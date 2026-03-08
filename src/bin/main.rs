//! # ESP32 WiFi Dashboard
//!
//! A bare-metal `no_std` Rust application for the ESP32 that:
//!   - Connects to a WPA2 WiFi network
//!   - Acquires an IP address via DHCP
//!   - Makes periodic HTTP GET requests
//!   - Renders a live dashboard on a 128×128 ST7735s TFT display
//!
//! ## Display layout (128×128 px)
//! ```
//! ┌────────────────────────┐  y=0
//! │  ESP32  DASHBOARD   ●  │  header bar (navy), wifi status dot top-right
//! ├════════════════════════┤  y=16  cyan accent rule
//! │ IP          10.0.0.42  │  y=18  DHCP-assigned IP address
//! │ UP          00:04:32   │  y=32  uptime since boot (HH:MM:SS)
//! ├────────────────────────┤  y=44  section divider
//! │ REQUESTS               │  y=46  section label
//! │ OK                  42 │  y=58  successful HTTP requests (2xx/3xx)
//! │ ERR                  3 │  y=70  failed requests (4xx/5xx or open error)
//! ├────────────────────────┤  y=82  section divider
//! │ LAST RESPONSE          │  y=84  section label
//! │          200           │  y=96  last HTTP status code (large, color-coded)
//! │       1842 bytes       │  y=120 response body size
//! └────────────────────────┘  y=128
//! ```
//!
//! ## Hardware wiring
//! | Signal | ESP32 GPIO |
//! |--------|-----------|
//! | DC     | GPIO2     |
//! | RST    | GPIO4     |
//! | CS     | GPIO5     |
//! | SCK    | GPIO18    |
//! | MOSI   | GPIO23    |
//!
//! ## Building
//! ```sh
//! SSID=YourNetwork PASSWORD=YourPassword cargo run --release
//! ```

#![no_std]   // No Rust standard library — we are running on bare metal
#![no_main]  // No Rust runtime entry point — esp-hal provides #[main]
#![deny(
    clippy::mem_forget,
    // mem::forget on esp-hal types is dangerous: peripherals and DMA buffers
    // must be properly dropped so hardware state is cleaned up correctly.
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

// ── Imports ───────────────────────────────────────────────────────────────

// embedded-graphics: 2D drawing library for displays.
// We use it for filled rectangles (UI panels/backgrounds) and anti-aliased
// mono-font text rendering. Rgb565 is the native 16-bit colour format of
// the ST7735s display controller.
use embedded_graphics::{
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_10X20}, // FONT_6X10 for labels, FONT_10X20 for status code
    },
    pixelcolor::Rgb565,
    prelude::*,                          // DrawTarget, Point, Size, etc.
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Text, TextStyleBuilder},
};

// mipidsi: driver for MIPI DCS-compatible display controllers over SPI.
// ST7735s is the specific controller on our 128×128 TFT module.
// SpiInterface wraps our SPI bus into the interface the driver expects.
// ColorOrder::Bgr corrects the red/blue channel swap on this panel variant.
use mipidsi::{
    Builder,
    interface::SpiInterface,
    models::ST7735s,
    options::ColorOrder,
};

// esp-hal: bare-metal HAL (Hardware Abstraction Layer) for the ESP32.
use esp_hal::{
    delay::Delay,                        // Busy-wait delay using the CPU cycle counter
    gpio::{Level, Output, OutputConfig}, // GPIO output pins for DC, RST, CS
    main,                                // Entry-point macro that sets up the Xtensa runtime
    spi::{
        Mode,                            // SPI clock polarity/phase: Mode::_0 = CPOL=0, CPHA=0
        master::{Config, Spi},           // SPI master peripheral
    },
    time::Rate,                          // Frequency type used for SPI clock configuration
};

// blocking-network-stack: a thin synchronous wrapper around smoltcp that
// provides a simple blocking Socket API (open / read / write / disconnect).
use blocking_network_stack::Stack;

// core::fmt::Write provides the write!() macro for formatting into
// heapless::String buffers (stack-allocated strings with fixed capacity).
// We import it as the bare name `Write` because that is what write!() looks for.
use core::fmt::Write;

// embedded-io: I/O traits for no_std. We use Read and Write (aliased IoWrite
// to avoid clashing with core::fmt::Write) for socket operations.
use embedded_io::{Read, Write as IoWrite};

use esp_hal::clock::CpuClock;           // CPU clock speed selection
use esp_hal::peripherals::Peripherals;  // Singleton struct holding all ESP32 peripherals
use esp_hal::rng::Rng;                  // Hardware random number generator (used for smoltcp seed)
use esp_hal::time::{Duration, Instant}; // Monotonic time types built on the ESP32 cycle counter
use esp_hal::timer::timg::TimerGroup;   // Timer group 0: required by esp-rtos for its tick source
use esp_println::{self as _, println};  // UART-backed println! (self as _ ensures the linker
                                        // keeps the UART init symbol even if only println! is used)
use esp_radio::wifi::{
    ClientConfig,    // WiFi STA (client) configuration: SSID + password
    ModeConfig,      // Enum wrapper: Client | AccessPoint | ApSta
    ScanConfig,      // Parameters for an active WiFi scan
    WifiController,  // Handle to the WiFi radio; used to start/connect/query state
};

// smoltcp: a small, no_std TCP/IP stack. We use it for:
//   - Interface: binds smoltcp to the WifiDevice network driver
//   - SocketSet / SocketStorage: storage arena for all sockets (DHCP + TCP)
use smoltcp::iface::{SocketSet, SocketStorage};
use smoltcp::wire::{
    DhcpOption, // Used to set a DHCP hostname option (option 12)
    IpAddress,  // Used to specify the remote HTTP server IP
};

// ── Panic handler ─────────────────────────────────────────────────────────

/// Minimal panic handler required by `no_std`.
///
/// In production embedded code a panic usually means an unrecoverable error.
/// We spin forever rather than attempting a restart to make the failure visible
/// (the watchdog will reset the chip if enabled, or you can attach a debugger).
#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

// The `alloc` crate provides heap allocation (Box, Vec, String, etc.).
// esp-alloc sets up the allocator via esp_alloc::heap_allocator!().
extern crate alloc;

// Generates the app descriptor block expected by the ESP-IDF second-stage
// bootloader. Without this the bootloader may refuse to run the image.
esp_bootloader_esp_idf::esp_app_desc!();

// ── Configuration ─────────────────────────────────────────────────────────

/// WiFi SSID — set at compile time via `SSID=<name> cargo build`.
const SSID: &str = env!("SSID");

/// WiFi password — set at compile time via `PASSWORD=<pass> cargo build`.
const PASSWORD: &str = env!("PASSWORD");

// ── Colour palette (Rgb565) ───────────────────────────────────────────────
//
// Rgb565::new(r, g, b) takes:
//   r: 0–31  (5 bits)
//   g: 0–63  (6 bits)
//   b: 0–31  (5 bits)

/// Background colour — pure black for maximum contrast.
const BG: Rgb565 = Rgb565::BLACK;

/// Header bar background — very dark navy blue.
const HEADER_BG: Rgb565 = Rgb565::new(0, 8, 16);

/// Primary accent — bright cyan. Used for IP label, header title, and scan line.
const ACCENT_CYAN: Rgb565 = Rgb565::new(0, 63, 31);

/// Success accent — lime green. Used for OK count, uptime label, progress bar.
const ACCENT_LIME: Rgb565 = Rgb565::new(4, 50, 4);

/// Error accent — red-orange. Used for ERR count and 4xx/5xx status codes.
const ACCENT_RED: Rgb565 = Rgb565::new(28, 10, 4);

/// Warning accent — amber/yellow. Used for 3xx redirect status codes.
const ACCENT_AMBER: Rgb565 = Rgb565::new(31, 40, 0);

/// Primary text — white. Used for all dynamic values (IP, uptime, counts).
const TEXT_WHITE: Rgb565 = Rgb565::WHITE;

/// Dimmed text — dark grey-green. Used for section labels and byte count.
const TEXT_DIM: Rgb565 = Rgb565::new(12, 22, 12);

/// Section divider line colour — very dark, barely visible rule.
const DIVIDER: Rgb565 = Rgb565::new(4, 10, 6);

/// WiFi status dot colour when connected — bright green.
const WIFI_ON: Rgb565 = Rgb565::new(4, 55, 4);

/// WiFi status dot colour when disconnected — dark red.
const WIFI_OFF: Rgb565 = Rgb565::new(28, 8, 4);

// ── Dashboard state ───────────────────────────────────────────────────────

/// All runtime values needed to render one frame of the dashboard.
///
/// Passed by reference into [`draw_dynamic_ui`] on every HTTP cycle.
/// Using a struct keeps the draw function signature clean and makes it easy
/// to add new fields later without changing call sites everywhere.
struct DashState<'a> {
    /// DHCP-assigned IP address as a pre-formatted string, e.g. "192.168.0.107".
    /// Set to "---" before WiFi is connected.
    ip: &'a str,

    /// Elapsed time since boot broken into (hours, minutes, seconds).
    /// Computed from `Instant::now() - start` on every loop iteration.
    uptime_hms: (u8, u8, u8),

    /// True if the WiFi STA interface is currently associated.
    /// Controls the colour of the status dot in the header.
    wifi_up: bool,

    /// Cumulative count of HTTP requests that returned a 2xx or 3xx status.
    ok_count: u32,

    /// Cumulative count of HTTP requests that failed to open, timed out,
    /// or returned a 4xx/5xx status code.
    err_count: u32,

    /// HTTP status code from the most recent response, e.g. 200.
    /// 0 means no response has been received yet.
    last_status: u16,

    /// Total number of bytes received in the most recent HTTP response body.
    /// 0 before the first successful request.
    last_bytes: usize,
}

// ── Entry point ───────────────────────────────────────────────────────────

/// Application entry point.
///
/// Initialises hardware in this order:
///   1. CPU clock, heap allocator, GPIO, SPI, display
///   2. Boot animation
///   3. WiFi radio, DHCP stack
///   4. Hands off to [`http_loop`] which runs forever
#[main]
fn main() -> ! {
    // Initialise the ESP32: sets CPU clock, returns the peripheral singleton.
    // Must be called before any peripheral is used.
    let peripherals = init_hardware();

    // Busy-wait delay used during display init and the boot animation.
    // Delay is zero-cost (no hardware resource) — it counts CPU cycles.
    let mut delay = Delay::new();

    // Allocate a second heap region in DRAM bank 2 (uninitialised memory).
    // This gives us ~96 KB extra heap beyond the 72 KB allocated in init_hardware().
    // The #[unsafe(link_section)] attribute places the backing array in the
    // correct linker section so the memory map is valid.
    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 98767);

    // ── GPIO setup ────────────────────────────────────────────────────
    // DC (Data/Command): LOW = command byte, HIGH = pixel data.
    // RST (Reset): active-low; initialised HIGH (not in reset).
    // CS  (Chip Select): active-low; initialised HIGH (deselected).
    let dc  = Output::new(peripherals.GPIO2, Level::Low,  OutputConfig::default());
    let rst = Output::new(peripherals.GPIO4, Level::High, OutputConfig::default());
    let cs  = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());

    // ── SPI bus ───────────────────────────────────────────────────────
    // SPI2 (HSPI) peripheral running in Mode 0 (CPOL=0, CPHA=0) at 40 MHz.
    // SCK on GPIO18, MOSI on GPIO23 — no MISO since the display is write-only.
    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO18)
    .with_mosi(peripherals.GPIO23);

    // ExclusiveDevice wraps the SPI bus + CS pin into a single `SpiDevice`
    // that automatically asserts/deasserts CS around each transaction.
    // `delay` is consumed here — it is used for the CS setup/hold timing.
    let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(spi, cs, delay).unwrap();

    // The SpiInterface needs a scratch buffer to stage pixel data before
    // writing it to the SPI FIFO. 512 bytes is sufficient for this display.
    let mut buffer = [0u8; 512];
    let di = SpiInterface::new(spi_device, dc, &mut buffer);

    // ── Display initialisation ────────────────────────────────────────
    // mipidsi::Builder configures the ST7735s controller:
    //   - display_size: physical pixel dimensions of this panel
    //   - color_order: Bgr fixes red/blue swap on this panel variant
    //   - init: sends the initialisation command sequence and returns the display
    let mut display = Builder::new(ST7735s, di)
        .reset_pin(rst)
        .display_size(128, 128)
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .unwrap();

    // Clear to black before drawing anything to avoid showing uninitialised GRAM.
    display.clear(BG).unwrap();

    // ── Boot animation then initial dashboard state ────────────────────
    boot_animation(&mut display, &mut delay);
    draw_static_ui(&mut display);

    // Show the "waiting for WiFi" state immediately so the screen isn't blank
    // while the radio initialises (which can take a few seconds).
    draw_dynamic_ui(&mut display, &DashState {
        ip:          "---",
        uptime_hms:  (0, 0, 0),
        wifi_up:     false,
        ok_count:    0,
        err_count:   0,
        last_status: 0,
        last_bytes:  0,
    });

    // ── WiFi / network setup ──────────────────────────────────────────

    // Timer group 0 is required by esp-rtos as its millisecond tick source.
    // esp_rtos::start() must be called before esp_radio::init().
    let timg0 = TimerGroup::new(peripherals.TIMG0);

    // Hardware RNG — used to seed the smoltcp TCP sequence number generator.
    // Providing a good random seed prevents TCP sequence number collisions.
    let rng = Rng::new();

    esp_rtos::start(timg0.timer0);

    // Initialise the radio subsystem. This powers on the RF frontend and
    // prepares the internal event loop used by esp-radio.
    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");

    // Create a WiFi controller and obtain the STA (station/client) network device.
    // `interfaces.sta` implements smoltcp's `Device` trait so smoltcp can
    // send and receive Ethernet frames through it.
    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let mut device = interfaces.sta;

    // ── smoltcp socket arena ──────────────────────────────────────────
    // smoltcp requires all socket storage to be provided by the caller
    // (no internal heap allocation). We preallocate 3 slots:
    //   slot 0: DHCPv4 socket (used during IP acquisition)
    //   slots 1–2: reserved for the TCP socket used by http_loop
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);

    // DHCPv4 socket: sends DISCOVER/REQUEST and processes OFFER/ACK messages.
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();

    // Set DHCP option 12 (hostname) so this device appears as "implRust"
    // in the router's DHCP lease table.
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"implRust",
    }]);
    socket_set.add(dhcp_socket);

    // smoltcp needs a monotonic millisecond clock. We provide a closure that
    // reads the ESP32's hardware cycle counter via esp-hal's Instant type.
    let now = || Instant::now().duration_since_epoch().as_millis();

    // blocking-network-stack wraps smoltcp's Interface + SocketSet behind a
    // simple synchronous API. rng.random() seeds TCP ISN generation.
    let mut stack = Stack::new(
        create_interface(&mut device),
        device,
        socket_set,
        now,
        rng.random(),
    );

    // ── WiFi connection sequence ──────────────────────────────────────
    configure_wifi(&mut wifi_controller); // Set SSID/password, start radio
    scan_wifi(&mut wifi_controller);      // Active scan — prints nearby APs to serial
    connect_wifi(&mut wifi_controller);   // Associate and authenticate; blocks until done
    obtain_ip(&mut stack);                // Run DHCP; blocks until an IP lease is received

    // Format the acquired IP address into a fixed-capacity stack string.
    // This string lives for the rest of the program and is passed by reference
    // into http_loop, avoiding repeated formatting on every loop iteration.
    let ip_info = stack.get_ip_info().unwrap();
    let mut ip_buf: heapless::String<16> = heapless::String::new();
    write!(ip_buf, "{}", ip_info.ip).ok();

    // Record the boot timestamp so we can compute uptime inside http_loop.
    let start = Instant::now();

    // Allocate TCP socket buffers on the stack. 1536 bytes (roughly one
    // Ethernet MTU) is enough for a small HTTP response.
    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    // Enter the main HTTP request loop — this never returns.
    http_loop(socket, &mut display, ip_buf.as_str(), start)
}

// ── HTTP request loop ─────────────────────────────────────────────────────

/// Main application loop: makes HTTP GET requests and updates the display.
///
/// This function never returns (`-> !`). On every iteration it:
///   1. Computes current uptime
///   2. Opens a TCP connection to the target server
///   3. Sends a minimal HTTP/1.0 GET request
///   4. Reads the response, tracking total bytes and parsing the status code
///   5. Updates the display via [`draw_dynamic_ui`]
///   6. Waits 5 + 1 seconds before the next request
///
/// # Why HTTP/1.0?
/// HTTP/1.0 uses connection-close semantics — the server closes the TCP
/// connection after sending the response. This gives us a clean EOF signal
/// (a `read()` returning `Ok(0)`) so we know when the response is complete
/// without needing to parse `Content-Length` or chunked transfer encoding.
///
/// # Generic parameters
/// The function is generic over the display interface (`DI`) and controller
/// model (`MODEL`) so it works with any mipidsi-compatible display without
/// requiring trait objects (which would need heap allocation).
fn http_loop<DI, MODEL>(
    mut socket: blocking_network_stack::Socket<'_, '_, esp_radio::wifi::WifiDevice<'_>>,
    display: &mut mipidsi::Display<DI, MODEL, Output<'_>>,
    ip_str: &str,
    start: Instant,
) -> !
where
    DI: mipidsi::interface::Interface,
    MODEL: mipidsi::models::Model<ColorFormat = Rgb565>,
    Rgb565: mipidsi::interface::InterfacePixelFormat<DI::Word>,
{
    let delay = Delay::new();
    let mut ok_count:  u32 = 0;  // Requests with 1xx–3xx status
    let mut err_count: u32 = 0;  // Requests that failed to open or got 4xx–5xx

    loop {
        // Pump the smoltcp stack at the top of each iteration to process any
        // pending ARP, TCP keepalive, or DHCP renewal packets.
        socket.work();

        // Compute uptime by subtracting the boot Instant from now.
        // as_secs() returns a u64; we cast each component to u8 which is
        // safe because hours wraps at 256 (about 10 days of uptime).
        let elapsed = (Instant::now() - start).as_secs();
        let h = (elapsed / 3600) as u8;
        let m = ((elapsed % 3600) / 60) as u8;
        let s = (elapsed % 60) as u8;

        println!("Making HTTP request");

        // Target: www.mobile-j.de — a small test HTTP server.
        // IP is hardcoded to avoid needing a DNS resolver.
        let remote_addr = IpAddress::v4(142, 250, 185, 115);
        let open_ok = socket.open(remote_addr, 80).is_ok();

        if open_ok {
            // Send a minimal HTTP/1.0 request. HTTP/1.0 (not 1.1) is used
            // deliberately because it guarantees the server will close the
            // connection after the response, giving us a clean EOF.
            socket
                .write(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
                .unwrap();

            // flush() ensures the request bytes are actually sent over the wire
            // and not held in a transmit buffer.
            socket.flush().unwrap();

            // Set a 20-second deadline. If the server takes longer than this
            // to send the complete response we abort and count it as an error.
            let deadline = Instant::now() + Duration::from_secs(20);
            let mut total_bytes: usize = 0;
            let mut status_code: u16   = 0;
            let mut first_chunk        = true; // True until we've parsed the status line
            let mut buf = [0u8; 512];          // Scratch buffer for each read chunk

            loop {
                // Pump smoltcp on every iteration of the read loop.
                // Without this, the stack cannot process incoming TCP segments
                // and socket.read() will block forever waiting for data that
                // has already arrived at the IP layer but not been delivered.
                socket.work();

                // Check deadline before blocking on read.
                if Instant::now() > deadline {
                    println!("Timeout");
                    break;
                }

                match socket.read(&mut buf) {
                    Ok(0) => {
                        // A zero-length read means the server has closed its
                        // end of the TCP connection (FIN received). This is
                        // the normal end-of-response signal for HTTP/1.0.
                        println!("EOF");
                        break;
                    }
                    Ok(len) => {
                        total_bytes += len;

                        // The HTTP status code sits at bytes 9–11 of the very
                        // first response line: "HTTP/1.x NNN Reason\r\n"
                        // We only parse it once (from the first chunk) because
                        // subsequent chunks are body data.
                        if first_chunk && len >= 12 {
                            first_chunk = false;
                            if let Ok(s) = core::str::from_utf8(&buf[9..12]) {
                                status_code = s.trim().parse().unwrap_or(0);
                            }
                        }

                        // Echo response text to the serial console for debugging.
                        if let Ok(text) = core::str::from_utf8(&buf[..len]) {
                            println!("{}", text);
                        }
                    }
                    Err(_) => {
                        // Any read error (connection reset, protocol error, etc.)
                        // exits the read loop cleanly. The display will show
                        // whatever partial data we collected.
                        println!("Read done");
                        break;
                    }
                }
            }

            // Classify the response: anything below 400 is "OK", everything
            // else (including status 0 meaning we never parsed a status line)
            // is counted as an error.
            if status_code > 0 {
                if status_code < 400 { ok_count += 1; } else { err_count += 1; }
            } else {
                err_count += 1;
            }

            // Redraw the dynamic section of the dashboard with fresh data.
            draw_dynamic_ui(display, &DashState {
                ip: ip_str,
                uptime_hms: (h, m, s),
                wifi_up: true,
                ok_count,
                err_count,
                last_status: status_code,
                last_bytes: total_bytes,
            });

            // Cleanly close the TCP connection before the next request.
            socket.disconnect();
        } else {
            // TCP open failed — server unreachable or stack busy.
            err_count += 1;
            draw_dynamic_ui(display, &DashState {
                ip: ip_str,
                uptime_hms: (h, m, s),
                wifi_up: false,
                ok_count,
                err_count,
                last_status: 0,
                last_bytes: 0,
            });
        }

        // Cool-down period: pump the stack for 5 seconds before the next
        // request. This gives smoltcp time to process the TCP TIME_WAIT state
        // and any pending DHCP renewals.
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            socket.work();
        }

        // Additional 1-second hard delay before the next iteration.
        delay.delay_millis(1000);
    }
}

// ── Boot animation ────────────────────────────────────────────────────────

/// Hacker-style boot animation displayed once at startup before the dashboard.
///
/// Total duration: approximately 3.5 seconds. Sequence:
///
/// 1. **Flash** — screen briefly goes white then cuts to black (simulates CRT power-on).
/// 2. **Scanline sweep** — a dim green horizontal band races from top to bottom.
/// 3. **Typewriter** — `"INITIALIZING..."` types out one character at a time
///    with a block cursor.
/// 4. **Glitch title** — the project name starts as random noise and resolves
///    character-by-character into `"WiFi Dashboard"`. Uses a deterministic LCG
///    (linear congruential generator) so no hardware RNG is needed.
/// 5. **Progress bar** — a cyan-outlined bar fills with lime green over 20 steps.
/// 6. **Wipe** — 16 black rows sweep top-to-bottom, clearing the screen ready
///    for the main dashboard.
fn boot_animation<DI, MODEL>(
    display: &mut mipidsi::Display<DI, MODEL, Output<'_>>,
    delay: &mut Delay,
)
where
    DI: mipidsi::interface::Interface,
    MODEL: mipidsi::models::Model<ColorFormat = Rgb565>,
    Rgb565: mipidsi::interface::InterfacePixelFormat<DI::Word>,
{
    let centered = TextStyleBuilder::new().alignment(Alignment::Center).build();
    let left     = TextStyleBuilder::new().alignment(Alignment::Left).build();

    // ── 1. Flash ─────────────────────────────────────────────────────
    // Briefly fill the screen white then back to black. The visual pop
    // draws attention and signals that the device is alive.
    display.clear(Rgb565::WHITE).unwrap();
    delay.delay_millis(60);
    display.clear(BG).unwrap();
    delay.delay_millis(40);

    // ── 2. Scanline sweep ─────────────────────────────────────────────
    // A narrow bright band moves from y=0 to y=128 in 8-pixel steps.
    // Each frame erases the previous band before drawing the new one,
    // creating a "flying line" effect reminiscent of old CRT scanlines.
    let scan_color = Rgb565::new(4, 40, 20); // Dim green tint
    for step in 0u16..=16 {
        // Erase the band drawn in the previous frame.
        if step > 0 {
            let prev_y = ((step - 1) * 8) as i32;
            Rectangle::new(Point::new(0, prev_y), Size::new(128, 8))
                .into_styled(PrimitiveStyle::with_fill(BG))
                .draw(display).unwrap();
        }
        // Draw the new band at the current position (3px tall inside an 8px step).
        let y = (step * 8) as i32;
        if y < 128 {
            Rectangle::new(Point::new(0, y), Size::new(128, 3))
                .into_styled(PrimitiveStyle::with_fill(scan_color))
                .draw(display).unwrap();
        }
        delay.delay_millis(18);
    }
    display.clear(BG).unwrap();
    delay.delay_millis(120);

    // ── 3. Typewriter ─────────────────────────────────────────────────
    // Append one character at a time to a heapless::String and redraw.
    // A filled rectangle after the last character simulates a cursor block.
    let init_msg = b"INITIALIZING...";
    let mut typed: heapless::String<20> = heapless::String::new();

    for &ch in init_msg {
        // Clear the text row before redrawing to prevent character ghosting.
        Rectangle::new(Point::new(0, 54), Size::new(128, 12))
            .into_styled(PrimitiveStyle::with_fill(BG))
            .draw(display).unwrap();

        typed.push(ch as char).ok(); // ok() discards the overflow error (capacity guaranteed)

        Text::with_text_style(
            typed.as_str(),
            Point::new(4, 64),
            MonoTextStyle::new(&FONT_6X10, ACCENT_LIME),
            left,
        ).draw(display).unwrap();

        // Cursor block: positioned 6 pixels (one FONT_6X10 character width) after
        // the last drawn character.
        let cursor_x = 4 + (typed.len() as i32) * 6;
        Rectangle::new(Point::new(cursor_x, 55), Size::new(5, 9))
            .into_styled(PrimitiveStyle::with_fill(ACCENT_LIME))
            .draw(display).unwrap();

        delay.delay_millis(45);
    }
    delay.delay_millis(200);

    // ── 4. Glitch title ───────────────────────────────────────────────
    // Each of 12 frames shows a mix of random characters and correct ones.
    // The resolved prefix grows by (len / 11) characters per frame so that
    // by frame 11 the entire string is correct.
    //
    // Random chars come from a Linear Congruential Generator seeded with a
    // fixed constant (0xDEAD_BEEF). Using a fixed seed makes the animation
    // deterministic (same noise pattern every boot) without consuming the
    // hardware RNG peripheral that smoltcp needs later.
    let target       = "WiFi Dashboard";
    let target_bytes = target.as_bytes();
    let len          = target_bytes.len();
    let mut lcg: u32 = 0xDEAD_BEEF;

    for frame in 0u8..12 {
        // Clear the title area before each frame.
        Rectangle::new(Point::new(0, 36), Size::new(128, 22))
            .into_styled(PrimitiveStyle::with_fill(BG))
            .draw(display).unwrap();

        let mut glitch: heapless::String<20> = heapless::String::new();

        // Number of characters that have "resolved" to their correct value.
        let resolved_up_to = (frame as usize * len) / 11;

        for i in 0..len {
            if i < resolved_up_to {
                // This character has resolved — use the real one.
                glitch.push(target_bytes[i] as char).ok();
            } else {
                // Still noisy — advance LCG and pick a printable ASCII char.
                // LCG recurrence: X_{n+1} = 1664525 * X_n + 1013904223 (mod 2^32)
                // This is the Numerical Recipes / ANSI C parameters.
                lcg = lcg.wrapping_mul(1664525).wrapping_add(1013904223);
                // Extract bits 16–21 to get a value in 0x21–0x3F ('!' to '>').
                let c = ((lcg >> 16) & 0x3E) as u8 + 0x21;
                glitch.push(c as char).ok();
            }
        }

        // Alternate colour each frame for a flicker effect.
        let color = if frame % 2 == 0 { ACCENT_CYAN } else { ACCENT_LIME };
        Text::with_text_style(
            glitch.as_str(),
            Point::new(64, 52),
            MonoTextStyle::new(&FONT_10X20, color),
            centered,
        ).draw(display).unwrap();

        delay.delay_millis(55);
    }

    // Draw the final resolved title in solid white.
    Rectangle::new(Point::new(0, 36), Size::new(128, 22))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(display).unwrap();

    Text::with_text_style(
        target,
        Point::new(64, 52),
        MonoTextStyle::new(&FONT_10X20, TEXT_WHITE),
        centered,
    ).draw(display).unwrap();

    delay.delay_millis(300);

    // ── 5. Progress bar ───────────────────────────────────────────────
    // A "BOOTING" label above a 100×8 pixel outlined bar that fills in
    // 20 incremental steps. Pure cosmetic — represents no real progress.
    Text::with_text_style(
        "BOOTING",
        Point::new(64, 82),
        MonoTextStyle::new(&FONT_6X10, TEXT_DIM),
        centered,
    ).draw(display).unwrap();

    // Draw the 1-pixel cyan outline around the bar area.
    Rectangle::new(Point::new(14, 88), Size::new(100, 8))
        .into_styled(PrimitiveStyle::with_stroke(ACCENT_CYAN, 1))
        .draw(display).unwrap();

    // Fill lime green from left to right, 96 inner pixels over 20 steps.
    // Each step extends the fill rectangle rather than drawing individual
    // segments, so there are no gaps between steps.
    for step in 1u32..=20 {
        let fill_w = (step * 96) / 20; // Integer division — grows 4–5 px per step
        Rectangle::new(Point::new(16, 90), Size::new(fill_w, 4))
            .into_styled(PrimitiveStyle::with_fill(ACCENT_LIME))
            .draw(display).unwrap();
        delay.delay_millis(35);
    }

    delay.delay_millis(250);

    // ── 6. Wipe to black ─────────────────────────────────────────────
    // Sweep 16 black rectangles (8px each) top-to-bottom to clear the
    // animation. The row-by-row approach creates a shutter/blind effect
    // rather than an abrupt screen clear.
    for row in 0u32..16 {
        Rectangle::new(Point::new(0, (row * 8) as i32), Size::new(128, 8))
            .into_styled(PrimitiveStyle::with_fill(BG))
            .draw(display).unwrap();
        delay.delay_millis(8);
    }
}

// ── Static UI chrome ──────────────────────────────────────────────────────

/// Draws the fixed (non-updating) chrome of the dashboard.
///
/// Called once after the boot animation. Renders elements that never change:
/// the header bar and title, section dividers, and section labels.
///
/// Separating static from dynamic rendering avoids redrawing the entire
/// screen on every HTTP cycle — only the value areas are repainted, which
/// prevents flicker and reduces SPI bus usage.
fn draw_static_ui<DI, MODEL>(display: &mut mipidsi::Display<DI, MODEL, Output<'_>>)
where
    DI: mipidsi::interface::Interface,
    MODEL: mipidsi::models::Model<ColorFormat = Rgb565>,
    Rgb565: mipidsi::interface::InterfacePixelFormat<DI::Word>,
{
    let left     = TextStyleBuilder::new().alignment(Alignment::Left).build();
    let centered = TextStyleBuilder::new().alignment(Alignment::Center).build();

    // Header bar — fills y=0..15 with navy background.
    Rectangle::new(Point::new(0, 0), Size::new(128, 16))
        .into_styled(PrimitiveStyle::with_fill(HEADER_BG))
        .draw(display).unwrap();

    // Accent rule — 1px cyan line immediately below the header (y=16).
    // Provides a clean visual boundary between the header and the data rows.
    Rectangle::new(Point::new(0, 16), Size::new(128, 1))
        .into_styled(PrimitiveStyle::with_fill(ACCENT_CYAN))
        .draw(display).unwrap();

    // Title text — centred on x=58 (slightly left of centre) to leave room
    // for the WiFi status dot at x=120–126.
    // Text baseline sits at y=12, which is vertically centred in the 16px header.
    Text::with_text_style(
        "ESP32  DASHBOARD",
        Point::new(58, 12),
        MonoTextStyle::new(&FONT_6X10, ACCENT_CYAN),
        centered,
    ).draw(display).unwrap();

    // Section divider 1 — thin dark rule separating the info rows from requests.
    Rectangle::new(Point::new(0, 44), Size::new(128, 1))
        .into_styled(PrimitiveStyle::with_fill(DIVIDER))
        .draw(display).unwrap();

    // "REQUESTS" section label. Baseline y=56, drawn in dim colour so it
    // recedes visually behind the brighter OK/ERR values below it.
    Text::with_text_style(
        "REQUESTS",
        Point::new(4, 56),
        MonoTextStyle::new(&FONT_6X10, TEXT_DIM),
        left,
    ).draw(display).unwrap();

    // Section divider 2 — separates the request counters from the HTTP response section.
    Rectangle::new(Point::new(0, 82), Size::new(128, 1))
        .into_styled(PrimitiveStyle::with_fill(DIVIDER))
        .draw(display).unwrap();

    // "LAST RESPONSE" section label. Baseline y=94.
    Text::with_text_style(
        "LAST RESPONSE",
        Point::new(4, 94),
        MonoTextStyle::new(&FONT_6X10, TEXT_DIM),
        left,
    ).draw(display).unwrap();
}

// ── Dynamic UI ────────────────────────────────────────────────────────────

/// Repaints all dynamic (changing) content of the dashboard.
///
/// Called after every HTTP request cycle with a freshly populated [`DashState`].
///
/// Each row is repainted in two steps:
///   1. A filled black rectangle erases the old value (prevents ghost text
///      when a shorter string replaces a longer one, e.g. "200" → "---").
///   2. The new text is drawn on top.
///
/// The static chrome drawn by [`draw_static_ui`] is intentionally not touched
/// here, keeping SPI traffic minimal.
fn draw_dynamic_ui<DI, MODEL>(
    display: &mut mipidsi::Display<DI, MODEL, Output<'_>>,
    state: &DashState<'_>,
)
where
    DI: mipidsi::interface::Interface,
    MODEL: mipidsi::models::Model<ColorFormat = Rgb565>,
    Rgb565: mipidsi::interface::InterfacePixelFormat<DI::Word>,
{
    let left     = TextStyleBuilder::new().alignment(Alignment::Left).build();
    let right    = TextStyleBuilder::new().alignment(Alignment::Right).build();
    let centered = TextStyleBuilder::new().alignment(Alignment::Center).build();

    // ── WiFi status dot ───────────────────────────────────────────────
    // 6×6 px square at the top-right corner of the header.
    // Green = connected, red = disconnected. No erase needed since the
    // dot always covers the same area regardless of colour.
    let dot_color = if state.wifi_up { WIFI_ON } else { WIFI_OFF };
    Rectangle::new(Point::new(120, 5), Size::new(6, 6))
        .into_styled(PrimitiveStyle::with_fill(dot_color))
        .draw(display).unwrap();

    // ── IP address row (y=18..30) ─────────────────────────────────────
    // Label "IP" left-aligned in cyan; value right-aligned in white.
    // Max value length: "255.255.255.255" = 15 chars × 6px = 90px, fits in 120px value area.
    Rectangle::new(Point::new(0, 18), Size::new(128, 13))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(display).unwrap();

    Text::with_text_style(
        "IP",
        Point::new(4, 28),
        MonoTextStyle::new(&FONT_6X10, ACCENT_CYAN),
        left,
    ).draw(display).unwrap();

    Text::with_text_style(
        state.ip,
        Point::new(124, 28),  // Right-aligned anchor at x=124 (4px margin from right edge)
        MonoTextStyle::new(&FONT_6X10, TEXT_WHITE),
        right,
    ).draw(display).unwrap();

    // ── Uptime row (y=32..44) ─────────────────────────────────────────
    // Label "UP" in lime; value "HH:MM:SS" right-aligned in white.
    // Max value width: "255:59:59" = 8 chars × 6px = 48px, well within range.
    Rectangle::new(Point::new(0, 32), Size::new(128, 13))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(display).unwrap();

    Text::with_text_style(
        "UP",
        Point::new(4, 42),
        MonoTextStyle::new(&FONT_6X10, ACCENT_LIME),
        left,
    ).draw(display).unwrap();

    // Format HH:MM:SS into a heapless::String<12>.
    // write!() requires core::fmt::Write to be in scope (imported at the top).
    let (h, m, s) = state.uptime_hms;
    let mut uptime_str: heapless::String<12> = heapless::String::new();
    write!(uptime_str, "{:02}:{:02}:{:02}", h, m, s).ok();

    Text::with_text_style(
        uptime_str.as_str(),
        Point::new(124, 42),
        MonoTextStyle::new(&FONT_6X10, TEXT_WHITE),
        right,
    ).draw(display).unwrap();

    // ── OK request count (y=58..69) ───────────────────────────────────
    // "OK" label in lime green; numeric count right-aligned in white.
    // Counts requests that received a 1xx–3xx HTTP status.
    Rectangle::new(Point::new(0, 58), Size::new(128, 12))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(display).unwrap();

    Text::with_text_style(
        "OK",
        Point::new(4, 68),
        MonoTextStyle::new(&FONT_6X10, ACCENT_LIME),
        left,
    ).draw(display).unwrap();

    let mut ok_str: heapless::String<10> = heapless::String::new();
    write!(ok_str, "{}", state.ok_count).ok();

    Text::with_text_style(
        ok_str.as_str(),
        Point::new(124, 68),
        MonoTextStyle::new(&FONT_6X10, TEXT_WHITE),
        right,
    ).draw(display).unwrap();

    // ── ERR request count (y=70..81) ──────────────────────────────────
    // "ERR" label in red-orange; numeric count right-aligned in white.
    // Counts requests that failed to open or received a 4xx–5xx status.
    Rectangle::new(Point::new(0, 70), Size::new(128, 12))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(display).unwrap();

    Text::with_text_style(
        "ERR",
        Point::new(4, 80),
        MonoTextStyle::new(&FONT_6X10, ACCENT_RED),
        left,
    ).draw(display).unwrap();

    let mut err_str: heapless::String<10> = heapless::String::new();
    write!(err_str, "{}", state.err_count).ok();

    Text::with_text_style(
        err_str.as_str(),
        Point::new(124, 80),
        MonoTextStyle::new(&FONT_6X10, TEXT_WHITE),
        right,
    ).draw(display).unwrap();

    // ── HTTP status code (y=96..119) ──────────────────────────────────
    // Displayed in FONT_10X20 (the largest available font) centred on the screen.
    // Color-coded by response class:
    //   2xx → lime   (success)
    //   3xx → amber  (redirect)
    //   4xx–5xx → red (client/server error)
    //   0 / unknown → dim grey
    Rectangle::new(Point::new(0, 96), Size::new(128, 24))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(display).unwrap();

    let status_color = match state.last_status {
        200..=299 => ACCENT_LIME,
        300..=399 => ACCENT_AMBER,
        400..=599 => ACCENT_RED,
        _         => TEXT_DIM,  // 0 (none yet) or unexpected value
    };

    let mut status_str: heapless::String<8> = heapless::String::new();
    if state.last_status == 0 {
        write!(status_str, "---").ok(); // Placeholder before first response
    } else {
        write!(status_str, "{}", state.last_status).ok();
    }

    // Baseline y=118: FONT_10X20 is 20px tall, so the top of the glyph is at y=98.
    Text::with_text_style(
        status_str.as_str(),
        Point::new(64, 118),
        MonoTextStyle::new(&FONT_10X20, status_color),
        centered,
    ).draw(display).unwrap();

    // ── Byte count (y=120..127) ───────────────────────────────────────
    // Shows the total bytes received in the last response, or "waiting..."
    // if no request has completed yet. Drawn in dim colour to de-emphasise.
    // Only 8px of vertical space remains before the bottom edge — FONT_6X10
    // with baseline at y=126 fits exactly (ascender at y=118, descender at y=128).
    Rectangle::new(Point::new(0, 120), Size::new(128, 8))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(display).unwrap();

    let mut bytes_str: heapless::String<20> = heapless::String::new();
    if state.last_bytes == 0 {
        write!(bytes_str, "waiting...").ok();
    } else {
        write!(bytes_str, "{} bytes", state.last_bytes).ok();
    }

    Text::with_text_style(
        bytes_str.as_str(),
        Point::new(64, 126),
        MonoTextStyle::new(&FONT_6X10, TEXT_DIM),
        centered,
    ).draw(display).unwrap();
}

// ── Network helpers ───────────────────────────────────────────────────────

/// Creates a smoltcp [`Interface`] bound to the WiFi STA network device.
///
/// The interface wraps the `WifiDevice` (which provides raw Ethernet frame
/// send/receive) with smoltcp's IP/ARP layer. The hardware Ethernet address
/// is read directly from the WiFi driver and used to configure smoltcp's
/// link-layer addressing.
///
/// [`Interface`]: smoltcp::iface::Interface
pub fn create_interface(device: &mut esp_radio::wifi::WifiDevice) -> smoltcp::iface::Interface {
    smoltcp::iface::Interface::new(
        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
        )),
        device,
        timestamp(), // Provides the current time for ARP cache timeouts etc.
    )
}

/// Returns the current time as a [`smoltcp::time::Instant`].
///
/// smoltcp requires a monotonic clock for managing TCP retransmit timers,
/// ARP cache expiry, and DHCP lease renewal. We convert from esp-hal's
/// `Instant` (based on the ESP32 cycle counter) to smoltcp's `Instant`
/// (microseconds since an arbitrary epoch).
fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros() as i64,
    )
}

/// Initialises the ESP32 hardware and returns the peripheral singleton.
///
/// Must be called exactly once at the start of `main`. Sets the CPU to its
/// maximum frequency and allocates a 72 KB heap in internal SRAM.
///
/// A second, larger heap region is allocated in `main` itself using
/// `#[link_section = ".dram2_uninit"]` to place it in a separate DRAM bank.
fn init_hardware() -> Peripherals {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // Primary heap: 72 KB in internal DRAM (fast, always available).
    // Used by smoltcp, esp-radio, and general alloc allocations.
    esp_alloc::heap_allocator!(size: 72 * 1024);

    peripherals
}

/// Configures and starts the WiFi radio in STA (client) mode.
///
/// Sets power saving to `None` (maximum radio activity) to avoid the latency
/// spikes that occur when the radio wakes from sleep between packets.
/// Power saving can be re-enabled if battery life matters more than latency.
fn configure_wifi(controller: &mut WifiController<'_>) {
    controller
        .set_power_saving(esp_radio::wifi::PowerSaveMode::None)
        .unwrap();

    let client_config = ModeConfig::Client(
        ClientConfig::default()
            .with_ssid(SSID.into())
            .with_password(PASSWORD.into()),
    );
    let res = controller.set_config(&client_config);
    println!("wifi_set_configuration returned {:?}", res);

    controller.start().unwrap();
    println!("is wifi started: {:?}", controller.is_started());
}

/// Performs an active WiFi scan and prints nearby access points to serial.
///
/// The scan result is informational only — we connect to the configured SSID
/// regardless of whether it appears in the scan. The scan is useful for
/// debugging signal strength and channel selection.
fn scan_wifi(controller: &mut WifiController<'_>) {
    println!("Start Wifi Scan");
    let scan_config = ScanConfig::default().with_max(10); // Cap at 10 results
    let res = controller.scan_with_config(scan_config).unwrap();
    for ap in res {
        println!("{:?}", ap);
    }
}

/// Connects to the configured WiFi network and blocks until associated.
///
/// Calls `controller.connect()` to initiate the 802.11 authentication and
/// association handshake, then polls `is_connected()` in a tight loop until
/// the radio driver confirms the link is up.
///
/// # Panics
/// Panics if `is_connected()` returns an error, which indicates a fatal
/// radio driver failure (not a normal connection timeout).
fn connect_wifi(controller: &mut WifiController<'_>) {
    println!("{:?}", controller.capabilities());
    println!("wifi_connect {:?}", controller.connect());

    println!("Wait to get connected");
    loop {
        match controller.is_connected() {
            Ok(true)  => break,
            Ok(false) => {}  // Still associating — keep polling
            Err(err)  => panic!("{:?}", err),
        }
    }
    println!("Connected: {:?}", controller.is_connected());
}

/// Runs the smoltcp stack until a DHCP lease is acquired.
///
/// Calls `stack.work()` in a tight loop, which drives the smoltcp event loop:
/// processing incoming Ethernet frames, running DHCP state machine transitions,
/// and sending outgoing packets. Blocks until `is_iface_up()` returns true,
/// which happens once smoltcp has assigned an IP address from the DHCP ACK.
fn obtain_ip(stack: &mut Stack<'_, esp_radio::wifi::WifiDevice<'_>>) {
    println!("Wait for IP address");
    loop {
        stack.work();
        if stack.is_iface_up() {
            println!("IP acquired: {:?}", stack.get_ip_info());
            break;
        }
    }
}