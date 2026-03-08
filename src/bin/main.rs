#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

use embedded_graphics::{
    mono_font::{
        MonoTextStyle,
        ascii::{FONT_6X10, FONT_10X20},
    },
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Text, TextStyleBuilder},
};

use mipidsi::{
    Builder,
    interface::SpiInterface,
    models::ST7735s,
    options::ColorOrder,
};
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    spi::{Mode, master::{Config, Spi}},
    time::Rate,
};
use blocking_network_stack::Stack;
use embedded_io::{Read, Write};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::Peripherals;
use esp_hal::rng::Rng;
use esp_hal::time::{Duration, Instant};
use esp_hal::timer::timg::TimerGroup;
use esp_println::{self as _, println};
use esp_radio::wifi::{ClientConfig, ModeConfig, ScanConfig, WifiController};
use smoltcp::iface::{SocketSet, SocketStorage};
use smoltcp::wire::{DhcpOption, IpAddress};
use core::fmt::Write as FmtWrite;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

// ── Colour palette ────────────────────────────────────────────────────────
const BG:           Rgb565 = Rgb565::BLACK;
const HEADER_BG:    Rgb565 = Rgb565::new(0, 8, 16);
const ACCENT_CYAN:  Rgb565 = Rgb565::new(0, 63, 31);
const ACCENT_LIME:  Rgb565 = Rgb565::new(4, 50, 4);
const ACCENT_RED:   Rgb565 = Rgb565::new(28, 10, 4);
const ACCENT_AMBER: Rgb565 = Rgb565::new(31, 40, 0);
const TEXT_WHITE:   Rgb565 = Rgb565::WHITE;
const TEXT_DIM:     Rgb565 = Rgb565::new(12, 22, 12);
const DIVIDER:      Rgb565 = Rgb565::new(4, 10, 6);
const WIFI_ON:      Rgb565 = Rgb565::new(4, 55, 4);
const WIFI_OFF:     Rgb565 = Rgb565::new(28, 8, 4);

// ── Dashboard state ───────────────────────────────────────────────────────
struct DashState<'a> {
    ip:          &'a str,
    uptime_hms:  (u8, u8, u8),
    wifi_up:     bool,
    ok_count:    u32,
    err_count:   u32,
    last_status: u16,   // 0 = not yet received
    last_bytes:  usize,
}

#[main]
fn main() -> ! {
    let peripherals = init_hardware();
    let mut delay = Delay::new();

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 98767);

    let dc = Output::new(peripherals.GPIO2, Level::Low,  OutputConfig::default());
    let rst = Output::new(peripherals.GPIO4, Level::High, OutputConfig::default());
    let cs  = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());

    let spi = Spi::new(
        peripherals.SPI2,
        Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sck(peripherals.GPIO18)
    .with_mosi(peripherals.GPIO23);

    let spi_device = embedded_hal_bus::spi::ExclusiveDevice::new(spi, cs, delay).unwrap();

    let mut buffer = [0u8; 512];
    let di = SpiInterface::new(spi_device, dc, &mut buffer);

    let mut display = Builder::new(ST7735s, di)
        .reset_pin(rst)
        .display_size(128, 128)           // ← 128×128
        .color_order(ColorOrder::Bgr)
        .init(&mut delay)
        .unwrap();

    display.clear(BG).unwrap();
    boot_animation(&mut display, &mut delay);

    draw_static_ui(&mut display);

    // Show "connecting…" state while WiFi comes up
    draw_dynamic_ui(&mut display, &DashState {
        ip:          "---",
        uptime_hms:  (0, 0, 0),
        wifi_up:     false,
        ok_count:    0,
        err_count:   0,
        last_status: 0,
        last_bytes:  0,
    });

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = Rng::new();

    esp_rtos::start(timg0.timer0);
    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");

    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let mut device = interfaces.sta;

    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"implRust",
    }]);
    socket_set.add(dhcp_socket);

    let now = || Instant::now().duration_since_epoch().as_millis();
    let mut stack = Stack::new(
        create_interface(&mut device),
        device,
        socket_set,
        now,
        rng.random(),
    );

    configure_wifi(&mut wifi_controller);
    scan_wifi(&mut wifi_controller);
    connect_wifi(&mut wifi_controller);
    obtain_ip(&mut stack);

    // Build IP string once
    let ip_info = stack.get_ip_info().unwrap();
    let mut ip_buf: heapless::String<16> = heapless::String::new();
    write!(ip_buf, "{}", ip_info.ip).ok();

    let start = Instant::now();

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    http_loop(socket, &mut display, ip_buf.as_str(), start)
}

// ── HTTP loop with live display updates ───────────────────────────────────
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
    let mut ok_count:  u32 = 0;
    let mut err_count: u32 = 0;

    loop {
        socket.work();

        // Uptime
        let elapsed = (Instant::now() - start).as_secs();
        let h = (elapsed / 3600) as u8;
        let m = ((elapsed % 3600) / 60) as u8;
        let s = (elapsed % 60) as u8;

        println!("Making HTTP request");

        let remote_addr = IpAddress::v4(142, 250, 185, 115);
        let open_ok = socket.open(remote_addr, 80).is_ok();

        if open_ok {
            socket
                .write(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
                .unwrap();
            socket.flush().unwrap();

            let deadline = Instant::now() + Duration::from_secs(20);
            let mut total_bytes: usize = 0;
            let mut status_code: u16   = 0;
            let mut first_chunk        = true;
            let mut buf = [0u8; 512];

            loop {
                // Keep the network stack alive while waiting for data
                socket.work();

                if Instant::now() > deadline {
                    println!("Timeout");
                    break;
                }

                match socket.read(&mut buf) {
                    Ok(0) => {
                        // EOF — server closed the connection
                        println!("EOF");
                        break;
                    }
                    Ok(len) => {
                        total_bytes += len;

                        // Parse "HTTP/1.x NNN" from first chunk
                        if first_chunk && len >= 12 {
                            first_chunk = false;
                            if let Ok(s) = core::str::from_utf8(&buf[9..12]) {
                                status_code = s.trim().parse().unwrap_or(0);
                            }
                        }

                        if let Ok(text) = core::str::from_utf8(&buf[..len]) {
                            println!("{}", text);
                        }
                    }
                    Err(_) => {
                        // Connection closed or error
                        println!("Read done");
                        break;
                    }
                }
            }

            if status_code > 0 {
                if status_code < 400 { ok_count += 1; } else { err_count += 1; }
            } else {
                err_count += 1;
            }

            draw_dynamic_ui(display, &DashState {
                ip: ip_str,
                uptime_hms: (h, m, s),
                wifi_up: true,
                ok_count,
                err_count,
                last_status: status_code,
                last_bytes: total_bytes,
            });

            socket.disconnect();
        } else {
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

        // Cool-down
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            socket.work();
        }
        delay.delay_millis(1000);
    }
}


// ── Boot animation ────────────────────────────────────────────────────────
//
// Sequence (total ≈ 3.5 s):
//   1. Screen flash          white → black       ~80 ms
//   2. Scanline sweep        bright line top→bot ~18 ms/step × 17 steps
//   3. "INITIALIZING..." types out               ~45 ms/char
//   4. Project name glitch   random → real       ~55 ms/frame × 12 frames
//   5. Progress bar fill                         ~35 ms/step  × 20 steps
//   6. Wipe to black                             ~8 ms/row    × 16 rows
//
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
    display.clear(Rgb565::WHITE).unwrap();
    delay.delay_millis(60);
    display.clear(BG).unwrap();
    delay.delay_millis(40);

    // ── 2. Scanline sweep ─────────────────────────────────────────────
    let scan_color = Rgb565::new(4, 40, 20);
    for step in 0u16..=16 {
        if step > 0 {
            let prev_y = ((step - 1) * 8) as i32;
            Rectangle::new(Point::new(0, prev_y), Size::new(128, 8))
                .into_styled(PrimitiveStyle::with_fill(BG))
                .draw(display).unwrap();
        }
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

    // ── 3. "INITIALIZING..." types out char by char ───────────────────
    let init_msg = b"INITIALIZING...";
    let mut typed: heapless::String<20> = heapless::String::new();

    for &ch in init_msg {
        Rectangle::new(Point::new(0, 54), Size::new(128, 12))
            .into_styled(PrimitiveStyle::with_fill(BG))
            .draw(display).unwrap();

        typed.push(ch as char).ok();

        Text::with_text_style(
            typed.as_str(),
            Point::new(4, 64),
            MonoTextStyle::new(&FONT_6X10, ACCENT_LIME),
            left,
        ).draw(display).unwrap();

        // Blinking cursor block
        let cursor_x = 4 + (typed.len() as i32) * 6;
        Rectangle::new(Point::new(cursor_x, 55), Size::new(5, 9))
            .into_styled(PrimitiveStyle::with_fill(ACCENT_LIME))
            .draw(display).unwrap();

        delay.delay_millis(45);
    }
    delay.delay_millis(200);

    // ── 4. Project name glitch ────────────────────────────────────────
    // Deterministic LCG gives "random-looking" chars without needing RNG
    let target       = "WiFi Dashboard";
    let target_bytes = target.as_bytes();
    let len          = target_bytes.len();
    let mut lcg: u32 = 0xDEAD_BEEF;

    for frame in 0u8..12 {
        Rectangle::new(Point::new(0, 36), Size::new(128, 22))
            .into_styled(PrimitiveStyle::with_fill(BG))
            .draw(display).unwrap();

        let mut glitch: heapless::String<20> = heapless::String::new();
        let resolved_up_to = (frame as usize * len) / 11;

        for i in 0..len {
            if i < resolved_up_to {
                glitch.push(target_bytes[i] as char).ok();
            } else {
                // LCG next — printable ASCII range 0x20–0x5E
                lcg = lcg.wrapping_mul(1664525).wrapping_add(1013904223);
                let c = ((lcg >> 16) & 0x3E) as u8 + 0x21; // '!' .. '>'
                glitch.push(c as char).ok();
            }
        }

        let color = if frame % 2 == 0 { ACCENT_CYAN } else { ACCENT_LIME };
        Text::with_text_style(
            glitch.as_str(),
            Point::new(64, 52),
            MonoTextStyle::new(&FONT_10X20, color),
            centered,
        ).draw(display).unwrap();

        delay.delay_millis(55);
    }

    // Final resolved name — solid white
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
    Text::with_text_style(
        "BOOTING",
        Point::new(64, 82),
        MonoTextStyle::new(&FONT_6X10, TEXT_DIM),
        centered,
    ).draw(display).unwrap();

    // Outline
    Rectangle::new(Point::new(14, 88), Size::new(100, 8))
        .into_styled(PrimitiveStyle::with_stroke(ACCENT_CYAN, 1))
        .draw(display).unwrap();

    // Fill in 20 steps
    for step in 1u32..=20 {
        let fill_w = (step * 96) / 20;
        Rectangle::new(Point::new(16, 90), Size::new(fill_w, 4))
            .into_styled(PrimitiveStyle::with_fill(ACCENT_LIME))
            .draw(display).unwrap();
        delay.delay_millis(35);
    }

    delay.delay_millis(250);

    // ── 6. Wipe to black row by row ───────────────────────────────────
    for row in 0u32..16 {
        Rectangle::new(Point::new(0, (row * 8) as i32), Size::new(128, 8))
            .into_styled(PrimitiveStyle::with_fill(BG))
            .draw(display).unwrap();
        delay.delay_millis(8);
    }
}

// ── Static chrome — call once ─────────────────────────────────────────────
fn draw_static_ui<DI, MODEL>(display: &mut mipidsi::Display<DI, MODEL, Output<'_>>)
where
    DI: mipidsi::interface::Interface,
    MODEL: mipidsi::models::Model<ColorFormat = Rgb565>,
    Rgb565: mipidsi::interface::InterfacePixelFormat<DI::Word>,
{
    let left     = TextStyleBuilder::new().alignment(Alignment::Left).build();
    let centered = TextStyleBuilder::new().alignment(Alignment::Center).build();

    // Header bar (y 0–15)
    Rectangle::new(Point::new(0, 0), Size::new(128, 16))
        .into_styled(PrimitiveStyle::with_fill(HEADER_BG))
        .draw(display).unwrap();

    // Cyan rule (y=16)
    Rectangle::new(Point::new(0, 16), Size::new(128, 1))
        .into_styled(PrimitiveStyle::with_fill(ACCENT_CYAN))
        .draw(display).unwrap();

    // Title — inset from right to leave space for wifi dot
    Text::with_text_style(
        "ESP32  DASHBOARD",
        Point::new(58, 12),
        MonoTextStyle::new(&FONT_6X10, ACCENT_CYAN),
        centered,
    ).draw(display).unwrap();

    // Divider 1 (y=44)
    Rectangle::new(Point::new(0, 44), Size::new(128, 1))
        .into_styled(PrimitiveStyle::with_fill(DIVIDER))
        .draw(display).unwrap();

    // "REQUESTS" label (baseline y=56)
    Text::with_text_style(
        "REQUESTS",
        Point::new(4, 56),
        MonoTextStyle::new(&FONT_6X10, TEXT_DIM),
        left,
    ).draw(display).unwrap();

    // Divider 2 (y=82)
    Rectangle::new(Point::new(0, 82), Size::new(128, 1))
        .into_styled(PrimitiveStyle::with_fill(DIVIDER))
        .draw(display).unwrap();

    // "LAST RESPONSE" label (baseline y=94)
    Text::with_text_style(
        "LAST RESPONSE",
        Point::new(4, 94),
        MonoTextStyle::new(&FONT_6X10, TEXT_DIM),
        left,
    ).draw(display).unwrap();
}

// ── Dynamic content — call every update ───────────────────────────────────
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

    // WiFi dot — top-right corner of header
    let dot_color = if state.wifi_up { WIFI_ON } else { WIFI_OFF };
    Rectangle::new(Point::new(120, 5), Size::new(6, 6))
        .into_styled(PrimitiveStyle::with_fill(dot_color))
        .draw(display).unwrap();

    // ── IP row (y=18, baseline=28) ────────────────────────────────────
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
        Point::new(124, 28),
        MonoTextStyle::new(&FONT_6X10, TEXT_WHITE),
        right,
    ).draw(display).unwrap();

    // ── Uptime row (y=32, baseline=42) ───────────────────────────────
    Rectangle::new(Point::new(0, 32), Size::new(128, 13))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(display).unwrap();

    Text::with_text_style(
        "UP",
        Point::new(4, 42),
        MonoTextStyle::new(&FONT_6X10, ACCENT_LIME),
        left,
    ).draw(display).unwrap();

    let (h, m, s) = state.uptime_hms;
    let mut uptime_str: heapless::String<12> = heapless::String::new();
    write!(uptime_str, "{:02}:{:02}:{:02}", h, m, s).ok();

    Text::with_text_style(
        uptime_str.as_str(),
        Point::new(124, 42),
        MonoTextStyle::new(&FONT_6X10, TEXT_WHITE),
        right,
    ).draw(display).unwrap();

    // ── OK count (y=58, baseline=68) ─────────────────────────────────
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

    // ── ERR count (y=70, baseline=80) ────────────────────────────────
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

    // ── HTTP status code — FONT_10X20, color-coded (y=96–119) ────────
    Rectangle::new(Point::new(0, 96), Size::new(128, 24))
        .into_styled(PrimitiveStyle::with_fill(BG))
        .draw(display).unwrap();

    let status_color = match state.last_status {
        200..=299 => ACCENT_LIME,
        300..=399 => ACCENT_AMBER,
        400..=599 => ACCENT_RED,
        _         => TEXT_DIM,
    };

    let mut status_str: heapless::String<8> = heapless::String::new();
    if state.last_status == 0 {
        write!(status_str, "---").ok();
    } else {
        write!(status_str, "{}", state.last_status).ok();
    }

    Text::with_text_style(
        status_str.as_str(),
        Point::new(64, 118),
        MonoTextStyle::new(&FONT_10X20, status_color),
        centered,
    ).draw(display).unwrap();

    // ── Bytes received (y=120, baseline=126) ─────────────────────────
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

// ── Networking helpers ────────────────────────────────────────────────────
pub fn create_interface(device: &mut esp_radio::wifi::WifiDevice) -> smoltcp::iface::Interface {
    smoltcp::iface::Interface::new(
        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
        )),
        device,
        timestamp(),
    )
}

fn timestamp() -> smoltcp::time::Instant {
    smoltcp::time::Instant::from_micros(
        esp_hal::time::Instant::now()
            .duration_since_epoch()
            .as_micros() as i64,
    )
}

fn init_hardware() -> Peripherals {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    esp_alloc::heap_allocator!(size: 72 * 1024);
    peripherals
}

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

fn scan_wifi(controller: &mut WifiController<'_>) {
    println!("Start Wifi Scan");
    let scan_config = ScanConfig::default().with_max(10);
    let res = controller.scan_with_config(scan_config).unwrap();
    for ap in res {
        println!("{:?}", ap);
    }
}

fn connect_wifi(controller: &mut WifiController<'_>) {
    println!("{:?}", controller.capabilities());
    println!("wifi_connect {:?}", controller.connect());

    println!("Wait to get connected");
    loop {
        match controller.is_connected() {
            Ok(true) => break,
            Ok(false) => {}
            Err(err) => panic!("{:?}", err),
        }
    }
    println!("Connected: {:?}", controller.is_connected());
}

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