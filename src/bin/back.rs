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
    primitives::{Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle},
    text::{Alignment, Text, TextStyleBuilder},
};

use blocking_network_stack::Stack;
use embedded_io::{Read, Write};
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::Peripherals;
use esp_hal::rng::Rng;
use esp_hal::time::{Duration, Instant};
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    main,
    spi::{
        Mode,
        master::{Config, Spi},
    },
    time::Rate,
};
use esp_println::{self as _, println};
use esp_radio::wifi::{ClientConfig, ModeConfig, ScanConfig, WifiController};
use heapless::String;
use mipidsi::{
    Builder,
    interface::SpiInterface,
    models::ST7735s,
    options::ColorOrder, // ← changed from ColorInversion
};
use smoltcp::iface::{SocketSet, SocketStorage};
use smoltcp::wire::{DhcpOption, IpAddress};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");
const BG: Rgb565 = Rgb565::BLACK;
const HEADER_BG: Rgb565 = Rgb565::new(0, 12, 20);
const ACCENT: Rgb565 = Rgb565::new(0, 31, 20);
const TEXT_MAIN: Rgb565 = Rgb565::WHITE;
const TEXT_DIM: Rgb565 = Rgb565::new(15, 20, 15);
const DIVIDER: Rgb565 = Rgb565::new(5, 10, 8);

#[main]
fn main() -> ! {
    // generator version: 1.0.0

    let peripherals = init_hardware();
    let mut delay = Delay::new();

    esp_alloc::heap_allocator!(#[unsafe(link_section = ".dram2_uninit")] size: 98767);

    let dc = Output::new(peripherals.GPIO2, Level::Low, OutputConfig::default());
    let rst = Output::new(peripherals.GPIO4, Level::High, OutputConfig::default());
    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());

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
        .display_size(128, 160)
        .color_order(ColorOrder::Bgr) // ← Fix 1
        .init(&mut delay)
        .unwrap();

    display.clear(BG).unwrap();
    draw_static_ui(&mut display);

    let mut elapsed_ms: u64 = 0;
    let tick_ms: u64 = 100;

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = Rng::new();

    esp_rtos::start(timg0.timer0);
    let radio_init = esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller");

    let (mut wifi_controller, interfaces) =
        esp_radio::wifi::new(&radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");

    let mut device = interfaces.sta;

    // let mut stack = setup_network_stack(device, &mut rng);
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let mut socket_set = SocketSet::new(&mut socket_set_entries[..]);
    let mut dhcp_socket = smoltcp::socket::dhcpv4::Socket::new();

    // we can set a hostname here (or add other DHCP options)
    dhcp_socket.set_outgoing_options(&[DhcpOption {
        kind: 12,
        data: b"implRust",
    }]);
    socket_set.add(dhcp_socket);
    // sta_socket_set.add(smoltcp::socket::dhcpv4::Socket::new());

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

    let mut rx_buffer = [0u8; 1536];
    let mut tx_buffer = [0u8; 1536];
    let socket = stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    http_loop(socket)
}

pub fn create_interface(device: &mut esp_radio::wifi::WifiDevice) -> smoltcp::iface::Interface {
    // users could create multiple instances but since they only have one WifiDevice
    // they probably can't do anything bad with that
    smoltcp::iface::Interface::new(
        smoltcp::iface::Config::new(smoltcp::wire::HardwareAddress::Ethernet(
            smoltcp::wire::EthernetAddress::from_bytes(&device.mac_address()),
        )),
        device,
        timestamp(),
    )
}

// some smoltcp boilerplate
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

fn http_loop(
    mut socket: blocking_network_stack::Socket<'_, '_, esp_radio::wifi::WifiDevice<'_>>,
) -> ! {
    println!("Starting HTTP client loop");
    let delay = Delay::new();
    loop {
        println!("Making HTTP request");
        socket.work();

        let remote_addr = IpAddress::v4(142, 250, 185, 115);
        socket.open(remote_addr, 80).unwrap();
        socket
            .write(b"GET / HTTP/1.0\r\nHost: www.mobile-j.de\r\n\r\n")
            .unwrap();
        socket.flush().unwrap();

        let deadline = Instant::now() + Duration::from_secs(20);
        let mut buffer = [0u8; 512];
        while let Ok(len) = socket.read(&mut buffer) {
            // let text = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };
            let Ok(text) = core::str::from_utf8(&buffer[..len]) else {
                panic!("Invalid UTF-8 sequence encountered");
            };

            println!("{}", text);

            if Instant::now() > deadline {
                println!("Timeout");
                break;
            }
        }

        socket.disconnect();
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            socket.work();
        }

        delay.delay_millis(1000);
    }
}

fn draw_static_ui<DI, MODEL>(display: &mut mipidsi::Display<DI, MODEL, Output<'_>>)
where
    DI: mipidsi::interface::Interface,
    MODEL: mipidsi::models::Model<ColorFormat = Rgb565>,
    Rgb565: mipidsi::interface::InterfacePixelFormat<DI::Word>,
{
    let centered = TextStyleBuilder::new().alignment(Alignment::Center).build();

    // Header
    Rectangle::new(Point::new(0, 0), Size::new(128, 36))
        .into_styled(PrimitiveStyle::with_fill(HEADER_BG))
        .draw(display)
        .unwrap();

    Rectangle::new(Point::new(0, 34), Size::new(128, 2))
        .into_styled(PrimitiveStyle::with_fill(ACCENT))
        .draw(display)
        .unwrap();

    Text::with_text_style(
        SSID,
        Point::new(64, 16),
        MonoTextStyle::new(&FONT_6X10, ACCENT),
        centered,
    )
    .draw(display)
    .unwrap();

    Text::with_text_style(
        "ESP32 + Rust",
        Point::new(64, 28),
        MonoTextStyle::new(&FONT_6X10, TEXT_DIM),
        centered,
    )
    .draw(display)
    .unwrap();

    // Label
    Text::with_text_style(
        "HH : MM : SS",
        Point::new(64, 58),
        MonoTextStyle::new(&FONT_6X10, TEXT_DIM),
        centered,
    )
    .draw(display)
    .unwrap();

    // Divider
    Line::new(Point::new(14, 118), Point::new(114, 118))
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_color(DIVIDER)
                .stroke_width(1)
                .build(),
        )
        .draw(display)
        .unwrap();
}
