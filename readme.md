>!WARNING
> WIP !!!!!


# ESP32 WiFi Dashboard

A bare-metal Rust application for the ESP32 that connects to WiFi, makes periodic HTTP requests, and displays a live dashboard on a 128×128 ST7735s TFT display.

Built with [`esp-hal`](https://github.com/esp-rs/esp-hal), [`embedded-graphics`](https://github.com/embedded-graphics/embedded-graphics), and [`mipidsi`](https://github.com/almindor/mipidsi). No `std`, no RTOS beyond the minimal `esp-rtos` timer bootstrap.

---

## What it does

- Connects to a WiFi network via WPA2
- Makes HTTP GET requests in a loop to a configurable endpoint
- Renders a live dashboard on the display showing:
  - **IP address** acquired via DHCP
  - **Uptime** (HH:MM:SS)
  - **Request counters** — OK and ERR totals
  - **Last HTTP status code** — color-coded (green 2xx, amber 3xx, red 4xx/5xx)
  - **Bytes received** in the last response
  - **WiFi status dot** — green when connected, red when not

---

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32 (tested on ESP32-WROOM-32) |
| Display | 128×128 ST7735s TFT via SPI |

### Wiring

| Signal | ESP32 GPIO |
|--------|-----------|
| DC     | GPIO2     |
| RST    | GPIO4     |
| CS     | GPIO5     |
| SCK    | GPIO18    |
| MOSI   | GPIO23    |

---

## Building

### Prerequisites

- Rust with the ESP toolchain: [esp-rs/rust](https://github.com/esp-rs/rust)
- [`espflash`](https://github.com/esp-rs/espflash) for flashing

Install the ESP Rust toolchain:
```sh
cargo install espup
espup install
```

### Run

Pass your WiFi credentials as environment variables:

```sh
SSID=YourNetwork PASSWORD=YourPassword cargo run --release
```

To just build without flashing:
```sh
SSID=YourNetwork PASSWORD=YourPassword cargo build --release
```

---

## Project structure

```
src/
└── bin/
    └── main.rs      # Everything: WiFi, HTTP client, display driver, UI
```

The UI is split into two functions:

- `draw_static_ui` — draws chrome (header, dividers, labels) once at startup
- `draw_dynamic_ui` — repaints only the value areas on every HTTP cycle, takes a `DashState` struct

---

## Dependencies

| Crate | Role |
|-------|------|
| `esp-hal` | Bare-metal ESP32 peripherals (SPI, GPIO, timers, RNG) |
| `esp-radio` | WiFi controller |
| `esp-rtos` | Minimal timer bootstrap required by `esp-radio` |
| `esp-alloc` | Heap allocator |
| `esp-bootloader-esp-idf` | App descriptor for the IDF bootloader |
| `esp-println` | `println!` over UART |
| `mipidsi` | Display driver for ST7735s |
| `embedded-graphics` | 2D drawing primitives and text |
| `embedded-hal-bus` | `ExclusiveDevice` SPI bus wrapper |
| `embedded-io` | `Read`/`Write` traits for the socket |
| `blocking-network-stack` | Blocking TCP socket over smoltcp |
| `smoltcp` | TCP/IP stack with DHCP |
| `heapless` | Stack-allocated `String` for formatting |

---

## Display layout

```
┌────────────────────────┐  y=0
│  ESP32  DASHBOARD   ●  │  header (navy), cyan text, wifi dot
├════════════════════════┤  y=16  cyan rule
│ IP          10.0.0.42  │  y=18  cyan label / white value
│ UP          00:04:32   │  y=32  lime label / white value
├────────────────────────┤  y=44  dim divider
│ REQUESTS               │  y=46  dim label
│ OK                  42 │  y=58  lime / white
│ ERR                  3 │  y=70  red  / white
├────────────────────────┤  y=82  dim divider
│ LAST RESPONSE          │  y=84  dim label
│                        │
│          200           │  y=96  large font, color-coded
│       1842 bytes       │  y=120 dim, centered
└────────────────────────┘  y=128
```

---

## License

MIT