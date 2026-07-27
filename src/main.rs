//! # esp-csi-litegui-rs
//!
//! `esp-csi-litegui-rs` is a Rust `no_std` firmware application that visualizes
//! Wi-Fi Channel State Information (CSI) on supported ESP32-S3 AMOLED boards.
//! It uses [`esp-csi-rs`](https://github.com/Connected-Motion-Research/esp-csi-rs)
//! for CSI collection and [`embedded-graphics`](https://github.com/embedded-graphics-rs/embedded-graphics)
//! for rendering.
//!
//! This binary initializes board peripherals, configures CSI collection,
//! and splits rendering/touch tasks per board profile:
//! - LilyGo: display + touch gesture tasks run on the second core.
//! - Waveshare: display runs on the second core, FT3x68 gesture task runs on
//!   the main executor to avoid render starvation from touch I2C reads.
//!
//! ## Supported boards
//!
//! - Waveshare ESP32-S3-Touch-AMOLED-1.8
//! - LilyGo T4-S3
//!
//! ## Requirements
//!
//! ### Hardware
//!
//! - One of the supported boards listed above.
//!
//! ### Software
//!
//! - Rust + ESP toolchain (`esp-rs`) installed.
//! - A flashing tool (recommended: `espflash`).
//! - Optional serial monitor (can also be done using `espflash monitor`).
//!
//! ## Feature flags
//!
//! ### Board feature (required)
//!
//! Enable exactly one:
//!
//! - `lilygo-t4`
//! - `waveshare-esp32-s3-touch-amoled-1_8`
//!
//! ### CSI mode features
//!
//! - `mode-sta`: Wi-Fi station mode CSI collection.
//! - `mode-now`: ESP-NOW mode CSI collection.
//! - `mode-snf`: Wi-Fi sniffer mode CSI collection.
//! - `mode-ap`: SoftAP collector — this board runs an AP (SSID `esp-csi-ap`)
//!   with a built-in DHCP server and pings the associated station; CSI is
//!   captured from its uplink replies. Pair with any Wi-Fi station, e.g. the
//!   esp-csi-rs `wifi_station` example.
//! - `mode-fast`: Fast one-to-one ESP-NOW collector — RX-only capture of a
//!   forced-MCS7 unicast flood. Pair with a peer flashed with the esp-csi-rs
//!   `esp_now_fast_source` example on the same channel.
//!
//! Notes:
//!
//! - `mode-now` is enabled by default.
//! - You can safely override the default mode by enabling one of `mode-sta`,
//!   `mode-snf`, `mode-ap`, or `mode-fast` without disabling default features.
//! - Only one override mode can be enabled at a time.
//!
//! ## Quick start
//!
//! 1. Clone this repository.
//! 2. Update station credentials in `src/main.rs` if using station mode
//!    (`ClientConfig::with_ssid(...)` and `with_password(...)`).
//! 3. Build/flash with board + mode features.
//!
//! ### Common commands
//!
//! LilyGo T4-S3 + default ESP-NOW mode:
//!
//! ```bash
//! cargo run --release --features="lilygo-t4"
//! ```
//!
//! LilyGo T4-S3 + ESP-NOW mode:
//!
//! ```bash
//! cargo run --release --features="lilygo-t4,mode-now"
//! ```
//!
//! Waveshare + station mode:
//!
//! ```bash
//! cargo run --release --features="waveshare-esp32-s3-touch-amoled-1_8,mode-sta"
//! ```
//!
//! Waveshare + sniffer mode:
//!
//! ```bash
//! cargo run --release --features="waveshare-esp32-s3-touch-amoled-1_8,mode-snf"
//! ```
//!
//! LilyGo + softAP collector mode:
//!
//! ```bash
//! cargo run --release --features="lilygo-t4,mode-ap"
//! ```
//!
//! Waveshare + fast ESP-NOW collector mode:
//!
//! ```bash
//! cargo run --release --features="waveshare-esp32-s3-touch-amoled-1_8,mode-fast"
//! ```
//!
//! ## Beginner tips
//!
//! - Start with station mode first; it is usually the easiest mode to validate.
//! - Check serial logs for the selected board and CSI mode.
//! - If the display stays blank in ESP-NOW/sniffer mode, verify peer/channel setup.
//! - Use touch gestures to switch between magnitude and phase views.

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;
use embassy_executor::Spawner;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_csi_rs::logging::logging::{LogMode, auto_log_backend_label, init_logger};
use esp_csi_rs::{CSINode, CSINodeClient, CSINodeHardware, CollectionMode, EspNowConfig, WifiApConfig, WifiSnifferConfig, WifiStationConfig, config::CsiConfig, install_static_espnow_recv, log_ln, set_csi_logging_enabled};
use esp_hal::dma_buffers;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    delay::Delay, gpio::{self, Input},
    i2c::master::{Config as I2cConfig, I2c},
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
};
#[cfg(feature = "lilygo-t4")]
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_backtrace as _;
use rm690b0_rs::{
    ColorMode, DMA_CHUNK_SIZE, Lgt4s3Driver, Rm690b0Driver,
};
use static_cell::StaticCell;
#[cfg(feature = "lilygo-t4")]
use cst226_rs::{CST226_DEVICE_ADDRESS, Cst226Driver};
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
use ft3x68_rs::{FT3168_DEVICE_ADDRESS, Ft3x68Driver};
use esp_radio::wifi::PowerSaveMode;
use esp_radio::wifi::ap::AccessPointConfig;
use esp_radio::wifi::sta::StationConfig;

use crate::alloc::string::ToString;
use config::{AP_PING_RATE_HZ, AP_SSID, BOARD_NAME, CSI_CHANNEL, CSI_OPERATION_MODE, CsiOperationMode, DISPLAY_SIZE, FB_SIZE};
use csi_task::configure_node_radio;
#[cfg(feature = "lilygo-t4")]
use gesture_task::display_gesture_runner;
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
use gesture_task::waveshare_touch_runner;
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
use reset::NoopResetDriver;
#[cfg(feature = "lilygo-t4")]
use reset::ResetDriver;
use state::{APP_CORE_STACK, MODE_INTENTS, WIFI_CONTROLLER};

mod config;
mod csi_processing;
mod csi_task;
mod display_task;
mod gesture_task;
mod reset;
mod state;

esp_app_desc!();

extern crate alloc;

/// Firmware entry point.
///
/// This function performs full system bring-up:
/// - clock and allocator initialization,
/// - board-specific display/touch wiring,
/// - CSI node creation and radio tuning,
/// - spawning CSI processing and display tasks,
/// - running touch gesture handling alongside the CSI node runtime.
#[esp_rtos::main]
async fn main(spawner: Spawner) {
    // Configure System Clock
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    // Take Peripherals
    let peripherals = esp_hal::init(config);

    // Required for `log_ln!` when `async-print` is enabled.
    init_logger(spawner, LogMode::Text);
    // `init_logger` opens the inline per-packet CSI UART log gate. Close it
    // immediately: the WiFi callback only falls through to that path while
    // delivery mode is `Off`, so any CSI packet that arrives before
    // `csi_task` first awaits `get_csi_data()` (which flips mode to `Async`)
    // would otherwise be dumped to UART.
    set_csi_logging_enabled(false);
    log_ln!("Log backend: {}", auto_log_backend_label());
    log_ln!("Board profile: {}", BOARD_NAME);

    esp_alloc::heap_allocator!(size: 61 * 1024);

    // Display Configuration //

    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let delay = Delay::new();

    // DMA Buffers for SPI
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_CHUNK_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // SPI Configuration (board-specific)
    // Using DMA for more efficient SPI communication.
    #[cfg(feature = "lilygo-t4")]
    let lcd_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(80_u32))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sio0(peripherals.GPIO14)
    .with_sio1(peripherals.GPIO10)
    .with_sio2(peripherals.GPIO16)
    .with_sio3(peripherals.GPIO12)
    .with_cs(peripherals.GPIO11)
    .with_sck(peripherals.GPIO15)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf);

    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    let lcd_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
        // Waveshare wiring is less tolerant at peak clock; 40 MHz is much more stable
        // for sustained partial flush traffic.
        .with_frequency(Rate::from_mhz(40_u32))
            .with_mode(Mode::_0),
    )
    .unwrap()
    .with_sio0(peripherals.GPIO4)
    .with_sio1(peripherals.GPIO5)
    .with_sio2(peripherals.GPIO6)
    .with_sio3(peripherals.GPIO7)
    .with_cs(peripherals.GPIO12)
    .with_sck(peripherals.GPIO11)
    .with_dma(peripherals.DMA_CH0)
    .with_buffers(dma_rx_buf, dma_tx_buf);

    // Touch Configuration //

    // I2C Configuration (board-specific)
    #[cfg(feature = "lilygo-t4")]
    let touch_i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO6)
    .with_scl(peripherals.GPIO7);

    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    let mut touch_i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO15)
    .with_scl(peripherals.GPIO14);

    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    {
        // Waveshare board uses XCA9554 I/O expander for reset/power lines.
        // Sequence from vendor demos: set pins 0..2 as outputs, pulse low then high.
        const XCA9554_ADDR: u8 = 0x20;
        touch_i2c.write(XCA9554_ADDR, &[0x03, 0xF8]).ok();
        touch_i2c.write(XCA9554_ADDR, &[0x01, 0x00]).ok();
        delay.delay_millis(20);
        touch_i2c.write(XCA9554_ADDR, &[0x01, 0x07]).ok();
        delay.delay_millis(20);
    }

    #[cfg(feature = "lilygo-t4")]
    let touch_reset_pin = Output::new(peripherals.GPIO17, Level::High, OutputConfig::default());

    // Initialize the reset driver.
    #[cfg(feature = "lilygo-t4")]
    let touch_reset_driver = ResetDriver::new(touch_reset_pin);

    // Initialize & power display driver
    #[cfg(feature = "lilygo-t4")]
    let display_reset_pin = Output::new(peripherals.GPIO13, Level::High, OutputConfig::default());
    #[cfg(feature = "lilygo-t4")]
    let _display_pwr_en = Output::new(peripherals.GPIO9, Level::High, OutputConfig::default());

    #[cfg(feature = "lilygo-t4")]
    let display_reset_driver = ResetDriver::new(display_reset_pin);
    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    let display_reset_driver = NoopResetDriver;

    let lg_driver = Lgt4s3Driver::new(lcd_spi);

    // Configure Pin to detect touch interrupts
    #[cfg(feature = "lilygo-t4")]
    let touch_int = Input::new(peripherals.GPIO8, gpio::InputConfig::default());

    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    let touch_int = Input::new(peripherals.GPIO21, gpio::InputConfig::default());

    // Instantiate the touch CST226 driver.
    #[cfg(feature = "lilygo-t4")]
    let mut touch = Cst226Driver::new(touch_i2c, CST226_DEVICE_ADDRESS, touch_reset_driver, delay);

    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    let mut touch = Ft3x68Driver::new(touch_i2c, FT3168_DEVICE_ADDRESS, NoopResetDriver, Delay::new());

    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    {
        if let Err(e) = touch.initialize() {
            log_ln!("Failed to initialize FT3x68 touch driver: {:?}", e);
        }
        if let Err(e) = touch.set_gesture_mode(true) {
            log_ln!("Failed to enable FT3x68 gesture mode: {:?}", e);
        }
    }

    // Instantiate and Initialize Display Driver
    log_ln!("Initializing Display...");
    let display_res = Rm690b0Driver::new_heap::<_, FB_SIZE>(
        lg_driver,
        display_reset_driver,
        ColorMode::Rgb888,
        DISPLAY_SIZE,
        delay,
    );
    let display = match display_res {
        Ok(d) => {
            log_ln!("Display initialized successfully.");
            d
        }
        Err(e) => {
            log_ln!("Error initializing display: {:?}", e);
            panic!("Display initialization failed: {:?}", e);
        }
    };

    //  WiFi & CSI Collection Configuration & Initialization //

    // Instantiate peripherals necessary to set up  WiFi
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Initialize WiFi Controller
    let config_radio = esp_radio::wifi::ControllerConfig::default();
    let (wifi_controller, mut interfaces) =
        esp_radio::wifi::new(peripherals.WIFI, config_radio)
            .expect("Failed to initialize Wi-Fi controller");

    // Replace esp-radio's heap-backed ESP-NOW receive queue with esp-csi-rs's
    // static pool as early as possible. `node.run()` performs the same takeover,
    // but display/second-core bring-up sits between here and `run()`, and a
    // nearby ESP-NOW transmitter can grow the heap queue during that window.
    // Harmless in non-ESP-NOW modes (the sniffer suspends recv at run start).
    install_static_espnow_recv();

    let controller = WIFI_CONTROLLER.init(wifi_controller);
    let _ = controller.set_power_saving(PowerSaveMode::None);

    log_ln!("WiFi Controller Initialized");

    // Create a CSINodeHardware instance which will be used by the CSINode to interact with the Wi-Fi hardware
    let csi_hardware = CSINodeHardware::new(&mut interfaces, controller);
    let mut node = match CSI_OPERATION_MODE {
        CsiOperationMode::WifiStation => {
            let client_config = StationConfig::default()
                .with_ssid("SSID")
                .with_password("PASS".to_string())
                .with_auth_method(esp_radio::wifi::AuthenticationMethod::Wpa2Personal);

                let station_config = WifiStationConfig::new(client_config);
            CSINode::new(
                esp_csi_rs::Node::Central(esp_csi_rs::CentralOpMode::WifiStation(station_config)),
                CollectionMode::Collector,
                Some(CsiConfig::default()),
                Some(1000),
                csi_hardware
            )
        }
        CsiOperationMode::EspNow => CSINode::new(
            esp_csi_rs::Node::Central(esp_csi_rs::CentralOpMode::EspNow(
                EspNowConfig::default().with_channel(CSI_CHANNEL),
            )),
            CollectionMode::Collector,
            Some(CsiConfig::default()),
            Some(1000),
            csi_hardware,
        ),
        CsiOperationMode::WifiSniffer => {
            let sniffer_config = WifiSnifferConfig::default().with_channel(CSI_CHANNEL);
            CSINode::new(
                esp_csi_rs::Node::Peripheral(esp_csi_rs::PeripheralOpMode::WifiSniffer(
                    sniffer_config,
                )),
                CollectionMode::Collector,
                Some(CsiConfig::default()),
                Some(1000),
                csi_hardware,
            )
        }
        CsiOperationMode::WifiAccessPoint => {
            let ap_radio_config = AccessPointConfig::default()
                .with_ssid(AP_SSID)
                .with_channel(CSI_CHANNEL);
            // Defaults: AP 192.168.13.1, single lease .2, DHCP server on.
            let ap_config = WifiApConfig::new(ap_radio_config, CSI_CHANNEL, None);
            CSINode::new(
                esp_csi_rs::Node::Central(esp_csi_rs::CentralOpMode::WifiAccessPoint(ap_config)),
                CollectionMode::Collector,
                Some(CsiConfig::default()),
                Some(AP_PING_RATE_HZ),
                csi_hardware,
            )
        }
        CsiOperationMode::EspNowFastCollector => CSINode::new(
            esp_csi_rs::Node::Central(esp_csi_rs::CentralOpMode::EspNowFastCollector(
                EspNowConfig::fast_default().with_channel(CSI_CHANNEL),
            )),
            CollectionMode::Collector,
            Some(CsiConfig::default()),
            // Collector is RX-only after discovery; the source generates the traffic.
            None,
            csi_hardware,
        ),
    };
    configure_node_radio(&mut node, CSI_OPERATION_MODE);
    log_ln!(
        "CSI mode configured: {:?} on channel {}",
        CSI_OPERATION_MODE,
        CSI_CHANNEL
    );

    // The new esp-csi-rs API delivers CSI via `CSINodeClient::get_csi_data().await`.
    // The first await lazily switches the lib's delivery mode to `Async` and
    // opens the publish gate, so no `set_csi_callback` / `set_csi_logging_enabled`
    // dance is needed here — `csi_task` just pulls from the lib's lock-free queue.
    let csi_client = CSINodeClient::new();

    spawner.spawn(csi_task::csi_task(csi_client).unwrap());
    spawner.spawn(csi_task::stats_task().unwrap());

    // Spawn display task(s) on second core.
    #[cfg(feature = "lilygo-t4")]
    {
        // Initalize touch
        touch
            .initialize()
            .expect("Failed to initialize touch driver");

        // Keep display rendering on a dedicated core for responsiveness.
        esp_rtos::start_second_core::<32768>(
            peripherals.CPU_CTRL,
            sw_int.software_interrupt1,
            unsafe { &mut *addr_of_mut!(APP_CORE_STACK) },
            move || {
                static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
                let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
                executor.run(|spawner| {
                    spawner.spawn(display_task::display_task(display).unwrap());
                    spawner.spawn(display_gesture_runner(
                        touch_int,
                        touch,
                        MODE_INTENTS.sender(),
                    ).unwrap());
                });
            },
        );
    }

    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    {
        // Run touch on the main executor so blocking I2C reads cannot stall display rendering.
        spawner.spawn(waveshare_touch_runner(
            touch_int,
            touch,
            MODE_INTENTS.sender(),
        ).unwrap());

        // Keep display rendering on a separate core.
        esp_rtos::start_second_core::<32768>(
            peripherals.CPU_CTRL,
            sw_int.software_interrupt1,
            unsafe { &mut *addr_of_mut!(APP_CORE_STACK) },
            move || {
                static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
                let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
                executor.run(|spawner| {
                    spawner.spawn(display_task::display_task(display).unwrap());
                });
            },
        );
    }

    // Keep CSI/node runtime isolated on the main core.
    node.run().await;
}
