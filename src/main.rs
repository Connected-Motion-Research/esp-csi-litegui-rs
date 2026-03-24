//! # esp-csi-litegui-rs
//!
//! `esp-csi-litegui-rs` is a Rust `no_std` firmware application that visualizes
//! Wi-Fi Channel State Information (CSI) on supported ESP32-S3 AMOLED boards.
//! It uses [`esp-csi-rs`](https://github.com/Connected-Motion-Research/esp-csi-rs)
//! for CSI collection and [`embedded-graphics`](https://github.com/embedded-graphics-rs/embedded-graphics)
//! for rendering.
//!
//! This binary initializes board peripherals, configures CSI collection,
//! starts the rendering task on the second core, and runs the touch/gesture
//! task on the primary core.
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
//!
//! Notes:
//!
//! - `mode-now` is enabled by default.
//! - You can safely override default mode by enabling `mode-sta` or `mode-snf`
//!   without disabling default features.
//! - `mode-sta` and `mode-snf` cannot be enabled together.
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
use esp_csi_rs::logging::logging::{LogMode, init_logger};
use esp_csi_rs::{CSINode, CSINodeClient, CSINodeHardware, CollectionMode, EspNowConfig, WifiSnifferConfig, WifiStationConfig, config::CsiConfig, log_ln};
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
use esp_radio::wifi::ClientConfig;

use crate::alloc::string::ToString;
use config::{BOARD_NAME, CSI_OPERATION_MODE, CsiOperationMode, DisplayMode, DISPLAY_SIZE, FB_SIZE};
use csi_task::configure_node_radio;
#[cfg(feature = "lilygo-t4")]
use gesture_task::display_gesture_runner;
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
use gesture_task::waveshare_touch_runner;
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
use reset::NoopResetDriver;
#[cfg(feature = "lilygo-t4")]
use reset::ResetDriver;
use state::{APP_CORE_STACK, DISPLAY_MODE, WIFI_CONTROLLER};

mod config;
mod csi_processing;
mod csi_task;
mod display_task;
mod gesture_task;
mod reset;
mod state;

esp_app_desc!();

extern crate alloc;

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

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
            loop {}
        }
    };

    //  WiFi & CSI Collection Configuration & Initialization //

    // Instantiate peripherals necessary to set up  WiFi
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    // Initialize WiFi Controller
    let radio_init = mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );

    let mut config_radio = esp_radio::wifi::Config::default();
    config_radio = config_radio.with_power_save_mode(esp_radio::wifi::PowerSaveMode::None);
    let (wifi_controller, mut interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, config_radio)
            .expect("Failed to initialize Wi-Fi controller");

    let controller = WIFI_CONTROLLER.init(wifi_controller);

    log_ln!("WiFi Controller Initialized");

    // Create a CSI Client Instance to handle CSI data and control messages
    let node_handle = CSINodeClient::new();
    // Create a CSINodeHardware instance which will be used by the CSINode to interact with the Wi-Fi hardware
    let csi_hardware = CSINodeHardware::new(&mut interfaces, controller);
    let mut node = match CSI_OPERATION_MODE {
        CsiOperationMode::WifiStation => {
            let client_config = ClientConfig::default()
                .with_ssid("Connected Motion ".to_string())
                .with_password("automotion@123".to_string())
                .with_auth_method(esp_radio::wifi::AuthMethod::Wpa2Personal)
                .with_channel(1);

                let station_config = WifiStationConfig { client_config };
            CSINode::new(
                esp_csi_rs::Node::Central(esp_csi_rs::CentralOpMode::WifiStation(station_config)),
                CollectionMode::Collector,
                Some(CsiConfig::default()),
                Some(1000),
                csi_hardware
            )
        }
        CsiOperationMode::EspNow => CSINode::new(
            esp_csi_rs::Node::Central(esp_csi_rs::CentralOpMode::EspNow(EspNowConfig::default())),
            CollectionMode::Collector,
            Some(CsiConfig::default()),
            Some(1000),
            csi_hardware,
        ),
        CsiOperationMode::WifiSniffer => CSINode::new(
            esp_csi_rs::Node::Peripheral(esp_csi_rs::PeripheralOpMode::WifiSniffer(
                WifiSnifferConfig::default(),
            )),
            CollectionMode::Collector,
            Some(CsiConfig::default()),
            Some(1000),
            csi_hardware,
        ),
    };
    configure_node_radio(&mut node, CSI_OPERATION_MODE);
    log_ln!("CSI mode configured: {:?}", CSI_OPERATION_MODE);

    spawner.spawn(csi_task::csi_task(node_handle)).ok();

    // Spawn Display Task on Second Core
    let sw_int = esp_hal::interrupt::software::SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    #[cfg(feature = "lilygo-t4")]
    {
        // Initalize touch
        touch
            .initialize()
            .expect("Failed to initialize touch driver");

        // Spawn display + gesture tasks on a separate core so touch I2C activity
        // cannot stall CSI/node runtime on the main core.
        esp_rtos::start_second_core::<32768>(
            peripherals.CPU_CTRL,
            sw_int.software_interrupt0,
            sw_int.software_interrupt1,
            unsafe { &mut *addr_of_mut!(APP_CORE_STACK) },
            move || {
                static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
                let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
                executor.run(|spawner| {
                    spawner.spawn(display_task::display_task(display)).ok();
                    spawner
                        .spawn(display_gesture_runner(
                            touch_int,
                            touch,
                            DISPLAY_MODE.sender(),
                        ))
                        .ok();
                });
            },
        );
    }

    #[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
    {
        // Spawn display + gesture tasks on a separate core so touch I2C activity
        // cannot stall CSI/node runtime on the main core.
        esp_rtos::start_second_core::<32768>(
            peripherals.CPU_CTRL,
            sw_int.software_interrupt0,
            sw_int.software_interrupt1,
            unsafe { &mut *addr_of_mut!(APP_CORE_STACK) },
            move || {
                static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
                let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
                executor.run(|spawner| {
                    spawner.spawn(display_task::display_task(display)).ok();
                    spawner
                        .spawn(waveshare_touch_runner(
                            touch_int,
                            touch_i2c,
                            DISPLAY_MODE.sender(),
                        ))
                        .ok();
                });
            },
        );
    }

    // Set the current display mode (default to Magnitude), acquire Watch sender, and update Watch variable.
    let current_display_mode = DisplayMode::Magnitude;
    let display_mode_watch = DISPLAY_MODE.sender();
    display_mode_watch.send(current_display_mode);

    // Keep CSI/node runtime isolated on the main core.
    node.run().await;
}
