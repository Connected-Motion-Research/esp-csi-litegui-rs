#![no_std]
#![no_main]

use core::ptr::addr_of_mut;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    channel::Channel,
    pubsub::{Subscriber, WaitResult},
    watch::Watch,
};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::i2c;
use embedded_hal_bus::i2c::AtomicDevice;
use embedded_hal_bus::util::AtomicCell;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_csi_rs::{
    config::{CSIConfig, TrafficConfig, TrafficType, WiFiConfig},
    CSICollector, NetworkArchitechture,
};
use esp_hal::dma::{DmaRxBuf, DmaTxBuf};
use esp_hal::dma_buffers;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::{
    delay::Delay,
    i2c::master::{Config as I2cConfig, Error, I2c},
    spi::{
        master::{Config as SpiConfig, Spi},
        Mode,
    },
    system::{CpuControl, Stack},
    time::Rate,
    Blocking,
};
use esp_hal_embassy::Executor;
use esp_println as _;
use esp_println::println;
use esp_wifi::{init, EspWifiController};
use heapless::Vec;
use micromath::F32Ext;
use sh8601_rs::{
    framebuffer_size, ColorMode, DisplaySize, ResetInterface as Sh8601ResetInterface, Sh8601Driver,
    Ws18AmoledDriver, DMA_CHUNK_SIZE,
};
use static_cell::StaticCell;

use embedded_graphics::{
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
};

use ft3x68_rs::{Ft3x68Driver, Gesture, ResetInterface, FT3168_DEVICE_ADDRESS};

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

// Enum to select the display mode
#[derive(Clone, Copy, Debug, PartialEq)]
enum DisplayMode {
    Magnitude,
    Phase,
}

// Keep in mind that Watch immediately overwrites the previous value when a new one is sent, without waiting for all receivers to read the previous value.
// If this poses a problem, changing to a PubSubChannel would be safer.
// The Watch sender is supposed to update only when a gesture is detected.
static DISPLAY_MODE: Watch<CriticalSectionRawMutex, DisplayMode, 2> = Watch::new();

static mut APP_CORE_STACK: Stack<8192> = Stack::new();

static CSI_PHASE: Channel<CriticalSectionRawMutex, [f32; VALID_SUBCARRIER_COUNT], 1> =
    Channel::new();
static CSI_MAG: Channel<CriticalSectionRawMutex, [f32; VALID_SUBCARRIER_COUNT], 1> = Channel::new();

// In CSI acquired array from ESP subcarriers are ordered as follows: [0 to 31,-32 to -1]
// Pilot Subcarriers -> (7:+7,21:+21,43:-21,57:-7)
// Null Subcarriers -> (0, ±25/±26, +27-31, -32--27)
// Refer to https://github.com/StevenMHernandez/ESP32-CSI-Tool/issues/12

const NON_DATA_SUBCARRIERS: &[usize] = &[
    0, // DC (+0)
    25, 26, 27, 28, 29, 30, 31, // +25 to +31
    32, 33, 34, 35, 36, 37, 38, // -32 to -26
];
const VALID_SUBCARRIER_COUNT: usize = 64 - NON_DATA_SUBCARRIERS.len(); // Now 49
const ORDERED_K: [i32; VALID_SUBCARRIER_COUNT] = [
    // Negatives: -25 to -1, incl. -21,-7 (25 items)
    -25, -24, -23, -22, -21, -20, -19, -18, -17, -16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6,
    -5, -4, -3, -2, -1, // Positives: +1 to +24, incl. +7,+21 (24 items)
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
];

// Set up the display size
const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);

// Calculate framebuffer size based on the display size and color mode
const FB_SIZE: usize = framebuffer_size(DISPLAY_SIZE, ColorMode::Rgb888);

struct DummyReset;
impl ResetInterface for DummyReset {
    type Error = Error;
    fn reset(&mut self) -> Result<(), Self::Error> {
        Ok(()) // No-op
    }
}

struct Sh8601ResetDriver;
// Empty Implementation for display since its handled by the touch driver
impl Sh8601ResetInterface for Sh8601ResetDriver {
    type Error = Error;
    fn reset(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

pub struct TouchResetDriver<I2C> {
    i2c: I2C,
}

impl<I2C> TouchResetDriver<I2C> {
    pub fn new(i2c: I2C) -> Self {
        TouchResetDriver { i2c }
    }
}

impl<I2C> ResetInterface for TouchResetDriver<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    type Error = Error;

    fn reset(&mut self) -> Result<(), Self::Error> {
        let delay = Delay::new();
        self.i2c.write(0x20, &[0x03, 0x00]).unwrap(); // Configure as output
        self.i2c.write(0x20, &[0x01, 0b0000_0010]).unwrap(); // Drive low
        delay.delay_millis(20);
        self.i2c.write(0x20, &[0x01, 0b0000_0111]).unwrap(); // Drive high
        delay.delay_millis(150);
        Ok(())
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Configure System Clock
    let config = esp_hal::Config::default().with_cpu_clock(esp_hal::clock::CpuClock::max());
    // Take Peripherals
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size:72 * 1024);

    // ------------------ Display Initialization ------------------ //

    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let delay = Delay::new();

    // DMA Buffers for SPI
    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(DMA_CHUNK_SIZE);
    let dma_rx_buf = DmaRxBuf::new(rx_descriptors, rx_buffer).unwrap();
    let dma_tx_buf = DmaTxBuf::new(tx_descriptors, tx_buffer).unwrap();

    // SPI Configuration for Waveshare ESP32-S3 1.8inch AMOLED Touch Display
    // Hardware is configured for QSPI. Pinout obtained from the schematic.
    // Schematic:
    // https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8.pdf
    // Using DMA for more efficient SPI communication.
    let lcd_spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(80_u32))
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

    // I2C Configuration for Waveshare ESP32-S3 1.8inch AMOLED Touch Display
    // Display uses an I2C IO Expander (TCA9554PWR) to control the LCD_RESET and LCD_DC lines.
    // Pinout:
    // SDA -> GPIO15
    // SCL -> GPIO14
    // Schematic:
    // https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8.pdf
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(300)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO15)
    .with_scl(peripherals.GPIO14);

    let i2c_cell = mk_static!(AtomicCell<I2c<'static, Blocking>>, AtomicCell::new(i2c));

    // Initialize I2C GPIO Reset Pin for the WaveShare 1.8" AMOLED display
    let reset = Sh8601ResetDriver {};
    let mut touch_reset = TouchResetDriver::new(i2c::AtomicDevice::new(i2c_cell));
    touch_reset.reset().unwrap();

    // Initialize display driver for the Waveshare 1.8" AMOLED display
    let ws_driver = Ws18AmoledDriver::new(lcd_spi);

    // Instantiate and Initialize Touch Driver
    let mut touch = Ft3x68Driver::new(
        i2c::AtomicDevice::new(i2c_cell),
        FT3168_DEVICE_ADDRESS,
        DummyReset {},
        delay,
    );

    // Instantiate and Initialize Display
    println!("Initializing SH8601 Display...");
    let display_res = Sh8601Driver::new_heap::<_, FB_SIZE>(
        ws_driver,
        reset,
        ColorMode::Rgb888,
        DISPLAY_SIZE,
        delay,
    );
    let display = match display_res {
        Ok(d) => {
            println!("Display initialized successfully.");
            d
        }
        Err(e) => {
            println!("Error initializing display: {:?}", e);
            loop {}
        }
    };

    // ------------------ WiFi & CSI Collection Initialization ------------------ //

    // Instantiate peripherals necessary to set up  WiFi
    let timer1 = esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG0);
    let wifi = peripherals.WIFI;
    let timer = timer1.timer0;
    let mut rng = Rng::new(peripherals.RNG);

    // Initialize WiFi Controller
    let init = &*mk_static!(EspWifiController<'static>, init(timer, rng,).unwrap());

    // Instantiate WiFi controller and interfaces
    let (controller, interfaces) = esp_wifi::wifi::new(&init, wifi).unwrap();

    // Obtain a random seed value
    let seed = rng.random() as u64;

    println!("WiFi Controller Initialized");

    // Create a CSI collector configuration
    // Device configured to default as Sniffer
    // Traffic generation, although default, is ignored
    // Network Architechture is Sniffer
    let csi_collector = CSICollector::new(
        WiFiConfig {
            ssid: "Connected Motion ".try_into().unwrap(),
            password: "automotion@123".try_into().unwrap(),
            ..Default::default()
        },
        esp_csi_rs::WiFiMode::Station,
        CSIConfig::default(),
        TrafficConfig {
            traffic_type: TrafficType::ICMPPing,
            traffic_interval_ms: 10,
        },
        true,
        NetworkArchitechture::AccessPointStation,
        None,
        false,
    );

    // Initalize CSI collector
    csi_collector
        .init(controller, interfaces, seed, &spawner)
        .unwrap();

    // ---------------- Embassy Intialization ---------------- //
    let timg1 = TimerGroup::new(peripherals.TIMG1);
    esp_hal_embassy::init(timg1.timer0);

    // Spawn CSI Task Collecting CSI Forever
    let csi = csi_collector.start(None);

    spawner.spawn(csi_task(csi)).ok();

    // Spawn Display Task on Second Core
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

    // Spawn display task on a seperate core
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, move || {
            static EXECUTOR: StaticCell<Executor> = StaticCell::new();
            let executor = EXECUTOR.init(Executor::new());
            executor.run(|spawner| {
                // spawner.spawn(touch_task(touch)).ok();
                spawner.spawn(display_task(display)).ok();
            });
        })
        .unwrap();

    touch
        .initialize()
        .expect("Failed to initialize touch driver");

    // Activate Gesture Mode to detect gestures
    touch
        .set_gesture_mode(true)
        .expect("Failed to set gesture mode");

    // Set the current display mode and send to Watch variable (default to Magnitude)
    let mut current_display_mode = DisplayMode::Magnitude;
    let display_mode_watch = DISPLAY_MODE.sender();
    display_mode_watch.send(current_display_mode);
    // Default to None gesture
    let mut current_gesture = Gesture::None;

    loop {
        let _ = touch.touch1();
        let new_gesture = touch.read_gesture().unwrap_or_else(|e| {
            println!("Error reading gesture: {:?}", e);
            Gesture::None
        });

        if new_gesture != current_gesture {
            match new_gesture {
                Gesture::SwipeLeft => {
                    current_display_mode = match current_display_mode {
                        DisplayMode::Magnitude => DisplayMode::Phase,
                        DisplayMode::Phase => DisplayMode::Magnitude,
                    };
                    display_mode_watch.send(current_display_mode);
                    current_gesture = new_gesture;
                }
                Gesture::SwipeRight => {
                    current_display_mode = match current_display_mode {
                        DisplayMode::Magnitude => DisplayMode::Phase,
                        DisplayMode::Phase => DisplayMode::Magnitude,
                    };
                    display_mode_watch.send(current_display_mode);
                    current_gesture = new_gesture;
                }
                _ => {}
            }
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn display_task(mut display_driver: Sh8601Driver<Ws18AmoledDriver, Sh8601ResetDriver>) {
    let row_height: u32 = DISPLAY_SIZE.height as u32 / VALID_SUBCARRIER_COUNT as u32;
    let column_width: u32 = 5; // Match phase_display_task
    let col_count = (DISPLAY_SIZE.width as u32 / column_width) as u32;
    let mut current_col: u32 = 0;

    let mut display_mode = DisplayMode::Magnitude;
    let mut display_watch = DISPLAY_MODE.receiver().unwrap();

    loop {
        // Check if new display mode is available
        // Optionally empty screen on change of display mode
        if let Some(new_mode) = display_watch.try_changed() {
            if new_mode != display_mode {
                display_mode = new_mode;
                println!("Display task switched to {:?}", display_mode); // Debug
                display_driver.clear(Rgb888::new(0, 0, 0)).unwrap();
                display_driver
                    .partial_flush(
                        0,
                        (DISPLAY_SIZE.width - 1) as u16,
                        0,
                        (DISPLAY_SIZE.height - 1) as u16,
                        ColorMode::Rgb888,
                    )
                    .unwrap();
            }
        }

        let csi_data = match display_mode {
            DisplayMode::Magnitude => CSI_MAG.receive().await,
            DisplayMode::Phase => CSI_PHASE.receive().await,
        };
        for row in 0..VALID_SUBCARRIER_COUNT {
            let normalized_data = csi_data[row];
            let color = Rgb888::new(
                (normalized_data * 255.0) as u8,
                ((1.0 - (2.0 * (normalized_data - 0.5).abs())) * 255.0) as u8,
                ((1.0 - normalized_data) * 255.0) as u8,
            );
            let x = current_col * column_width;
            let y = row as u32 * row_height;
            let square_width = if current_col == col_count - 1 {
                DISPLAY_SIZE.width as u32 - (current_col * column_width)
            } else {
                column_width
            };
            let square_height = if row == VALID_SUBCARRIER_COUNT - 1 {
                DISPLAY_SIZE.height as u32 - (row as u32 * row_height)
            } else {
                row_height
            };

            if square_width > 0 && square_height > 0 {
                Rectangle::new(
                    Point::new(x as i32, y as i32),
                    Size::new(square_width, square_height),
                )
                .into_styled(PrimitiveStyleBuilder::new().fill_color(color).build())
                .draw(&mut display_driver)
                .unwrap();
            }
        }

        let x_start = (current_col * column_width) as u16;
        let x_end = if current_col == col_count - 1 {
            (DISPLAY_SIZE.width - 1) as u16
        } else {
            ((current_col + 1) * column_width - 1) as u16
        };
        let y_start = 0u16;
        let y_end = (DISPLAY_SIZE.height - 1) as u16;

        if let Err(e) =
            display_driver.partial_flush(x_start, x_end, y_start, y_end, ColorMode::Rgb888)
        {
            println!("Error flushing column {}: {:?}", current_col, e);
        }

        current_col = (current_col + 1) % col_count;
    }
}

fn smooth_phase(
    phase: &[f32; VALID_SUBCARRIER_COUNT],
    window: usize,
) -> [f32; VALID_SUBCARRIER_COUNT] {
    let mut smoothed = [0.0; VALID_SUBCARRIER_COUNT];
    for i in 0..VALID_SUBCARRIER_COUNT {
        let start = i.saturating_sub(window / 2);
        let end = (i + window / 2 + 1).min(VALID_SUBCARRIER_COUNT);
        let sum: f32 = phase[start..end].iter().sum();
        smoothed[i] = sum / (end - start) as f32;
    }
    smoothed
}

fn reorder_subcarriers(data: &[f32; VALID_SUBCARRIER_COUNT]) -> [f32; VALID_SUBCARRIER_COUNT] {
    const POSITIVE_SUBCARRIER_COUNT: usize = 24;
    let mut ordered_data = [0.0; VALID_SUBCARRIER_COUNT];
    let mut idx = 0;
    // Negative subcarriers are at the end of the input array
    for i in POSITIVE_SUBCARRIER_COUNT..VALID_SUBCARRIER_COUNT {
        ordered_data[idx] = data[i];
        idx += 1;
    }
    // Positive subcarriers are at the beginning
    for i in 0..POSITIVE_SUBCARRIER_COUNT {
        ordered_data[idx] = data[i];
        idx += 1;
    }
    ordered_data
}

#[embassy_executor::task]
async fn csi_task(
    mut csi_buffer: Subscriber<'static, CriticalSectionRawMutex, Vec<i8, 616>, 4, 2, 1>,
) {
    let mut display_mode = DisplayMode::Magnitude;
    let mut display_watch = DISPLAY_MODE.receiver().unwrap();

    loop {
        // Check if new display mode is available
        if let Some(new_mode) = display_watch.try_changed() {
            display_mode = new_mode
        };

        match csi_buffer.next_message().await {
            WaitResult::Lagged(_) => {
                println!("CSI task lagged, skipping message");
                continue;
            }
            WaitResult::Message(data) => {
                let csi_payload = &data[4..];
                if csi_payload.len() < 128 {
                    continue;
                }

                match display_mode {
                    DisplayMode::Phase => {
                        let mut phase: [f32; VALID_SUBCARRIER_COUNT] =
                            [0.0; VALID_SUBCARRIER_COUNT];
                        let mut valid_index = 0;

                        for i in 0..64 {
                            if NON_DATA_SUBCARRIERS.contains(&i) {
                                continue;
                            }
                            if valid_index >= VALID_SUBCARRIER_COUNT {
                                break;
                            }

                            let real = csi_payload[2 * i] as f32;
                            let imag = csi_payload[2 * i + 1] as f32;

                            if real == 0.0 && imag == 0.0 {
                                if valid_index > 0 {
                                    phase[valid_index] = phase[valid_index - 1];
                                } else {
                                    phase[valid_index] = 0.0;
                                }
                            } else {
                                phase[valid_index] = imag.atan2(real);
                            }
                            valid_index += 1;
                        }

                        let mut ordered_phase = reorder_subcarriers(&phase);

                        unwrap_phase(&mut ordered_phase);

                        let mut sum_k = 0.0f32;
                        let mut sum_phase = 0.0f32;
                        let mut sum_k2 = 0.0f32;
                        let mut sum_k_phase = 0.0f32;
                        let n = VALID_SUBCARRIER_COUNT as f32;

                        for i in 0..VALID_SUBCARRIER_COUNT {
                            let ki = ORDERED_K[i] as f32;
                            let pi = ordered_phase[i];
                            sum_k += ki;
                            sum_phase += pi;
                            sum_k2 += ki * ki;
                            sum_k_phase += ki * pi;
                        }

                        let denominator = n * sum_k2 - sum_k * sum_k;
                        let slope = if denominator.abs() > f32::EPSILON {
                            (n * sum_k_phase - sum_k * sum_phase) / denominator
                        } else {
                            0.0
                        };
                        let intercept = (sum_phase - slope * sum_k) / n;

                        for i in 0..VALID_SUBCARRIER_COUNT {
                            let ki = ORDERED_K[i] as f32;
                            ordered_phase[i] -= slope * ki + intercept;
                        }

                        let smoothed_phase = smooth_phase(&ordered_phase, 3);
                        let max_phase = smoothed_phase
                            .iter()
                            .cloned()
                            .fold(f32::NEG_INFINITY, f32::max);
                        let min_phase =
                            smoothed_phase.iter().cloned().fold(f32::INFINITY, f32::min);
                        let phase_range = max_phase - min_phase;
                        let mut normalized_phase = smoothed_phase;
                        if phase_range > 0.0 {
                            for i in 0..VALID_SUBCARRIER_COUNT {
                                normalized_phase[i] = (smoothed_phase[i] - min_phase) / phase_range;
                            }
                        } else {
                            for i in 0..VALID_SUBCARRIER_COUNT {
                                normalized_phase[i] = 0.5;
                            }
                        }
                        CSI_PHASE.send(normalized_phase).await;
                    }
                    DisplayMode::Magnitude => {
                        let mut amplitude: [f32; VALID_SUBCARRIER_COUNT] =
                            [0.0; VALID_SUBCARRIER_COUNT];
                        let mut valid_index = 0;

                        for i in 0..64 {
                            if NON_DATA_SUBCARRIERS.contains(&i) {
                                continue;
                            }
                            if valid_index >= VALID_SUBCARRIER_COUNT {
                                break;
                            }

                            let real = csi_payload[2 * i] as f32;
                            let imag = csi_payload[2 * i + 1] as f32;

                            if real == 0.0 && imag == 0.0 {
                                if valid_index > 0 {
                                    amplitude[valid_index] = amplitude[valid_index - 1];
                                } else {
                                    amplitude[valid_index] = 0.0;
                                }
                            } else {
                                amplitude[valid_index] = (real * real + imag * imag).sqrt();
                            }
                            valid_index += 1;
                        }

                        let ordered_amplitude = reorder_subcarriers(&amplitude);

                        let smoothed_amplitude = smooth_amplitude(&ordered_amplitude, 7);
                        let max_amplitude = smoothed_amplitude
                            .iter()
                            .cloned()
                            .fold(f32::NEG_INFINITY, f32::max);
                        let min_amplitude = smoothed_amplitude
                            .iter()
                            .cloned()
                            .fold(f32::INFINITY, f32::min);
                        let amplitude_range = if max_amplitude - min_amplitude > 0.0 {
                            max_amplitude - min_amplitude
                        } else {
                            1.0
                        };
                        let mut normalized_amplitude = smoothed_amplitude;
                        for i in 0..VALID_SUBCARRIER_COUNT {
                            normalized_amplitude[i] =
                                (smoothed_amplitude[i] - min_amplitude) / amplitude_range;
                            normalized_amplitude[i] = normalized_amplitude[i].clamp(0.0, 1.0);
                        }
                        CSI_MAG.send(normalized_amplitude).await;
                    }
                }
            }
        }
    }
}

fn smooth_amplitude(
    amplitude: &[f32; VALID_SUBCARRIER_COUNT],
    window: usize,
) -> [f32; VALID_SUBCARRIER_COUNT] {
    let mut smoothed = [0.0; VALID_SUBCARRIER_COUNT];
    for i in 0..VALID_SUBCARRIER_COUNT {
        let start = i.saturating_sub(window / 2);
        let end = (i + window / 2 + 1).min(VALID_SUBCARRIER_COUNT);
        let sum: f32 = amplitude[start..end].iter().sum();
        smoothed[i] = sum / (end - start) as f32;
    }
    smoothed
}

fn unwrap_phase(phase: &mut [f32]) {
    if phase.is_empty() {
        return;
    }
    let mut prev = phase[0];
    for i in 1..phase.len() {
        let current = phase[i];
        let delta = current - prev;
        let adjustment =
            (delta / (2.0 * core::f32::consts::PI)).round() * 2.0 * core::f32::consts::PI;
        phase[i] = current - adjustment;
        prev = phase[i];
    }
}
