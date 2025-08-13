#![no_std]
#![no_main]

use core::ptr::addr_of_mut;
use embassy_executor::Spawner;
use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, pubsub::Subscriber,
    watch::Watch,
};
use embedded_hal_bus::i2c;
use embedded_hal_bus::util::AtomicCell;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_csi_rs::{
    config::{CSIConfig, TrafficConfig, TrafficType, WiFiConfig},
    CSICollector, NetworkArchitechture,
};
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
use esp_hal::{
    dma::{DmaRxBuf, DmaTxBuf},
    gpio::{self, Input},
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
    mono_font::MonoTextStyle,
    pixelcolor::Rgb888,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle},
    text::Text,
};
use profont::{PROFONT_14_POINT, PROFONT_24_POINT};

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

impl DisplayMode {
    pub fn next(&self) -> Self {
        match self {
            DisplayMode::Magnitude => DisplayMode::Phase,
            DisplayMode::Phase => DisplayMode::Magnitude,
        }
    }

    pub fn previous(&self) -> Self {
        match self {
            DisplayMode::Magnitude => DisplayMode::Phase,
            DisplayMode::Phase => DisplayMode::Magnitude,
        }
    }
}

// The Watch construct immediately overwrites the previous value when a new one is sent, without waiting for all receivers to read the previous value.
// The Watch sender is supposed to update only when a gesture is detected.
static DISPLAY_MODE: Watch<CriticalSectionRawMutex, DisplayMode, 2> = Watch::new();

// Stack Instance for Second Core
static mut APP_CORE_STACK: Stack<8192> = Stack::new();

static CSI_PHASE: Channel<CriticalSectionRawMutex, [f32; VALID_SUBCARRIER_COUNT], 1> =
    Channel::new();
static CSI_MAG: Channel<CriticalSectionRawMutex, [f32; VALID_SUBCARRIER_COUNT], 1> = Channel::new();

// According to documentation the acquired array from ESP is an LLTF array with subcarriers ordered as follows: [0 to 31,-32 to -1]
// There are several formats depending on supported training algortihim
// Core library would have to integrate some indication for the type of CSI recieved if to be accomodated here

// Pilot Subcarriers -> (+7/-7,+21/-21)
// Null Subcarriers -> (0, +27~31, -27~-32)
// This is assuming LLTF
// Refer to https://github.com/StevenMHernandez/ESP32-CSI-Tool/issues/12

const VALID_SUBCARRIER_COUNT: usize = 52;

// Set up the display size
// The Waveshare 1.8" AMOLED display has a resolution of 368x448 pixels
const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);

// Heatmap placement configuration
// This allows for the heatmap to be placed on different parts of the display
// Starting point is the upper left corner of the heatmap:
const HEATMAP_START_X: u32 = 20;
const HEATMAP_START_Y: u32 = 80;
const HEATMAP_WIDTH: u32 = 350;
const HEATMAP_HEIGHT: u32 = 350;

// Make sure to clip to fit within display bounds
const HEATMAP_MAX_WIDTH: u32 = DISPLAY_SIZE.width as u32 - HEATMAP_START_X;
const HEATMAP_MAX_HEIGHT: u32 = DISPLAY_SIZE.height as u32 - HEATMAP_START_Y;
const HEATMAP_EFFECTIVE_WIDTH: u32 = if HEATMAP_WIDTH < HEATMAP_MAX_WIDTH {
    HEATMAP_WIDTH
} else {
    HEATMAP_MAX_WIDTH
};
const HEATMAP_EFFECTIVE_HEIGHT: u32 = if HEATMAP_HEIGHT < HEATMAP_MAX_HEIGHT {
    HEATMAP_HEIGHT
} else {
    HEATMAP_MAX_HEIGHT
};

// Calculate framebuffer size based on the display size and color mode
const FB_SIZE: usize = framebuffer_size(DISPLAY_SIZE, ColorMode::Rgb888);

// The display driver and the touch driver are reset by the same I2C-based I/O expander on the Waveshare display
// Also both drivers require a ResetInterface trait implmentation, however only one implementation is sufficient.
// This is a dummy reset implementation that does nothing to statisfy the requirements of the driver.
struct DummyReset;
impl ResetInterface for DummyReset {
    type Error = Error;
    fn reset(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
impl Sh8601ResetInterface for DummyReset {
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

    // Display Configuration //

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

    // Touch Configuration //

    // I2C Configuration for Waveshare ESP32-S3 1.8inch AMOLED Touch Display
    // Display uses an I2C IO Expander (TCA9554PWR) to control the LCD_RESET and LCD_DC lines.
    // Pinout:
    // SDA -> GPIO15
    // SCL -> GPIO14
    // Schematic:
    // https://files.waveshare.com/wiki/ESP32-S3-Touch-AMOLED-1.8/ESP32-S3-Touch-AMOLED-1.8.pdf
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO15)
    .with_scl(peripherals.GPIO14);

    let i2c_cell = mk_static!(AtomicCell<I2c<'static, Blocking>>, AtomicCell::new(i2c));

    // Initialize I2C GPIO Reset Pin for the WaveShare 1.8" AMOLED display
    let mut touch_reset = TouchResetDriver::new(i2c::AtomicDevice::new(i2c_cell));
    touch_reset.reset().unwrap();

    // Initialize display driver for the Waveshare 1.8" AMOLED display
    let ws_driver: Ws18AmoledDriver = Ws18AmoledDriver::new(lcd_spi);

    // Configure Pin to detect touch interrupts
    // Touch interrupt is connected to GPIO21 acording to WaveShare Schematic
    let mut touch_int = Input::new(peripherals.GPIO21, gpio::InputConfig::default());

    // Instantiate and Initialize Touch Driver
    let mut touch = Ft3x68Driver::new(
        i2c::AtomicDevice::new(i2c_cell),
        FT3168_DEVICE_ADDRESS,
        DummyReset {},
        delay,
    );

    // Instantiate and Initialize Display Driver
    println!("Initializing SH8601 Display...");
    let display_res = Sh8601Driver::new_heap::<_, FB_SIZE>(
        ws_driver,
        DummyReset {},
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

    //  WiFi & CSI Collection Configuration & Initialization //

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
    let csi_collector = CSICollector::new(
        WiFiConfig {
            ssid: "SSID".try_into().unwrap(),
            password: "PASSWORD".try_into().unwrap(),
            ..Default::default()
        },
        esp_csi_rs::WiFiMode::Station,
        CSIConfig::default(),
        TrafficConfig {
            traffic_type: TrafficType::ICMPPing,
            traffic_interval_ms: 5,
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

    // Embassy Intialization //
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

    // Initalize touch
    touch
        .initialize()
        .expect("Failed to initialize touch driver");

    // Activate Gesture Mode to detect gestures
    touch
        .set_gesture_mode(true)
        .expect("Failed to set gesture mode");

    // Default detected gestrue to None
    let mut current_gesture = Gesture::None;

    // Set the current display mode (default to Magnitude), acquire Watch sender, and update Watch variable.
    let mut current_display_mode = DisplayMode::Magnitude;
    let display_mode_watch = DISPLAY_MODE.sender();
    display_mode_watch.send(current_display_mode);

    loop {
        // Wait for touch event to happen
        touch_int.wait_for(gpio::Event::FallingEdge).await;
        let new_gesture = touch.read_gesture().unwrap_or_else(|e| {
            println!("Error reading gesture: {:?}", e);
            Gesture::None
        });

        // If a new gesture is detected, update the display mode and send a new value to the Watch sender
        if new_gesture != current_gesture {
            let mut mode_changed = true;

            current_display_mode = match new_gesture {
                Gesture::SwipeRight => current_display_mode.next(),
                Gesture::SwipeLeft => current_display_mode.previous(),
                _ => {
                    mode_changed = false;
                    current_display_mode
                }
            };

            if mode_changed {
                display_mode_watch.send(current_display_mode);
            }

            current_gesture = new_gesture;
        }
    }
}

#[embassy_executor::task]
async fn display_task(mut display_driver: Sh8601Driver<Ws18AmoledDriver, DummyReset>) {
    // Initial full-screen clear
    display_driver.clear(Rgb888::new(0, 0, 0)).unwrap();

    // Text styles
    let title_text_style = MonoTextStyle::new(&PROFONT_24_POINT, Rgb888::YELLOW);
    let static_text_style = MonoTextStyle::new(&PROFONT_14_POINT, Rgb888::WHITE);
    let clear_style = PrimitiveStyleBuilder::new()
        .fill_color(Rgb888::new(0, 0, 0))
        .build();

    // Static element drawing in framebuffer
    // Draw static y-axis label vertically, character-by-character
    let yaxis_label = "Subcarriers";
    let yaxis_label_style = static_text_style.clone();
    let char_height = yaxis_label_style.font.character_size.height as i32;
    let char_width = yaxis_label_style.font.character_size.width as i32;
    let yaxis_label_height = yaxis_label.chars().count() as i32 * char_height;

    // Calculate position to be centered vertically next to the heatmap
    let label_start_y =
        HEATMAP_START_Y as i32 + (HEATMAP_EFFECTIVE_HEIGHT as i32 - yaxis_label_height) / 2;
    let label_x = (HEATMAP_START_X as i32 - char_width - 5).max(0); // 5px padding

    // Loop and draw each character indivudually
    for (i, ch) in yaxis_label.chars().enumerate() {
        let mut buffer = [0u8; 4];
        let char_str = ch.encode_utf8(&mut buffer);
        Text::new(
            char_str,
            Point::new(label_x, label_start_y + (i as i32 * char_height)),
            yaxis_label_style.clone(),
        )
        .draw(&mut display_driver)
        .unwrap();
    }

    // Draw static x-axis label horizontally & center
    let xaxis_label_str = "Packets";
    let xaxis_label = Text::new(xaxis_label_str, Point::zero(), static_text_style.clone());
    let xaxis_label_width = xaxis_label.bounding_box().size.width as i32;
    let xaxis_label_x =
        HEATMAP_START_X as i32 + (HEATMAP_EFFECTIVE_WIDTH as i32 - xaxis_label_width) / 2;
    let xaxis_label_y = HEATMAP_START_Y as i32 + HEATMAP_EFFECTIVE_HEIGHT as i32 + 15;
    Text::new(
        xaxis_label_str,
        Point::new(xaxis_label_x, xaxis_label_y),
        static_text_style.clone(),
    )
    .draw(&mut display_driver)
    .unwrap();

    let mut current_display_mode = DisplayMode::Magnitude;
    let mut display_watch = DISPLAY_MODE.receiver().unwrap();

    let text_strip_height: i32 = 40;
    let text_strip_y: i32 = (HEATMAP_START_Y as i32 - text_strip_height - 5).max(0);

    // Draw title text
    let draw_title = |display: &mut Sh8601Driver<_, _>, mode: DisplayMode| {
        let mode_str = match mode {
            DisplayMode::Magnitude => "Magnitude",
            DisplayMode::Phase => "Phase",
        };
        let title_text = Text::new(mode_str, Point::zero(), title_text_style.clone());
        let text_width = title_text.bounding_box().size.width as i32;
        let centered_x = HEATMAP_START_X as i32 + (HEATMAP_EFFECTIVE_WIDTH as i32 - text_width) / 2;

        Rectangle::new(
            Point::new(HEATMAP_START_X as i32, text_strip_y),
            Size::new(HEATMAP_EFFECTIVE_WIDTH, text_strip_height as u32),
        )
        .into_styled(clear_style)
        .draw(display)
        .unwrap();

        Text::new(
            mode_str,
            Point::new(centered_x, HEATMAP_START_Y as i32 - 20),
            title_text_style.clone(),
        )
        .draw(display)
        .unwrap();
    };

    draw_title(&mut display_driver, current_display_mode);

    // Flush static text changes
    display_driver
        .partial_flush(
            0,
            (DISPLAY_SIZE.width - 1) as u16,
            0,
            (DISPLAY_SIZE.height - 1) as u16,
            ColorMode::Rgb888,
        )
        .unwrap();

    let column_width: u32 = 5;
    let col_count = HEATMAP_EFFECTIVE_WIDTH / column_width;
    let mut current_col: u32 = 0;

    // Enter loop for heatmap updates
    loop {
        if let Some(new_mode) = display_watch.try_changed() {
            if new_mode != current_display_mode {
                current_display_mode = new_mode;

                draw_title(&mut display_driver, current_display_mode);

                Rectangle::new(
                    Point::new(HEATMAP_START_X as i32, HEATMAP_START_Y as i32),
                    Size::new(HEATMAP_EFFECTIVE_WIDTH, HEATMAP_EFFECTIVE_HEIGHT),
                )
                .into_styled(clear_style)
                .draw(&mut display_driver)
                .unwrap();

                current_col = 0;

                display_driver
                    .partial_flush(
                        HEATMAP_START_X as u16,
                        (HEATMAP_START_X + HEATMAP_EFFECTIVE_WIDTH - 1) as u16,
                        text_strip_y as u16,
                        (HEATMAP_START_Y + HEATMAP_EFFECTIVE_HEIGHT - 1) as u16,
                        ColorMode::Rgb888,
                    )
                    .unwrap();
            }
        }

        let csi_data = match current_display_mode {
            DisplayMode::Magnitude => CSI_MAG.receive().await,
            DisplayMode::Phase => CSI_PHASE.receive().await,
        };

        if current_col == 0 {
            Rectangle::new(
                Point::new(HEATMAP_START_X as i32, HEATMAP_START_Y as i32),
                Size::new(HEATMAP_EFFECTIVE_WIDTH, HEATMAP_EFFECTIVE_HEIGHT),
            )
            .into_styled(clear_style)
            .draw(&mut display_driver)
            .unwrap();
        }

        let total_height: u32 = HEATMAP_EFFECTIVE_HEIGHT;
        let base_row_height: u32 = total_height / VALID_SUBCARRIER_COUNT as u32;
        let extra_rows: u32 = total_height % VALID_SUBCARRIER_COUNT as u32;
        let mut y_pos: u32 = HEATMAP_START_Y;

        if current_col == 0 {
            // Clear the entire heatmap area at the start of a new cycle
            Rectangle::new(
                Point::new(HEATMAP_START_X as i32, HEATMAP_START_Y as i32),
                Size::new(HEATMAP_EFFECTIVE_WIDTH, HEATMAP_EFFECTIVE_HEIGHT),
            )
            .into_styled(clear_style)
            .draw(&mut display_driver)
            .unwrap();
        }

        for row in 0..VALID_SUBCARRIER_COUNT {
            let normalized_data = csi_data[row];
            let color = Rgb888::new(
                (normalized_data * 255.0) as u8,
                ((1.0 - (2.0 * (normalized_data - 0.5).abs())) * 255.0) as u8,
                ((1.0 - normalized_data) * 255.0) as u8,
            );
            let x = HEATMAP_START_X + current_col * column_width;
            let square_width = if current_col == col_count - 1 {
                HEATMAP_EFFECTIVE_WIDTH - (current_col * column_width)
            } else {
                column_width
            };
            let mut square_height = base_row_height + if (row as u32) < extra_rows { 1 } else { 0 };
            if row == VALID_SUBCARRIER_COUNT - 1 {
                let target_bottom = HEATMAP_START_Y + HEATMAP_EFFECTIVE_HEIGHT;
                square_height = target_bottom - y_pos;
            }

            if square_width > 0 && square_height > 0 {
                Rectangle::new(
                    Point::new(x as i32, y_pos as i32),
                    Size::new(square_width, square_height),
                )
                .into_styled(PrimitiveStyleBuilder::new().fill_color(color).build())
                .draw(&mut display_driver)
                .unwrap();
            }

            y_pos += square_height;
        }

        let x_start = (HEATMAP_START_X + current_col * column_width) as u16;
        let x_end = (x_start + column_width as u16 - 1)
            .min((HEATMAP_START_X + HEATMAP_EFFECTIVE_WIDTH - 1) as u16);
        let y_end = (HEATMAP_START_Y + HEATMAP_EFFECTIVE_HEIGHT - 1) as u16;

        if let Err(e) = display_driver.partial_flush(
            x_start,
            x_end,
            HEATMAP_START_Y as u16,
            y_end,
            ColorMode::Rgb888,
        ) {
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

        // Grab the next data whether lagged or not
        let mut data = csi_buffer.next_message_pure().await;

        let csi_payload = &mut data[4..];

        // Order the subcarrier data correctly
        swap_upper_lower(csi_payload);

        match display_mode {
            DisplayMode::Phase => {
                let mut phase: [f32; 64] = [0.0; 64];
                let mut valid_index = 0;

                for i in 0..64 {
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

                unwrap_phase(&mut phase);

                // Select valid subcarriers, skipping DC (index 32)
                let mut valid_phase: [f32; VALID_SUBCARRIER_COUNT] = [0.0; VALID_SUBCARRIER_COUNT];
                valid_phase[0..26].copy_from_slice(&phase[6..32]);
                valid_phase[26..52].copy_from_slice(&phase[33..59]);

                let smoothed_phase = smooth_phase(&valid_phase, 3);

                let max_phase = smoothed_phase
                    .iter()
                    .cloned()
                    .fold(f32::NEG_INFINITY, f32::max);
                let min_phase = smoothed_phase.iter().cloned().fold(f32::INFINITY, f32::min);
                let phase_range = max_phase - min_phase;

                let mut normalized_phase: [f32; VALID_SUBCARRIER_COUNT] = smoothed_phase;

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
                let mut amplitude: [f32; 64] = [0.0; 64];
                let mut valid_index = 0;

                for i in 0..64 {
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

                // Select valid subcarriers, skipping DC (index 32)
                let mut valid_amplitude: [f32; VALID_SUBCARRIER_COUNT] =
                    [0.0; VALID_SUBCARRIER_COUNT];
                valid_amplitude[0..26].copy_from_slice(&amplitude[6..32]);
                valid_amplitude[26..52].copy_from_slice(&amplitude[33..59]);

                let smoothed_amplitude = smooth_amplitude(&valid_amplitude, 3);

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
                let mut normalized_amplitude: [f32; VALID_SUBCARRIER_COUNT] = smoothed_amplitude;
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

fn swap_upper_lower<T>(arr: &mut [T]) {
    let mid = arr.len() / 2;
    for i in 0..mid {
        arr.swap(i, i + mid);
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
