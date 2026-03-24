//! Touch/gesture tasks that control display mode switching.

use cst226_rs::{Cst226Driver, Gesture};
use embassy_executor::task;
use embassy_futures::yield_now;
use embassy_sync::watch::Sender;
use embassy_time::{Duration, Instant};
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
use embassy_time::Timer;
use esp_csi_rs::log_ln;
use esp_hal::{
    delay::Delay,
    gpio::Input,
    i2c::master::I2c,
    sync::RawPriorityLimitedMutex,
};

use crate::{config::DisplayMode, reset::ResetDriver};

const MODE_SWITCH_COOLDOWN: Duration = Duration::from_millis(350);

/// Gesture runner for LilyGo touch controller.
///
/// Swipe gestures map to cycling between [`DisplayMode`] variants.
#[task]
pub async fn display_gesture_runner(
    touch_int: Input<'static>,
    mut touch: Cst226Driver<I2c<'static, esp_hal::Blocking>, Delay, ResetDriver<esp_hal::gpio::Output<'static>>>,
    display_mode_watch: Sender<'static, RawPriorityLimitedMutex, DisplayMode, 2>,
) {
    let mut current_gesture = Gesture::None;
    let mut current_display_mode = DisplayMode::Magnitude;
    let mut last_mode_switch: Option<Instant> = None;
    let mut touch_armed = true;

    loop {
        let pressed = touch_int.is_low();
        if !pressed {
            touch_armed = true;
            yield_now().await;
            continue;
        }

        if !touch_armed {
            yield_now().await;
            continue;
        }
        touch_armed = false;

        let new_gesture = touch.get_gesture().unwrap_or_else(|e| {
            log_ln!("Error reading gesture: {:?}", e);
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
                let cooldown_ok = last_mode_switch
                    .map(|t| Instant::now().saturating_duration_since(t) >= MODE_SWITCH_COOLDOWN)
                    .unwrap_or(true);
                if cooldown_ok {
                    display_mode_watch.send(current_display_mode);
                    last_mode_switch = Some(Instant::now());
                }
            }

            current_gesture = new_gesture;
        }

        yield_now().await;
    }
}

#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
/// Gesture/touch runner for Waveshare FT3x68-compatible touch controller.
///
/// Uses bounded per-touch sampling to detect horizontal swipes without
/// continuously polling I2C (for long-run stability).
#[task]
pub async fn waveshare_touch_runner(
    touch_int: Input<'static>,
    mut touch_i2c: I2c<'static, esp_hal::Blocking>,
    display_mode_watch: Sender<'static, RawPriorityLimitedMutex, DisplayMode, 2>,
) {
    const FT3X68_ADDR: u8 = 0x38;
    const REG_GEST_ID: u8 = 0x01;
    const REG_TD_STATUS: u8 = 0x02;
    const GESTURE_SWIPE_RIGHT: u8 = 0x14;
    const GESTURE_SWIPE_LEFT: u8 = 0x1C;

    let mut current_display_mode = DisplayMode::Magnitude;
    let mut last_mode_switch: Option<Instant> = None;
    let mut touch_armed = true;

    const SAMPLE_INTERVAL: Duration = Duration::from_millis(16);
    const MAX_SAMPLES_PER_TOUCH: usize = 8;
    const SWIPE_THRESHOLD_PX: i32 = 32;

    loop {
        let pressed = touch_int.is_low();
        if !pressed {
            touch_armed = true;
            yield_now().await;
            continue;
        }

        if !touch_armed {
            yield_now().await;
            continue;
        }

        touch_armed = false;

        // Sample only for a short bounded window during each touch press.
        // This keeps I2C activity bounded and avoids hogging the executor.
        let mut start_x: Option<i32> = None;
        let mut start_y: Option<i32> = None;
        let mut last_x: Option<i32> = None;
        let mut last_y: Option<i32> = None;
        let mut swipe_dir: i8 = 0; // +1: right, -1: left

        for _ in 0..MAX_SAMPLES_PER_TOUCH {
            if !touch_int.is_low() {
                break;
            }

            // Fast path: use controller gesture classification if available.
            let mut gest = [0u8; 1];
            if touch_i2c
                .write_read(FT3X68_ADDR, &[REG_GEST_ID], &mut gest)
                .is_ok()
            {
                match gest[0] {
                    GESTURE_SWIPE_RIGHT => {
                        swipe_dir = 1;
                        break;
                    }
                    GESTURE_SWIPE_LEFT => {
                        swipe_dir = -1;
                        break;
                    }
                    _ => {}
                }
            }

            // Fallback: derive swipe direction from sampled coordinates.
            let mut p1 = [0u8; 5];
            if touch_i2c
                .write_read(FT3X68_ADDR, &[REG_TD_STATUS], &mut p1)
                .is_ok()
                && (p1[0] & 0x0F) > 0
            {
                let x = ((((p1[1] & 0x0F) as u16) << 8) | p1[2] as u16) as i32;
                let y = ((((p1[3] & 0x0F) as u16) << 8) | p1[4] as u16) as i32;

                if start_x.is_none() {
                    start_x = Some(x);
                    start_y = Some(y);
                }
                last_x = Some(x);
                last_y = Some(y);
            }

            Timer::after(SAMPLE_INTERVAL).await;
        }

        if swipe_dir == 0 {
            if let (Some(sx), Some(sy), Some(ex), Some(ey)) = (start_x, start_y, last_x, last_y) {
                let dx = ex - sx;
                let dy = ey - sy;
                let adx = if dx >= 0 { dx } else { -dx };
                let ady = if dy >= 0 { dy } else { -dy };

                if adx >= SWIPE_THRESHOLD_PX && adx > (ady + ady) {
                    swipe_dir = if dx > 0 { 1 } else { -1 };
                }
            }
        }

        let cooldown_ok = last_mode_switch
            .map(|t| Instant::now().saturating_duration_since(t) >= MODE_SWITCH_COOLDOWN)
            .unwrap_or(true);

        if cooldown_ok && swipe_dir != 0 {
            current_display_mode = if swipe_dir > 0 {
                current_display_mode.next()
            } else {
                current_display_mode.previous()
            };
            display_mode_watch.send(current_display_mode);
            last_mode_switch = Some(Instant::now());
        }

        yield_now().await;
    }
}
