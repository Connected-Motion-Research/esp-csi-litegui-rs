//! Touch/gesture tasks that control display mode switching.

use cst226_rs::{Cst226Driver, Gesture};
use embassy_executor::task;
use embassy_futures::yield_now;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Sender};
use embassy_time::{Duration, Instant};
use esp_csi_rs::log_ln;
use esp_hal::{
    delay::Delay,
    gpio::Input,
    i2c::master::I2c,
};

use crate::{
    config::DisplayMode,
    reset::ResetDriver,
};

const MODE_SWITCH_COOLDOWN: Duration = Duration::from_millis(350);

/// Gesture runner for LilyGo touch controller.
///
/// Swipe gestures map to cycling between [`DisplayMode`] variants.
#[task]
pub async fn display_gesture_runner(
    touch_int: Input<'static>,
    mut touch: Cst226Driver<I2c<'static, esp_hal::Blocking>, Delay, ResetDriver<esp_hal::gpio::Output<'static>>>,
    display_mode_watch: Sender<'static, CriticalSectionRawMutex, DisplayMode, 2>,
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
/// Conservative non-blocking mode switch handler.
///
/// Uses touch interrupt edges and cooldown only (no I2C touch reads) to avoid
/// potential runtime stalls from blocking touch transactions.
#[task]
pub async fn waveshare_touch_runner(
    touch_int: Input<'static>,
    _touch_i2c: I2c<'static, esp_hal::Blocking>,
    display_mode_watch: Sender<'static, CriticalSectionRawMutex, DisplayMode, 2>,
) {
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

        let cooldown_ok = last_mode_switch
            .map(|t| Instant::now().saturating_duration_since(t) >= MODE_SWITCH_COOLDOWN)
            .unwrap_or(true);

        if cooldown_ok {
            current_display_mode = current_display_mode.next();
            display_mode_watch.send(current_display_mode);
            last_mode_switch = Some(Instant::now());
        }

        yield_now().await;
    }
}
