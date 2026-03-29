//! Touch/gesture tasks that control display mode switching.

use cst226_rs::{Cst226Driver, Gesture};
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
use ft3x68_rs::{Ft3x68Driver, Gesture as FtGesture, TouchPoint, TouchState};
use embassy_executor::task;
use embassy_futures::yield_now;
use embassy_sync::channel::Sender;
use embassy_time::{Duration, Instant};
use esp_csi_rs::log_ln;
use esp_hal::{
    delay::Delay,
    gpio::Input,
    i2c::master::I2c,
};

use crate::{
    reset::ResetDriver,
    state::ModeIntent,
};
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
use crate::reset::NoopResetDriver;

const MODE_SWITCH_COOLDOWN: Duration = Duration::from_millis(350);

#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
/// Minimum horizontal pixel distance required before treating movement as a swipe.
const SWIPE_MIN_DELTA_PX: i32 = 36;

#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
/// Horizontal dominance factor for swipe classification.
///
/// Movement is treated as horizontal when $|dx| \ge 2 \cdot |dy|$.
const SWIPE_AXIS_RATIO_NUM: i32 = 2;

#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
/// Maps FT3x68 hardware gesture IDs to mode-intent events.
fn mode_intent_from_ft_gesture(gesture: FtGesture) -> Option<ModeIntent> {
    match gesture {
        FtGesture::SwipeRight => Some(ModeIntent::Next),
        FtGesture::SwipeLeft => Some(ModeIntent::Previous),
        _ => None,
    }
}

#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
/// Infers a left/right swipe intent from two touch points.
///
/// Returns `None` when movement is too short or not sufficiently horizontal.
fn infer_horizontal_swipe(
    start: TouchPoint,
    current: TouchPoint,
) -> Option<ModeIntent> {
    let dx = current.x as i32 - start.x as i32;
    let dy = current.y as i32 - start.y as i32;
    let abs_dx = dx.abs();
    let abs_dy = dy.abs();

    if abs_dx < SWIPE_MIN_DELTA_PX {
        return None;
    }

    if abs_dx < abs_dy * SWIPE_AXIS_RATIO_NUM {
        return None;
    }

    if dx > 0 {
        Some(ModeIntent::Next)
    } else {
        Some(ModeIntent::Previous)
    }
}

/// Gesture runner for LilyGo touch controller.
///
/// Swipe gestures map to cycling between [`crate::config::DisplayMode`] variants.
#[task]
pub async fn display_gesture_runner(
    touch_int: Input<'static>,
    mut touch: Cst226Driver<I2c<'static, esp_hal::Blocking>, Delay, ResetDriver<esp_hal::gpio::Output<'static>>>,
    mode_intents: Sender<'static, esp_sync::RawMutex, ModeIntent, 8>,
) {
    let mut current_gesture = Gesture::None;
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

        // If a new gesture is detected, publish a relative mode intent.
        if new_gesture != current_gesture {
            let intent = match new_gesture {
                Gesture::SwipeRight => Some(ModeIntent::Next),
                Gesture::SwipeLeft => Some(ModeIntent::Previous),
                _ => None,
            };

            if let Some(intent) = intent {
                let cooldown_ok = last_mode_switch
                    .map(|t| Instant::now().saturating_duration_since(t) >= MODE_SWITCH_COOLDOWN)
                    .unwrap_or(true);
                if cooldown_ok {
                    let _ = mode_intents.try_send(intent);
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
/// Uses a two-stage gesture strategy:
/// - Prefer FT3x68 hardware gesture IDs.
/// - Fall back to coordinate-based left/right swipe inference.
///
/// Cooldown and per-touch arming are used to avoid repeated mode changes from a
/// single continuous touch.
#[task]
pub async fn waveshare_touch_runner(
    touch_int: Input<'static>,
    mut touch: Ft3x68Driver<I2c<'static, esp_hal::Blocking>, Delay, NoopResetDriver>,
    mode_intents: Sender<'static, esp_sync::RawMutex, ModeIntent, 8>,
) {
    let mut last_mode_switch: Option<Instant> = None;
    let mut touch_armed = true;
    let mut swipe_start: Option<TouchPoint> = None;

    loop {
        let pressed = touch_int.is_low();
        if !pressed {
            touch_armed = true;
            swipe_start = None;
            yield_now().await;
            continue;
        }

        let mut detected_intent = touch
            .read_gesture()
            .ok()
            .and_then(mode_intent_from_ft_gesture);

        if detected_intent.is_none() {
            match touch.touch1() {
                Ok(TouchState::Pressed(point)) => {
                    if let Some(start) = swipe_start.as_ref() {
                        detected_intent = infer_horizontal_swipe(
                            TouchPoint {
                                x: start.x,
                                y: start.y,
                            },
                            point,
                        );
                    } else {
                        swipe_start = Some(point);
                    }
                }
                Ok(TouchState::Released) => {
                    swipe_start = None;
                }
                Err(_) => {}
            }
        }

        if let Some(intent) = detected_intent {
            let cooldown_ok = last_mode_switch
                .map(|t| Instant::now().saturating_duration_since(t) >= MODE_SWITCH_COOLDOWN)
                .unwrap_or(true);

            if cooldown_ok && touch_armed {
                let _ = mode_intents.try_send(intent);
                last_mode_switch = Some(Instant::now());
                touch_armed = false;
            }
        }

        yield_now().await;
    }
}
