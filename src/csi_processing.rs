//! Signal-processing helpers for CSI amplitude and phase preparation.

use crate::config::VALID_SUBCARRIER_COUNT;
use micromath::F32Ext;

/// Swaps upper and lower halves of an array/slice in place.
///
/// This matches CSI subcarrier ordering expected by downstream processing.
pub fn swap_upper_lower<T>(arr: &mut [T]) {
    let mid = arr.len() / 2;
    for i in 0..mid {
        arr.swap(i, i + mid);
    }
}

/// Fast 3-tap smoothing specialized for amplitude data.
pub fn smooth_amplitude3(
    amplitude: &[f32; VALID_SUBCARRIER_COUNT],
) -> [f32; VALID_SUBCARRIER_COUNT] {
    let mut smoothed = [0.0; VALID_SUBCARRIER_COUNT];
    if VALID_SUBCARRIER_COUNT == 0 {
        return smoothed;
    }

    if VALID_SUBCARRIER_COUNT == 1 {
        smoothed[0] = amplitude[0];
        return smoothed;
    }

    smoothed[0] = (amplitude[0] + amplitude[1]) * 0.5;
    for i in 1..(VALID_SUBCARRIER_COUNT - 1) {
        smoothed[i] = (amplitude[i - 1] + amplitude[i] + amplitude[i + 1]) / 3.0;
    }
    smoothed[VALID_SUBCARRIER_COUNT - 1] =
        (amplitude[VALID_SUBCARRIER_COUNT - 2] + amplitude[VALID_SUBCARRIER_COUNT - 1]) * 0.5;

    smoothed
}

/// Fast 3-tap smoothing specialized for phase data.
pub fn smooth_phase3(
    phase: &[f32; VALID_SUBCARRIER_COUNT],
) -> [f32; VALID_SUBCARRIER_COUNT] {
    let mut smoothed = [0.0; VALID_SUBCARRIER_COUNT];
    if VALID_SUBCARRIER_COUNT == 0 {
        return smoothed;
    }

    if VALID_SUBCARRIER_COUNT == 1 {
        smoothed[0] = phase[0];
        return smoothed;
    }

    smoothed[0] = (phase[0] + phase[1]) * 0.5;
    for i in 1..(VALID_SUBCARRIER_COUNT - 1) {
        smoothed[i] = (phase[i - 1] + phase[i] + phase[i + 1]) / 3.0;
    }
    smoothed[VALID_SUBCARRIER_COUNT - 1] =
        (phase[VALID_SUBCARRIER_COUNT - 2] + phase[VALID_SUBCARRIER_COUNT - 1]) * 0.5;

    smoothed
}

/// Unwraps a phase series by removing $2\pi$ discontinuities.
pub fn unwrap_phase(phase: &mut [f32]) {
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
