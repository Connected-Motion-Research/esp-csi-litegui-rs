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

/// Applies a centered moving average to amplitude values.
///
/// `window` controls the smoothing kernel width.
pub fn smooth_amplitude(
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

/// Applies a centered moving average to phase values.
///
/// `window` controls the smoothing kernel width.
pub fn smooth_phase(
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
