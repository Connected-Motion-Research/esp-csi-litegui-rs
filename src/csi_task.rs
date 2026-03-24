//! CSI capture task and radio configuration helpers.

use crate::{
    config::{CsiOperationMode, DisplayMode, VALID_SUBCARRIER_COUNT},
    csi_processing::{smooth_amplitude, smooth_phase, swap_upper_lower, unwrap_phase},
    state::{CSI_FRAMES, CsiFrame, DISPLAY_MODE},
};
use embassy_executor::task;
use embassy_futures::yield_now;
use esp_csi_rs::{CSINode, CSINodeClient};
use esp_radio::{esp_now::WifiPhyRate, wifi::Protocol};
use micromath::F32Ext;

/// Applies mode-specific radio settings used for CSI collection.
pub fn configure_node_radio(node: &mut CSINode<'_>, mode: CsiOperationMode) {
    match mode {
        CsiOperationMode::WifiStation => {
            // Station mode CSI capture - lock to highest 11n rate for maximum packet density.
            node.set_protocol(Protocol::P802D11BGN);
            node.set_rate(WifiPhyRate::RateMcs0Lgi);
        }
        CsiOperationMode::EspNow => {
            // For interoperability, avoid forcing a fixed PHY rate in ESP-NOW mode.
            // Different peers (e.g. ESP32-C6 examples) may transmit at non-MCS rates,
            // and forcing MCS0 can reduce/lose CSI capture.
            node.set_protocol(Protocol::P802D11BGNLR);
            node.set_rate(WifiPhyRate::RateMcs0Lgi);
        }
        CsiOperationMode::WifiSniffer => {
            // Sniffer captures packets on the CURRENT tuned Wi-Fi channel.
            // Current esp-csi-rs/esp-radio path does not expose automatic channel hopping.
            // Protocol mask below broadens frame/rate compatibility on that channel.
            node.set_protocol(Protocol::P802D11BGNLR);
            // Use MCS0 to maximize capture of low-rate frames which are common in beacons/management.
            node.set_rate(WifiPhyRate::RateMcs0Lgi);
        }
    }
}

/// Background task that receives raw CSI packets and publishes normalized frames.
///
/// The task computes both magnitude and phase representations and stores the
/// most recent values into [`CSI_FRAMES`](crate::state::CSI_FRAMES).
#[task]
pub async fn csi_task(mut node_handle: CSINodeClient) {
    let mut display_mode = DisplayMode::Magnitude;
    let mut display_watch = DISPLAY_MODE.receiver().unwrap();
    let mut packets_since_yield: u8 = 0;

    let mut cached_magnitude = [0.0; VALID_SUBCARRIER_COUNT];
    let mut cached_phase = [0.5; VALID_SUBCARRIER_COUNT];

    loop {
        if let Some(new_mode) = display_watch.try_changed() {
            display_mode = new_mode;
        }

        // Grab the next data whether lagged or not
        let mut packet = node_handle.get_csi_data().await;

        // Some CSI packets can arrive shorter than the expected 64 complex subcarriers (128 bytes).
        // Handle variable-length packets safely.
        let raw = &mut packet.csi_data;
        let csi_payload = if raw.len() >= 4 + 128 {
            &mut raw[4..(4 + 128)]
        } else if raw.len() >= 128 {
            &mut raw[..128]
        } else {
            continue;
        };

        // Order the subcarrier data correctly
        swap_upper_lower(csi_payload);

        match display_mode {
            DisplayMode::Magnitude => {
                let mut amplitude: [f32; 64] = [0.0; 64];
                let sample_pairs = (csi_payload.len() / 2).min(64);

                for i in 0..sample_pairs {
                    let real = csi_payload[2 * i] as f32;
                    let imag = csi_payload[2 * i + 1] as f32;

                    if real == 0.0 && imag == 0.0 {
                        amplitude[i] = if i > 0 { amplitude[i - 1] } else { 0.0 };
                    } else {
                        amplitude[i] = (real * real + imag * imag).sqrt();
                    }
                }

                // Select valid subcarriers, skipping DC (index 32)
                let mut valid_amplitude: [f32; VALID_SUBCARRIER_COUNT] = [0.0; VALID_SUBCARRIER_COUNT];
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

                for i in 0..VALID_SUBCARRIER_COUNT {
                    cached_magnitude[i] = (smoothed_amplitude[i] - min_amplitude) / amplitude_range;
                    cached_magnitude[i] = cached_magnitude[i].clamp(0.0, 1.0);
                }
            }
            DisplayMode::Phase => {
                let mut phase: [f32; 64] = [0.0; 64];
                let sample_pairs = (csi_payload.len() / 2).min(64);

                for i in 0..sample_pairs {
                    let real = csi_payload[2 * i] as f32;
                    let imag = csi_payload[2 * i + 1] as f32;

                    if real == 0.0 && imag == 0.0 {
                        phase[i] = if i > 0 { phase[i - 1] } else { 0.0 };
                    } else {
                        phase[i] = imag.atan2(real);
                    }
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

                if phase_range > 0.0 {
                    for i in 0..VALID_SUBCARRIER_COUNT {
                        cached_phase[i] = (smoothed_phase[i] - min_phase) / phase_range;
                    }
                } else {
                    for i in 0..VALID_SUBCARRIER_COUNT {
                        cached_phase[i] = 0.5;
                    }
                }
            }
        }

        let frame = CsiFrame {
            magnitude: cached_magnitude,
            phase: cached_phase,
        };

        if CSI_FRAMES.try_send(frame).is_err() {
            let _ = CSI_FRAMES.try_receive();
            let _ = CSI_FRAMES.try_send(frame);
        }

        // Cooperative scheduling: avoid starving other async tasks (e.g. radio runtime)
        // when CSI arrives at high packet rates for extended periods.
        packets_since_yield = packets_since_yield.wrapping_add(1);
        if packets_since_yield >= 8 {
            packets_since_yield = 0;
            yield_now().await;
        }
    }
}
