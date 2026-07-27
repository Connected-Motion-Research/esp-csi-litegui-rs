//! CSI capture task and radio configuration helpers.

use crate::{
    config::{CsiOperationMode, VALID_SUBCARRIER_COUNT},
    csi_processing::{smooth_amplitude3, smooth_phase3, swap_upper_lower, unwrap_phase},
    state::{CSI_FRAMES, CsiFrame, STATS},
};
use core::sync::atomic::Ordering;
use embassy_executor::task;
use embassy_futures::yield_now;
use embassy_time::{Duration, Timer};
use esp_csi_rs::{CSINode, CSINodeClient, log_ln};
use esp_radio::{esp_now::WifiPhyRate, wifi::Protocol};
use micromath::F32Ext;

/// Length of the LLTF / HT-LTF I/Q payload in bytes (64 complex pairs of i8).
const RAW_CSI_PAYLOAD_LEN: usize = 128;

/// Applies mode-specific radio settings used for CSI collection.
pub fn configure_node_radio(node: &mut CSINode<'_>, mode: CsiOperationMode) {
    match mode {
        CsiOperationMode::WifiStation => {
            // Station mode CSI capture - lock to highest 11n rate for maximum packet density.
            node.set_protocol(Protocol::N);
        }
        CsiOperationMode::EspNow => {
            // For interoperability, avoid forcing a fixed PHY rate in ESP-NOW mode.
            // Different peers (e.g. ESP32-C6 examples) may transmit at non-MCS rates,
            // and forcing MCS0 can reduce/lose CSI capture.
            node.set_protocol(Protocol::N);
            node.set_rate(WifiPhyRate::RateMcs0Lgi);
        }
        CsiOperationMode::WifiSniffer => {
            // Sniffer captures packets on the CURRENT tuned Wi-Fi channel.
            // Current esp-csi-rs/esp-radio path does not expose automatic channel hopping.
            // LR broadens frame/rate compatibility on that channel.
            node.set_protocol(Protocol::LR);
        }
        CsiOperationMode::WifiAccessPoint => {
            // 11n uplink from the associated station maximizes HT-LTF CSI
            // density from the ICMP echo replies.
            node.set_protocol(Protocol::N);
        }
        CsiOperationMode::EspNowFastCollector => {
            // Protocol::N only — EspNowConfig::fast_default() already forces
            // the per-peer PHY to MCS7; set_rate here would clobber it.
            node.set_protocol(Protocol::N);
        }
    }
}

/// Background task that pulls CSI packets from the node's lock-free queue
/// via [`CSINodeClient::get_csi_data`] and publishes normalized
/// magnitude/phase frames into [`CSI_FRAMES`](crate::state::CSI_FRAMES).
///
/// The first `get_csi_data().await` lazily switches the lib's delivery mode
/// to `Async` and opens the publish gate; from there the WiFi callback
/// enqueues directly into `CSI_QUEUE` and we get woken when a packet is
/// available. No user-side ring or `set_csi_callback` registration is
/// needed — the lib owns the SPSC path internally.
#[task]
pub async fn csi_task(mut client: CSINodeClient) {
    let mut cached_magnitude = [0.0; VALID_SUBCARRIER_COUNT];
    let mut cached_phase = [0.5; VALID_SUBCARRIER_COUNT];

    loop {
        let packet = client.get_csi_data().await;
        STATS.csi_recv.fetch_add(1, Ordering::Relaxed);

        let raw = packet.csi_data.as_slice();
        let mut payload_buf = [0i8; RAW_CSI_PAYLOAD_LEN];
        if raw.len() >= 4 + RAW_CSI_PAYLOAD_LEN {
            payload_buf.copy_from_slice(&raw[4..(4 + RAW_CSI_PAYLOAD_LEN)]);
        } else if raw.len() >= RAW_CSI_PAYLOAD_LEN {
            payload_buf.copy_from_slice(&raw[..RAW_CSI_PAYLOAD_LEN]);
        } else {
            STATS.csi_short.fetch_add(1, Ordering::Relaxed);
            yield_now().await;
            continue;
        }

        let csi_payload = &mut payload_buf[..];

        // Order the subcarrier data correctly
        swap_upper_lower(csi_payload);

        let mut amplitude: [f32; 64] = [0.0; 64];
        let mut phase: [f32; 64] = [0.0; 64];
        let sample_pairs = (csi_payload.len() / 2).min(64);

        for i in 0..sample_pairs {
            let real = csi_payload[2 * i] as f32;
            let imag = csi_payload[2 * i + 1] as f32;

            if real == 0.0 && imag == 0.0 {
                amplitude[i] = if i > 0 { amplitude[i - 1] } else { 0.0 };
                phase[i] = if i > 0 { phase[i - 1] } else { 0.0 };
            } else {
                // Skip sqrt to reduce per-packet FP overhead.
                amplitude[i] = real * real + imag * imag;
                phase[i] = imag.atan2(real);
            }
        }

        unwrap_phase(&mut phase);

        // Select valid subcarriers, skipping DC (index 32)
        let mut valid_amplitude: [f32; VALID_SUBCARRIER_COUNT] = [0.0; VALID_SUBCARRIER_COUNT];
        valid_amplitude[0..26].copy_from_slice(&amplitude[6..32]);
        valid_amplitude[26..52].copy_from_slice(&amplitude[33..59]);

        let mut valid_phase: [f32; VALID_SUBCARRIER_COUNT] = [0.0; VALID_SUBCARRIER_COUNT];
        valid_phase[0..26].copy_from_slice(&phase[6..32]);
        valid_phase[26..52].copy_from_slice(&phase[33..59]);

        let smoothed_amplitude = smooth_amplitude3(&valid_amplitude);
        let mut min_amplitude = f32::INFINITY;
        let mut max_amplitude = f32::NEG_INFINITY;
        for value in smoothed_amplitude {
            min_amplitude = min_amplitude.min(value);
            max_amplitude = max_amplitude.max(value);
        }
        let amplitude_range = if max_amplitude - min_amplitude > 0.0 {
            max_amplitude - min_amplitude
        } else {
            1.0
        };

        for i in 0..VALID_SUBCARRIER_COUNT {
            cached_magnitude[i] = (smoothed_amplitude[i] - min_amplitude) / amplitude_range;
            cached_magnitude[i] = cached_magnitude[i].clamp(0.0, 1.0);
        }

        let smoothed_phase = smooth_phase3(&valid_phase);
        let mut min_phase = f32::INFINITY;
        let mut max_phase = f32::NEG_INFINITY;
        for value in smoothed_phase {
            min_phase = min_phase.min(value);
            max_phase = max_phase.max(value);
        }
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

        let frame = CsiFrame {
            magnitude: cached_magnitude,
            phase: cached_phase,
        };

        if CSI_FRAMES.try_send(frame).is_ok() {
            STATS.frames_pushed.fetch_add(1, Ordering::Relaxed);
        } else {
            STATS.frames_evicted.fetch_add(1, Ordering::Relaxed);
            let _ = CSI_FRAMES.try_receive();
            let _ = CSI_FRAMES.try_send(frame);
        }

        yield_now().await;
    }
}

/// Periodic instrumentation dump. Prints per-second deltas for each pipeline
/// stage so a starvation/overrun can be located by inspecting the log:
///
///   - `csi=`  CSI packets pulled from the lib's queue / short payloads dropped
///   - `frames=`  publish rate into `CSI_FRAMES` / evictions due to a backed-up consumer
///   - `render=`  display_task render rate / flush rate / idle (no frame in 20ms)
#[task]
pub async fn stats_task() {
    let mut prev_csi_recv: usize = 0;
    let mut prev_csi_short: usize = 0;
    let mut prev_frames_pushed: usize = 0;
    let mut prev_frames_evicted: usize = 0;
    let mut prev_render_iters: usize = 0;
    let mut prev_render_cols: usize = 0;
    let mut prev_render_flushes: usize = 0;
    let mut prev_render_idle: usize = 0;

    loop {
        Timer::after(Duration::from_secs(1)).await;

        let csi_recv = STATS.csi_recv.load(Ordering::Relaxed);
        let csi_short = STATS.csi_short.load(Ordering::Relaxed);
        let frames_pushed = STATS.frames_pushed.load(Ordering::Relaxed);
        let frames_evicted = STATS.frames_evicted.load(Ordering::Relaxed);
        let render_iters = STATS.render_iters.load(Ordering::Relaxed);
        let render_cols = STATS.render_cols.load(Ordering::Relaxed);
        let render_flushes = STATS.render_flushes.load(Ordering::Relaxed);
        let render_idle = STATS.render_idle.load(Ordering::Relaxed);

        log_ln!(
            "[stats/1s] csi=recv{} short{} | frames=pushed{} evicted{} | render=iters{} cols{} flushes{} idle{}",
            csi_recv.wrapping_sub(prev_csi_recv),
            csi_short.wrapping_sub(prev_csi_short),
            frames_pushed.wrapping_sub(prev_frames_pushed),
            frames_evicted.wrapping_sub(prev_frames_evicted),
            render_iters.wrapping_sub(prev_render_iters),
            render_cols.wrapping_sub(prev_render_cols),
            render_flushes.wrapping_sub(prev_render_flushes),
            render_idle.wrapping_sub(prev_render_idle),
        );

        prev_csi_recv = csi_recv;
        prev_csi_short = csi_short;
        prev_frames_pushed = frames_pushed;
        prev_frames_evicted = frames_evicted;
        prev_render_iters = render_iters;
        prev_render_cols = render_cols;
        prev_render_flushes = render_flushes;
        prev_render_idle = render_idle;
    }
}
