# esp-csi-litegui-rs

esp-csi-litegui-rs is a no_std firmware application for ESP32-S3 boards that renders Wi-Fi CSI (Channel State Information) as a live AMOLED heatmap.

It is built on:
- esp-csi-rs for CSI collection
- embedded-graphics for drawing
- rm690b0-rs for panel driving

<p align="center">
  <img src="assets/espheatmap.gif" alt="CSI Heatmap" width="300"/>
</p>

## Supported Boards
- [Waveshare ESP32-S3-Touch-AMOLED-1.8](https://www.waveshare.com/esp32-s3-touch-amoled-1.8.htm)
- [LilyGo T4-S3](https://lilygo.cc/products/t4-s3)

## Runtime Architecture
- CSI processing task runs on the main executor.
- Display rendering runs on a dedicated second core executor.
- Touch handling is board-specific:
- LilyGo uses CST226 gesture reads.
- Waveshare uses FT3x68 gesture IDs with coordinate fallback swipe inference.

This split keeps display updates responsive under high CSI packet rates.

## Requirements
### Hardware
- One supported board.

### Software
- Rust toolchain with ESP target support.
- espflash for flashing and monitoring.

Setup guides:
- https://docs.esp-rs.org/book/installation/index.html
- https://docs.esp-rs.org/book/tooling/espflash.html

## Cargo Features

### Board Feature (required)
Enable exactly one:
- lilygo-t4
- waveshare-esp32-s3-touch-amoled-1_8

### CSI Mode Features

This board is always a **collector** — it renders a CSI heatmap, so it needs CSI to render. The modes differ only in where the measurable frames come from.

- mode-snf (default) — promiscuous capture on a locked channel. Needs no peer configuration.
- mode-sta — associate to an AP and capture CSI from that link.
- mode-ap — softAP collector: the board runs an AP (SSID `esp-csi-ap`) with a built-in DHCP server and pings the associated station; CSI is captured from its uplink replies.

Rules:
- mode-snf is enabled by default.
- Only one of mode-sta / mode-ap can be enabled.
- You can enable an override mode without disabling defaults.

Pairing notes:
- mode-snf: pair with an **emitter** board running the esp-csi-rs `ht20_emitter` or `ht40_emitter` example on the same channel (this app uses channel 1; the examples default to 7 — change one side to match). An emitter sounds the channel with raw injected frames and needs no association, so nothing has to be configured on this side beyond the channel. Ambient traffic on the channel is also captured.
- mode-ap: any Wi-Fi station joining `esp-csi-ap` works (e.g. the esp-csi-rs `wifi_station` example); clients get a 192.168.13.x lease from the built-in DHCP server, and CSI flows once a station associates.

### Logging/Debug Features (optional)
- println
- async-print — force non-blocking async logging (no longer implied by jtag-serial as of esp-csi-rs 0.8; the default `auto` feature already picks the async drain at runtime when USB-Serial-JTAG is active)
- defmt — esp-csi-rs 0.8 registers its own rzcobs-encoded defmt global logger
- external-defmt-logger — use your own defmt global logger instead
- jtag-serial — force the JTAG transport
- uart — force the UART transport (do not combine with async-print)

## Build and Run

### 1. Clone and enter project
```bash
git clone <repo-url>
cd esp-csi-litegui-rs
```

### 2. Configure Wi-Fi credentials (station mode only)
Update SSID and password in src/main.rs where ClientConfig is created.

### 3. Build/run examples

LilyGo + default sniffer mode:
```bash
cargo run --release --features="lilygo-t4"
```

LilyGo + station mode:
```bash
cargo run --release --features="lilygo-t4,mode-sta"
```

LilyGo + explicit sniffer mode:
```bash
cargo run --release --features="lilygo-t4,mode-snf"
```

Waveshare + default sniffer mode:
```bash
cargo run --release --features="waveshare-esp32-s3-touch-amoled-1_8"
```

Waveshare + station mode:
```bash
cargo run --release --features="waveshare-esp32-s3-touch-amoled-1_8,mode-sta"
```

Waveshare + explicit sniffer mode:
```bash
cargo run --release --features="waveshare-esp32-s3-touch-amoled-1_8,mode-snf"
```

LilyGo + softAP collector mode:
```bash
cargo run --release --features="lilygo-t4,mode-ap"
```

Waveshare + softAP collector mode:
```bash
cargo run --release --features="waveshare-esp32-s3-touch-amoled-1_8,mode-ap"
```

## Gesture Behavior
- LilyGo: CST226 hardware gestures (left/right swipe)
- Waveshare: FT3x68 hardware gestures first, coordinate-based left/right fallback when needed

Both boards publish mode-intent events consumed by the display task.

## Troubleshooting
- Blank display: verify correct board feature and flash target.
- No CSI movement: verify selected mode and radio environment.
- No gesture response:
- LilyGo: verify CST226 init and touch IRQ pin.
- Waveshare: verify FT3x68 init and XCA9554 setup sequence.

## License
Copyright 2026 The esp-csi Team

Licensed under the Apache License, Version 2.0.
http://www.apache.org/licenses/LICENSE-2.0

---

Made with 🦀 for ESP chips
