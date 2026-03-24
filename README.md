# esp-csi-litegui-rs

`esp-csi-litegui-rs` is a Rust `no-std` bare metal lightweight graphical user interface (GUI) application that runs on top of the [`esp-csi-rs`](https://github.com/Connected-Motion-Research/esp-csi-rs) crate.  `esp-csi-litegui-rs` provides an emebedded display graphical interface to visualize Wi-Fi Channel State Information (CSI) collected from ESP devices.

This application leverages the [`embedded-graphics`](https://github.com/embedded-graphics-rs/embedded-graphics) crate to draw the CSI data.

<p align="center">
  <img src="assets/espheatmap.gif" alt="CLI Snapshot" width="300"/>
</p>

## Currently Supported Displays
This application currently supports the following displays:
- [Waveshare ESP32-S3-Touch-AMOLED 1.8](https://www.waveshare.com/esp32-s3-touch-amoled-1.8.htm)
- [LilyGo T4-S3](https://lilygo.cc/products/t4-s3)

> 🚨 ***Note***: While more devices will be added with time, it should not take a lot of effort to adapt the the application in this repository to support other displays that are compatible with the `embedded-graphics` crate. Most of the work lies in modifying the display driver initialization code and dimension constants in `main.rs` to match your display. This is granted that a display driver exists and supports `embedded-graphics`. Alternatively, you would need to write a custom display driver (or extend an existing driver) that implements the `DrawTarget` trait from the `embedded-graphics` crate.

## Minimum Requirements
### 🛠️ Hardware
You would need one of the supported development boards listed above.

### 📀 Software
At a minimum, you would need the following:
* Rust toolchain with ESP target support installed. Full instructions for setting up a development environment are available [here](https://docs.esp-rs.org/book/installation/index.html). 
* Tool for flashing the firmware. It is recommended to use `esp-flash`. Installation instructions are available [here](https://docs.esp-rs.org/book/tooling/espflash.html).
* (Optional) A terminal program to view the output. It is also recommended to use `esp-flash` which was installed in the previous step.

> ‼️ Installing `espflash` requires a Rust installation. If you don't have Rust installed, follow the instructions on the [rustup](https://rustup.rs/) website.


## 📋 Usage
There are a few ways in which we can obtain CSI data. One is either through a connection between a Station and an Access point, or alternatively sniffing network packets. For better looking heatmaps, its recommended to establish a connection.

CSI mode is selected using Cargo features.

### Board feature (required)
Select exactly one board feature:
- `lilygo-t4`
- `waveshare-esp32-s3-touch-amoled-1_8`

### CSI mode features
- `mode-sta`
- `mode-now`
- `mode-snf`

Mode selection behavior:
- `mode-now` is the default mode.
- You can safely override default mode by adding `mode-sta` or `mode-snf`
  without disabling default features.
- `mode-sta` and `mode-snf` cannot be enabled together.

### Steps to Run:
1. ***Setup Project***: Clone this repository and `cd` into the repo root.
2. ***Configure Wi-Fi credentials*** (required for station mode): update SSID/password in `src/main.rs`.

```rust
WiFiConfig {
    ssid: "SSID".try_into().unwrap(),
    password: "PASSWORD".try_into().unwrap(),
    ..Default::default()
},
```

3. ***Build & Run*** with your board + optional CSI mode override:

- LilyGo + default ESP-NOW mode:
```bash
cargo run --release --features="lilygo-t4"
```

- Waveshare + ESP-NOW mode:
```bash
cargo run --release --features="waveshare-esp32-s3-touch-amoled-1_8,mode-now"
```

- Waveshare + Wi-Fi sniffer mode:
```bash
cargo run --release --features="waveshare-esp32-s3-touch-amoled-1_8,mode-snf"
```

## License
Copyright 2025 The Embedded Rustacean

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

---

Made with 🦀 for ESP chips
