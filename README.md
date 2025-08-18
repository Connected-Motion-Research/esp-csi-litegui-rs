# esp-csi-litegui-rs

`esp-csi-litegui-rs` is a Rust no-std bare metal lightweight graphical user interface (GUI) application that runs on top of the [`esp-csi-rs`](https://github.com/Connected-Motion-Research/esp-csi-rs) crate.  `esp-csi-litegui-rs` provides an emebedded display graphical interface to visualize Wi-Fi Channel State Information (CSI) collected from ESP devices.

This application takes advantage of the [`embedded-graphics`](https://github.com/embedded-graphics-rs/embedded-graphics) crate to draw the CSI data.

<p align="center">
  <img src="assets/espheatmap.gif" alt="CLI Snapshot" width="300"/>
</p>

## Supported Boards
This application currently supports the following displays:
- [Waveshare ESP32-S3-Touch-AMOLED 1.8](https://www.waveshare.com/esp32-s3-touch-amoled-1.8.htm)

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

To operate in sniffer mode, you'd need to change the configuration of the `CSICollector` abstraction in `main.rs`. For more detail, you are recommended to refer to the  [`esp-csi-rs`](https://github.com/Connected-Motion-Research/esp-csi-rs) crate documentation.

1. ***Setup Project***: Clone this repository.
2. ***Modify*** **`main.rs`**: Head to the `main.rs` file modify the `"SSID"` and `"PASSWORD"` strings in the following code to match your access point:

```rust
WiFiConfig {
    ssid: "SSID".try_into().unwrap(),
    password: "PASSWORD".try_into().unwrap(),
    ..Default::default()
},
```

3. ***Build & Run***: execute the following command in the terminal to build then run the project:
```bash
cargo run --release
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
