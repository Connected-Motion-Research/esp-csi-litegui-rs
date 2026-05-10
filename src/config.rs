//! Compile-time configuration for board profiles, CSI operating mode, and display geometry.

use rm690b0_rs::{ColorMode, DisplaySize, framebuffer_size};

#[cfg(all(feature = "lilygo-t4", feature = "waveshare-esp32-s3-touch-amoled-1_8"))]
compile_error!("Enable only one board feature: 'lilygo-t4' or 'waveshare-esp32-s3-touch-amoled-1_8'");

#[cfg(not(any(feature = "lilygo-t4", feature = "waveshare-esp32-s3-touch-amoled-1_8")))]
compile_error!("Please enable a board feature: 'lilygo-t4' or 'waveshare-esp32-s3-touch-amoled-1_8'");

/// Display mode shown on the heatmap.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DisplayMode {
    /// Render normalized CSI magnitude.
    Magnitude,
    /// Render normalized CSI phase.
    Phase,
}

impl DisplayMode {
    /// Returns the next mode in the cycle.
    pub fn next(&self) -> Self {
        match self {
            DisplayMode::Magnitude => DisplayMode::Phase,
            DisplayMode::Phase => DisplayMode::Magnitude,
        }
    }

    /// Returns the previous mode in the cycle.
    pub fn previous(&self) -> Self {
        match self {
            DisplayMode::Magnitude => DisplayMode::Phase,
            DisplayMode::Phase => DisplayMode::Magnitude,
        }
    }
}

/// Runtime CSI acquisition topology.
#[derive(Clone, Copy, Debug)]
#[allow(dead_code)]
pub enum CsiOperationMode {
    /// Station mode CSI, typically using an AP link.
    WifiStation,
    /// ESP-NOW based CSI collection.
    EspNow,
    /// Passive Wi-Fi sniffer CSI collection.
    WifiSniffer,
}

#[cfg(all(feature = "mode-sta", feature = "mode-snf"))]
compile_error!(
    "Enable only one override mode: 'mode-sta' or 'mode-snf'"
);

/// Selected CSI operating mode derived from Cargo features.
#[cfg(feature = "mode-snf")]
pub const CSI_OPERATION_MODE: CsiOperationMode = CsiOperationMode::WifiSniffer;

/// Selected CSI operating mode derived from Cargo features.
#[cfg(all(not(feature = "mode-snf"), feature = "mode-sta"))]
pub const CSI_OPERATION_MODE: CsiOperationMode = CsiOperationMode::WifiStation;

/// Selected CSI operating mode derived from Cargo features.
#[cfg(all(not(feature = "mode-snf"), not(feature = "mode-sta")))]
pub const CSI_OPERATION_MODE: CsiOperationMode = CsiOperationMode::EspNow;

/// 2.4 GHz channel (1–14) used for CSI capture in `EspNow` and `WifiSniffer`
/// modes. Both this device and the transmitting peer MUST be on the same
/// channel — a mismatch is silent and looks like "no packets are arriving".
pub const CSI_CHANNEL: u8 = 1;

// According to documentation the acquired array from ESP is an LLTF array with subcarriers ordered as follows: [0 to 31,-32 to -1]
// There are several formats depending on supported training algortihim
// Core library would have to integrate some indication for the type of CSI recieved if to be accomodated here

// Pilot Subcarriers -> (+7/-7,+21/-21)
// Null Subcarriers -> (0, +27~31, -27~-32)
// This is assuming LLTF
// Refer to https://github.com/StevenMHernandez/ESP32-CSI-Tool/issues/12

/// Number of valid LLTF subcarriers after removing null/pilot carriers.
pub const VALID_SUBCARRIER_COUNT: usize = 52;

// Set up the display size
// LilyGo T4-S3 display resolution is 450x600.
/// Display dimensions for LilyGo T4-S3.
#[cfg(feature = "lilygo-t4")]
pub const DISPLAY_SIZE: DisplaySize = DisplaySize::new(450, 600);

// Waveshare ESP32-S3-Touch-AMOLED-1.8 panel resolution is 368x448.
/// Display dimensions for Waveshare ESP32-S3-Touch-AMOLED-1.8.
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
pub const DISPLAY_SIZE: DisplaySize = DisplaySize::new(368, 448);

/// Human-readable board profile name.
#[cfg(feature = "lilygo-t4")]
pub const BOARD_NAME: &str = "lilygo-t4";

/// Human-readable board profile name.
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
pub const BOARD_NAME: &str = "waveshare-esp32-s3-touch-amoled-1_8";

// Heatmap placement configuration
// This allows for the heatmap to be placed on different parts of the display.

/// Heatmap left/top origin for LilyGo profile.
#[cfg(feature = "lilygo-t4")]
pub const HEATMAP_START_X: u32 = 40;
/// Heatmap left/top origin for LilyGo profile.
#[cfg(feature = "lilygo-t4")]
pub const HEATMAP_START_Y: u32 = 80;
/// Requested heatmap width for LilyGo profile.
#[cfg(feature = "lilygo-t4")]
pub const HEATMAP_WIDTH: u32 = 450;
/// Requested heatmap height for LilyGo profile.
#[cfg(feature = "lilygo-t4")]
pub const HEATMAP_HEIGHT: u32 = 450;

/// Heatmap left/top origin for Waveshare profile.
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
pub const HEATMAP_START_X: u32 = 24;
/// Heatmap left/top origin for Waveshare profile.
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
pub const HEATMAP_START_Y: u32 = 64;
/// Requested heatmap width for Waveshare profile.
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
pub const HEATMAP_WIDTH: u32 = 320;
/// Requested heatmap height for Waveshare profile.
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
pub const HEATMAP_HEIGHT: u32 = 320;

/// Maximum drawable heatmap width after clipping to display bounds.
pub const HEATMAP_MAX_WIDTH: u32 = DISPLAY_SIZE.width as u32 - HEATMAP_START_X;
/// Maximum drawable heatmap height after clipping to display bounds.
pub const HEATMAP_MAX_HEIGHT: u32 = DISPLAY_SIZE.height as u32 - HEATMAP_START_Y;
/// Effective heatmap width after clipping.
pub const HEATMAP_EFFECTIVE_WIDTH: u32 = if HEATMAP_WIDTH < HEATMAP_MAX_WIDTH {
    HEATMAP_WIDTH
} else {
    HEATMAP_MAX_WIDTH
};
/// Effective heatmap height after clipping.
pub const HEATMAP_EFFECTIVE_HEIGHT: u32 = if HEATMAP_HEIGHT < HEATMAP_MAX_HEIGHT {
    HEATMAP_HEIGHT
} else {
    HEATMAP_MAX_HEIGHT
};

/// RGB888 framebuffer size in bytes for the selected display profile.
pub const FB_SIZE: usize = framebuffer_size(DISPLAY_SIZE, ColorMode::Rgb888);

/// Board-specific display reset driver type for LilyGo profile.
#[cfg(feature = "lilygo-t4")]
pub type DisplayResetDriver = crate::reset::ResetDriver<esp_hal::gpio::Output<'static>>;

/// Board-specific display reset driver type for Waveshare profile.
#[cfg(feature = "waveshare-esp32-s3-touch-amoled-1_8")]
pub type DisplayResetDriver = crate::reset::NoopResetDriver;
