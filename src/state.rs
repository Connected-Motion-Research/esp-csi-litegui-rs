//! Shared runtime state and inter-task channels.

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, watch::Watch,
};
use esp_hal::system::Stack;
use esp_radio::wifi::WifiController;
use static_cell::StaticCell;

use crate::config::{DisplayMode, VALID_SUBCARRIER_COUNT};

/// Global Wi-Fi controller storage initialized during startup.
pub static WIFI_CONTROLLER: StaticCell<WifiController<'static>> = StaticCell::new();

// The Watch construct immediately overwrites the previous value when a new one is sent, without waiting for all receivers to read the previous value.
// The Watch sender is supposed to update only when a gesture is detected.
/// Display mode watch channel used by gesture and display tasks.
pub static DISPLAY_MODE: Watch<CriticalSectionRawMutex, DisplayMode, 2> = Watch::new();

/// Dedicated stack used for the second CPU core executor.
///
/// This core runs both display and gesture tasks, so it needs additional headroom.
pub static mut APP_CORE_STACK: Stack<32768> = Stack::new();

/// Processed CSI data frame sent to the display task.
#[derive(Clone, Copy)]
pub struct CsiFrame {
    /// Normalized magnitude values in range $[0, 1]$.
    pub magnitude: [f32; VALID_SUBCARRIER_COUNT],
    /// Normalized phase values in range $[0, 1]$.
    pub phase: [f32; VALID_SUBCARRIER_COUNT],
}

/// Channel carrying processed CSI frames from capture to renderer.
pub static CSI_FRAMES: Channel<CriticalSectionRawMutex, CsiFrame, 32> = Channel::new();
