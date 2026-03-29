//! Shared runtime state and inter-task channels.

use embassy_sync::channel::Channel;
use esp_hal::system::Stack;
use esp_radio::wifi::WifiController;
use esp_sync::RawMutex;
use static_cell::StaticCell;

use crate::config::VALID_SUBCARRIER_COUNT;

/// Global Wi-Fi controller storage initialized during startup.
pub static WIFI_CONTROLLER: StaticCell<WifiController<'static>> = StaticCell::new();

/// Relative display-mode change event produced by gesture tasks.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ModeIntent {
    /// Move to the next display mode in the mode cycle.
    Next,
    /// Move to the previous display mode in the mode cycle.
    Previous,
}

/// Ordered gesture intent channel consumed only by the display task.
pub static MODE_INTENTS: Channel<RawMutex, ModeIntent, 8> = Channel::new();

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
pub static CSI_FRAMES: Channel<RawMutex, CsiFrame, 32> = Channel::new();
