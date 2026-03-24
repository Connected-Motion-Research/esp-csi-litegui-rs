//! Reset driver adapters shared by touch and display controllers.

use cst226_rs::ResetInterface;
use core::convert::Infallible;
use esp_hal::delay::Delay;
use rm690b0_rs::ResetInterface as Rm6090b0ResetInterface;

/// GPIO-based reset implementation reusable across supported drivers.
pub struct ResetDriver<OUT> {
    output: OUT,
}

impl<OUT> ResetDriver<OUT> {
    /// Creates a new GPIO-backed reset driver.
    pub fn new(output: OUT) -> Self {
        ResetDriver { output }
    }
}

impl<OUT> ResetInterface for ResetDriver<OUT>
where
    OUT: embedded_hal::digital::OutputPin,
{
    type Error = OUT::Error;

    fn reset(&mut self) -> Result<(), Self::Error> {
        let delay = Delay::new();
        self.output.set_low()?;
        delay.delay_millis(20);
        self.output.set_high()?;
        delay.delay_millis(150);
        Ok(())
    }
}

impl<OUT> Rm6090b0ResetInterface for ResetDriver<OUT>
where
    OUT: embedded_hal::digital::OutputPin,
{
    type Error = OUT::Error;

    fn reset(&mut self) -> Result<(), Self::Error> {
        let delay = Delay::new();
        self.output.set_low()?;
        delay.delay_millis(20);
        self.output.set_high()?;
        delay.delay_millis(150);
        Ok(())
    }
}

/// No-op reset implementation for hardware where reset is managed externally.
pub struct NoopResetDriver;

impl ResetInterface for NoopResetDriver {
    type Error = Infallible;

    fn reset(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl Rm6090b0ResetInterface for NoopResetDriver {
    type Error = Infallible;

    fn reset(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
