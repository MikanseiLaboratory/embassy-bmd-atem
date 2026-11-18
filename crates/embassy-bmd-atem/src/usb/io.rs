//! USB bulk host transport trait.
//!
//! Backends implement [`UsbBulkHost`] for one claimed bulk endpoint pair of the
//! switcher's control interface. Reads return one bulk IN transfer at a time (a
//! transfer ends at a short packet); a ZLP read yields `Ok(0)`.

#![allow(async_fn_in_trait)]

/// Async USB bulk host transport for the ATEM control endpoint pair.
pub trait UsbBulkHost: core::fmt::Debug {
    /// Backend error type (device unplugged, driver failure, claim loss, ...).
    type Error: core::fmt::Debug + core::fmt::Display;

    /// Read up to `buf.len()` bytes from the bulk IN endpoint; returns the number of
    /// bytes received (0 for a zero-length packet).
    async fn read_bulk(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error>;

    /// Write `data` to the bulk OUT endpoint. `data.len() == 0` sends a
    /// zero-length keepalive packet.
    async fn write_bulk(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Reset endpoint state before (re)connecting (e.g. clear HALT flags after
    /// an error or a device re-enumeration).
    async fn reset_state(&mut self) -> Result<(), Self::Error>;

    /// Wait for `ms` milliseconds. Used for send pacing (pyatem waits ~4 ms
    /// per USB write) and keepalive scheduling.
    async fn delay_ms(&mut self, ms: u32);
}
