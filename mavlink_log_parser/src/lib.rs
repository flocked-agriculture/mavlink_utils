#![crate_name = "mavlink_log_parser"]
#![doc = include_str!("../README.md")]
#![doc = include_str!("../docs/home.md")]
#![doc = include_str!("../docs/mav_log_file_format.md")]
#![doc = include_str!("../docs/tlog_file_format.md")]

use std::option::Option;

use mavlink::error::MessageReadError;
use mavlink::{MavHeader, Message};

#[cfg(feature = "MavLog")]
/// Module for parsing MAVLink log files.
pub mod mav_parser;

#[cfg(feature = "Tlog")]
/// Module for parsing telemetry log (TLog) files.
pub mod tlog_parser;

/// Represents a single log entry in a MAVLink log or telemetry log.
///
/// # Type Parameters
/// - `M`: A type that implements the `Message` trait, representing a MAVLink message.
///
/// # Fields
/// - `timestamp`: The timestamp of the log entry, if available.
/// - `mav_header`: The MAVLink header associated with the message, if available.
/// - `mav_message`: The MAVLink message, if available.
/// - `text`: Any textual information associated with the log entry, if available.
/// - `raw`: The raw binary data of the log entry, if available.
pub struct LogEntry<M: Message> {
    pub timestamp: Option<u64>,
    pub mav_header: Option<MavHeader>,
    pub mav_message: Option<M>,
    pub text: Option<String>,
    pub raw: Option<Vec<u8>>,
}

impl<M: Message> Default for LogEntry<M> {
    /// Provides a default implementation for `LogEntry`.
    ///
    /// All fields are initialized to `None`.
    fn default() -> Self {
        Self {
            timestamp: None,
            mav_header: None,
            mav_message: None,
            text: None,
            raw: None,
        }
    }
}

/// A trait for parsing MAVLink logs or telemetry logs.
///
/// # Associated Types
/// - `M`: A type that implements the `Message` trait, representing a MAVLink message.
///
/// # Required Methods
/// - `next`: Reads the next log entry from the log source.
///
/// # Errors
/// Returns a `MessageReadError` if there is an issue reading the next log entry. This
/// applies to EOF as well.
pub trait MavParser {
    type M: Message;

    /// Reads the next log entry from the log source.
    ///
    /// # Returns
    /// - `Ok(LogEntry<Self::M>)`: The next log entry if successfully read.
    /// - `Err(MessageReadError)`: An error if the log entry could not be read.
    fn next(&mut self) -> Result<LogEntry<Self::M>, MessageReadError>;
}
