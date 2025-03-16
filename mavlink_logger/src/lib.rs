//! # Mavlink Logger
//! The `mavlink_logger` crate provides a simple interface for logging mavlink messages to a file.
#![crate_name = "mavlink_logger"]
#![doc = include_str!("../docs/file_management.md")]
#![doc = include_str!("../docs/file_format.md")]

pub mod rotating_mav_logger;
pub mod rotating_tlog;

use mavlink::{MavFrame, Message};

pub trait MavLogger {
    fn write_mavlink<M: Message>(&mut self, frame: MavFrame<M>) -> std::io::Result<()>;
}
