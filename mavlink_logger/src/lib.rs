#![crate_name = "mavlink_logger"]
#![doc = include_str!("../README.md")]
#![doc = include_str!("../docs/home.md")]
#![doc = include_str!("../docs/mav_log_file_format.md")]
#![doc = include_str!("../docs/tlog_file_format.md")]

#[cfg(feature = "MavLog")]
pub mod rotating_mav_logger;

#[cfg(feature = "Tlog")]
pub mod rotating_tlog;

use mavlink::{MavFrame, Message};

pub trait MavLogger {
    fn write_mavlink<M: Message>(&mut self, frame: MavFrame<M>) -> std::io::Result<()>;
}
