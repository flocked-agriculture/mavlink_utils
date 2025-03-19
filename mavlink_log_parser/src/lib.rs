use std::option::Option;

use mavlink::error::MessageReadError;
use mavlink::{MavHeader, Message};

#[cfg(feature = "MavLog")]
pub mod mav_parser;

#[cfg(feature = "Tlog")]
pub mod tlog_parser;

pub struct LogEntry<M: Message> {
    pub timestamp: Option<u64>,
    pub mav_header: Option<MavHeader>,
    pub mav_message: Option<M>,
    pub text: Option<String>,
    pub raw: Option<Vec<u8>>,
}

impl<M: Message> Default for LogEntry<M> {
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

pub trait MavParser<M: Message> {
    fn next(&mut self) -> Result<LogEntry<M>, MessageReadError>;
}
