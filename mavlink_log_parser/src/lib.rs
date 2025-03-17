use mavlink::Message;
use std::option::Option;

#[cfg(feature = "MavLog")]
pub mod mav_parser;

#[cfg(feature = "Tlog")]
pub mod tlog_parser;

pub struct LogEntry<M: Message> {
    pub timestamp: Option<u64>,
    pub message: Option<M>,
    pub text: Option<String>,
    pub raw: Option<Vec<u8>>,
}

pub trait MavParser<M: Message> {
    fn next(&mut self) -> Option<LogEntry<M>>;
}
