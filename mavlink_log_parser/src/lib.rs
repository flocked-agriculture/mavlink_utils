use mavlink::common::MavMessage; // TODO: remove dependency on dialect since that will change depending on file
use std::option::Option;

#[cfg(feature = "MavLog")]
pub mod mav_parser;

#[cfg(feature = "Tlog")]
pub mod tlog_parser;

pub struct LogEntry {
    pub timestamp: Option<u64>,
    pub message: Option<MavMessage>,
    pub text: Option<String>,
    pub raw: Option<Vec<u8>>,
}

pub trait MavParser {
    fn next(&mut self) -> Option<LogEntry>;
}
