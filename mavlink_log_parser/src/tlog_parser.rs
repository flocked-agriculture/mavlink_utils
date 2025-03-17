use std::fs::File;
use std::io::BufReader;

use mavlink::MavConnection;
use mavlink::common::MavMessage; // TODO: remove dependency on dialect since that will change depending on file

use crate::MavParser;

pub struct TlogParser {
    file_conn: MavConnection,
}

impl TlogParser {
    pub fn new(file_path: &str) -> Self {
        let connection_string = format!("file:{file_path}");
        let vehicle = mavlink::connect::<MavMessage>(&connection_string);
        assert!(vehicle.is_ok(), "An invalid file path was provided");
        Self { vehicle }
    }
}

impl MavParser for TlogParser {
    fn next(&mut self) -> Option<LogEntry> {
        match self.file_conn.recv() {
            Ok((header, msg)) => Some(LogEntry {
                timestamp: None,
                message: Some(msg),
                text: None,
                raw: None,
            }),
            Err(MessageReadError::Io(e)) => None,
            _ => None,
        }
    }
}
