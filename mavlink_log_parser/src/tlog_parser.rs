use mavlink::{MavConnection, Message};

use crate::LogEntry;
use crate::MavParser;

pub struct TlogParser<M: Message> {
    file_conn: Box<dyn MavConnection<M> + Sync + Send>,
}

impl<M: Message> TlogParser<M> {
    pub fn new<M: Message>(file_path: &str) -> Self {
        let connection_string = format!("file:{}", file_path);
        let vehicle =
            mavlink::connect::<M>(&connection_string).expect("An invalid file path was provided");
        Self { file_conn: vehicle }
    }
}

impl<M: Message> MavParser<M> for TlogParser<M> {
    fn next(&mut self) -> Option<LogEntry<M>> {
        match self.file_conn.recv() {
            Ok((_header, msg)) => Some(LogEntry {
                timestamp: None,
                message: Some(msg),
                text: None,
                raw: None,
            }),
            Err(_) => None,
        }
    }
}
