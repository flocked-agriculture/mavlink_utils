use mavlink::error::MessageReadError;
use mavlink::{MavConnection, Message};

use crate::LogEntry;
use crate::MavParser;

pub struct TlogParser<M: Message> {
    file_conn: Box<dyn MavConnection<M> + Sync + Send>,
}

impl<M: Message> TlogParser<M> {
    pub fn new(file_path: &str) -> Self {
        let connection_string = format!("file:{}", file_path);
        let vehicle =
            mavlink::connect::<M>(&connection_string).expect("An invalid file path was provided");
        Self { file_conn: vehicle }
    }
}

impl<M: Message> MavParser for TlogParser<M> {
    type M = M;

    fn next(&mut self) -> Result<LogEntry<Self::M>, MessageReadError> {
        match self.file_conn.recv() {
            Ok((header, msg)) => Ok(LogEntry {
                timestamp: None,
                mav_header: Some(header),
                mav_message: Some(msg),
                text: None,
                raw: None,
            }),
            Err(err) => return Err(err),
        }
    }
}
