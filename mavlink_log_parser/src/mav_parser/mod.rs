mod header;

use std::fs::File;

use mavlink::Message;
use mavlink::peek_reader::PeekReader;

use crate::{LogEntry, MavParser};
use header::FileHeader;

pub struct MavLogParser<M: Message> {
    reader: PeekReader<File>,
}

impl<M: Message> MavLogParser<M> {
    pub fn new(file_path: &str) -> Self {
        let file: File = File::open(file_path).expect("Failed to open file");
        let mut reader: PeekReader<File> = PeekReader::new(file);

        // handle file header
        let header_bytes: &[u8] = reader.read_exact(108).expect("Failed to read file header.");

        Self { reader }
    }
}

impl<M: Message> MavParser<M> for MavLogParser {
    fn next(&mut self) -> Option<LogEntry<M>> {
        match self.reader.next() {
            Ok((timestamp, msg)) => Some(LogEntry {
                timestamp: Some(timestamp),
                message: Some(msg),
                text: None,
                raw: None,
            }),
            Err(_) => None,
        }
    }
}
