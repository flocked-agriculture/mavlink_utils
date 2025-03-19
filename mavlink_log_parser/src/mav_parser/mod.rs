mod header;

use std::convert::TryFrom;
use std::convert::TryInto;
use std::fs::File;
use std::u32;

use mavlink::bytes;
use mavlink::error::{MessageReadError, ParserError};
use mavlink::peek_reader::PeekReader;
use mavlink::{MavHeader, MavlinkVersion, Message, read_versioned_msg};

use crate::{LogEntry, MavParser};
use header::{FileHeader, MavlinkDefinitionPayloadType};

enum EntryType {
    Raw = 0,
    Mavlink = 1,
    Utf8Text = 2,
}

impl TryFrom<u8> for EntryType {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(EntryType::Raw),
            1 => Ok(EntryType::Mavlink),
            2 => Ok(EntryType::Utf8Text),
            _ => Err(()),
        }
    }
}

pub trait Parser {
    fn next<M: Message>(&mut self) -> Result<LogEntry<M>, MessageReadError>;
}

struct MavlinkOnlyNoTimestampParser {
    reader: PeekReader<File>,
    mav_version: MavlinkVersion,
}

impl Parser for MavlinkOnlyNoTimestampParser {
    fn next<M: Message>(&mut self) -> Result<LogEntry<M>, MessageReadError> {
        let mut entry: LogEntry<M> = LogEntry::default();
        // TODO: think about handling error and trying next message
        let (header, message) = read_versioned_msg::<M, File>(&mut self.reader, self.mav_version)?;
        entry.mav_header = Some(header);
        entry.mav_message = Some(message);
        Ok(entry)
    }
}

struct TimestampedMavlinkOnlyParser {
    reader: PeekReader<File>,
    mav_version: MavlinkVersion,
}

impl Parser for TimestampedMavlinkOnlyParser {
    fn next<M: Message>(&mut self) -> Result<LogEntry<M>, MessageReadError> {
        let mut entry: LogEntry<M> = LogEntry::default();

        let magic_number: u8 = match self.mav_version {
            MavlinkVersion::V1 => mavlink::MAV_STX,
            MavlinkVersion::V2 => mavlink::MAV_STX_V2,
        };
        if self.reader.peek_exact(5)?[4] == magic_number {
            // only get timestamp if it precedes a mavlink message magic number
            // otherwise our data is misaligned
            let timestamp_raw: &[u8] = self.reader.read_exact(8)?;
            entry.timestamp = match timestamp_raw.try_into() {
                Ok(bytes) => Some(u64::from_le_bytes(bytes)),
                Err(_) => None,
            };
        } else {
            // if misaligned, ignore timestamp and let read mavlink logic jump to next magic number
        }
        let (header, message) = read_versioned_msg::<M, File>(&mut self.reader, self.mav_version)?;
        entry.mav_header = Some(header);
        entry.mav_message = Some(message);
        Ok(entry)
    }
}

pub struct MixedParser {
    timestamped: bool,
    reader: PeekReader<File>,
}

impl Parser for MixedParser {
    fn next<M: Message>(&mut self) -> Result<LogEntry<M>, MessageReadError> {
        let mut entry: LogEntry<M> = LogEntry::default();
        let entry_type: EntryType = self
            .reader
            .read_u8()
            .map(|value| value.try_into().unwrap_or(EntryType::Raw))?; // Default to Raw on error

        // handle timestamp
        if self.timestamped {
            let timestamp_raw: &[u8] = self.reader.read_exact(8)?;
            entry.timestamp = match timestamp_raw.try_into() {
                Ok(bytes) => Some(u64::from_le_bytes(bytes)),
                Err(_) => None,
            };
        }

        // handle payload
        let payload_size: u16 = u16::from_le_bytes(
            self.reader
                .read_exact(2)?
                .try_into()
                // This should never happen and is a fatal error if it does.
                .expect("Failed to read log entry payload size. Unable to continue."),
        );
        let payload = self.reader.read_exact(payload_size as usize)?;
        match entry_type {
            EntryType::Raw => {
                entry.raw = Some(payload.to_vec());
            }
            EntryType::Mavlink => {
                let (header, message) =
                    read_versioned_msg::<M, File>(&mut self.reader, MavlinkVersion::V1)?;
                entry.mav_header = Some(header);
                entry.mav_message = Some(message);
                return Ok(entry);
            }
            EntryType::Utf8Text => {
                entry.text = match String::from_utf8(payload.to_vec()) {
                    Ok(text) => Some(text),
                    Err(_) => {
                        return Err(MessageReadError::Io(std::io::Error::new(
                            std::io::ErrorKind::InvalidData,
                            "Failed to decode UTF-8 text from payload",
                        )));
                    }
                };
            }
        }

        Ok(entry)
    }
}

pub struct MavLogParser<P: Parser> {
    parser: P,
}

impl<P: Parser> MavLogParser<P> {
    pub fn new(file_path: &str) -> Self {
        let file: File = File::open(file_path).expect("Failed to open file");
        let mut reader: PeekReader<File> = PeekReader::new(file);

        // handle file header
        let header_bytes: [u8; 108] = reader
            .read_exact(108)
            .expect("Failed to read file header.")
            .try_into()
            .expect("Failed to read file header.");
        let mut header: FileHeader = FileHeader::unpack(&header_bytes);
        let definitions_raw: &[u8] = reader
            .read_exact(header.message_definition.size as usize)
            .expect("Failed to read message definitions.");
        header.message_definition.unpack_payload(definitions_raw);

        // Check supported payload types
        match header.message_definition.payload_type {
            MavlinkDefinitionPayloadType::None => {
                // No additional processing needed for this payload type
            }
            MavlinkDefinitionPayloadType::Utf8SpaceDelimitedUrlsForXMLFiles => {
                panic!("Custom XML files for message definitions are not supported.");
            }
            MavlinkDefinitionPayloadType::Utf8Xml => {
                panic!("XML for message definitions is not supported.");
            }
        }

        // check supported file format versions
        match header.format_version {
            1 => {
                // No additional processing needed for this format version
            }
            _ => {
                panic!("Unsupported file format version.");
            }
        }

        // check supported mavlink versions
        let mav_version = match header.message_definition.version_major {
            2 => MavlinkVersion::V2,
            1 => MavlinkVersion::V1,
            _ => {
                panic!("Unsupported MAVLink version.");
            }
        };

        if header.format_flags.mavlink_only && header.format_flags.not_timestamped {
            return MavLogParser {
                parser: MavlinkOnlyNoTimestampParser {
                    reader,
                    mav_version,
                },
            };
        } else if header.format_flags.mavlink_only && !header.format_flags.not_timestamped {
            return MavLogParser {
                parser: TimestampedMavlinkOnlyParser {
                    reader,
                    mav_version,
                },
            };
        } else {
            return MavLogParser {
                parser: MixedParser {
                    timestamped: !header.format_flags.not_timestamped,
                    reader,
                },
            };
        }

        panic!("Unsupported file format.");
    }
}

impl<M: Message, P: Parser> MavParser<M> for MavLogParser<P> {
    fn next(&mut self) -> Result<LogEntry<M>, MessageReadError> {
        self.parser.next::<M>()
    }
}
