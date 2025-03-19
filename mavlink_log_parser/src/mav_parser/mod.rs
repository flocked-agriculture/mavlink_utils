mod header;

use std::convert::TryFrom;
use std::convert::TryInto;
use std::fs::File;

use mavlink::error::MessageReadError;
use mavlink::peek_reader::PeekReader;
use mavlink::{MavlinkVersion, Message, read_versioned_msg};

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

struct MavlinkOnlyNoTimestampParser<M: Message> {
    reader: PeekReader<File>,
    mav_version: MavlinkVersion,
    _phantom: std::marker::PhantomData<M>,
}

impl<M: Message> MavParser for MavlinkOnlyNoTimestampParser<M> {
    type M = M;

    fn next(&mut self) -> Result<LogEntry<M>, MessageReadError> {
        let mut entry: LogEntry<M> = LogEntry::default();
        let (header, message) = read_versioned_msg::<M, File>(&mut self.reader, self.mav_version)?;
        entry.mav_header = Some(header);
        entry.mav_message = Some(message);
        Ok(entry)
    }
}

struct TimestampedMavlinkOnlyParser<M: Message> {
    reader: PeekReader<File>,
    mav_version: MavlinkVersion,
    _phantom: std::marker::PhantomData<M>,
}

impl<M: Message> MavParser for TimestampedMavlinkOnlyParser<M> {
    type M = M;

    fn next(&mut self) -> Result<LogEntry<M>, MessageReadError> {
        let mut entry: LogEntry<M> = LogEntry::default();
        let magic_number: u8 = match self.mav_version {
            MavlinkVersion::V1 => mavlink::MAV_STX,
            MavlinkVersion::V2 => mavlink::MAV_STX_V2,
        };
        if self.reader.peek_exact(5)?[4] == magic_number {
            let timestamp_raw: &[u8] = self.reader.read_exact(8)?;
            entry.timestamp = match timestamp_raw.try_into() {
                Ok(bytes) => Some(u64::from_le_bytes(bytes)),
                Err(_) => None,
            };
        }
        let (header, message) = read_versioned_msg::<M, File>(&mut self.reader, self.mav_version)?;
        entry.mav_header = Some(header);
        entry.mav_message = Some(message);
        Ok(entry)
    }
}

pub struct MixedParser<M: Message> {
    timestamped: bool,
    reader: PeekReader<File>,
    _phantom: std::marker::PhantomData<M>,
}

impl<M: Message> MavParser for MixedParser<M> {
    type M = M;

    fn next(&mut self) -> Result<LogEntry<M>, MessageReadError> {
        let mut entry: LogEntry<M> = LogEntry::default();
        let entry_type: EntryType = self
            .reader
            .read_u8()
            .map(|value| value.try_into().unwrap_or(EntryType::Raw))?;
        if self.timestamped {
            let timestamp_raw: &[u8] = self.reader.read_exact(8)?;
            entry.timestamp = match timestamp_raw.try_into() {
                Ok(bytes) => Some(u64::from_le_bytes(bytes)),
                Err(_) => None,
            };
        }
        let payload_size: u16 = u16::from_le_bytes(
            self.reader
                .read_exact(2)?
                .try_into()
                .expect("Failed to read log entry payload size."),
        );
        let payload = self.reader.read_exact(payload_size as usize)?;
        match entry_type {
            EntryType::Raw => entry.raw = Some(payload.to_vec()),
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

pub struct MavLogParser<M: Message + 'static> {
    parser: Box<dyn MavParser<M = M>>,
}

impl<M: Message + 'static> MavLogParser<M> {
    pub fn new(file_path: &str) -> Self {
        let file: File = File::open(file_path).expect("Failed to open file");
        let mut reader: PeekReader<File> = PeekReader::new(file);

        let header = Self::read_file_header(&mut reader);

        let mav_version = Self::determine_mavlink_version(&header);

        let parser: Box<dyn MavParser<M = M>> = if header.format_flags.mavlink_only {
            if header.format_flags.not_timestamped {
                Box::new(MavlinkOnlyNoTimestampParser {
                    reader,
                    mav_version,
                    _phantom: std::marker::PhantomData,
                })
            } else {
                Box::new(TimestampedMavlinkOnlyParser {
                    reader,
                    mav_version,
                    _phantom: std::marker::PhantomData,
                })
            }
        } else {
            Box::new(MixedParser {
                timestamped: !header.format_flags.not_timestamped,
                reader,
                _phantom: std::marker::PhantomData,
            })
        };

        MavLogParser { parser }
    }

    fn read_file_header(reader: &mut PeekReader<File>) -> FileHeader {
        let header_bytes: [u8; 108] = reader
            .read_exact(108)
            .expect("Failed to read file header.")
            .try_into()
            .expect("Failed to read file header.");
        let mut header = FileHeader::unpack(&header_bytes);
        let definitions_raw: &[u8] = reader
            .read_exact(header.message_definition.size as usize)
            .expect("Failed to read message definitions.");
        header.message_definition.unpack_payload(definitions_raw);

        match header.message_definition.payload_type {
            MavlinkDefinitionPayloadType::None => {}
            MavlinkDefinitionPayloadType::Utf8SpaceDelimitedUrlsForXMLFiles => {
                panic!("Custom XML files for message definitions are not supported.");
            }
            MavlinkDefinitionPayloadType::Utf8Xml => {
                panic!("XML for message definitions is not supported.");
            }
        }

        match header.format_version {
            1 => {}
            _ => panic!("Unsupported file format version."),
        }

        header
    }

    fn determine_mavlink_version(header: &FileHeader) -> MavlinkVersion {
        match header.message_definition.version_major {
            2 => MavlinkVersion::V2,
            1 => MavlinkVersion::V1,
            _ => panic!("Unsupported MAVLink version."),
        }
    }
}

impl<M: Message + 'static> MavParser for MavLogParser<M> {
    type M = M;

    fn next(&mut self) -> Result<LogEntry<M>, MessageReadError> {
        self.parser.next()
    }
}
