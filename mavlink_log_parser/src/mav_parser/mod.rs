mod header;

use std::convert::TryFrom;
use std::convert::TryInto;
use std::fs::File;

use mavlink::error::MessageReadError;
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

pub struct MavLogParser {
    mav_version: MavlinkVersion,
    header: FileHeader,
    reader: PeekReader<File>,
}

impl MavLogParser {
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

        Self {
            mav_version,
            header,
            reader,
        }
    }

    fn read_timestamp(&mut self) -> Result<Option<u64>, MessageReadError> {
        if !self.header.format_flags.not_timestamped {
            match self
                .reader
                .read_exact(8)
                .map(|bytes| Some(u64::from_le_bytes(bytes.try_into().unwrap())))
            {
                Ok(timestamp) => Ok(timestamp),
                Err(MessageReadError::Io(e)) => {
                    if e.kind() == std::io::ErrorKind::UnexpectedEof {
                        return Err(MessageReadError::Io(e));
                    }
                    return Ok(None);
                }
                Err(_) => Ok(None),
            }
        } else {
            Ok(None)
        }
    }

    fn read_mavlink<M: Message>(&mut self) -> Result<Option<(MavHeader, M)>, MessageReadError> {
        match read_versioned_msg::<M, File>(&mut self.reader, self.mav_version) {
            Ok(msg) => Ok(Some(msg)),
            Err(MessageReadError::Io(e)) => {
                if e.kind() == std::io::ErrorKind::UnexpectedEof {
                    return Err(MessageReadError::Io(e));
                }
                return Ok(None);
            }
            Err(_) => Ok(None),
        }
    }
}

impl<M: Message> MavParser<M> for MavLogParser {
    fn next(&mut self) -> Result<Option<LogEntry<M>>, MessageReadError> {
        let mut entry: LogEntry<M> = LogEntry::default();
        // Handle Mavlink Only
        if self.header.format_flags.mavlink_only {
            entry.timestamp = self.read_timestamp()?;
            match self.read_mavlink() {
                Ok(None) => return Ok(None),
                Ok(Some((header, message))) => {
                    entry.mav_header = Some(header);
                    entry.mav_message = Some(message);
                    return Ok(Some(entry));
                }
                Err(e) => return Err(e),
            }
        }

        let entry_type: EntryType = match self
            .reader
            .read_u8()
            .ok()
            .and_then(|type_| type_.try_into().ok())
        {
            Some(entry_type) => entry_type,
            None => EntryType::Raw,
        };

        // handle timestamp
        entry.timestamp = self.read_timestamp()?;

        let payload_size: u16 = u16::from_le_bytes(
            self.reader
                .read_exact(2)?
                .try_into()
                .expect("Failed to read entry size. Unable to proceed."),
        );
        let payload = self.reader.read_exact(payload_size as usize)?;
        match entry_type {
            EntryType::Raw => {
                entry.raw = Some(payload.to_vec());
            }
            EntryType::Mavlink => match self.read_mavlink() {
                Ok(None) => return Ok(None),
                Ok(Some((header, message))) => {
                    entry.mav_header = Some(header);
                    entry.mav_message = Some(message);
                    return Ok(Some(entry));
                }
                Err(e) => return Err(e),
            },
            EntryType::Utf8Text => {
                entry.text = String::from_utf8(payload.to_vec()).ok();
                if entry.text.is_none() {
                    return Ok(None);
                }
            }
        }

        Ok(Some(entry))
    }
}
