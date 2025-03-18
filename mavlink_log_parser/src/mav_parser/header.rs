use std::convert::TryFrom;
use std::convert::TryInto;

use uuid::Uuid;

/// Struct representing format flags for the log file.
///
/// `FormatFlags` contains options that modify the format of the log file.
/// - `mavlink_only`: If set, only MAVLink messages are logged allowing for a more compact log file.
/// - `not_timestamped`: If set, timestamps per entry are not included in the log file.
pub struct FormatFlags {
    /// If set, only MAVLink messages are logged allowing for a more compact log file.
    pub mavlink_only: bool,
    /// If set, timestamps per entry are not included in the log file.
    pub not_timestamped: bool,
}

impl FormatFlags {
    pub fn unpack(packed_data: u16) -> Self {
        FormatFlags {
            mavlink_only: packed_data & 0x01 != 0,
            not_timestamped: packed_data & 0x02 != 0,
        }
    }
}

/// Enum representing the payload type for MAVLink message definitions.
///
/// `MavlinkDefinitionPayloadType` specifies the type of payload used to identify message definitions.
/// - `None`: No payload. Use MAVLink main XML definition as default.
/// - `Utf8CommaDelimitedUrlsForXMLFiles`: UTF-8 encoded comma delimited URLs for XML files.
/// - `Utf8Xml`: UTF-8 encoded XML.
#[derive(PartialEq, Copy, Clone, Debug)]
pub enum MavlinkDefinitionPayloadType {
    /// No payload. Use MAVLink main XML definition as default.
    None = 0,
    /// UTF-8 encoded comma delimited URLs for XML files.
    Utf8SpaceDelimitedUrlsForXMLFiles = 1,
    /// UTF-8 encoded XML.
    Utf8Xml = 2,
}

impl TryFrom<u16> for MavlinkDefinitionPayloadType {
    type Error = ();

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(MavlinkDefinitionPayloadType::None),
            1 => Ok(MavlinkDefinitionPayloadType::Utf8SpaceDelimitedUrlsForXMLFiles),
            2 => Ok(MavlinkDefinitionPayloadType::Utf8Xml),
            _ => Err(()),
        }
    }
}

/// Struct representing a MAVLink message definition.
///
/// `MavlinkMessageDefinition` contains information about the MAVLink protocol version, dialect, payload type, and the actual payload.
pub struct MavlinkMessageDefinition {
    /// MAVLink protocol major version number.
    pub version_major: u32,
    /// MAVLink protocol minor version number.
    pub version_minor: u32,
    /// String identifying mavlink dialect.
    pub dialect: String,
    /// Type of payload used to identify message definition.
    pub payload_type: MavlinkDefinitionPayloadType,
    /// Size of message definition payload in bytes.
    pub size: u32,
    /// Variable size message definition payload.
    pub payload: Option<Vec<u8>>,
}

impl MavlinkMessageDefinition {
    pub fn unpack(packed_data: &[u8; 46]) -> Self {
        // stop at the first null byte when unpacking a string
        let end_dialect_ind: usize = match packed_data[8..40].iter().position(|&x| x == 0) {
            Some(index) => index + 8,
            None => 40,
        };
        MavlinkMessageDefinition {
            version_major: u32::from_le_bytes(packed_data[0..4].try_into().unwrap()),
            version_minor: u32::from_le_bytes(packed_data[4..8].try_into().unwrap()),
            dialect: String::from_utf8(packed_data[8..end_dialect_ind].to_vec()).unwrap(),
            payload_type: u16::from_le_bytes(packed_data[40..42].try_into().unwrap())
                .try_into()
                .unwrap(),
            size: u32::from_le_bytes(packed_data[42..46].try_into().unwrap()),
            payload: None,
        }
    }

    pub fn unpack_payload(&mut self, packed_data: &[u8]) {
        match self.payload_type {
            MavlinkDefinitionPayloadType::Utf8SpaceDelimitedUrlsForXMLFiles => {
                self.payload = Some(packed_data.to_vec());
            }
            MavlinkDefinitionPayloadType::Utf8Xml => {
                self.payload = Some(packed_data.to_vec());
            }
            _ => {}
        }
    }
}

/// Struct representing the file header for the log file.
///
/// `FileHeader` contains metadata about the log file, including a unique identifier, timestamp, source application ID,
/// format version, format flags, and message definitions.
pub struct FileHeader {
    /// Unique id for log file. It is expected the uuid library will be used to generate this.
    pub uuid: Uuid,
    /// The system unix timestamp in microseconds when the logger was initialized.
    pub timestamp_us: u64,
    /// String identifying the application used to generate the log file.
    pub src_application_id: String,
    /// A format version number. This is to allow compatability detection for future changes to the log file format.
    pub format_version: u32,
    /// A struct inidicating optional log file format changes.
    pub format_flags: FormatFlags,
    /// The message definitions for the log file.
    pub message_definition: MavlinkMessageDefinition,
}

impl FileHeader {
    pub fn unpack(packed_data: &[u8; 108]) -> Self {
        let id_end: usize = match packed_data[24..56].iter().position(|&x| x == 0) {
            Some(index) => index + 24,
            None => 56,
        };
        let src_application_id: String = match String::from_utf8(packed_data[24..id_end].to_vec()) {
            Ok(v) => v,
            Err(_e) => "".to_string(),
        };

        FileHeader {
            uuid: Uuid::from_bytes(packed_data[0..16].try_into().unwrap()),
            timestamp_us: u64::from_le_bytes(packed_data[16..24].try_into().unwrap()),
            src_application_id,
            format_version: u32::from_le_bytes(packed_data[56..60].try_into().unwrap()),
            format_flags: FormatFlags::unpack(u16::from_le_bytes(
                packed_data[60..62].try_into().unwrap(),
            )),
            message_definition: MavlinkMessageDefinition::unpack(
                packed_data[62..].try_into().unwrap(),
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_format_flags_unpack() {
        let packed_data: u16 = 0b11;
        let flags = FormatFlags::unpack(packed_data);
        assert!(flags.mavlink_only);
        assert!(flags.not_timestamped);

        let packed_data: u16 = 0b01;
        let flags = FormatFlags::unpack(packed_data);
        assert!(flags.mavlink_only);
        assert!(!flags.not_timestamped);

        let packed_data: u16 = 0b10;
        let flags = FormatFlags::unpack(packed_data);
        assert!(!flags.mavlink_only);
        assert!(flags.not_timestamped);

        let packed_data: u16 = 0b00;
        let flags = FormatFlags::unpack(packed_data);
        assert!(!flags.mavlink_only);
        assert!(!flags.not_timestamped);
    }

    #[test]
    fn test_mavlink_definition_payload_type_try_from() {
        assert_eq!(
            MavlinkDefinitionPayloadType::try_from(0).unwrap(),
            MavlinkDefinitionPayloadType::None
        );
        assert_eq!(
            MavlinkDefinitionPayloadType::try_from(1).unwrap(),
            MavlinkDefinitionPayloadType::Utf8SpaceDelimitedUrlsForXMLFiles
        );
        assert_eq!(
            MavlinkDefinitionPayloadType::try_from(2).unwrap(),
            MavlinkDefinitionPayloadType::Utf8Xml
        );
        assert!(MavlinkDefinitionPayloadType::try_from(3).is_err());
    }

    #[test]
    fn test_mavlink_message_definition_unpack() {
        let packed_data: [u8; 46] = [
            1, 0, 0, 0, // version_major
            2, 0, 0, 0, // version_minor
            b't', b'e', b's', b't', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, // dialect
            0, 0, // payload_type
            0, 0, 0, 0, // size
        ];
        let definition = MavlinkMessageDefinition::unpack(&packed_data);
        assert_eq!(definition.version_major, 1);
        assert_eq!(definition.version_minor, 2);
        assert_eq!(definition.dialect, "test");
        assert_eq!(definition.payload_type, MavlinkDefinitionPayloadType::None);
        assert_eq!(definition.size, 0);
        assert!(definition.payload.is_none());

        let mut packed_data: [u8; 46] = [
            1, 0, 0, 2, // version_major
            2, 0, 0, 1, // version_minor
            b't', b'e', b's', b't', b' ', b'1', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, // dialect
            1, 0, // payload_type
            0, 0, 0, 0, // size
        ];
        let urls_str: String = String::from("http://example.com http://example.2.com");
        let encoded_urls: &[u8] = urls_str.as_bytes();
        packed_data[42..46].copy_from_slice(&(encoded_urls.len() as u32).to_le_bytes());
        let mut definition = MavlinkMessageDefinition::unpack(&packed_data);
        assert_eq!(definition.version_major, 0x02000001);
        assert_eq!(definition.version_minor, 0x01000002);
        assert_eq!(definition.dialect, "test 1");
        assert_eq!(
            definition.payload_type,
            MavlinkDefinitionPayloadType::Utf8SpaceDelimitedUrlsForXMLFiles
        );
        assert_eq!(definition.size, encoded_urls.len() as u32);
        assert!(definition.payload.is_none());
        definition.unpack_payload(encoded_urls);
        assert_eq!(definition.payload, Some(encoded_urls.to_vec()));
    }

    #[test]
    fn test_file_header_unpack() {
        let packed_data: [u8; 108] = [
            // file header
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, // uuid
            16, 0, 0, 0, 0, 0, 0, 17, // timestamp_us
            b'a', b'p', b'p', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, // src_application_id
            1, 0, 0, 2, // format_version
            3, 4, // format_flags
            // message_definition
            4, 0, 0, 5, // version_major
            6, 0, 0, 7, // version_minor
            b't', b'e', b's', b't', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, // dialect
            2, 0, // payload_type
            10, 0, 0, 0, // size
        ];
        let header = FileHeader::unpack(&packed_data);
        assert_eq!(
            header.uuid,
            Uuid::from_bytes([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15])
        );
        assert_eq!(header.timestamp_us, 0x1100000000000010);
        assert_eq!(header.src_application_id, "app");
        assert_eq!(header.format_version, 0x02000001);
        assert!(header.format_flags.mavlink_only);
        assert!(header.format_flags.not_timestamped);
        assert_eq!(header.message_definition.version_major, 0x05000004);
        assert_eq!(header.message_definition.version_minor, 0x07000006);
        assert_eq!(header.message_definition.dialect, "test");
        assert_eq!(
            header.message_definition.payload_type,
            MavlinkDefinitionPayloadType::Utf8Xml
        );
        assert_eq!(header.message_definition.size, 10);
        assert!(header.message_definition.payload.is_none());
    }
}
