use std::time::SystemTime;

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

impl Default for FormatFlags {
    /// Provides default values for `FormatFlags`.
    ///
    /// By default, both `mavlink_only` and `not_timestamped` are set to `false`.
    fn default() -> Self {
        FormatFlags {
            mavlink_only: false,
            not_timestamped: false,
        }
    }
}

impl FormatFlags {
    /// Packs the `FormatFlags` into a 2-byte array.
    ///
    /// This method converts the `FormatFlags` into a 2-byte array where each flag is represented by a bit.
    ///
    /// # Returns
    /// A `[u8; 2]` array containing the packed representation of the `FormatFlags`.
    pub fn pack(&self) -> [u8; 2] {
        let flags: u16 = (self.mavlink_only as u16) | ((self.not_timestamped as u16) << 1);
        flags.to_le_bytes()
    }
}

/// Enum representing the payload type for MAVLink message definitions.
///
/// `MavlinkDefinitionPayloadType` specifies the type of payload used to identify message definitions.
/// - `None`: No payload. Use MAVLink main XML definition as default.
/// - `Utf8CommaDelimitedUrlsForXMLFiles`: UTF-8 encoded comma delimited URLs for XML files.
/// - `Utf8Xml`: UTF-8 encoded XML.
#[derive(PartialEq, Copy, Clone)]
pub enum MavlinkDefinitionPayloadType {
    /// No payload. Use MAVLink main XML definition as default.
    None = 0,
    /// UTF-8 encoded comma delimited URLs for XML files.
    Utf8SpaceDelimitedUrlsForXMLFiles = 1,
    /// UTF-8 encoded XML.
    Utf8Xml = 2,
}

/// Struct representing a MAVLink message definition.
///
/// `MavlinkMessageDefinition` contains information about the MAVLink protocol version, dialect, payload type, and the actual payload.
pub struct MavlinkMessageDefinition {
    /// MAVLink protocol major version number.
    pub version_major: u32,
    /// MAVLink protocol minor version number.
    pub version_minor: u32,
    /// UTF-8 string identifying mavlink dialect.
    pub dialect: String, // needs to be packed into a c-type char array of len 32
    /// Type of payload used to identify message definition.
    pub payload_type: MavlinkDefinitionPayloadType,
    /// Size of message definition payload in bytes.
    pub size: u32,
    /// Variable size message definition payload.
    pub payload: Vec<u8>,
}

impl MavlinkMessageDefinition {
    /// Default dialect for MAVLink message definitions.
    pub const DEFAULT_DIALECT: &str = "common";

    /// Packs the `MavlinkMessageDefinition` into a vector of bytes.
    ///
    /// This function serializes the `MavlinkMessageDefinition` into a byte vector
    /// by converting its fields into their respective byte representations and
    /// appending them to the vector. The packed data includes the major and minor
    /// version numbers, the dialect, the payload type, and the size. If the payload
    /// type is not `None`, the payload is also included in the packed data.
    ///
    /// # Returns
    ///
    /// A `Vec<u8>` containing the packed byte representation of the `MavlinkMessageDefinition`.
    pub fn pack(&self) -> Vec<u8> {
        assert!(self.dialect.len() <= 32, "dialect must be 32 bytes or less");
        let mut dialect_bytes = [0u8; 32];
        dialect_bytes[..self.dialect.len()].copy_from_slice(self.dialect.as_bytes());

        let mut packed: Vec<u8> = Vec::new();
        packed.extend_from_slice(&self.version_major.to_le_bytes());
        packed.extend_from_slice(&self.version_minor.to_le_bytes());
        packed.extend_from_slice(&dialect_bytes);
        let payload_type: u16 = self.payload_type as u16;
        packed.extend_from_slice(&payload_type.to_le_bytes());
        packed.extend_from_slice(&self.size.to_le_bytes());
        if self.payload_type != MavlinkDefinitionPayloadType::None {
            packed.extend_from_slice(&self.payload);
        }
        packed
    }
}

impl Default for MavlinkMessageDefinition {
    /// Provides default values for `MavlinkMessageDefinition`.
    ///
    /// By default, the major version is set to 2, minor version to 0, dialect to `DEFAULT_DIALECT`,
    /// payload type to `None`, size to 0, and payload to an empty vector.
    fn default() -> Self {
        MavlinkMessageDefinition {
            version_major: 2,
            version_minor: 0,
            dialect: String::from(MavlinkMessageDefinition::DEFAULT_DIALECT),
            payload_type: MavlinkDefinitionPayloadType::None,
            size: 0,
            payload: Vec::new(),
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
    /// UTF-8 string identifying the application used to generate the log file.
    pub src_application_id: String, // needs to be packed into a c-type char array of len 32
    /// A format version number. This is to allow compatability detection for future changes to the log file format.
    pub format_version: u32,
    /// A struct inidicating optional log file format changes.
    pub format_flags: FormatFlags,
    /// The message definitions for the log file.
    pub message_definition: MavlinkMessageDefinition,
}

impl FileHeader {
    /// Minimum size of the file header in bytes. Can be more if message definitions are included.
    pub const MIN_SIZE: usize = 108;
    /// Currently supported file format version.
    pub const FILE_FORMAT_VERSION: u32 = 1;
    /// Default source application ID.
    pub const SRC_APPLICATION_ID: &str = "mavlink_logger";

    /// Creates a new `FileHeader` with the provided format flags and message definition.
    ///
    /// This method initializes a new `FileHeader` with a unique UUID, the current timestamp in microseconds,
    /// the source application ID, format version, format flags, and message definition.
    ///
    /// # Arguments
    ///
    /// * `format_flags` - A `FormatFlags` struct indicating optional log file format changes.
    /// * `message_definition` - A `MavlinkMessageDefinition` struct containing the message definitions for the log file.
    ///
    /// # Returns
    ///
    /// A new `FileHeader` instance.
    pub fn new(
        format_flags: FormatFlags,
        message_definition: MavlinkMessageDefinition,
    ) -> FileHeader {
        let timestamp_us: u64 = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .expect("Time went backwards")
            .as_micros() as u64;

        FileHeader {
            uuid: Uuid::new_v4(),
            timestamp_us,
            src_application_id: String::from(FileHeader::SRC_APPLICATION_ID),
            format_version: FileHeader::FILE_FORMAT_VERSION,
            format_flags,
            message_definition,
        }
    }

    /// Packs the `FileHeader` into a vector of bytes.
    ///
    /// This method serializes the `FileHeader` fields into a byte vector in the following order:
    /// - UUID (16 bytes)
    /// - Timestamp in microseconds (8 bytes)
    /// - Source application ID (32 bytes, UTF-8 encoded)
    /// - Format version (8 bytes)
    /// - Format flags (2 bytes, packed)
    /// - Message definition (variable length, packed)
    /// All bytes are packed in little-endian format.
    ///
    /// # Returns
    /// A `Vec<u8>` containing the packed representation of the `FileHeader`.
    pub fn pack(&self) -> Vec<u8> {
        assert!(
            self.src_application_id.len() <= 32,
            "src_application_id must be 32 bytes or less"
        );
        let mut app_id_bytes = [0u8; 32];
        app_id_bytes[..self.src_application_id.len()]
            .copy_from_slice(self.src_application_id.as_bytes());

        let mut packed: Vec<u8> = Vec::new();
        packed.extend_from_slice(self.uuid.as_bytes());
        packed.extend_from_slice(&self.timestamp_us.to_le_bytes());
        packed.extend_from_slice(&app_id_bytes);
        packed.extend_from_slice(&self.format_version.to_le_bytes());
        packed.extend_from_slice(&self.format_flags.pack());
        packed.extend_from_slice(&self.message_definition.pack());
        packed
    }
}

impl Default for FileHeader {
    /// Provides default values for `FileHeader`.
    ///
    /// By default, the UUID is generated using the `uuid` library, the timestamp is set to the current time in microseconds,
    /// the source application ID is set to `SRC_APPLICATION_ID`, the format version is set to `FILE_FORMAT_VERSION`,
    /// the format flags are set to `FormatFlags::default()`, and the message definition is set to `MavlinkMessageDefinition::default()`.
    fn default() -> Self {
        let timestamp_us: u64 = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .expect("Time went backwards")
            .as_micros() as u64;

        FileHeader {
            uuid: Uuid::new_v4(),
            timestamp_us,
            src_application_id: String::from(FileHeader::SRC_APPLICATION_ID),
            format_version: FileHeader::FILE_FORMAT_VERSION,
            format_flags: FormatFlags::default(),
            message_definition: MavlinkMessageDefinition::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Tests the `pack` method of `FormatFlags`.
    ///
    /// This test verifies that the `pack` method correctly converts the `FormatFlags`
    /// struct into a 2-byte array representation. It checks various combinations of
    /// the `mavlink_only` and `not_timestamped` flags to ensure the correct bit
    /// representation in the packed array.
    fn test_format_flags_pack() {
        let flags = FormatFlags {
            mavlink_only: false,
            not_timestamped: false,
        };
        assert_eq!(flags.pack(), [0, 0]);

        let flags = FormatFlags {
            mavlink_only: true,
            not_timestamped: false,
        };
        assert_eq!(flags.pack(), [1, 0]);

        let flags = FormatFlags {
            mavlink_only: false,
            not_timestamped: true,
        };
        assert_eq!(flags.pack(), [2, 0]);

        let flags = FormatFlags {
            mavlink_only: true,
            not_timestamped: true,
        };
        assert_eq!(flags.pack(), [3, 0]);
    }

    #[test]
    /// Tests the `pack` method of `MavlinkMessageDefinition`.
    ///
    /// This test verifies that the `pack` method correctly serializes the
    /// `MavlinkMessageDefinition` struct into a byte vector. It checks the
    /// packed representation for both default and custom values of the
    /// `MavlinkMessageDefinition` fields, ensuring that each field is
    /// correctly converted to its byte representation and appended to the
    /// vector in the correct order.
    fn test_mavlink_message_definition_pack() {
        let definition = MavlinkMessageDefinition {
            version_major: 2,
            version_minor: 0,
            dialect: String::from(MavlinkMessageDefinition::DEFAULT_DIALECT),
            payload_type: MavlinkDefinitionPayloadType::None,
            size: 0,
            payload: Vec::new(),
        };
        let packed = definition.pack();
        assert_eq!(packed.len(), 46);
        assert_eq!(&packed[0..4], &[2, 0, 0, 0]);
        assert_eq!(&packed[4..8], &[0, 0, 0, 0]);
        assert_eq!(
            String::from_utf8(packed[8..40].to_vec()).unwrap(),
            "common\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"
        );
        assert_eq!(&packed[40..42], &[0, 0]);
        assert_eq!(&packed[42..46], &[0, 0, 0, 0]);

        let definition = MavlinkMessageDefinition {
            version_major: 0x01020304,
            version_minor: 0x04050607,
            dialect: String::from(MavlinkMessageDefinition::DEFAULT_DIALECT),
            payload_type: MavlinkDefinitionPayloadType::Utf8Xml,
            size: 5,
            payload: b"hello".to_vec(),
        };
        let packed = definition.pack();
        assert_eq!(packed.len(), 51);
        assert_eq!(&packed[0..4], &[4, 3, 2, 1]);
        assert_eq!(&packed[4..8], &[7, 6, 5, 4]);
        assert_eq!(
            String::from_utf8(packed[8..40].to_vec()).unwrap(),
            "common\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"
        );
        assert_eq!(&packed[40..42], &[2, 0]);
        assert_eq!(&packed[42..46], &[5, 0, 0, 0]);
        assert_eq!(&packed[46..51], b"hello");
    }

    #[test]
    /// Tests the `pack` method of `FileHeader`.
    ///
    /// This test verifies that the `pack` method correctly serializes the
    /// `FileHeader` struct into a byte vector. It checks the packed representation
    /// for a `FileHeader` instance with custom values for the `format_flags` and
    /// `message_definition` fields. The test ensures that each field is correctly
    /// converted to its byte representation and appended to the vector in the
    /// correct order, including the UUID, timestamp, source application ID,
    /// format version, format flags, and message definition.
    fn test_file_header_pack() {
        let format_flags = FormatFlags {
            mavlink_only: true,
            not_timestamped: false,
        };
        let message_definition = MavlinkMessageDefinition {
            version_major: 2,
            version_minor: 0,
            dialect: String::from(MavlinkMessageDefinition::DEFAULT_DIALECT),
            payload_type: MavlinkDefinitionPayloadType::Utf8Xml,
            size: 5,
            payload: b"hello".to_vec(),
        };
        let header = FileHeader::new(format_flags, message_definition);
        let packed = header.pack();
        assert_eq!(packed.len(), 113);
        assert_eq!(&packed[16..24], &header.timestamp_us.to_le_bytes()); // timestamp
        assert_eq!(
            String::from_utf8(packed[24..56].to_vec()).unwrap(),
            "mavlink_logger\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0"
        ); // src application id
        assert_eq!(&packed[56..60], &[1, 0, 0, 0]); // file version
        assert_eq!(&packed[60..62], &[1, 0]); // format flags
        assert_eq!(&packed[62..113], &header.message_definition.pack()[..]);
    }
}
