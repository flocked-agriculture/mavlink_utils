/// This module defines a rotating file logger for MAVLink messages.
/// It supports logging raw data, text, and MAVLink messages with optional
/// format flags and message definitions.
use std::option::Option;
use std::option::Option::Some;
use std::time::SystemTime;

use mavlink::{MAVLinkV1MessageRaw, MAVLinkV2MessageRaw};
use mavlink::{MavFrame, Message};
use rotating_file_handler::RotatingFileHandler;

use super::header::{FileHeader, FormatFlags, MavlinkMessageDefinition};
use crate::MavLogger;

/// Enum representing the type of log entry.
#[derive(PartialEq, Debug)]
enum EntryType {
    Raw = 0,
    Mavlink = 1,
    Text = 2,
}

/// Struct representing a rotating file logger for MAVLink messages.
pub struct RotatingFileMavLogger {
    header: FileHeader,
    time: SystemTime,
    file_handler: RotatingFileHandler,
}

impl RotatingFileMavLogger {
    /// Creates a new `RotatingFileMavLogger`.
    ///
    /// # Arguments
    ///
    /// * `base_path` - The base path for the log files.
    /// * `max_bytes` - The maximum size of a log file before it is rotated.
    /// * `backup_count` - The number of backup files to keep.
    /// * `format_flags` - Optional format flags for the log file.
    /// * `mavlink_definitions` - Optional MAVLink message definitions.
    ///
    /// # Returns
    ///
    /// A `Result` containing the new `RotatingFileMavLogger` or an `io::Error`.
    pub fn new(
        base_path: &str,
        max_bytes: u64,
        backup_count: usize,
        format_flags: Option<FormatFlags>,
        mavlink_definitions: Option<MavlinkMessageDefinition>,
    ) -> std::io::Result<Self> {
        // Handle optional format flags
        let flags: FormatFlags;
        match format_flags {
            Some(f) => flags = f,
            None => flags = FormatFlags::default(),
        }
        // Handle optional mavlink message definitions
        let msg_definition: MavlinkMessageDefinition;
        match mavlink_definitions {
            Some(d) => msg_definition = d,
            None => msg_definition = MavlinkMessageDefinition::default(),
        }
        // Create the file header
        let header: FileHeader = FileHeader::new(flags, msg_definition);

        // Create the rotating file handler
        let file_handler =
            RotatingFileHandler::new(base_path, max_bytes, backup_count, Some(header.pack()))?;

        Ok(Self {
            header,
            time: SystemTime::now(),
            file_handler,
        })
    }
}

impl MavLogger for RotatingFileMavLogger {
    /// Writes a MAVLink message to the log.
    ///
    /// # Arguments
    ///
    /// * `frame` - The MavFrame to log. This contains the MAVLink version, message, and header.
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure.
    fn write_mavlink<M: Message>(&mut self, frame: MavFrame<M>) -> std::io::Result<()> {
        match frame.protocol_version {
            mavlink::MavlinkVersion::V1 => {
                let mut msg: MAVLinkV1MessageRaw = MAVLinkV1MessageRaw::new();
                msg.serialize_message(frame.header, &frame.msg);
                return self.write(EntryType::Mavlink, msg.raw_bytes());
            }
            mavlink::MavlinkVersion::V2 => {
                let mut msg: MAVLinkV2MessageRaw = MAVLinkV2MessageRaw::new();
                msg.serialize_message(frame.header, &frame.msg);
                return self.write(EntryType::Mavlink, msg.raw_bytes());
            }
        }
    }
}

impl RotatingFileMavLogger {
    /// Writes a text message to the log.
    ///
    /// # Arguments
    ///
    /// * `text` - The text message to log.
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure.
    pub fn write_text(&mut self, text: &str) -> std::io::Result<()> {
        let text_bytes: &[u8] = text.as_bytes();
        self.write(EntryType::Text, text_bytes)
    }

    /// Writes raw data to the log.
    ///
    /// # Arguments
    ///
    /// * `data` - The raw data to log.
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure.
    pub fn write_raw(&mut self, data: &[u8]) -> std::io::Result<()> {
        self.write(EntryType::Raw, data)
    }

    /// Writes a log entry to the file.
    ///
    /// # Arguments
    ///
    /// * `entry_type` - The type of log entry (Raw, Mavlink, or Text).
    /// * `data` - The data to log.
    ///
    /// # Returns
    ///
    /// A `Result` indicating success or failure.
    fn write(&mut self, entry_type: EntryType, data: &[u8]) -> std::io::Result<()> {
        // If we are in MAVLink only mode and there is an attempt to write a non MAVLink entry, return an error.
        if entry_type != EntryType::Mavlink && self.header.format_flags.mavlink_only {
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "This logger accepts only mavlink messages",
            ));
        }

        // Construct the log entry
        let mut record_bytes: Vec<u8> = Vec::new();
        if !self.header.format_flags.mavlink_only {
            // If mavlink only, there is no need to track the entry type
            record_bytes.extend_from_slice(&(entry_type as u8).to_le_bytes());
        }
        if !self.header.format_flags.not_timestamped {
            // If tracking log entry time, add the timestamp
            let timestamp_us: u64 = match self.time.elapsed() {
                Ok(elapsed) => elapsed.as_micros() as u64,
                Err(_) => {
                    self.time = SystemTime::now();
                    0
                }
            };
            record_bytes.extend_from_slice(&timestamp_us.to_le_bytes());
        }
        if !self.header.format_flags.mavlink_only {
            // If mavlink only, no need to add the payload size
            let size: u16 = data.len() as u16;
            record_bytes.extend_from_slice(&size.to_le_bytes());
        }
        record_bytes.extend_from_slice(data);
        self.file_handler.emit(&record_bytes)?;

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use mavlink::MavHeader;
    use mavlink::MavlinkVersion;
    use mavlink::common::{HEARTBEAT_DATA, MavMessage};
    use std::fs::File;
    use std::io::Read;

    use super::*;

    /// Helper function to populate the log file with MAVLink, text, and raw data entries.
    fn populate_log_file(logger: &mut RotatingFileMavLogger) {
        // Define a MAVLink message to log
        let mavlink_message: MavFrame<MavMessage> = MavFrame {
            header: MavHeader::default(),
            msg: MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: mavlink::common::MavType::MAV_TYPE_SUBMARINE,
                autopilot: mavlink::common::MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode: mavlink::common::MavModeFlag::empty(),
                system_status: mavlink::common::MavState::MAV_STATE_STANDBY,
                mavlink_version: 0x3,
            }),
            protocol_version: MavlinkVersion::V2,
        };

        for _ in 0..10 {
            logger.write_mavlink(mavlink_message.clone()).unwrap();
        }

        if logger.header.format_flags.mavlink_only {
            return;
        }
        for _ in 0..10 {
            logger.write_text("Test log entry").unwrap();
        }
        for _ in 0..10 {
            logger.write_raw(&[1, 2, 3, 4, 5]).unwrap();
        }
        for _ in 0..2 {
            logger.write_mavlink(mavlink_message.clone()).unwrap();
            logger.write_text("Test log entry").unwrap();
            logger.write_raw(&[1, 2, 3, 4, 5]).unwrap();
        }
    }

    /// Test writing a mix of MAVLink, text, and raw data entries without any optimizations.
    #[test]
    fn test_write_mix_no_optimization() {
        // Define the test file name and remove it if it exists
        const CASE_FILE_NAME: &str = "test_mix_write_log.mav";
        std::fs::remove_file(CASE_FILE_NAME).unwrap_or_else(|_| {});

        // Create a new logger instance
        let mut logger: RotatingFileMavLogger =
            RotatingFileMavLogger::new(CASE_FILE_NAME, 1000, 0, None, None)
                .expect("Failed to create logger");

        // Populate the log file
        populate_log_file(&mut logger);

        // Read the log file and verify its content
        let mut file: File = File::open(CASE_FILE_NAME).unwrap();
        let mut content: Vec<u8> = Vec::new();
        file.read_to_end(&mut content).unwrap();
        assert_eq!(content.len(), 984);

        // Verify the file header
        assert_eq!(&content[0..16], logger.header.uuid.as_bytes());
        assert_eq!(content[16..24], logger.header.timestamp_us.to_le_bytes());
        assert_eq!(content[24..56], FileHeader::SRC_APPLICATION_ID);
        assert_eq!(
            content[56..60],
            FileHeader::FILE_FORMAT_VERSION.to_le_bytes()
        );
        assert_eq!(content[60..62], [0, 0]); // flags

        // Verify MAVLink entries
        for i in 0..10 {
            let offset = 108 + i * 32;
            assert_eq!(content[offset], 1); // type
            assert_ne!(content[offset + 1..offset + 9], [0, 0, 0, 0, 0, 0, 0, 0]); // timestamp
            assert_eq!(content[offset + 9..offset + 11], [21, 0]); // payload size
            assert_eq!(
                content[offset + 11..offset + 32],
                [
                    253, 9, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 12, 3, 0, 3, 3, 98, 190
                ]
            ); // payload
        }

        // Verify text entries
        for i in 0..10 {
            let offset = 428 + i * 25;
            assert_eq!(content[offset], 2); // type
            assert_ne!(content[offset + 1..offset + 9], [0, 0, 0, 0, 0, 0, 0, 0]); // timestamp
            assert_eq!(content[offset + 9..offset + 11], [14, 0]); // payload size
            assert_eq!(
                content[offset + 11..offset + 25],
                [
                    84, 101, 115, 116, 32, 108, 111, 103, 32, 101, 110, 116, 114, 121
                ]
            ); // payload
        }

        // Verify raw entries
        for i in 0..10 {
            let offset = 678 + i * 16;
            assert_eq!(content[offset], 0); // type
            assert_ne!(content[offset + 1..offset + 9], [0, 0, 0, 0, 0, 0, 0, 0]); // timestamp
            assert_eq!(content[offset + 9..offset + 11], [5, 0]); // payload size
            assert_eq!(content[offset + 11..offset + 16], [1, 2, 3, 4, 5]); // payload
        }

        // Verify mixed entries
        for i in 0..2 {
            let offset = 838 + i * 73;
            assert_eq!(content[offset], 1); // type
            assert_ne!(content[offset + 1..offset + 9], [0, 0, 0, 0, 0, 0, 0, 0]); // timestamp
            assert_eq!(content[offset + 9..offset + 11], [21, 0]); // payload size
            assert_eq!(
                content[offset + 11..offset + 32],
                [
                    253, 9, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 12, 3, 0, 3, 3, 98, 190
                ]
            ); // payload
            let offset_text = offset + 32;
            assert_eq!(content[offset_text], 2); // type
            assert_ne!(
                content[offset_text + 1..offset_text + 9],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ); // timestamp
            assert_eq!(content[offset_text + 9..offset_text + 11], [14, 0]); // payload size
            assert_eq!(
                content[offset_text + 11..offset_text + 25],
                [
                    84, 101, 115, 116, 32, 108, 111, 103, 32, 101, 110, 116, 114, 121
                ]
            ); // payload

            let offset_raw = offset + 57;
            assert_eq!(content[offset_raw], 0); // type
            assert_ne!(
                content[offset_raw + 1..offset_raw + 9],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ); // timestamp
            assert_eq!(content[offset_raw + 9..offset_raw + 11], [5, 0]); // payload size
            assert_eq!(content[offset_raw + 11..offset_raw + 16], [1, 2, 3, 4, 5]); // payload
        }

        // Clean up the test file
        std::fs::remove_file(CASE_FILE_NAME).unwrap();
    }

    /// Test writing a mix of MAVLink, text, and raw data entries with no timestamp optimization.
    #[test]
    fn test_write_mix_no_timestamp_optimization() {
        // Define the test file name and remove it if it exists
        const CASE_FILE_NAME: &str = "test_write_mix_no_timestamp.mav";
        std::fs::remove_file(CASE_FILE_NAME).unwrap_or_else(|_| {});

        // Define format flags with no timestamp optimization
        let format_flags = FormatFlags {
            not_timestamped: true,
            ..Default::default()
        };

        // Create a new logger instance with the format flags
        let mut logger: RotatingFileMavLogger =
            RotatingFileMavLogger::new(CASE_FILE_NAME, 1000, 0, Some(format_flags), None)
                .expect("Failed to create logger");

        // Populate the log file
        populate_log_file(&mut logger);

        // Read the log file and verify its content
        let mut file: File = File::open(CASE_FILE_NAME).unwrap();
        let mut content: Vec<u8> = Vec::new();
        file.read_to_end(&mut content).unwrap();
        assert_eq!(content.len(), 696);

        // Verify the file header
        assert_eq!(content[60..62], [2, 0]); // flags

        // Verify MAVLink entries
        for i in 0..10 {
            let offset = 108 + i * 24;
            assert_eq!(content[offset], 1); // type
            assert_eq!(content[offset + 1..offset + 3], [21, 0]); // payload size
            assert_eq!(
                content[offset + 3..offset + 24],
                [
                    253, 9, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 12, 3, 0, 3, 3, 98, 190
                ]
            ); // payload
        }

        // Verify text entries
        for i in 0..10 {
            let offset = 348 + i * 17;
            assert_eq!(content[offset], 2); // type
            assert_eq!(content[offset + 1..offset + 3], [14, 0]); // payload size
            assert_eq!(
                content[offset + 3..offset + 17],
                [
                    84, 101, 115, 116, 32, 108, 111, 103, 32, 101, 110, 116, 114, 121
                ]
            ); // payload
        }

        // Verify raw entries
        for i in 0..10 {
            let offset = 518 + i * 8;
            assert_eq!(content[offset], 0); // type
            assert_eq!(content[offset + 1..offset + 3], [5, 0]); // payload size
            assert_eq!(content[offset + 3..offset + 8], [1, 2, 3, 4, 5]); // payload
        }

        // Verify mixed entries
        for i in 0..2 {
            let offset = 598 + i * 49;
            assert_eq!(content[offset], 1); // type
            assert_eq!(content[offset + 1..offset + 3], [21, 0]); // payload size
            assert_eq!(
                content[offset + 3..offset + 24],
                [
                    253, 9, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 12, 3, 0, 3, 3, 98, 190
                ]
            ); // payload
            let offset_text = offset + 24;
            assert_eq!(content[offset_text], 2); // type
            assert_eq!(content[offset_text + 1..offset_text + 3], [14, 0]); // payload size
            assert_eq!(
                content[offset_text + 3..offset_text + 17],
                [
                    84, 101, 115, 116, 32, 108, 111, 103, 32, 101, 110, 116, 114, 121
                ]
            ); // payload

            let offset_raw = offset + 41;
            assert_eq!(content[offset_raw], 0); // type
            assert_eq!(content[offset_raw + 1..offset_raw + 3], [5, 0]); // payload size
            assert_eq!(content[offset_raw + 3..offset_raw + 8], [1, 2, 3, 4, 5]); // payload
        }

        // Clean up the test file
        std::fs::remove_file(CASE_FILE_NAME).unwrap();
    }

    /// Test writing only MAVLink entries with MAVLink only optimization.
    #[test]
    fn test_write_mavlink_only_optimization() {
        // Define the test file name and remove it if it exists
        const CASE_FILE_NAME: &str = "test_write_mavlink_only.mav";
        std::fs::remove_file(CASE_FILE_NAME).unwrap_or_else(|_| {});

        // Define format flags with MAVLink only optimization
        let format_flags = FormatFlags {
            mavlink_only: true,
            ..Default::default()
        };

        // Create a new logger instance with the format flags
        let mut logger: RotatingFileMavLogger =
            RotatingFileMavLogger::new(CASE_FILE_NAME, 1000, 0, Some(format_flags), None)
                .expect("Failed to create logger");

        populate_log_file(&mut logger);

        // Read the log file and verify its content
        let mut file: File = File::open(CASE_FILE_NAME).unwrap();
        let mut content: Vec<u8> = Vec::new();
        file.read_to_end(&mut content).unwrap();
        assert_eq!(content.len(), 108 + 10 * 29);

        // Verify the file header
        assert_eq!(content[60..62], [1, 0]); // flags

        // Verify MAVLink entries
        for i in 0..10 {
            let offset = 108 + i * 29;
            assert_ne!(content[offset..offset + 8], [0, 0, 0, 0, 0, 0, 0, 0]); // timestamp
            assert_eq!(
                content[offset + 8..offset + 29],
                [
                    253, 9, 0, 0, 0, 255, 0, 0, 0, 0, 0, 0, 0, 0, 12, 3, 0, 3, 3, 98, 190
                ]
            ); // payload
        }

        // Clean up the test file
        std::fs::remove_file(CASE_FILE_NAME).unwrap();
    }
}
