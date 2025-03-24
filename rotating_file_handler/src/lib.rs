//! # Rotating File Handler
//!
//! A simple rotating file handler for logging generic binary data. It works by providing a base
//! file name such a `log.txt`, a maximum file size in bytes, and a number of backup files to keep.
//! When the log file reaches the maximum size, it rotates the log files by renaming the existing
//! files containing the base name. It does so by adding a suffix with a number starting from `0`.
//! If the file name already has a suffix, it increments the number by `1` up to the limit of the
//! specified backup count.
//!
//! Example: base=log.txt max_bytes=1024 backup_count=3
//! log.txt -> log.txt.0 -> log.txt.1 -> log.txt.2
//!
//! Example usage of the `RotatingFileHandler`.
//!
//! ```
//! use rotating_file_handler::RotatingFileHandler;
//! use std::io::Write;
//! use std::fs;
//!
//! fn main() -> std::io::Result<()> {
//!     let mut handler = RotatingFileHandler::new("docs_log.txt", 1024, 3, None)?;
//!     handler.emit(b"Hello, world!")?;
//!     handler.emit(b"Logging some more data...")?;
//!     fs::remove_file("docs_log.txt");
//!     Ok(())
//! }
//! ```

use std::fs::{File, OpenOptions};
use std::io::{self, Write};
use std::option::Option;
use std::path::Path;

/// A handler for rotating log files.
///
/// This struct manages a log file that rotates when it reaches a specified size.
/// It keeps a specified number of backup files.
pub struct RotatingFileHandler {
    base_path: String,
    max_bytes: u64,
    backup_count: usize,
    current_size: u64,
    file: File,
    header: Option<Vec<u8>>,
}

impl RotatingFileHandler {
    /// Creates a new `RotatingFileHandler`.
    ///
    /// # Arguments
    ///
    /// * `base_path` - The base path of the log file.
    /// * `max_bytes` - The maximum size of the log file in bytes before it rotates.
    /// * `backup_count` - The number of backup files to keep.
    /// * `header` - An optional header to write to the log file.
    ///
    /// # Returns
    ///
    /// An `io::Result` containing the new `RotatingFileHandler` or an error.
    pub fn new(
        base_path: &str,
        max_bytes: u64,
        backup_count: usize,
        header: Option<Vec<u8>>,
    ) -> io::Result<Self> {
        let mut file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(base_path)?;
        if let Some(ref header) = header {
            if header.len() as u64 > max_bytes {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "Header size exceeds maximum file size",
                ));
            }
            file.write_all(header)?;
        }
        let current_size = file.metadata()?.len();
        Ok(Self {
            base_path: base_path.to_string(),
            max_bytes,
            backup_count,
            current_size,
            file,
            header,
        })
    }

    /// Rotates the log files.
    ///
    /// This method renames the current log file and creates a new one.
    /// It keeps a specified number of backup files.
    fn rotate(&mut self) -> io::Result<()> {
        self.file.flush()?; // Ensure all data is written to the file before rotating.
        for i in (1..self.backup_count).rev() {
            let src = format!("{}.{}", self.base_path, i - 1);
            let dst = format!("{}.{}", self.base_path, i);
            if Path::new(&src).exists() {
                std::fs::rename(src, dst)?; // Rename the backup files.
            }
        }
        std::fs::rename(&self.base_path, format!("{}.0", self.base_path))?; // Rename the current log file.
        self.file = OpenOptions::new()
            .append(true)
            .create(true)
            .open(&self.base_path)?; // Create a new log file.
        if let Some(ref header) = self.header {
            self.file.write_all(header)?; // Write the header to the new log file.
        }
        self.current_size = 0; // Reset the current size.
        Ok(())
    }

    /// Writes bytes to the log file.
    ///
    /// This method writes the provided bytes to the log file. If the file size
    /// exceeds the maximum size, it rotates the log files.
    ///
    /// # Arguments
    ///
    /// * `bytes` - The bytes to write to the log file.
    ///
    /// # Returns
    ///
    /// An `io::Result` indicating success or failure.
    pub fn emit(&mut self, bytes: &[u8]) -> io::Result<()> {
        if self.current_size + bytes.len() as u64 > self.max_bytes {
            self.rotate()?; // Rotate the log files if the size exceeds the maximum.
        }
        self.file.write_all(bytes)?; // Write the bytes to the log file.
        self.current_size += bytes.len() as u64; // Update the current size.
        Ok(())
    }
}

impl Write for RotatingFileHandler {
    /// Writes data to the log file.
    ///
    /// This method is part of the `Write` trait implementation. It writes the provided buffer
    /// to the log file using the `emit` method. If the file size exceeds the maximum size,
    /// it triggers a rotation of the log files.
    ///
    /// # Arguments
    ///
    /// * `buf` - The buffer containing the data to write.
    ///
    /// # Returns
    ///
    /// An `io::Result` containing the number of bytes written or an error.
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.emit(buf)?;
        Ok(buf.len())
    }

    /// Flushes the log file.
    ///
    /// This method ensures that all buffered data is written to the log file.
    ///
    /// # Returns
    ///
    /// An `io::Result` indicating success or failure.
    fn flush(&mut self) -> io::Result<()> {
        self.file.flush()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::io::Write;

    /// Test that the log file rotates when the maximum file size is reached.
    #[test]
    fn test_rotation_on_max_file_size() {
        let mut handler = RotatingFileHandler::new("test_case_1_log.txt", 10, 3, None).unwrap();

        // Emit data to reach the maximum file size but not exceed it.
        handler.emit(b"12345").unwrap();
        handler.emit(b"67890").unwrap();
        assert!(Path::new("test_case_1_log.txt").exists());
        assert!(!Path::new("test_case_1_log.txt.0").exists());

        // Emit more data to exceed the maximum file size and trigger rotation.
        handler.emit(b"abcde").unwrap();
        handler.emit(b"fghij").unwrap();
        assert!(Path::new("test_case_1_log.txt").exists());
        assert!(Path::new("test_case_1_log.txt.0").exists());
        assert!(!Path::new("test_case_1_log.txt.1").exists());

        let content = fs::read_to_string("test_case_1_log.txt").unwrap();
        assert_eq!(content, "abcdefghij");

        let content = fs::read_to_string("test_case_1_log.txt.0").unwrap();
        assert_eq!(content, "1234567890");

        let _ = fs::remove_file("test_case_1_log.txt");
        for i in 0..1 {
            let _ = fs::remove_file(format!("test_case_1_log.txt.{}", i));
        }
    }

    /// Test that the log file rotates when the maximum backup count is reached.
    #[test]
    fn test_rotation_on_max_count() {
        let mut handler = RotatingFileHandler::new("test_case_2_log.txt", 10, 2, None).unwrap();
        handler.emit(b"1234567890").unwrap();
        handler.emit(b"abcdefghij").unwrap(); // This should trigger a rotation.
        handler.emit(b"klmnopqrst").unwrap(); // This should trigger a rotation.
        handler.emit(b"uvwxyzabcd").unwrap(); // This should trigger a rotation.

        assert!(Path::new("test_case_2_log.txt").exists());
        assert!(Path::new("test_case_2_log.txt.0").exists());
        assert!(Path::new("test_case_2_log.txt.1").exists());
        assert!(!Path::new("test_case_2_log.txt.2").exists()); // Max 2 backups should exist.

        let content = fs::read_to_string("test_case_2_log.txt").unwrap();
        assert_eq!(content, "uvwxyzabcd");

        let content = fs::read_to_string("test_case_2_log.txt.0").unwrap();
        assert_eq!(content, "klmnopqrst");

        let content = fs::read_to_string("test_case_2_log.txt.1").unwrap();
        assert_eq!(content, "abcdefghij");

        let _ = fs::remove_file("test_case_2_log.txt");
        for i in 0..2 {
            let _ = fs::remove_file(format!("test_case_2_log.txt.{}", i));
        }
    }

    /// Test that the `emit` method writes data to the log file.
    #[test]
    fn test_emit() {
        let mut handler = RotatingFileHandler::new("test_case_3_log.txt", 50, 1, None).unwrap();
        handler.emit(b"Hello, world!").unwrap();
        handler.emit(b" More data.").unwrap();

        let content = fs::read_to_string("test_case_3_log.txt").unwrap();
        assert_eq!(content, "Hello, world! More data.");

        let _ = fs::remove_file("test_case_3_log.txt");
    }

    /// Test that the `write` method writes data to the log file.
    #[test]
    fn test_write_trait() {
        let mut handler = RotatingFileHandler::new("test_case_4_log.txt", 50, 1, None).unwrap();
        write!(handler, "Hello, world!").unwrap();
        write!(handler, " More data.").unwrap();

        let content = fs::read_to_string("test_case_4_log.txt").unwrap();
        assert_eq!(content, "Hello, world! More data.");

        let _ = fs::remove_file("test_case_4_log.txt");
    }
}
