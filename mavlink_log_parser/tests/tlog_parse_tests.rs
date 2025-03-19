#[cfg(feature = "Tlog")]
mod tlog_parse_tests {
    use mavlink::Message;
    use mavlink::ardupilotmega::MavMessage;
    use mavlink::error::MessageReadError;
    use mavlink_log_parser::tlog_parser::TlogParser;
    use mavlink_log_parser::{LogEntry, MavParser};

    #[test]
    fn test_tlog_parse() {
        let mut tlog = TlogParser::<MavMessage>::new("tests/data/tlog_data_0.tlog");
        let mut count: u64 = 0;
        loop {
            let entry: Result<LogEntry<MavMessage>, MessageReadError> = tlog.next();
            match entry {
                Ok(entry) => {
                    count += 1;
                    println!("Message");
                    println!(
                        "Message name: {:?}",
                        entry.mav_message.unwrap().message_name()
                    );
                }
                Err(_) => break,
            }
        }
        assert_eq!(count, 1426);
    }
}
