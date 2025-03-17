#[cfg(feature = "Tlog")]
mod tlog_parse_tests {
    use mavlink::Message;
    use mavlink::ardupilotmega::MavMessage;
    use mavlink_log_parser::tlog_parser::TlogParser;
    use mavlink_log_parser::{LogEntry, MavParser};
    use std::option::Option;

    #[test]
    fn test_tlog_parse() {
        let mut tlog = TlogParser::<MavMessage>::new("tests/data/tlog_data_0.tlog");
        let mut count: u64 = 0;
        loop {
            let entry: Option<LogEntry<MavMessage>> = tlog.next();
            match entry {
                Some(entry) => {
                    count += 1;
                    println!("Message");
                    println!("Message name: {:?}", entry.message.unwrap().message_name());
                }
                None => break,
            }
        }
        assert_eq!(count, 1426);
    }
}
