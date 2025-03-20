# Mavlink Log Parser

This crate contains utilities for parsing mavlink log files.

## Known Issues

### read_versioned_msg

The read_versioned_msg function from rust_mavlink is not built for mixed data streams which can cause problems on the mixed stream log file. Specifically it will search through the data for the mavlink packet start key. This only a problem for our parser if the file data got corrupted and an unexpected message defintion was used. Meaning it tried to read the current mavlink packet and failed. This method will immediately search for the next valid MAVLink message. It should in theory recover on the next mavlink message but until then, it is looping over potentially valid non mavlink file records or there could be false positives.

We would rather it exit on failure so we can take the data as raw and move on to the next entry.

> **NOTE**
> This is only relevant if the format flags MAVLINK_ONLY=true or NO_TIMESTAMP=false

## Future Work

- URGENT - create a new read_versioned_msg for the more complex log file types
- async
- extract timestamps from tlog
- strict feature flag to exit on any parsing issues rather than trying to continue
