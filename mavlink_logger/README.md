# Mavlink Logger

This crate is a generic mavlink logger.

## Technical

Simple rotating file logging system.

## Documentation

- All user facing documentation should be a markdown file in the /docs folder
- all developer facing documentation should be provided through inline rust comment documentation or through this readme

## Future Work

- pass through features to the mavlink crate
- use rust features to select between .tlog and .mav loggers
- use rust features to select for certain optimizations such as no timestamps or mavlink only
- add support for logging in embedded systems
