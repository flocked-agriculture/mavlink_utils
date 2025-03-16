# File Management

This logger should be continuously logging every message in full fidelity to a binary file with the extension as documented in the file format section below using a rotating file handler that caps the file size to limit based on underlying compute specifications. There should be an additional limit to the number of files allowed before the oldest file is replaced. Each filename should end with its count like follows: log.bin, log.bin.0, log.bin.1, etc. It is recommended each file have a unique name by giving the base name a suffix of the created timestamp per the [RFC 3339](https://datatracker.ietf.org/doc/html/rfc3339) spec format. So an example file name might look like "dispersion_1996-12-19T16:39:57.112.bin.12".

## Requirements

- Must be a binary file
- Must enforce a file size limit
- Must enforce a file count limit
- Must flush on every new record
- Must replace oldest file when max file count occurs
- Must rotate files upon reaching size limit in such a way the oldest file has the largest suffix number
- Must create either a [tlog file](https://docs.qgroundcontrol.com/master/en/qgc-dev-guide/file_formats/mavlink.html) or a [mav file](file_format.md)
