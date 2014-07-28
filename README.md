Skytraq
=======

Skytraq Venus 6 GPS binary log protocol implentation

# Purpose

I've bought a nameless GPS USB DATA LOGGER on ebay (ref: *SJ-5282-DL*). This
GPS device outputs NMEA messages using a USB to serial port converter. It
also has the ability to store gps logs (date, position and speed) to its
internal memory.
To acces those gps logs, this device was sold with a proprietary software
called *GPS Photo Tagger*, that only works on windows.
My goal was to be able to access those logs using linux and Mac OS, in order
to add gps data to my pictures.
After some research, it appeared that this device was using a
*Skytraq Venus 6 GPS* module (many other gps modules uses Skytraq Venus 6
module, so it might work with other devices as well).
Skytraq uses a proprietary binary protocol to communicate with the module.
This protocol is specified in two application notes:
- [Binary Messages of Venus 6 GPS](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/GPS/AN0003_v1.4.19.pdf)
- [Data Logging Extension for Venus 6 GPS](http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/GPS/Venus/638/doc/AN0008_v1.4.11-datalogging.pdf)

# Status

## Working features:
- read/write navigation mode
- read/write waas status
- read software version / crc
- read data logs from device
- configure serial port speed
- probe serial port speed
- write ephemeris data (agps) to device (UNTESTED)

## TODO
- read/write data logging parameters
- read/write nmea output configuration
- read/write output format
- read/write position output rate
- restart system
- reset to factory default

# Usage

- to extract log data a sample program extract_log.py is provided. It
dumps the logs from the device and generate a dump.gpx file.
- to update ephemeris data use agps.py (UNTESTED)

# Reference to other similar projects

- https://github.com/makefu/skytraq-datalogger quite working but some agps features seems
different from what I've got.
