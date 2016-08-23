#!/usr/bin/env python3
# -*- coding: utf8 -*-

import binascii
from skytraq.venus6 import Venus6

serial_speed = None # 9600

# open com port
gps = Venus6('/dev/ttyUSB0', serial_speed, debug=False)

if serial_speed == None:
  # try to guess serial speed
  serial_speed = gps.guessSerialSpeed()

print("===================================")
print("====        device info       =====")
print("===================================")
print("> serial port speed: ", serial_speed)
print("> soft version", gps.getSoftwareVersion(0))
print("> soft crc", binascii.hexlify(gps.getSoftwareCRC()).decode('utf8').upper())

print("===================================")
print("====    getting log status    =====")
print("===================================")

(log_wr_ptr, sector_left, total_sector,
      min_time, max_time,
      min_distance, max_distance,
      min_speed, max_speed,
      data_log_enable, log_fifo_mode) = gps.getLogStatus()
print("> log_wr_ptr: 0x%X" % log_wr_ptr)
print("> sector_left: %d / %d" % (sector_left, total_sector))
print("> time min: %ds / max %ds" % (min_time, max_time))
print("> distance min: %d m / max %d m" % (min_distance, max_distance))
print("> speed min: %d km/h / max %d km/h" % (min_speed, max_speed))
print("> data_log_enable: %d" % data_log_enable)
print("> log_fifo_mode: %d" % log_fifo_mode)

print("===================================")
print("====         read logs        =====")
print("===================================")

if serial_speed != 115200:
  print("/!\\ CHANGING SERIAL SPEED TO 115200 /!\\")
  gps.setSerialSpeed(115200)

entries = []
with open('raw.bin', 'wb') as raw:
  for s in range(total_sector - sector_left + 1):
    print("start reading sector %d" % s)
    data = gps.readLog(s, 1)
    print("got data: ", data[0:32])
    raw.write(data)

print("Saved data to raw.bin")

# restore serial speed
if serial_speed != 115200:
  print("/!\\ RESTORE SERIAL SPEED TO %d /!\\", serial_speed)
  gps.setSerialSpeed(serial_speed)

# gps.clearLogs()
