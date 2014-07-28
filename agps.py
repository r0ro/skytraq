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

waas_enabled = gps.getWaasStatus()
print("> waas: ", waas_enabled)
nav_mode = gps.getNavigationMode()
print("> navigation mode: ", nav_mode)

# ensure waas is enabled
if not waas_enabled:
  print("Enable WAAS")
  gps.setWaasStatus(True, True)
  print("> waas: ", gps.getWaasStatus())

# ensure we are in pedestrian mode
if nav_mode != "pedestrian":
  print("Enable pedestrian mode")
  gps.setNavigationMode(True, True)
  print("> navigation mode: ", gps.getNavigationMode())

print("===================================")
print("====   update ephemeris info  =====")
print("===================================")

gps.updateEphemeris()

print("DONE")

#eph = gps.getEphemeris(0)
#print(eph)
