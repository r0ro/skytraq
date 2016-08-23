#!/usr/bin/env python3
# -*- coding: utf8 -*-

import binascii
from skytraq.venus6 import Venus6
from datetime import timedelta, datetime
from pytz import timezone
import pytz
import sys

local_zone = timezone('America/Toronto')

with open('raw.bin', 'rb') as raw:
	data = raw.read()
	entries = Venus6.decodeLog(data)

	# dump entries to gpx file
	with open('dump.gpx', 'w') as gpx:

	  gpx.write("""<?xml version="1.0" encoding="UTF-8" standalone="no" ?>
	<gpx xmlns="http://www.topografix.com/GPX/1/1" creator="r0ro"
	  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
	  xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd">
	  <trk>
	    <trkseg>

	  """)

	  prev_date = None
	  max_diff = timedelta(hours=1)
	  max_age = timedelta(days=20)
	  now = datetime.now(pytz.utc)
	  for (date, lat, lon, alt, speed) in entries:
	    if now - date > max_age:
	      continue

	    local_date = date.astimezone(local_zone)
	    print("%s | %0.8f | %0.8f" % (local_date, lat, lon))

	    if prev_date == None:
	      prev_date = date
	    else:
	      if date - prev_date > max_diff:
	        gpx.write('</trkseg>\n</trk>\n\n<trk>\n<trkseg name="%s">\n' % (date))
	    gpx.write('<trkpt lat="%f" lon="%f"><ele>%f</ele><time>%s</time><speed>%d</speed></trkpt>\n' %
	      (lat, lon, alt, date.isoformat(), speed))
	    prev_date = date

	  gpx.write("""
	    </trkseg>
	  </trk>
	</gpx>""")

	# for (date, lat, lon, alt, speed) in entries:
	# 	print("%s | %0.8f | %0.8f" % (date, lat, lon))
