# Skytraq Venus module

import serial
import time
import datetime
import math
from ftplib import FTP

class Venus6:
  "Venus6 GPS object"

  MSG_TYPE_SOFT_VERSION_Q     = 0x02
  MSG_TYPE_SOFT_CRC_Q         = 0x03

  MSG_TYPE_CONF_SERIAL        = 0x05

  MSG_TYPE_LOG_STATUS_Q       = 0x17
  MSG_TYPE_LOG_CLEAR          = 0x19
  MSG_TYPE_LOG_READ_BATCH     = 0x1D
  MSG_TYPE_EPHEMERIS_GET      = 0x30
  MSG_TYPE_WAAS_SET           = 0x37
  MSG_TYPE_WAAS_GET           = 0x38
  MSG_TYPE_NAV_MODE_SET       = 0x3C
  MSG_TYPE_NAV_MODE_GET       = 0x3D

  MSG_TYPE_SOFT_VERSION_R     = 0x80
  MSG_TYPE_SOFT_CRC_R         = 0x81

  MSG_TYPE_ACK                = 0x83
  MSG_TYPE_NACK               = 0x84

  MSG_TYPE_LOG_STATUS_R       = 0x94

  MSG_TYPE_EPHEMERIS_GET_R    = 0xB1
  MSG_TYPE_WAAS_GET_R         = 0xB3
  MSG_TYPE_NAV_MODE_GET_R     = 0xB5

  def __init__(self, serialport, baudrate, debug=False):
    if baudrate == None:
      baudrate = 9600
    self.serial = serial.Serial(serialport, baudrate, timeout=5)
    self.debug = debug

  def __del__(self):
    if hasattr(self, 'serial'):
      self.serial.close()

  def readline(self):
    "read a response line from gps host (terminated by \r\n)"
    prev = 0
    line = bytearray()
    while True:
      c = self.serial.read()
      line += c
      if c == b'\n' and prev == b'\r':
        return line
      prev = c

  def isNMEA(buf):
    "return if line content is NMEA cf: http://en.wikipedia.org/wiki/NMEA_0183"
    return buf[0] == ord('$')

  def setSerialSpeed(self, speed):
    "Set the host serial port speed"

    speedIdx = -1
    if speed == 4800:
      speedIdx = 0
    elif speed == 9600:
      speedIdx = 1
    elif speed == 19200:
      speedIdx = 2
    elif speed == 38400:
      speedIdx = 3
    elif speed == 57600:
      speedIdx = 4
    elif speed == 115200:
      speedIdx = 5

    if speedIdx < 0:
      raise Exception("invalid speed", speed)

    self.sendCmd(self.MSG_TYPE_CONF_SERIAL,
      bytearray([
        0x00, # COM1
        speedIdx,
        0x00 # only update sram
      ]))

    print("speed changed to", speed)

    self.serial.flush()

    # update baudrate
    self.serial.baudrate = speed

    print(self.readline())

  def guessSerialSpeed(self):
    "Attempt to guess host serial port speed"
    for speed in [ 9600, 115200, 4800, 19200, 38400, 57600 ]:
      self.serial.baudrate = speed
      try:
        rep = self.getSoftwareVersion(0)
        if self.debug:
          print("got software version", rep, "at speed", speed)
        return speed
      except Exception as e:
        print("failed to get soft version at speed", speed, e)
    raise Exception("failed to guess serial speed")

  def readResponse(self, expectedRespId=None, expectedLen=0, maxAttempts=256):
    "read a response from gps, discarding NMEA output"
    attempt = 0
    prev = 0
    response = bytearray()
    tmp = bytearray()

    # look for start of sequence
    while attempt < maxAttempts:

      c = self.serial.read()
      if c[0] == 0xA1 and prev == 0xA0:
        break
      tmp += c
      prev = c[0]
      attempt += 1

    if attempt >= maxAttempts:
      raise Exception("failed to get response after reading %d bytes" % attempt, tmp)

    # read length
    l = self.serial.read(2)
    payloadLen = l[0] << 8 | l[1]
    msgId = self.serial.read()[0]
    payload = self.serial.read(payloadLen - 1)

    # read checksum
    checksum = self.serial.read()[0]
    # compare checksum
    check = 0 ^ msgId
    for b in payload:
      check ^= b
    if check != checksum:
      raise Exception("received msg with invalid checksum", check, checksum)

    # read end of sequence
    eos = self.serial.read(2)
    if eos != b"\r\n":
      raise Exception("invalid end of sequence", eos)

    if self.debug:
      print("RX <-", payloadLen, msgId, payload)

    if expectedRespId and msgId != expectedRespId:
      raise Exception("unexpected reponse from gps", msgId,
        "expected", expectedRespId)
    if expectedLen and len(payload) != expectedLen:
      raise Exception("unexpected reponse length",
        len(payload), "expected", expectedLen)
    return msgId, payload

  def sendCmd(self, msgId, payload, maxAttempts=5, expectAck=True):
    "sends a binary message to venus gps"
    msg = bytearray([0xA0, 0xA1]) # start of sequence
    payloadLen = 1 + len(payload)
    checksum = 0
    # payload length
    msg.append(payloadLen >> 8)
    msg.append(payloadLen & 0xFF)
    # msg_id
    msg.append(msgId)
    checksum ^= msgId
    # payload
    msg += payload
    for b in payload:
      checksum ^= b
    # checksum
    msg.append(checksum)
    # end of sequence
    msg += bytearray([0x0D, 0x0A])
    if self.debug:
      print("TX ->", msg)
    self.serial.write(msg)

    # check ACK
    if expectAck:
      i = 0
      while i < maxAttempts:
        repId, repPayload = self.readResponse()
        if repId == self.MSG_TYPE_NACK:
          if repPayload[0] == msgId:
            raise Exception("got NACK from gps")
        elif repId == self.MSG_TYPE_ACK:
          if repPayload[0] == msgId:
            # ok got ack
            break
        elif self.debug:
          print("received unexpected", repId, repPayload)

      if i >= maxAttempts:
        raise Exception("failed to get ack for query")

  def getSoftwareVersion(self, versionType):
    "Get running software version on host"
    self.sendCmd(self.MSG_TYPE_SOFT_VERSION_Q, bytearray([versionType]))

    # read response
    msgId, payload = self.readResponse(self.MSG_TYPE_SOFT_VERSION_R, 13)
    versionType = payload[0]
    kernelVersion = "%d.%d.%d" % (payload[1] << 8 | payload[2],
                                  payload[3], payload[4])
    odmVersion = "%d.%d.%d" % (payload[5] << 8 | payload[6],
                                  payload[7], payload[8])
    revision = "%d/%d/%d" % (payload[9] << 8 | payload[10],
                                  payload[11], payload[12])
    return (versionType, kernelVersion, odmVersion, revision)

  def getSoftwareCRC(self):
    "Get CRC of software running on host"
    self.sendCmd(self.MSG_TYPE_SOFT_CRC_Q, bytearray([0x01]))
    # read response
    msgId, payload = self.readResponse(self.MSG_TYPE_SOFT_CRC_R, 3)
    return (payload[1:3])

  def getLogStatus(self):
    "Get log buffer status / NOTE: number transmitted in little endian"
    self.sendCmd(self.MSG_TYPE_LOG_STATUS_Q, bytearray())
    # read response
    msgId, payload = self.readResponse(self.MSG_TYPE_LOG_STATUS_R)

    if (len(payload) < 34):
      raise Exception("unexpected reponse length for status log",
                        len(payload))

    offset = 0
    log_wr_ptr = (payload[offset+3] << 24 | payload[offset+2] << 16
                  | payload[offset+1] << 8 | payload[offset])
    offset += 4
    sector_left = payload[offset+1] << 8 | payload[offset]
    offset += 2
    total_sector = payload[offset+1] << 8 | payload[offset]
    offset += 2
    max_time = (payload[offset+3] << 24 | payload[offset+2] << 16
                  | payload[offset+1] << 8 | payload[offset])
    offset += 4
    min_time = (payload[offset+3] << 24 | payload[offset+2] << 16
                  | payload[offset+1] << 8 | payload[offset])
    offset += 4
    max_distance = (payload[offset+3] << 24 | payload[offset+2] << 16
                  | payload[offset+1] << 8 | payload[offset])
    offset += 4
    min_distance = (payload[offset+3] << 24 | payload[offset+2] << 16
                  | payload[offset+1] << 8 | payload[offset])
    offset += 4
    max_speed = (payload[offset+3] << 24 | payload[offset+2] << 16
                  | payload[offset+1] << 8 | payload[offset])
    offset += 4
    min_speed = (payload[offset+3] << 24 | payload[offset+2] << 16
                  | payload[offset+1] << 8 | payload[offset])
    offset += 4
    data_log_enable = payload[offset]
    offset += 1
    log_fifo_mode = payload[offset]
    offset += 1

    return (log_wr_ptr, sector_left, total_sector,
      min_time, max_time, min_distance, max_distance,
      min_speed, max_speed, data_log_enable, log_fifo_mode)

  def readLogResponse(self, sector, length):
    "read a log response from gps"

    # read response
    response = self.serial.read(length)

    # compute data checksum
    mysum = 0
    for i in range(length):
      mysum ^= response[i]

    end = self.serial.read(13)
    if (end != b"END\x00CHECKSUM="):
      print("unexpected end", end, response)
      while True:
        print(self.readline())
      raise Exception("unexpected response, check device page size", response, end)

    checksum = self.serial.read(1)[0]
    if checksum != mysum:
      raise Exception("checksum mismatch: 0x%x vs 0x%x" % (checksum, mysum))

    # check sector read
    tmp = self.serial.read(2)
    sector_verif = tmp[1] << 8 | tmp[0]
    if sector_verif != sector:
      raise Exception("sector mismatch %d vs %d" % (sector_verif, sector))

    # discard trailing crap
    self.serial.read(3)

    # wait for next line before attempting to issue
    # another command
    self.readline()

    return response

  def readLog(self, sector, nb_sector=1, maxAttempts=3):
    "Read a log sector form gps"
    attempt = 0
    while True:
      try:
        self.sendCmd(self.MSG_TYPE_LOG_READ_BATCH,
          bytearray([
            # start sector
            (sector >> 16) & 0xFF,
            sector & 0xFF,
            # nb sector
            (nb_sector >> 16) & 0xFF,
            nb_sector & 0xFF
          ])
        )
        data = self.readLogResponse(sector, nb_sector * 4096) # sector length is 4096 bytes
        break
      except Exception as e:
        print("FAILED TO READ SECTOR", e)
        attempt += 1
        if attempt > maxAttempts:
          raise Exception("failed to read sector")
    return data

  @staticmethod
  def __ecef_to_geo(x, y, z):
    # see: http://fr.wikipedia.org/wiki/WGS_84
    # see: http://microem.ru/files/2012/08/GPS.G1-X-00006.pdf
    a = 6378137.0         # earth semimajor axis in meters
    e = 0.081819190842622 # excentricity
    e_2 = 0.0066943799901414 # e ^ 2
    b = 6356752.314245179497563967 # earth semi minor axis
    ep = 0.08209443794969568 # first eccentricity sqrt((a^2-b^2)/b^2)
    ep_2 = 0.006739496742276433 # ep ^ 2

    p = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    th = math.atan2(a*z, b*p)
    lon = math.atan2(y,x)
    lat = math.atan2(
      z + ep_2 * b * math.pow(math.sin(th), 3),
      p - e_2 * a * math.pow(math.cos(th), 3)
    )
    N = a / math.sqrt(1 - e_2 * math.pow(math.sin(lat), 2))
    alt = p / math.cos(lat) - N
    return (lon * 180 / math.pi, lat * 180 / math.pi, alt)

  @staticmethod
  def __gps_time_to_timestamp(week_number, time_of_week):
    refDate = datetime.datetime(1980, 1, 6, tzinfo=datetime.timezone.utc)
    date = refDate + datetime.timedelta(weeks=week_number, seconds=time_of_week)
    return date

  @staticmethod
  def __decodeFull(data, offset):
    speed = (data[offset] & 0x03) << 8 | data[offset + 1]
    wn = data[offset + 3] | ((data[offset + 2] & 0x03) << 8)
    wn += 1024 # counter wrapped 1 time
    tow = (((data[offset + 2] >> 4) & 0x0F)
          | (data[offset + 5] << 4)
          | (data[offset + 4] << 12))
    offset += 6

    ecef_x = data[offset + 1] + (data[offset] << 8) + (data[offset + 3] << 16) + (data[offset + 2] << 24);
    offset += 4
    ecef_y = data[offset + 1] + (data[offset] << 8) + (data[offset + 3] << 16) + (data[offset + 2] << 24);
    offset += 4
    ecef_z = data[offset + 1] + (data[offset] << 8) + (data[offset + 3] << 16) + (data[offset + 2] << 24);
    offset += 4

    # print(">> speed %d km/h" % speed)
    # print(">> wn %d" % wn)
    # print(">> tow %d" % tow)
    # print(">> ecef_x %d" % ecef_x)
    # print(">> ecef_y %d" % ecef_y)
    # print(">> ecef_z %d" % ecef_z)

    return (speed, wn, tow, ecef_x, ecef_y, ecef_z)

  @staticmethod
  def __decodeCompact(data, offset):
    speed = (data[offset] & 0x03) << 8 | data[offset + 1]
    d_tow = data[offset + 2] << 8 | data[offset + 3]
    d_x = data[offset + 4] << 2 | (data[offset + 5] >> 6) & 0x03
    d_y = (data[offset + 5] & 0x3F) | (((data[offset + 6] >> 4) & 0x0F) << 6)
    d_z = ((data[offset + 6] & 0x03) << 8) | data[offset + 7]

    # convert to signed
    if (d_x >= 512):
      d_x = 511 - d_x
    if (d_y >= 512):
      d_y = 511 - d_y
    if (d_z >= 512):
      d_z = 511 - d_z

    return (speed, d_tow, d_x, d_y, d_z)

  @staticmethod
  def decodeLog(data):
    entries = []
    offset = 0
    length = len(data)
    while offset < length:
      b = data[offset]
      entryType = b >> 5

      if entryType == 2 or entryType == 3:
        # full fix
        (speed, wn, tow, ecef_x, ecef_y, ecef_z) = Venus6.__decodeFull(data, offset)

        # convert ecef to geodetic system
        (lon, lat, alt) = Venus6.__ecef_to_geo(ecef_x, ecef_y, ecef_z)

        # convert time
        date = Venus6.__gps_time_to_timestamp(wn, tow)

        # add entry
        entries.append([date, lat, lon, alt, speed])
        offset += 18
      elif entryType == 4:
        # read compact
        (speed, d_tow, d_x, d_y, d_z) = Venus6.__decodeCompact(data, offset)

        tow += d_tow
        ecef_x += d_x
        ecef_y += d_y
        ecef_z += d_z

        # convert ecef to geodetic system
        (lon, lat, alt) = Venus6.__ecef_to_geo(ecef_x, ecef_y, ecef_z)

        # convert time
        date = Venus6.__gps_time_to_timestamp(wn, tow)

        # add entry
        entries.append([date, lat, lon, alt, speed])

        offset += 8
      elif entryType == 7:
        # empty, skip
        offset += 2
      else:
        print("WARN: unknown entry type %d", entryType)
        offset += 1

    return entries


  def clearLogs(self):
    "clear all gps logs"
    self.sendCmd(self.MSG_TYPE_LOG_CLEAR, bytearray())

  def getWaasStatus(self):
    "get waas status from host"
    self.sendCmd(self.MSG_TYPE_WAAS_GET, bytearray())
    # read response
    msgId, payload = self.readResponse(self.MSG_TYPE_WAAS_GET_R, 1)
    return (payload[0])

  def setWaasStatus(self, enabled, persist=True):
    "enable/disable waas"
    self.sendCmd(self.MSG_TYPE_WAAS_SET, bytearray([
      1 if enabled else 0,
      1 if persist else 0,
    ]))
    # give time to reboot
    time.sleep(1)
    # drop received data
    self.serial.flushInput()

  def getNavigationMode(self):
    "get navigation from host"
    self.sendCmd(self.MSG_TYPE_NAV_MODE_GET, bytearray())
    # read response
    msgId, payload = self.readResponse(self.MSG_TYPE_NAV_MODE_GET_R, 1)
    return "pedestrian" if payload[0] else "car"

  def setNavigationMode(self, pedestrian, persist=True):
    "set navigation mode"
    self.sendCmd(self.MSG_TYPE_NAV_MODE_SET, bytearray([
      1 if pedestrian else 0,
      1 if persist else 0,
    ]))
    # give time to reboot
    time.sleep(1)
    # drop received data
    self.serial.flushInput()

  def getEphemeris(self, sv):
    "get ephemeris data from host"
    self.sendCmd(self.MSG_TYPE_EPHEMERIS_GET, bytearray([sv]))
    # read response
    msgId, payload = self.readResponse(self.MSG_TYPE_EPHEMERIS_GET_R, 86)
    return payload

  def updateEphemeris(self):
    "update current gps ephemeris"

    # download ephemeris file
    if self.debug:
      print("Start downloading ephemeris file")
    with FTP('60.250.205.31') as ftp:
      ftp.login('skytraq', 'skytraq')
      ftp.cwd('ephemeris')
      eph_data = bytearray()
      ftp.retrbinary('RETR Eph_4.dat', eph_data.extend)
      if self.debug:
        print('got data', len(eph_data), eph_data)

    # send content to host
    if self.debug:
      print("Start uploading data to host")
    self.serial.write(eph_data)
    if self.debug:
      print("Done")

    # give time to process
    time.sleep(1)
    # drop received data
    self.serial.flushInput()
