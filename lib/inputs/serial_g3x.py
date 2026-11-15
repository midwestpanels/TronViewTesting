#!/usr/bin/env python

# Serial input source
# Garmin G3X
# 01/30/2019 Brian Chesteen, credit to Topher for developing template for input modules.
# 06/18/2023 C.Jones fix for live serial data input.
# 07/28/2024 A.O. grab DA and TAS from G3X Sentence ID 2 versus calculating from SID 1
# 08/06/2024 A.O. Fix SentID parsing, GPS AGL Parsing, LatLongHemi Parsing
# 11/6/2024  Added IMU data.
# 02/09/2025 DataShip refactor. Topher
# 11/15/2026 Line-based framing with checksum validation, underscore (invalid) field
#            handling, binary playback, and parse errors no longer stop TronView.
#            Modeled on the IntelliKeyNGv2 ESP32 G3X decoder.
#
# Format: G3X/G3X Touch Avionics Installation Manual 190-01115-01 Appendix C.
# Sentences start with '=' (or '@' for GPS PVT), are fixed width per sentence ID and end
# with CR/LF. '=' sentences end with a 2 char ASCII hex checksum: the 8 bit sum of every
# byte before it, including the leading '='. Fields filled with underscores are invalid.
# All field offsets below are from the start of the line ('=' or '@' is offset 0).

from ._input import Input
from lib import hud_utils
from lib import hud_text
from lib import geomag
from . import _utils
import serial
import struct
import math, sys
import time
from contextlib import suppress
from lib.common.dataship.dataship import Dataship
from lib.common.dataship.dataship_air import AirData
from lib.common.dataship.dataship_imu import IMUData
from lib.common.dataship.dataship_engine_fuel import EngineData, FuelData
from lib.common.dataship.dataship_nav import NavData
from lib.common.dataship.dataship_gps import GPSData
import traceback

# Max bytes to skip while hunting for a sentence start before giving other inputs a turn.
MAX_SYNC_BYTES = 512
# If this many bytes are waiting on the serial port we are falling behind; drop the backlog
# so the display stays current. (~0.35 sec of data at 115200 baud)
MAX_BACKLOG_BYTES = 4096

def checkInputVal(value):
    try:
        newval = int(value)
        return True
    except:
        return False

def safeInt(value):
    try:
        newval = int(value)
        return newval
    except:
        return None

def g3xInt(line, off, width):
    '''Fixed width integer field. Returns None if the field is missing, underscore filled
    (the G3X invalid marker) or not a number.'''
    if off + width > len(line):
        return None
    field = line[off:off + width]
    if b"_" in field:
        return None
    try:
        return int(field)
    except ValueError:
        return None

def g3xChar(line, off):
    '''Single character field as str, or None if missing/invalid.'''
    if off >= len(line):
        return None
    c = chr(line[off])
    if c == "_":
        return None
    return c

def g3xChecksumOK(line):
    '''line is the sentence without CR/LF. 8 bit sum of every byte before the 2 checksum
    chars (including the leading '=') must match the checksum as ASCII hex.'''
    if len(line) < 4:
        return False
    try:
        given = int(line[-2:], 16)
    except ValueError:
        return False
    return (sum(line[:-2]) & 0xFF) == given

class serial_g3x(Input):
    def __init__(self):
        self.name = "serial_g3x"
        self.version = 1.3
        self.inputtype = "serial"

        # Setup moving averages to smooth a bit
        self.readings = []
        self.max_samples = 10
        self.readings1 = []
        self.max_samples1 = 20
        self.imuData = IMUData()
        self.airData = AirData()
        self.navData = NavData()
        self.gpsData = GPSData()
        self.engineData = EngineData()
        self.fuelData = FuelData()


    def initInput(self,num, dataship:Dataship):
        Input.initInput(self,num, dataship)  # call parent init Input.
        self.msg_bad = 0
        self.msg_unknown = 0
        self.msg_checksum_bad = 0
        if(self.PlayFile!=None and self.PlayFile!=False):
            # play a log file?
            if self.PlayFile==True:
                defaultTo = "garmin_g3x_data1.txt"
                self.PlayFile = hud_utils.readConfig(self.name, "playback_file", defaultTo)
            # binary mode so playback sees exactly the bytes the serial port would.
            self.ser,self.input_logFileName = Input.openLogFile(self,self.PlayFile,"rb")
            if self.ser is None:
                print("G3X: unable to open playback file: "+str(self.PlayFile))
                dataship.errorFoundNeedToExit = True
                return
            self.isPlaybackMode = True
        else:
            self.efis_data_port = hud_utils.readConfig(self.name, "port", "/dev/ttyS0")
            self.efis_data_baudrate = hud_utils.readConfigInt(self.name, "baudrate", 115200)
            # open serial connection.
            try:
                self.ser = serial.Serial(
                    port=self.efis_data_port,
                    baudrate=self.efis_data_baudrate,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS,
                    timeout=1,
                )
            except Exception as e:
                print("G3X: unable to open serial port "+str(self.efis_data_port)+" : "+str(e))
                print("G3X: set the port in config.cfg under [serial_g3x] port = ...")
                dataship.errorFoundNeedToExit = True
                return
        print("G3X input init'd")

        # create a empty imu object.
        self.imuData = IMUData()
        self.imuData.id = "g3x_imu"+str(len(dataship.imuData))
        self.imuData.name = self.name
        self.imu_index = len(dataship.imuData)  # Start at 0
        dataship.imuData.append(self.imuData)
        self.last_read_time = time.time()
        print("G3X: IMU Init")

        # create a empty air object.
        self.airData = AirData()
        self.airData.id = "g3x_air"+str(len(dataship.airData))
        self.airData.name = self.name
        self.air_index = len(dataship.airData)  # Start at 0
        dataship.airData.append(self.airData)
        print("G3X: AirData Init")

        # create a empty nav object.
        self.navData = NavData()
        self.navData.id = "g3x_nav"+str(len(dataship.navData))
        self.navData.name = self.name
        self.nav_index = len(dataship.navData)  # Start at 0
        dataship.navData.append(self.navData)
        print("G3X: NavData Init")

        # create a empty gps object.
        self.gpsData = GPSData()
        self.gpsData.id = "g3x_gps"+str(len(dataship.gpsData))
        self.gpsData.name = self.name
        self.gps_index = len(dataship.gpsData)  # Start at 0
        dataship.gpsData.append(self.gpsData)
        print("G3X: GPS Data Init")

        # create a empty engine object.
        self.engineData = EngineData()
        self.engineData.id = "g3x_engine"+str(len(dataship.engineData))
        self.engineData.name = self.name
        self.engine_index = len(dataship.engineData)  # Start at 0
        dataship.engineData.append(self.engineData)
        print("G3X: Engine Data Init")


    # close this input source
    def closeInput(self, dataship:Dataship):
        if self.ser is not None:
            self.ser.close()

    #############################################
    ## Function: _readSentence
    ## Returns the next complete sentence (bytes, starting with '=' or '@', CR/LF stripped)
    ## and the raw bytes as received (for logging). Returns (None, None) if nothing complete
    ## was available this call.
    def _readSentence(self):
        if not self.isPlaybackMode:
            # keep latency bounded if we fall behind, without discarding data on every message.
            waiting = getattr(self.ser, "in_waiting", 0)
            if waiting > MAX_BACKLOG_BYTES:
                self.ser.reset_input_buffer()

        # hunt for start of sentence.
        start = None
        for _ in range(MAX_SYNC_BYTES):
            t = self.ser.read(1)
            if len(t) == 0:
                if self.isPlaybackMode:
                    self.ser.seek(0)  # end of file, loop playback.
                return None, None
            if t == b"=" or t == b"@":
                start = t
                break
        if start is None:
            return None, None

        rest = self.ser.readline()  # works for serial and binary files. includes the \n
        if len(rest) == 0 or not rest.endswith(b"\n"):
            # partial line (serial timeout or end of file)
            return None, None
        raw = start + rest
        line = raw.rstrip(b"\r\n")
        # if a sentence start shows up mid line we joined onto a partial sentence; resync there.
        idx = max(line.rfind(b"="), line.rfind(b"@"))
        if idx > 0:
            line = line[idx:]
            raw = raw[idx:]
        return line, raw

    def _log(self, raw):
        if self.output_logFile != None:
            Input.addToLog(self,self.output_logFile,raw)

    def _debugHz(self, dataship:Dataship):
        if dataship.debug_mode > 0:
            current_time = time.time() # calculate hz.
            dt = current_time - self.last_read_time
            if dt > 0:
                self.imuData.hz = round(1 / dt, 1)
            self.last_read_time = current_time
            print("System Time: " + str(current_time))
            print("GPS Time: " + str(self.gpsData.GPSTime_string))

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship:Dataship):
        if dataship.errorFoundNeedToExit:
            return dataship
        if self.ser is None:
            return dataship
        try:
            line, raw = self._readSentence()
            if line is None:
                return dataship
            if(dataship.debug_mode>1):
                print("g3x: "+str(raw))

            if line[0:1] == b"@":
                self._parsePVT(dataship, line, raw)
                return dataship

            # '=' sentences carry a checksum.
            if not g3xChecksumOK(line):
                self.msg_checksum_bad += 1
                self.msg_bad += 1
                return dataship

            SentID = line[1:2]
            if SentID == b"1":
                self._parseAttitude(dataship, line, raw)
            elif SentID == b"2":
                self._parseAttitude2(dataship, line, raw)
            elif SentID == b"7":
                self._parseGPSData(dataship, line, raw)
            else:
                # =3 engine, =5 EIS, =C CNI, etc. Not handled yet.
                self.msg_unknown += 1
                if self.isPlaybackMode:
                    time.sleep(0.01)

        except (serial.SerialException, OSError) as e:
            # the port itself failed (unplugged, etc.)
            print("G3X serial port error")
            print(e)
            traceback.print_exc()
            dataship.errorFoundNeedToExit = True
        except Exception as e:
            # a bad sentence should never take down the display. count it and move on.
            self.msg_bad += 1
            if self.msg_bad < 10 or dataship.debug_mode > 0:
                print("G3X parse exception (sentence skipped)")
                print(type(e))
                print(e)
                traceback.print_exc()

        return dataship

    #############################################
    ## '@' GPS PVT (Garmin simple text). No checksum.
    ## @YYMMDDhhmmss + lat/lon/status/HPE/alt/velocities. 55 chars without CR/LF.
    def _parsePVT(self, dataship:Dataship, line, raw):
        if len(line) != 55:
            self.gpsData.msg_bad += 1
            return
        if(dataship.debug_mode>1):
            print("g3x: GPS Sentence")
        self.gpsData.msg_count += 1

        hh = g3xInt(line, 7, 2)
        mm = g3xInt(line, 9, 2)
        ss = g3xInt(line, 11, 2)
        if None not in (hh, mm, ss):
            self.gpsData.sys_time_string = "%d:%d:%d"%(hh, mm, ss)
            self.gpsData.GPSTime_string = self.gpsData.sys_time_string
            if(dataship.debug_mode>1):
                print("GPS Time: " + self.gpsData.GPSTime_string)

        GPSAlt = g3xInt(line, 34, 6)
        if GPSAlt is not None:
            self.gpsData.Alt = GPSAlt * 3.28084  # meters to feet

        EWVelDir = g3xChar(line, 40)  # E or W
        EWVelmag = g3xInt(line, 41, 4)
        NSVelDir = g3xChar(line, 45)  # N or S
        NSVelmag = g3xInt(line, 46, 4)
        VVelDir = g3xChar(line, 50)   # U or D
        VVelmag = g3xInt(line, 51, 4)
        if EWVelDir is not None: self.gpsData.EWVelDir = EWVelDir
        if EWVelmag is not None: self.gpsData.EWVelmag = EWVelmag * 0.1
        if NSVelDir is not None: self.gpsData.NSVelDir = NSVelDir
        if NSVelmag is not None: self.gpsData.NSVelmag = NSVelmag * 0.1
        if VVelDir is not None: self.gpsData.VVelDir = VVelDir
        if VVelmag is not None: self.gpsData.VVelmag = VVelmag * 0.1

        # position. only valid once the GPS has a fix (underscores before that).
        LatHemi = g3xChar(line, 13)
        LatDeg = g3xInt(line, 14, 2)
        LatMin = g3xInt(line, 16, 5)
        LonHemi = g3xChar(line, 21)
        LonDeg = g3xInt(line, 22, 3)
        LonMin = g3xInt(line, 25, 5)
        if (LatHemi in ("N", "S") and LonHemi in ("E", "W")
                and None not in (LatDeg, LatMin, LonDeg, LonMin)):
            self.gpsData.Mag_Decl, self.gpsData.Lat, self.gpsData.Lon = _utils.calc_geomag(
                LatHemi, LatDeg, LatMin * 0.001,
                LonHemi, LonDeg, LonMin * 0.001,
            )

        #self.gpsData.GndSpeed = _utils.gndspeed(EWVelmag, NSVelmag) * 1.15078 # convert back to mph - commented due to G3X sentence already existing
        if (EWVelDir in ("E", "W") and NSVelDir in ("N", "S")
                and EWVelmag is not None and NSVelmag is not None):
            self.gpsData.GndTrack = _utils.gndtrack(EWVelDir, EWVelmag, NSVelDir, NSVelmag)

        if(dataship.debug_mode>1):
            current_time = time.time() # calculate hz.
            dt = current_time - self.last_read_time
            if dt > 0:
                self.imuData.hz = round(1 / dt, 1)
            self.last_read_time = current_time
            print("System Time: " + str(current_time))
        self._log(raw)

    def _setAltitudes(self, PressAlt, AltSet):
        self.airData.Alt_pres = PressAlt
        if AltSet is not None:
            self.airData.Baro = (AltSet + 2750.0) / 100.0
            self.airData.Baro_diff = self.airData.Baro - 29.9213
        if PressAlt is not None:
            baro_diff = self.airData.Baro_diff if self.airData.Baro_diff is not None else 0
            self.airData.Alt = int(PressAlt + (baro_diff / 0.00108))  # 0.00108 of inches of mercury change per foot.
        self.airData.BALT = self.airData.Alt

    def _mean(self, nums):
        return float(sum(nums)) / max(len(nums), 1)

    #############################################
    ## =1 Attitude / Air Data. 57 chars (full) or 45 chars (older, no ROT/accels/AOA).
    def _parseAttitude(self, dataship:Dataship, line, raw):
        self.imuData.msg_last = raw
        if line[2:3] != b"1" or len(line) not in (57, 45):
            self.airData.msg_bad += 1
            return
        full = len(line) == 57
        if(dataship.debug_mode>1):
            print("g3x: Attitude/Air Data Sentence (id:1)" + ("" if full else " short"))

        Pitch = g3xInt(line, 11, 4)
        Roll = g3xInt(line, 15, 5)
        Heading = g3xInt(line, 20, 3)
        Airspeed = g3xInt(line, 23, 4)
        PressAlt = g3xInt(line, 27, 6)
        if full:
            RateofTurn = g3xInt(line, 33, 4)
            LatAcc = g3xInt(line, 37, 3)
            VertAcc = g3xInt(line, 40, 3)
            AOA = g3xInt(line, 43, 2)
            VertSpeed = g3xInt(line, 45, 4)
            OAT = g3xInt(line, 49, 3)
            AltSet = g3xInt(line, 52, 3)
        else:
            VertSpeed = g3xInt(line, 33, 4)
            OAT = g3xInt(line, 37, 3)
            AltSet = g3xInt(line, 40, 3)

        if Roll is not None:
            self.imuData.roll = (Roll / 10) * -1
        if Pitch is not None:
            self.imuData.pitch = Pitch / 10
        if Airspeed is not None:
            self.airData.IAS = Airspeed * 0.115078 # convert knots to mph * 0.1
        if OAT is not None:
            self.airData.OAT = (OAT * 1.8) + 32 # c to f
        self.imuData.mag_head = Heading
        self.imuData.yaw = Heading
        self._setAltitudes(PressAlt, AltSet)
        if VertSpeed is not None:
            self.airData.VSI = VertSpeed * 10 # vertical speed in fpm

        if full:
            if AOA is not None:
                self.readings1.append(AOA)
                if len(self.readings1) > self.max_samples1:
                    self.readings1.pop(0)
                self.airData.AOA = self._mean(self.readings1)  # Moving average to smooth a bit
            else:
                self.airData.AOA = 0
            if RateofTurn is not None:
                self.imuData.turn_rate = RateofTurn * 0.1
            if VertAcc is not None:
                self.imuData.vert_G = VertAcc * 0.1
            if LatAcc is not None:
                self.readings.append(LatAcc * 0.01)
                if len(self.readings) > self.max_samples:
                    self.readings.pop(0)
                self.imuData.slip_skid = self._mean(self.readings)  # Moving average to smooth a bit

        self.imuData.msg_count += 1
        if (self.isPlaybackMode):  # if playback mode then add a delay.  Else reading a file is way to fast.
            time.sleep(0.08)
        self._log(raw)
        self._debugHz(dataship)

    #############################################
    ## =2 Attitude / Air Data set 2. 40 chars (full) or 28 chars (TAS only).
    def _parseAttitude2(self, dataship:Dataship, line, raw):
        self.airData.msg_last = raw
        if line[2:3] != b"1" or len(line) not in (40, 28):
            self.airData.msg_bad += 1
            return
        TAS = g3xInt(line, 11, 4)
        if TAS is not None:
            self.airData.TAS = TAS * 0.115078 # convert knots to mph * 0.1
        if len(line) == 40:
            if(dataship.debug_mode>1):
                print("g3x: Attitude/Air Data Sentence (id:2) TAS, DAlt, HeadingSel, AltSel, AirspeedSel, VSSel")
            self.airData.Alt_da = g3xInt(line, 15, 6)
            self.navData.HeadBug = g3xInt(line, 21, 3)
            self.navData.AltBug = g3xInt(line, 24, 6)
            AirspeedSel = g3xInt(line, 30, 4)
            if AirspeedSel is not None:
                self.navData.ASIBug = AirspeedSel * 0.115078 # convert knots to mph * 0.1
            VSSel = g3xInt(line, 34, 4)
            if VSSel is not None:
                self.navData.VSBug = VSSel * 10 # multiply up to hundreds of feet
            self.navData.msg_count += 1
        else:
            if(dataship.debug_mode>1):
                print("g3x: Attitude/Air Data Sentence (id:2) TAS")
            self.airData.msg_count += 1
        if (self.isPlaybackMode):  # if playback mode then add a delay.  Else reading a file is way to fast.
            time.sleep(0.08)
        self._log(raw)

    #############################################
    ## =7 GPS AGL data. 20 chars (AGL + ground speed) or 16 chars (older, AGL only).
    def _parseGPSData(self, dataship:Dataship, line, raw):
        if line[2:3] != b"1" or len(line) not in (20, 16):
            self.airData.msg_bad += 1
            return
        if(dataship.debug_mode>1):
            print("g3x: GPS AGL data message")
        HeightAGL = g3xInt(line, 11, 3)
        if HeightAGL is not None:
            self.airData.Alt_agl = HeightAGL * 100
        if len(line) == 20:
            GroundSpeed = g3xInt(line, 14, 4)
            if GroundSpeed is not None:
                self.gpsData.GndSpeed = GroundSpeed * 0.115078 # convert knots to mph * 0.1
        self.gpsData.msg_count += 1
        self._log(raw)

    # =3 Engine Data is not parsed yet (needs testing). Note from the IntelliKeyNGv2 decoder:
    # on a TWIN this sentence alternates between engines with no engine identifier, so its
    # per-engine fields can't be attributed; =5 (EIS) is the authority for per-engine data.
    # Table C-3 offsets (from line start): oil press 11, oil temp 14, RPM 18, MAP 26,
    # fuel flow 29, fuel press 35, fuel qty 38/41, calc fuel 44, volts 47/50, amps 53,
    # airframe hours 57 (w5), engine hours 62 (w5), then CHT6/EGT6/CHT5/EGT5... from 67 in steps of 4.


# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python
