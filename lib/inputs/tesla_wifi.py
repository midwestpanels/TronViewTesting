#!/usr/bin/env python

# Tesla CAN bus over WiFi (panda UDP protocol)
# 11/17/2026 A.O.
#
# Works with the Enhance Auto S3XY Commander (and CANserver), which share the car's CAN
# bus over WiFi using the "panda" UDP protocol that ScanMyTesla and similar apps use.
#
# Commander setup: in the Commander app enable ScanMyTesla support, then join the
# Commander's WiFi network (S3XY_OBD). The Commander is at 192.168.4.1, UDP port 1338.
#
# Panda UDP protocol (see github.com/joshwardell/CANserver tools/pandaTest_v2.py):
#   client -> server  "ehllo"                 connect (filtered mode). Must be re-sent every
#                                             few seconds as a heartbeat or the stream stops.
#   client -> server  0x0F + (bus,id)...      add frame filters  ("!B" + "!BH" per frame)
#   client -> server  0x18                    clear all filters
#   client -> server  "hello"                 old (v1) mode: send every frame, no filters
#   client -> server  "bye"                   disconnect
#   server -> client  16 byte frames, several per datagram:
#                     <I  f1: CAN id << 21
#                     <I  f2: length (low 4 bits), bus number (bits 4+)
#                     8 data bytes
#                     Bus 15 is the server itself: id 6 = connect ack, id 7 = connect refused.
#
# Signal definitions are from commaai/opendbc tesla_model3_party.dbc (Model 3 / Y).
# All signals used here are little endian (Intel).
#
# Data recording / playback uses the candump log format, one frame per line:
#   (1700000000.123456) can0 39B#0102030405060708
# so recordings also open in SavvyCAN, cantools, python-can, etc.

from ._input import Input
from lib import hud_utils
from lib.common import shared
from lib.common.dataship.dataship import Dataship
from lib.common.dataship.dataship_vehicle import VehicleData
import socket
import struct
import time
import traceback

KPH_TO_MPH = 0.621371

# server (bus 15) status frame ids
PANDA_BUS_SERVER = 15
PANDA_ACK = 6
PANDA_REFUSED = 7

HEARTBEAT_SEC = 3.0     # server drops us after ~5 sec without a heartbeat.
RECONNECT_SEC = 5.0     # no frames for this long = not connected; keep sending ehllo.
MAX_DATAGRAMS_PER_READ = 50  # drain this many UDP packets per readMessage() call.

# CAN ids
ID_DI_systemStatus = 0x118
ID_SCCM_steeringAngle = 0x129
ID_DI_speed = 0x257
ID_DI_state = 0x286
ID_DAS_control = 0x2B9
ID_UI_warning = 0x311
ID_DAS_status2 = 0x389
ID_DAS_status = 0x39B

FRAME_IDS = [
    ID_DI_systemStatus, ID_SCCM_steeringAngle, ID_DI_speed, ID_DI_state,
    ID_DAS_control, ID_UI_warning, ID_DAS_status,
]

GEARS = {1: "P", 2: "R", 3: "N", 4: "D"}

AUTOPILOT_STATES = {
    0: "DISABLED", 1: "UNAVAILABLE", 2: "AVAILABLE", 3: "ACTIVE_NOMINAL",
    4: "ACTIVE_RESTRICTED", 5: "ACTIVE_NAV", 6: "ACTIVE_FSD", 8: "ABORTING",
    9: "ABORTED", 14: "FAULT",
}
AUTOPILOT_ACTIVE_STATES = (3, 4, 5, 6)

HANDS_ON_STATES = {
    0: "NOT_REQD", 1: "REQD_DETECTED", 2: "REQD_NOT_DETECTED", 3: "REQD_VISUAL",
    4: "REQD_CHIME_1", 5: "REQD_CHIME_2", 6: "REQD_SLOWING", 7: "REQD_STRUCK_OUT",
    8: "SUSPENDED",
}

CRUISE_STATES = {
    0: "UNAVAILABLE", 1: "STANDBY", 2: "ENABLED", 3: "STANDSTILL", 4: "OVERRIDE",
    5: "FAULT", 6: "PRE_FAULT", 7: "PRE_CANCEL",
}
CRUISE_ACTIVE_STATES = (2, 3, 4)

LANE_DEPARTURE = {
    0: "NONE", 1: "LEFT_WARNING", 2: "RIGHT_WARNING", 3: "LEFT_WARNING_SEVERE",
    4: "RIGHT_WARNING_SEVERE",
}

SIDE_COLLISION = {0: "NONE", 1: "WARN_LEFT", 2: "WARN_RIGHT", 3: "WARN_LEFT_RIGHT"}


def sig(data, start, length):
    '''Unsigned little endian (Intel) signal from an 8 byte CAN payload.'''
    return (int.from_bytes(data, "little") >> start) & ((1 << length) - 1)


def parse_panda_frames(datagram):
    '''Yield (bus, can_id, data) for each 16 byte panda frame in a UDP datagram.'''
    for off in range(0, len(datagram) - 15, 16):
        f1, f2 = struct.unpack_from("<II", datagram, off)
        length = f2 & 0x0F
        if length > 8:
            length = 8
        yield (f2 >> 4), (f1 >> 21), datagram[off + 8:off + 8 + length]


def parse_candump_line(line):
    '''"(ts) canN ID#DATA" -> (ts, bus, can_id, data) or None.'''
    try:
        parts = line.split()
        if len(parts) < 3:
            return None
        ts = float(parts[0].strip("()"))
        bus = int("".join(c for c in parts[1] if c.isdigit()) or 0)
        ident, payload = parts[2].split("#", 1)
        return ts, bus, int(ident, 16), bytes.fromhex(payload)
    except ValueError:
        return None


class tesla_wifi(Input):
    def __init__(self):
        self.name = "tesla_wifi"
        self.version = 1.0
        self.inputtype = "network"
        self.vehicleData = VehicleData()

    def initInput(self, num, dataship: Dataship):
        Input.initInput(self, num, dataship)
        self.output_logBinary = False
        self.msg_unknown = 0
        self.msg_bad = 0
        self.last_heartbeat = 0
        self.last_frame_time = 0
        self.connect_refused = False
        self.frame_count_window = 0
        self.hz_window_start = time.time()

        self.host = hud_utils.readConfig(self.name, "host", "192.168.4.1")
        self.port = hud_utils.readConfigInt(self.name, "port", 1338)
        # filter = true asks the Commander for only the frames we decode (less WiFi traffic).
        # filter = false uses the older "hello" mode and receives every frame on the bus.
        self.use_filter = hud_utils.readConfigBool(self.name, "filter", True)
        # Only decode frames from this bus number. -1 = any bus.
        self.bus = hud_utils.readConfigInt(self.name, "bus", -1)

        self.vehicleData = VehicleData()
        self.vehicleData.id = "tesla_vehicle" + str(len(dataship.vehicleData))
        self.vehicleData.name = self.name
        self.vehicleData.inputSrcName = self.name
        self.vehicleData.inputSrcNum = num
        dataship.vehicleData.append(self.vehicleData)

        if self.PlayFile is not None and self.PlayFile is not False:
            if self.PlayFile is True:
                self.PlayFile = hud_utils.readConfig(self.name, "playback_file", "tesla_simulated_1.log")
            self.ser, self.input_logFileName = Input.openLogFile(self, self.PlayFile, "r")
            if self.ser is None:
                print("tesla_wifi: unable to open playback file: " + str(self.PlayFile))
                dataship.errorFoundNeedToExit = True
                return
            self.isPlaybackMode = True
            self.playback_t0_log = None
            self.playback_t0_wall = None
        else:
            self.ser = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            self.ser.bind(("", hud_utils.readConfigInt(self.name, "local_port", 0)))
            self.ser.setblocking(False)
            print("tesla_wifi: connecting to %s:%d" % (self.host, self.port))
            self._connect()

    def closeInput(self, dataship: Dataship):
        if self.ser is None:
            return
        if not self.isPlaybackMode:
            self._send(b"bye")
        self.ser.close()

    #############################################
    ## connection handling (live only)
    def _send(self, payload):
        try:
            self.ser.sendto(payload, (self.host, self.port))
        except OSError:
            # not on the Commander's WiFi yet, etc. keep trying on the next heartbeat.
            pass

    def _connect(self):
        if self.use_filter:
            self._send(b"ehllo")
            filt = struct.pack("!B", 0x0F)
            for can_id in FRAME_IDS:
                buses = (0, 1) if self.bus < 0 else (self.bus,)
                for bus in buses:
                    filt += struct.pack("!BH", bus, can_id)
            self._send(filt)
        else:
            self._send(b"hello")
        self.last_heartbeat = time.time()

    def _heartbeat(self):
        now = time.time()
        if now - self.last_heartbeat < HEARTBEAT_SEC:
            return
        if self.vehicleData.connected and now - self.last_frame_time > RECONNECT_SEC:
            self.vehicleData.connected = False
            shared.GrowlManager.add_message(self.name + ": lost connection to " + self.host)
        if self.vehicleData.connected:
            # heartbeat. filters stay in place on the server.
            self._send(b"ehllo" if self.use_filter else b"hello")
            self.last_heartbeat = now
        else:
            self._connect()

    #############################################
    ## Function: readMessage
    def readMessage(self, dataship: Dataship):
        if self.shouldExit:
            dataship.errorFoundNeedToExit = True
        if dataship.errorFoundNeedToExit or self.skipReadInput or self.ser is None:
            return dataship
        try:
            if self.isPlaybackMode:
                self._readPlayback(dataship)
            else:
                self._heartbeat()
                for _ in range(MAX_DATAGRAMS_PER_READ):
                    try:
                        datagram, _addr = self.ser.recvfrom(4096)
                    except (BlockingIOError, socket.timeout):
                        break
                    except OSError:
                        break  # e.g. ICMP port unreachable while the Commander is away.
                    now = time.time()
                    for bus, can_id, data in parse_panda_frames(datagram):
                        if self.output_logFile is not None:
                            Input.addToLog(self, self.output_logFile,
                                           "(%.6f) can%d %03X#%s\n" % (now, bus, can_id, data.hex().upper()))
                        self._processFrame(dataship, bus, can_id, data)
        except Exception as e:
            # one bad frame should never take down the display.
            self.msg_bad += 1
            self.vehicleData.msg_bad += 1
            if self.msg_bad < 10 or dataship.debug_mode > 0:
                print("tesla_wifi parse exception (frame skipped): " + str(e))
                traceback.print_exc()
        return dataship

    def _readPlayback(self, dataship: Dataship):
        '''Replay a candump log at the speed it was recorded.'''
        now = time.time()
        for _ in range(500):
            pos = self.ser.tell()
            line = self.ser.readline()
            if line == "":
                self.ser.seek(0)  # loop
                self.playback_t0_log = None
                return
            frame = parse_candump_line(line)
            if frame is None:
                continue
            ts, bus, can_id, data = frame
            if self.playback_t0_log is None:
                self.playback_t0_log = ts
                self.playback_t0_wall = now
            if ts - self.playback_t0_log > now - self.playback_t0_wall:
                self.ser.seek(pos)  # not time for this frame yet.
                return
            self._processFrame(dataship, bus, can_id, data)

    #############################################
    ## Frame decoding
    def _processFrame(self, dataship: Dataship, bus, can_id, data):
        v = self.vehicleData
        now = time.time()

        if bus == PANDA_BUS_SERVER:
            if can_id == PANDA_ACK:
                if not v.connected:
                    print("tesla_wifi: connected to " + self.host)
                    shared.GrowlManager.add_message(self.name + ": connected to " + self.host)
                v.connected = True
                self.connect_refused = False
                self.last_frame_time = now
            elif can_id == PANDA_REFUSED and not self.connect_refused:
                self.connect_refused = True
                print("tesla_wifi: connection refused by " + self.host + " (is ScanMyTesla support enabled in the Commander app?)")
                shared.GrowlManager.add_message(self.name + ": connection refused by " + self.host)
            return

        if self.bus >= 0 and bus != self.bus:
            return
        if len(data) < 8:
            data = data + bytes(8 - len(data))

        self.last_frame_time = now
        if not v.connected:
            v.connected = True
            shared.GrowlManager.add_message(self.name + ": receiving CAN data")

        if can_id == ID_DI_speed:
            raw = sig(data, 12, 12)
            if raw != 4095:
                v.Speed = round((raw * 0.08 - 40) * KPH_TO_MPH, 2)
            ui = sig(data, 24, 8)
            if ui != 255:
                v.SpeedDisplayed = ui
            v.SpeedUnits = "kph" if sig(data, 32, 1) else "mph"

        elif can_id == ID_DAS_status:
            ap = sig(data, 0, 4)
            if ap == 15:
                v.AutopilotState = v.AutopilotStateName = v.AutopilotActive = None
            else:
                v.AutopilotState = ap
                v.AutopilotStateName = AUTOPILOT_STATES.get(ap, str(ap))
                v.AutopilotActive = ap in AUTOPILOT_ACTIVE_STATES
            left = sig(data, 4, 2)
            right = sig(data, 6, 2)
            v.BlindSpotLeft = None if left == 3 else left
            v.BlindSpotRight = None if right == 3 else right
            limit = sig(data, 8, 5)
            v.SpeedLimit = None if limit in (0, 31) else limit * 5
            fcw = sig(data, 22, 2)
            v.ForwardCollision = None if fcw == 3 else fcw == 1
            v.SideCollision = SIDE_COLLISION.get(sig(data, 32, 2))
            v.LaneDeparture = LANE_DEPARTURE.get(sig(data, 37, 3))
            hands = sig(data, 42, 4)
            v.AutopilotHandsOn = None if hands == 15 else HANDS_ON_STATES.get(hands, str(hands))
            v.AutoLaneChangeState = sig(data, 46, 5)

        elif can_id == ID_DI_systemStatus:
            gear = sig(data, 21, 3)
            v.Gear = GEARS.get(gear)
            brake = sig(data, 19, 2)
            v.BrakePedal = None if brake == 2 else brake == 1
            pedal = sig(data, 32, 8)
            v.AccelPedal = None if pedal == 255 else round(pedal * 0.4, 1)

        elif can_id == ID_DI_state:
            cruise = sig(data, 12, 3)
            v.CruiseState = CRUISE_STATES.get(cruise)
            v.CruiseActive = cruise in CRUISE_ACTIVE_STATES

        elif can_id == ID_DAS_control:
            setspeed = sig(data, 0, 12)
            if setspeed == 4095:  # 409.5 kph = SNA (no set speed)
                v.CruiseSetSpeed = None
            else:
                v.CruiseSetSpeed = round(setspeed * 0.1 * KPH_TO_MPH, 1)

        elif can_id == ID_SCCM_steeringAngle:
            if sig(data, 30, 2) == 1:  # validity: 1 = valid
                v.SteeringAngle = round(sig(data, 16, 14) * 0.1 - 819.2, 1)

        elif can_id == ID_UI_warning:
            v.BlinkerLeft = sig(data, 22, 1) == 1
            v.BlinkerRight = sig(data, 23, 1) == 1

        else:
            self.msg_unknown += 1
            return

        v.msg_count += 1
        v.msg_last = "%03X#%s" % (can_id, data.hex().upper())
        self.frame_count_window += 1
        if now - self.hz_window_start >= 1.0:
            v.hz = round(self.frame_count_window / (now - self.hz_window_start), 1)
            self.frame_count_window = 0
            self.hz_window_start = now

    #############################################
    ## data recorder: candump text log
    def startLog(self, dataship):
        if self.output_logFile is None:
            self.output_logFile, self.output_logFileName = Input.createLogFile(self, ".log")
            print("Creating log output: %s" % (self.output_logFileName))
        else:
            print("Already logging to: " + self.output_logFileName)


# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python
