#!/usr/bin/env python3
"""
Flight log replay -> Garmin G3X serial text output.

Reads a Garmin G3X / G3X Touch data_log CSV (log_YYYYMMDD_HHMMSS_XXX.csv) or a Garmin AXIS
aircraft-log CSV, interpolates the 1 Hz recording up to 10 Hz with a smooth, non-overshooting
cubic (monotone PCHIP), and emits Garmin G3X serial text sentences that TronView's serial_g3x
input parses:

    =1  Attitude / Air Data       10 Hz   pitch, roll, hdg, IAS, press alt, ROT, lat/vert G,
                                          AOA, VS, OAT, altimeter setting
    =2  Attitude / Air Data set 2  2 Hz   TAS, density alt, heading/alt/airspeed/VS bugs
    @   GPS PVT                    1 Hz   date/time, lat/lon, fix, EPH, GPS alt, velocities
    =7  GPS Data                   1 Hz   height AGL, ground speed

Rates and sentence order match a real G3X capture (lib/inputs/_example_data/g3x_1.dat).
Format reference: G3X/G3X Touch Avionics Installation Manual 190-01115-01 Appendix C.

Output goes to one of:
    --port /dev/tty.usbserial-XXXX     a real serial port (needs pyserial), e.g. to a Pi
    --virtual [LINK]                   a virtual serial port (pty) on this machine; point
                                       TronView's [serial_g3x] port at LINK
                                       (default /tmp/tronview_g3x). macOS / Linux only.
    --out FILE                         a capture file TronView can play back with -c
    --stdout                           print the sentences (debugging)

Examples:
    python3 util/flightlog_replay.py log_20251114_090719_Y83.csv --virtual
    python3 util/flightlog_replay.py aircraft-log-N915MW.csv --port /dev/tty.usbserial-A10 --start takeoff --speed 2
    python3 util/flightlog_replay.py log.csv --out flightlog/n915mw_g3x.dat
"""

import argparse
import bisect
import csv
import datetime as dt
import math
import os
import signal
import sys
import time

KT_TO_MS = 0.514444
FT_TO_M = 0.3048

# --------------------------------------------------------------------------------------------
# Channels. kind: "smooth" = cubic/linear interpolated number, "angle" = same but unwrapped
# across 0/360, "hold" = step (last value held), used for bugs and status fields.
# --------------------------------------------------------------------------------------------
CHANNELS = {
    "pitch": "smooth", "roll": "smooth", "heading": "angle", "ias": "smooth",
    "palt": "smooth", "latacc": "smooth", "normacc": "smooth", "aoa": "smooth",
    "vs": "smooth", "oat": "smooth", "baro": "hold", "tas": "smooth", "dalt": "smooth",
    "sel_hdg": "hold", "sel_alt": "hold", "sel_ias": "hold", "sel_vs": "hold",
    "agl": "smooth", "gs": "smooth", "lat": "smooth", "lon": "smooth", "gpsalt": "smooth",
    "fix": "hold", "hfom": "hold", "vel_e": "smooth", "vel_n": "smooth", "vel_u": "smooth",
}
STRING_CHANNELS = {"fix"}

# Column names per log format. First match wins. G3X logs carry a long-name header row and a
# short-name row; both are listed so older/newer software versions resolve.
G3X_COLS = {
    "pitch": ["Pitch (deg)", "Pitch"],
    "roll": ["Roll (deg)", "Roll"],
    "heading": ["Magnetic Heading (deg)", "HDG"],
    "ias": ["Indicated Airspeed (kt)", "IAS"],
    "palt": ["Pressure Altitude (ft)", "AltP"],
    "latacc": ["Lateral Acceleration (G)", "LatAc"],
    "normacc": ["Normal Acceleration (G)", "NormAc"],
    "aoa": ["AOA"],
    "vs": ["Vertical Speed (ft/min)", "VSpd"],
    "oat": ["Outside Air Temp (deg C)", "OAT"],
    "baro": ["Baro Setting (inch Hg)", "Baro", "BaroA"],
    "tas": ["True Airspeed (kt)", "TAS"],
    "dalt": ["Density Altitude (ft)", "AltD"],
    "sel_hdg": ["Selected Heading (deg)", "SelHDG"],
    "sel_alt": ["Selected Altitude (ft)", "SelALT"],
    "sel_ias": ["Selected Airspeed (kt)", "SelIAS"],
    "sel_vs": ["Selected Vertical Speed (ft/min)", "SelVSpd"],
    "agl": ["Height Above Ground (ft)", "AGL"],
    "gs": ["GPS Ground Speed (kt)", "GndSpd"],
    "lat": ["Latitude (deg)", "Latitude"],
    "lon": ["Longitude (deg)", "Longitude"],
    "gpsalt": ["GPS Altitude (ft)", "AltGPS"],
    "fix": ["GPS Fix Status", "GPSfix"],
    "hfom": ["GPS HFOM (ft)", "HFOM"],
    "vel_e": ["GPS Velocity E (m/sec)", "GPSVelE"],
    "vel_n": ["GPS Velocity N (m/sec)", "GPSVelN"],
    "vel_u": ["GPS Velocity U (m/sec)", "GPSVelU"],
}
AXIS_COLS = {
    "pitch": ["attitudePitchAngle"],
    "roll": ["attitudeRollAngle"],
    "heading": ["heading"],
    "ias": ["indicatedAirspeed"],
    "palt": ["pressureAltitude"],
    "latacc": ["lateralYAcceleration"],
    "normacc": ["verticalZAcceleration"],
    "aoa": ["aoa"],
    "vs": ["verticalSpeed"],
    "oat": ["outsideAirTemperature"],
    "baro": ["selectedBarometricPressureSetting"],
    "tas": ["trueAirspeed"],
    "dalt": ["densityAltitude"],
    "sel_hdg": ["selectedHeading"],
    "sel_alt": ["selectedAltitude"],
    "sel_ias": ["selectedAirspeed"],
    "sel_vs": ["selectedVerticalSpeed"],
    "agl": ["heightAboveGround"],
    "gs": ["groundSpeed"],
    "lat": ["latitudePosition"],
    "lon": ["longitudePosition"],
    "gpsalt": ["gpsWgs84Altitude"],
    "fix": ["gpsFix"],
    "hfom": ["gpsHorizontalFigureOfMerit"],
    "vel_e": ["gpsVelocityE"],
    "vel_n": ["gpsVelocityN"],
    "vel_u": ["gpsVelocityU"],
}


def _num(s):
    if s is None:
        return None
    s = s.strip()
    if s == "" or s.startswith("_"):
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def _parse_offset(s):
    """'-05:00' -> timedelta(hours=-5)."""
    s = (s or "").strip()
    if not s:
        return dt.timedelta(0)
    sign = -1 if s[0] == "-" else 1
    hh, _, mm = s.lstrip("+-").partition(":")
    return sign * dt.timedelta(hours=int(hh or 0), minutes=int(mm or 0))


class FlightLog:
    """Loaded log: utc0 (datetime), t (seconds from start), cols {channel: [values]},
    local0 (datetime, local time of first row), fmt, ident."""

    def __init__(self, path):
        self.path = path
        with open(path, newline="", encoding="utf-8-sig", errors="replace") as f:
            rows = list(csv.reader(f))
        if not rows:
            raise ValueError("empty file")
        first = ",".join(rows[0])
        if first.startswith("#airframe_info"):
            self._load_g3x(rows)
        elif first.startswith("#") and any("attitudePitchAngle" in r for r in rows[:5]):
            self._load_axis(rows)
        else:
            raise ValueError("unrecognised log format (expected G3X data_log or AXIS aircraft-log CSV)")

    def _resolve(self, header_rows, colmap):
        idx = {}
        for ch, names in colmap.items():
            for hdr in header_rows:
                pos = {h.strip(): i for i, h in enumerate(hdr) if h.strip()}
                hit = next((pos[n] for n in names if n in pos), None)
                if hit is not None:
                    idx[ch] = hit
                    break
        return idx

    def _store(self, times, idx, data_rows):
        self.t = []
        self.cols = {ch: [] for ch in CHANNELS}
        last = None
        for when, r in zip(times, data_rows):
            if when is None:
                continue
            t = (when - self.utc0).total_seconds()
            if last is not None and t <= last:
                continue  # duplicate / out of order timestamp
            last = t
            self.t.append(t)
            for ch in CHANNELS:
                i = idx.get(ch)
                raw = r[i] if i is not None and i < len(r) else None
                if ch in STRING_CHANNELS:
                    self.cols[ch].append(raw.strip() if raw and raw.strip() else None)
                else:
                    self.cols[ch].append(_num(raw))
        self.missing = sorted(ch for ch in CHANNELS if ch not in idx)

    def _load_g3x(self, rows):
        self.fmt = "G3X"
        meta = dict(kv.split("=", 1) for kv in rows[0][1:] if "=" in kv)
        self.ident = meta.get("aircraft_ident", "").strip('"')
        long_hdr, short_hdr = rows[1], rows[2] if len(rows) > 2 else []
        idx = self._resolve([long_hdr, short_hdr], G3X_COLS)
        pos = {h.strip(): i for i, h in enumerate(long_hdr)}
        spos = {h.strip(): i for i, h in enumerate(short_hdr)}
        di = pos.get("Date (yyyy-mm-dd)", spos.get("Lcl Date"))
        ti = pos.get("Time (hh:mm:ss)", spos.get("Lcl Time"))
        oi = pos.get("UTC Offset (hh:mm)", spos.get("UTCOfst"))
        if di is None or ti is None:
            raise ValueError("G3X log has no date/time columns")
        data = [r for r in rows[3:] if len(r) > max(di, ti)]
        times, locals_ = [], []
        for r in data:
            try:
                local = dt.datetime.strptime(r[di].strip() + " " + r[ti].strip(), "%Y-%m-%d %H:%M:%S")
                off = _parse_offset(r[oi]) if oi is not None else dt.timedelta(0)
                times.append(local - off)
                locals_.append(local)
            except (ValueError, IndexError):
                times.append(None)
                locals_.append(None)
        good = [i for i, x in enumerate(times) if x is not None]
        if not good:
            raise ValueError("no timestamped rows in G3X log")
        self.utc0 = times[good[0]]
        self.local_offset = locals_[good[0]] - self.utc0
        self._store(times, idx, data)

    def _load_axis(self, rows):
        self.fmt = "AXIS"
        self.ident = ""
        hrow = next(i for i, r in enumerate(rows[:5]) if "attitudePitchAngle" in r)
        hdr = rows[hrow]
        idx = self._resolve([hdr], AXIS_COLS)
        pos = {h: i for i, h in enumerate(hdr)}
        tsi = pos["timestamp"]
        oi = pos.get("utcOffset")
        data = [r for r in rows[hrow + 1:] if len(r) > tsi]
        times = []
        for r in data:
            try:
                times.append(dt.datetime.fromisoformat(r[tsi].strip().replace("Z", "+00:00")).replace(tzinfo=None))
            except ValueError:
                times.append(None)
        good = [i for i, x in enumerate(times) if x is not None]
        if not good:
            raise ValueError("no timestamped rows in AXIS log")
        self.utc0 = times[good[0]]
        self.local_offset = _parse_offset(data[good[0]][oi]) if oi is not None else dt.timedelta(0)
        self._store(times, idx, data)

    @property
    def duration(self):
        return self.t[-1] if self.t else 0.0


# --------------------------------------------------------------------------------------------
# Interpolation
# --------------------------------------------------------------------------------------------
def _moving_average(x, y, window):
    """Centered time-window average (pre-smoothing for noisy channels)."""
    if window <= 0 or len(x) < 3:
        return y
    half = window / 2.0
    out, lo, hi, acc = [], 0, 0, 0.0
    for i, xi in enumerate(x):
        while hi < len(x) and x[hi] <= xi + half:
            acc += y[hi]
            hi += 1
        while x[lo] < xi - half:
            acc -= y[lo]
            lo += 1
        out.append(acc / (hi - lo))
    return out


def _pchip_slopes(x, y):
    """Fritsch-Carlson monotone cubic tangents: smooth (C1) but never overshoots the samples."""
    n = len(x)
    if n < 2:
        return [0.0] * n
    h = [x[i + 1] - x[i] for i in range(n - 1)]
    d = [(y[i + 1] - y[i]) / h[i] for i in range(n - 1)]
    m = [0.0] * n
    m[0], m[-1] = d[0], d[-1]
    for k in range(1, n - 1):
        if d[k - 1] * d[k] <= 0:
            m[k] = 0.0
        else:
            w1 = 2 * h[k] + h[k - 1]
            w2 = h[k] + 2 * h[k - 1]
            m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k])
    return m


class Track:
    """One channel. value(t) -> (value, rate_per_sec) or (None, None) when there's no data
    within max_gap of t (so TronView sees the field as invalid, like a real G3X)."""

    def __init__(self, times, values, kind, method, max_gap, smooth):
        pts = [(t, v) for t, v in zip(times, values) if v is not None]
        self.kind = kind
        self.max_gap = max_gap
        self.x = [p[0] for p in pts]
        y = [p[1] for p in pts]
        if kind == "hold":
            self.y = y
            return
        if kind == "angle" and y:
            un = [y[0]]
            for v in y[1:]:
                dv = (v - un[-1] + 180.0) % 360.0 - 180.0
                un.append(un[-1] + dv)
            y = un
        # Split into contiguous runs so tangents never reach across a data gap.
        self.runs = []
        start = 0
        for i in range(1, len(self.x) + 1):
            if i == len(self.x) or self.x[i] - self.x[i - 1] > max_gap:
                rx, ry = self.x[start:i], y[start:i]
                ry = _moving_average(rx, ry, smooth)
                rm = _pchip_slopes(rx, ry) if method == "cubic" else None
                self.runs.append((rx, ry, rm))
                start = i
        self.run_starts = [r[0][0] for r in self.runs]

    def value(self, t):
        if not self.x:
            return None, None
        if self.kind == "hold":
            i = bisect.bisect_right(self.x, t) - 1
            if i < 0 or t - self.x[i] > self.max_gap:
                return None, None
            return self.y[i], 0.0
        r = bisect.bisect_right(self.run_starts, t) - 1
        if r < 0:
            return None, None
        x, y, m = self.runs[r]
        if t > x[-1]:
            return None, None
        if len(x) == 1:
            return self._wrap(y[0]), 0.0
        k = min(bisect.bisect_right(x, t) - 1, len(x) - 2)
        h = x[k + 1] - x[k]
        s = (t - x[k]) / h
        if m is None:
            return self._wrap(y[k] + s * (y[k + 1] - y[k])), (y[k + 1] - y[k]) / h
        s2, s3 = s * s, s * s * s
        v = ((2 * s3 - 3 * s2 + 1) * y[k] + (s3 - 2 * s2 + s) * h * m[k]
             + (-2 * s3 + 3 * s2) * y[k + 1] + (s3 - s2) * h * m[k + 1])
        dv = ((6 * s2 - 6 * s) / h * y[k] + (3 * s2 - 4 * s + 1) * m[k]
              + (-6 * s2 + 6 * s) / h * y[k + 1] + (3 * s2 - 2 * s) * m[k + 1])
        return self._wrap(v), dv

    def _wrap(self, v):
        return v % 360.0 if self.kind == "angle" else v


# --------------------------------------------------------------------------------------------
# G3X sentence encoding (Appendix C). Field = sign + digits, or underscores when invalid.
# --------------------------------------------------------------------------------------------
def fld(v, digits, scale=1.0, signed=False, lo=None):
    width = digits + (1 if signed else 0)
    if v is None:
        return "_" * width
    n = int(round(v * scale))
    top = 10 ** digits - 1
    if lo is not None:
        n = max(n, lo)
    n = max(-top, min(top, n))
    if signed:
        return ("+" if n >= 0 else "-") + str(abs(n)).zfill(digits)
    return str(max(0, n)).zfill(digits)


def with_checksum(body):
    return body + "%02X" % (sum(body.encode("ascii")) & 0xFF) + "\r\n"


def hms_cs(utc):
    return utc.strftime("%H%M%S") + "%02d" % (utc.microsecond // 10000)


def sentence_attitude(utc, v, aoa_pct):
    hdg = None if v["heading"] is None else int(round(v["heading"])) % 360
    altset = None if v["baro"] is None else (v["baro"] - 27.5)
    return with_checksum(
        "=11" + hms_cs(utc)
        + fld(v["pitch"], 3, 10, True)
        + fld(v["roll"], 4, 10, True)
        + (("%03d" % hdg) if hdg is not None else "___")
        + fld(v["ias"], 4, 10)
        + fld(v["palt"], 5, 1, True)
        + fld(v["rot"], 3, 10, True)
        + fld(v["latacc"], 2, 100, True)
        + fld(v["normacc"], 2, 10, True)
        + fld(aoa_pct, 2)
        + fld(v["vs"], 3, 0.1, True)
        + fld(v["oat"], 2, 1, True)
        + fld(altset, 3, 100)
    )


def sentence_attitude2(utc, v):
    hsel = None if v["sel_hdg"] is None else int(round(v["sel_hdg"])) % 360
    return with_checksum(
        "=21" + hms_cs(utc)
        + fld(v["tas"], 4, 10)
        + fld(v["dalt"], 5, 1, True)
        + (("%03d" % hsel) if hsel is not None else "___")
        + fld(v["sel_alt"], 5, 1, True)
        + fld(v["sel_ias"], 4, 10)
        + fld(v["sel_vs"], 3, 0.1, True)
    )


def sentence_gpsdata(utc, v):
    agl = None if v["agl"] is None else max(0.0, v["agl"])
    return with_checksum("=71" + hms_cs(utc) + fld(agl, 3, 0.01) + fld(v["gs"], 4, 10))


FIX_CODES = {"3DDIFF": "D", "2DDIFF": "d", "3D": "G", "3D-": "G", "2D": "g", "SIM": "S"}


def _latlon(val, deg_digits, pos, neg):
    a = abs(val)
    deg = int(a)
    mins = int(round((a - deg) * 60000))
    if mins >= 60000:
        deg, mins = deg + 1, 0
    return (pos if val >= 0 else neg) + str(deg).zfill(deg_digits) + str(mins).zfill(5)


def sentence_pvt(utc, v):
    fix = FIX_CODES.get((v["fix"] or "").upper().replace(" ", ""), "_")
    have_pos = fix != "_" and v["lat"] is not None and v["lon"] is not None
    if have_pos:
        pos = _latlon(v["lat"], 2, "N", "S") + _latlon(v["lon"], 3, "E", "W")
    else:
        pos = "_" * 8 + "_" * 9
        fix = "_"
    eph = None if v["hfom"] is None else v["hfom"] * FT_TO_M
    alt_m = None if v["gpsalt"] is None else v["gpsalt"] * FT_TO_M

    def vel(x, pos_c, neg_c, scale):
        if x is None or not have_pos:
            return "_" * 5
        return (pos_c if x >= 0 else neg_c) + fld(abs(x), 4, scale)

    return (
        "@" + utc.strftime("%y%m%d%H%M%S") + pos + fix
        + (fld(eph, 3) if have_pos else "___")
        + (fld(alt_m, 5, 1, True) if have_pos else "_" * 6)
        + vel(v["vel_e"], "E", "W", 10)
        + vel(v["vel_n"], "N", "S", 10)
        + vel(v["vel_u"], "U", "D", 100)
        + "\r\n"
    )


# --------------------------------------------------------------------------------------------
# Output sinks
# --------------------------------------------------------------------------------------------
class SerialSink:
    realtime = True

    def __init__(self, port, baud):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is required for --port (pip install pyserial)")
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=0, write_timeout=0)
        self.desc = "serial port %s @ %d baud" % (port, baud)
        if baud < 38400:
            print("warning: %d baud is below the G3X's 115200; 10 Hz output may not fit" % baud, file=sys.stderr)

    def write(self, data):
        try:
            self.ser.write(data)
        except Exception as e:  # write timeout = buffer full; drop like a real UART would
            if "timeout" not in str(e).lower():
                raise

    def close(self):
        self.ser.close()


class VirtualSink:
    """pty pair. TronView opens the slave side (or the symlink) as its serial port."""
    realtime = True

    def __init__(self, link):
        import pty, tty, termios
        self.termios = termios
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)  # no echo / newline translation, like a real UART
        tty.setraw(self.master)
        os.set_blocking(self.master, False)
        self.name = os.ttyname(self.slave)
        self.link = None
        if link:
            if os.path.lexists(link):
                if not os.path.islink(link):
                    sys.exit("%s exists and is not a symlink; pick another --virtual path" % link)
                os.unlink(link)
            os.symlink(self.name, link)
            self.link = link
        self.desc = "virtual serial port %s%s" % (self.name, (" (linked as %s)" % link) if link else "")
        self.dropped = 0

    def write(self, data):
        try:
            os.write(self.master, data)
        except BlockingIOError:
            # Nobody is reading (TronView not running). Discard the stale backlog so a reader
            # that connects later gets current data, then retry once.
            self.termios.tcflush(self.slave, self.termios.TCIFLUSH)
            try:
                os.write(self.master, data)
            except BlockingIOError:
                self.dropped += 1

    def close(self):
        if self.link and os.path.islink(self.link):
            os.unlink(self.link)
        os.close(self.master)
        os.close(self.slave)


class FileSink:
    realtime = False

    def __init__(self, path):
        self.f = open(path, "wb")
        self.desc = "file " + path

    def write(self, data):
        self.f.write(data)

    def close(self):
        self.f.close()


class StdoutSink:
    realtime = True
    desc = "stdout"

    def write(self, data):
        sys.stdout.write(data.decode("ascii").replace("\r\n", "\n"))
        sys.stdout.flush()

    def close(self):
        pass


# --------------------------------------------------------------------------------------------
# Replay
# --------------------------------------------------------------------------------------------
def parse_when(s, log, tracks):
    """--start/--end: seconds into log, 'HH:MM:SS' local time of day, or 'takeoff'/'landing'."""
    if s is None:
        return None
    s = s.strip().lower()
    if s in ("takeoff", "landing"):
        ias = log.cols["ias"]
        fast = [i for i, v in enumerate(ias) if v is not None and v >= 45]
        if not fast:
            sys.exit("no airborne section found (IAS never above 45 kt)")
        if s == "takeoff":
            return max(0.0, log.t[fast[0]] - 30)
        return min(log.duration, log.t[fast[-1]] + 30)
    if ":" in s:
        parts = [int(p) for p in s.split(":")]
        while len(parts) < 3:
            parts.append(0)
        local0 = log.utc0 + log.local_offset
        target = local0.replace(hour=parts[0], minute=parts[1], second=parts[2], microsecond=0)
        if target < local0:
            target += dt.timedelta(days=1)
        return (target - local0).total_seconds()
    return float(s)


def build_tracks(log, args):
    tracks = {}
    for ch, kind in CHANNELS.items():
        smooth = args.smooth if kind != "hold" else 0
        tracks[ch] = Track(log.t, log.cols[ch], kind, args.interp, args.max_gap, smooth)
    return tracks


def sample(tracks, t):
    v = {}
    for ch, tr in tracks.items():
        v[ch], rate = tr.value(t)
        if ch == "heading":
            v["rot"] = rate  # deg/sec, from the smoothed heading curve
    if v["heading"] is None:
        v["rot"] = None
    return v


def aoa_to_pct(aoa, lo, hi):
    if aoa is None:
        return None
    return max(0.0, min(99.0, (aoa - lo) / (hi - lo) * 99.0))


def replay(log, tracks, sink, args, t_start, t_end):
    step = 1.0 / args.rate
    att2_every = max(1, int(round(args.rate / 2.0)))  # =2 at ~2 Hz
    realtime = sink.realtime and not args.fast
    status = sys.stderr.isatty() and not isinstance(sink, StdoutSink)
    passes = 0
    while True:
        n_ticks = int(math.floor((t_end - t_start) / step)) + 1
        wall0 = time.monotonic()
        last_status = -1
        for k in range(n_ticks):
            t = t_start + k * step
            if realtime:
                delay = wall0 + (k * step) / args.speed - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                elif delay < -2.0:
                    wall0 = time.monotonic() - (k * step) / args.speed  # fell behind; resync
            utc = log.utc0 + dt.timedelta(seconds=t)
            v = sample(tracks, t)
            out = sentence_attitude(utc, v, aoa_to_pct(v["aoa"], args.aoa_min, args.aoa_max))
            if k % att2_every == 0:
                out += sentence_attitude2(utc, v)
            # 1 Hz sentences on whole log seconds
            if int(math.floor(t + 1e-6)) != int(math.floor(t - step + 1e-6)) or k == 0:
                out += sentence_pvt(utc, v) + sentence_gpsdata(utc, v)
            sink.write(out.encode("ascii"))

            if status and int(t) != last_status:
                last_status = int(t)
                local = utc + log.local_offset

                def f(x, fmt):
                    return "---" if x is None else fmt % x
                sys.stderr.write(
                    "\r%s  %5.1f%%  IAS %s  ALT %s  HDG %s  PIT %s  ROL %s  VS %s   "
                    % (local.strftime("%H:%M:%S"), 100.0 * (t - t_start) / max(1e-6, t_end - t_start),
                       f(v["ias"], "%5.1f"), f(v["palt"], "%6.0f"), f(v["heading"], "%3.0f"),
                       f(v["pitch"], "%5.1f"), f(v["roll"], "%6.1f"), f(v["vs"], "%5.0f")))
                sys.stderr.flush()
        passes += 1
        if not args.loop:
            break
    if status:
        sys.stderr.write("\n")


def main():
    ap = argparse.ArgumentParser(
        description="Replay a Garmin G3X or AXIS flight log as G3X serial output for TronView.",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__.split("Examples:")[1])
    ap.add_argument("log", help="G3X data_log CSV or AXIS aircraft-log CSV")
    dst = ap.add_mutually_exclusive_group(required=True)
    dst.add_argument("--port", help="real serial port to write to (e.g. /dev/tty.usbserial-XXXX, COM3)")
    dst.add_argument("--virtual", nargs="?", const="/tmp/tronview_g3x", metavar="LINK",
                     help="create a virtual serial port (pty) and symlink it at LINK "
                          "(default /tmp/tronview_g3x). Set [serial_g3x] port = LINK in config.cfg")
    dst.add_argument("--out", metavar="FILE", help="write a G3X capture file (for TronView -c playback)")
    dst.add_argument("--stdout", action="store_true", help="print sentences to the terminal")
    ap.add_argument("--baud", type=int, default=115200, help="baud for --port (default 115200)")
    ap.add_argument("--rate", type=float, default=10.0, help="=1 sentence rate in Hz (default 10)")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier (default 1.0)")
    ap.add_argument("--fast", action="store_true", help="don't pace output in real time")
    ap.add_argument("--start", help="where to start: seconds into log, local HH:MM[:SS], or 'takeoff'")
    ap.add_argument("--end", help="where to stop: seconds into log, local HH:MM[:SS], or 'landing'")
    ap.add_argument("--loop", action="store_true", help="repeat forever")
    ap.add_argument("--interp", choices=["cubic", "linear"], default="cubic",
                    help="interpolation between 1 Hz samples (default cubic: smooth, no overshoot)")
    ap.add_argument("--smooth", type=float, default=0.0, metavar="SEC",
                    help="extra centered moving-average window in seconds applied before "
                         "interpolation, e.g. 3 to calm noisy logs (default 0 = off)")
    ap.add_argument("--max-gap", type=float, default=3.0, metavar="SEC",
                    help="don't interpolate across gaps longer than this; field goes invalid (default 3)")
    ap.add_argument("--aoa-min", type=float, default=-0.5,
                    help="logged (normalized) AOA that maps to 0%% (default -0.5)")
    ap.add_argument("--aoa-max", type=float, default=1.0,
                    help="logged (normalized) AOA that maps to 99%% (default 1.0)")
    args = ap.parse_args()
    if args.rate <= 0 or args.speed <= 0:
        ap.error("--rate and --speed must be positive")
    if args.aoa_max <= args.aoa_min:
        ap.error("--aoa-max must be greater than --aoa-min")

    try:
        log = FlightLog(args.log)
    except (OSError, ValueError, StopIteration) as e:
        sys.exit("can't read %s: %s" % (args.log, e))
    if len(log.t) < 2:
        sys.exit("log has fewer than 2 usable rows")
    tracks = build_tracks(log, args)
    t_start = parse_when(args.start, log, tracks) or 0.0
    t_end = parse_when(args.end, log, tracks)
    t_end = log.duration if t_end is None else min(t_end, log.duration)
    t_start = max(0.0, min(t_start, t_end))

    local0 = log.utc0 + log.local_offset
    print("%s log %s: %d rows, %s local, %.0f min. Replaying %s -> %s at %gx, %g Hz"
          % (log.fmt, (log.ident + " ") if log.ident else "", len(log.t), local0.strftime("%Y-%m-%d %H:%M:%S"),
             log.duration / 60, (local0 + dt.timedelta(seconds=t_start)).strftime("%H:%M:%S"),
             (local0 + dt.timedelta(seconds=t_end)).strftime("%H:%M:%S"), args.speed, args.rate),
          file=sys.stderr)
    if log.missing:
        print("not in this log (sent as invalid): " + ", ".join(log.missing), file=sys.stderr)

    if args.port:
        sink = SerialSink(args.port, args.baud)
    elif args.virtual:
        if os.name == "nt":
            sys.exit("--virtual needs macOS/Linux. On Windows use a com0com pair with --port.")
        sink = VirtualSink(args.virtual)
    elif args.out:
        sink = FileSink(args.out)
    else:
        sink = StdoutSink()
    print("output: " + sink.desc, file=sys.stderr)
    if isinstance(sink, VirtualSink):
        print("TronView: set  [serial_g3x] port = %s  in config.cfg, then run with -i serial_g3x"
              % (sink.link or sink.name), file=sys.stderr)
    if isinstance(sink, FileSink) and args.loop:
        sys.exit("--loop makes no sense with --out")

    def _stop(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _stop)
    try:
        replay(log, tracks, sink, args, t_start, t_end)
    except KeyboardInterrupt:
        sys.stderr.write("\nstopped\n")
    finally:
        sink.close()


if __name__ == "__main__":
    main()
