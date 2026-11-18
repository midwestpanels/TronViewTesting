#!/usr/bin/env python3
"""
Simulated Tesla CAN data for testing the tesla_wifi input without a car.

Live mode acts like an Enhance Auto Commander / CANserver: a panda UDP server that
answers "ehllo"/"hello", honours frame filters and stops streaming when heartbeats stop.

    python3 util/tesla_panda_sim.py                 # serve on 127.0.0.1:1338
    python3 main.py --in1 tesla_wifi                # with [tesla_wifi] host = 127.0.0.1

Log mode writes a candump style file usable with -e / --playfile1:

    python3 util/tesla_panda_sim.py --write lib/inputs/_example_data/tesla_simulated_1.log --seconds 60

The drive is made up (not recorded from a car): accelerate to ~65 mph, engage
Autopilot, a car passes through each blind spot, a signalled lane change, a speed
limit change, Autopilot off, slow down and park. It loops every 60 seconds.
"""

import argparse
import math
import socket
import struct
import time

KPH_PER_MPH = 1.609344


def put(bits, start, length, value):
    value = int(value) & ((1 << length) - 1)
    return bits | (value << start)


def frame(bits):
    return bits.to_bytes(8, "little")


def state_at(t):
    """Vehicle state at t seconds into the 60 second loop."""
    t = t % 60.0
    s = {}
    if t < 3:
        speed, gear = 0, "P"
    elif t < 20:
        speed, gear = min(65, (t - 3) * 4.5), "D"
    elif t < 50:
        speed, gear = 65 + 2 * math.sin(t / 3), "D"
    elif t < 57:
        speed, gear = max(0, 65 - (t - 50) * 10), "D"
    else:
        speed, gear = 0, "P"
    s["speed"] = speed
    s["gear"] = gear
    s["accel"] = 35 if 3 <= t < 18 else (8 if 18 <= t < 22 else 0)
    s["brake"] = 50 <= t < 58
    # autopilot: available once moving, active 22-48, aborting briefly at 48
    if t < 5 or t >= 57:
        ap = 1  # UNAVAILABLE
    elif 22 <= t < 48:
        ap = 5 if t >= 36 else 3  # NAV after 36s
    elif 48 <= t < 49:
        ap = 8  # ABORTING
    else:
        ap = 2  # AVAILABLE
    s["ap"] = ap
    s["cruise"] = 2 if 20 <= t < 49 else (1 if gear == "D" else 0)
    s["set_speed_mph"] = 67 if 20 <= t < 49 else None
    s["hands_on"] = 3 if 40 <= t < 43 else (1 if 22 <= t < 48 else 0)
    # blind spots: car on the left 26-30 s, right 32-36 s (level 2 while signalling into it)
    s["bs_left"] = 1 if 26 <= t < 30 else 0
    s["bs_right"] = (2 if 34 <= t < 36 else 1) if 32 <= t < 36 else 0
    s["blinker_right"] = 34 <= t < 40 and int(t * 2.5) % 2 == 0
    s["blinker_left"] = False
    s["alc"] = 9 if 37 <= t < 40 else 0
    s["limit_mph"] = 55 if t < 30 else 65
    s["lane_dep"] = 0
    s["steer"] = 4 * math.sin(t / 2) + (25 if 37 <= t < 40 else 0)
    return s


def build_frames(t):
    """List of (can_id, 8 byte payload) for one 50 ms cycle."""
    s = state_at(t)
    out = []

    # 0x257 DI_speed
    b = put(0, 12, 12, (s["speed"] * KPH_PER_MPH + 40) / 0.08)
    b = put(b, 24, 8, round(s["speed"]))
    b = put(b, 32, 1, 0)  # mph
    out.append((0x257, frame(b)))

    # 0x39B DAS_status
    b = put(0, 0, 4, s["ap"])
    b = put(b, 4, 2, s["bs_left"])
    b = put(b, 6, 2, s["bs_right"])
    b = put(b, 8, 5, s["limit_mph"] / 5)
    b = put(b, 22, 2, 0)
    b = put(b, 32, 2, 0)
    b = put(b, 37, 3, s["lane_dep"])
    b = put(b, 42, 4, s["hands_on"])
    b = put(b, 46, 5, s["alc"])
    out.append((0x39B, frame(b)))

    # 0x118 DI_systemStatus
    b = put(0, 19, 2, 1 if s["brake"] else 0)
    b = put(b, 21, 3, {"P": 1, "R": 2, "N": 3, "D": 4}[s["gear"]])
    b = put(b, 32, 8, s["accel"] / 0.4)
    out.append((0x118, frame(b)))

    # 0x286 DI_state
    out.append((0x286, frame(put(0, 12, 3, s["cruise"]))))

    # 0x2B9 DAS_control
    sp = s["set_speed_mph"]
    out.append((0x2B9, frame(put(0, 0, 12, 4095 if sp is None else sp * KPH_PER_MPH / 0.1))))

    # 0x129 SCCM_steeringAngleSensor
    b = put(0, 16, 14, (s["steer"] + 819.2) / 0.1)
    b = put(b, 30, 2, 1)
    out.append((0x129, frame(b)))

    # 0x311 UI_warning
    b = put(0, 22, 1, s["blinker_left"])
    b = put(b, 23, 1, s["blinker_right"])
    out.append((0x311, frame(b)))
    return out


def panda_frame(bus, can_id, data):
    return struct.pack("<II", can_id << 21, (bus << 4) | len(data)) + data.ljust(8, b"\0")


def serve(host, port, bus):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    sock.setblocking(False)
    print("panda sim listening on %s:%d" % (host, port))
    clients = {}  # addr -> {"last": time, "filter": set or None}
    t0 = time.time()
    while True:
        while True:
            try:
                msg, addr = sock.recvfrom(1024)
            except BlockingIOError:
                break
            now = time.time()
            if msg in (b"hello", b"ehllo"):
                if addr not in clients:
                    print("client connected:", addr, msg.decode())
                    clients[addr] = {"filter": set() if msg == b"ehllo" else None}
                    sock.sendto(panda_frame(15, 6, b""), addr)
                clients[addr]["last"] = now
            elif msg == b"bye":
                print("client bye:", addr)
                clients.pop(addr, None)
            elif msg[:1] == b"\x0f" and addr in clients:
                f = clients[addr]["filter"]
                for i in range(1, len(msg) - 2, 3):
                    fb, fid = struct.unpack("!BH", msg[i:i + 3])
                    if f is not None:
                        f.add((fb, fid))
                print("client filter:", addr, sorted("%d:%03X" % x for x in (f or [])))
            elif msg[:1] == b"\x18" and addr in clients and clients[addr]["filter"] is not None:
                clients[addr]["filter"].clear()

        now = time.time()
        for addr in [a for a, c in clients.items() if now - c["last"] > 5]:
            print("client timed out:", addr)
            del clients[addr]

        frames = build_frames(now - t0)
        for addr, c in clients.items():
            payload = b"".join(panda_frame(bus, i, d) for i, d in frames
                               if c["filter"] is None or (bus, i) in c["filter"])
            if payload:
                sock.sendto(payload, addr)
        time.sleep(0.05)


def write_log(path, seconds, bus):
    t0 = 1763400000.0  # fixed start time so the file is reproducible
    with open(path, "w") as f:
        t = 0.0
        while t < seconds:
            for i, (can_id, data) in enumerate(build_frames(t)):
                f.write("(%.6f) can%d %03X#%s\n" % (t0 + t + i * 0.0005, bus, can_id, data.hex().upper()))
            t += 0.1  # 10 Hz keeps the example file small
    print("wrote", path)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1338)
    ap.add_argument("--bus", type=int, default=0, help="bus number to report frames on")
    ap.add_argument("--write", metavar="FILE", help="write a candump log instead of serving UDP")
    ap.add_argument("--seconds", type=float, default=60)
    args = ap.parse_args()
    if args.write:
        write_log(args.write, args.seconds, args.bus)
    else:
        serve(args.host, args.port, args.bus)
