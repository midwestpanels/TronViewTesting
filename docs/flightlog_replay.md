# Flight log replay (G3X / AXIS → G3X serial)

`util/flightlog_replay.py` turns a Garmin flight log into a live Garmin G3X serial text stream
so TronView (or anything else that reads G3X serial) can be tested with real flight data.

Supported logs:
- **G3X / G3X Touch** `data_log/log_YYYYMMDD_HHMMSS_XXX.csv`
- **AXIS** `aircraft-log-<tail>-<timestamp>.csv`

The logs are 1 Hz. The tool interpolates them to 10 Hz with a monotone cubic (PCHIP): it passes
exactly through every logged sample, is smooth between them, and never overshoots. Heading is
unwrapped across 360/000, and turn rate is derived from the smoothed heading. Bugs and the
baro setting are held (stepped), not interpolated. Gaps longer than `--max-gap` (3 s) are sent as
invalid (underscore) fields, the way a real G3X reports missing data.

| Sentence | Rate | Content |
|---|---|---|
| `=1` | 10 Hz | pitch, roll, heading, IAS, pressure alt, rate of turn, lateral/normal G, AOA, VS, OAT, altimeter |
| `=2` | 2 Hz | TAS, density alt, heading/altitude/airspeed/VS bugs |
| `@`  | 1 Hz | UTC date/time, lat/lon, fix, EPH, GPS alt, E/N/U velocity |
| `=7` | 1 Hz | height AGL, ground speed |

Engine data (`=3` / `=5`) is not generated yet.

## Local TronView via a virtual serial port (macOS / Linux)

```bash
python3 util/flightlog_replay.py ~/logs/log_20251114_090719_Y83.csv --virtual --start takeoff
```

This creates a pty and symlinks it at `/tmp/tronview_g3x`. In `config.cfg`:

```
[serial_g3x]
port = /tmp/tronview_g3x
```

Then run TronView with `-i serial_g3x` in a second terminal. The symlink is removed when the replay stops.

## Several readers at once

A serial port (real or virtual) delivers each byte to only one reader. If two scripts open the
same port, each gets roughly half the sentences. Instead, give each reader its own port. One
replay fans the identical stream out to all of them, in sync:

```bash
# three virtual ports: /tmp/tronview_g3x, /tmp/tronview_g3x_2, /tmp/tronview_g3x_3
python3 util/flightlog_replay.py log.csv --readers 3

# or name them, and mix in a real port and a capture file
python3 util/flightlog_replay.py log.csv --virtual /tmp/tronview --virtual /tmp/my_logger \
    --port /dev/tty.usbserial-XXXX --out run1.dat
```

You can also run separate replays side by side, for example a G3X log and an AXIS log, as long as
each uses its own `--virtual` path. A replay writes `LINK.pid` next to its link and refuses to
take over a link that another running replay owns. A link left behind by a crashed replay is
reclaimed automatically.

## Real serial port (e.g. to a Pi running TronView)

```bash
python3 util/flightlog_replay.py aircraft-log-N915MW.csv --port /dev/tty.usbserial-XXXX --baud 115200
```

On Windows, use `--port COM3`. For a virtual pair on Windows, install com0com and point TronView at the other end of the pair.

## Capture file for TronView playback

```bash
python3 util/flightlog_replay.py log.csv --out n915mw.dat
```

Copy the `.dat` into TronView's flight data recorder folder and play it with `-i serial_g3x -c n915mw.dat`.

## Useful options

- `--start` / `--end`: seconds into the log, local `HH:MM[:SS]`, or `takeoff` / `landing`
- `--speed 4`: play back 4× faster
- `--loop`: repeat forever
- `--interp linear`: plain linear interpolation instead of cubic
- `--smooth 3`: add a 3 s centered moving average before interpolating, for noisy logs
- `--aoa-min` / `--aoa-max`: Garmin logs a normalized AOA. The defaults map -0.5 → 0 % and 1.0 → 99 %.
  TronView's AOA bands are in percent (stall 89 % by default), so tune these per airframe.
- `--stdout`: print the sentences to the terminal
