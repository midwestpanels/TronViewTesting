#!/usr/bin/env python

# Ground vehicle data (cars). First used by the tesla_wifi input.
# 11/17/2026 A.O.

#############################################
## Class: VehicleData
## Speeds are in mph (same as the rest of the dataship) so the text unit conversions
## (:kph, :kts) work. Values that the vehicle has not sent yet (or reports as SNA) are None.
class VehicleData(object):
    def __init__(self):
        self.id = None
        self.name = None
        self.inputSrcName = None
        self.inputSrcNum = None
        self.connected = False      # True while the data source is sending us frames.

        # Motion
        self.Speed = None           # vehicle speed (mph)
        self.SpeedDisplayed = None  # speed exactly as shown on the car's screen (in SpeedUnits)
        self.SpeedUnits = None      # "mph" or "kph", the units the car's screen is set to
        self.Gear = None            # "P", "R", "N", "D"
        self.AccelPedal = None      # accelerator pedal position (0-100 %)
        self.BrakePedal = None      # True when the brake pedal is pressed
        self.SteeringAngle = None   # steering wheel angle (deg), sign as reported by the car
        self.BlinkerLeft = None     # True while the left turn signal is on
        self.BlinkerRight = None    # True while the right turn signal is on

        # Driver assist / Autopilot
        self.AutopilotState = None       # raw state number (see AutopilotStateName)
        self.AutopilotStateName = None   # "DISABLED", "UNAVAILABLE", "AVAILABLE", "ACTIVE_NOMINAL", "ACTIVE_RESTRICTED", "ACTIVE_NAV", "ACTIVE_FSD", "ABORTING", "ABORTED", "FAULT"
        self.AutopilotActive = None      # True while Autosteer / Nav / FSD is steering
        self.AutopilotHandsOn = None     # hands-on request name, "NOT_REQD" when no nag
        self.CruiseState = None          # "UNAVAILABLE", "STANDBY", "ENABLED", "STANDSTILL", "OVERRIDE", "FAULT", ...
        self.CruiseActive = None         # True while cruise (TACC) is controlling speed
        self.CruiseSetSpeed = None       # cruise set speed (mph)
        self.SpeedLimit = None           # detected road speed limit (in SpeedUnits)
        self.AutoLaneChangeState = None  # raw auto lane change state number

        # Warnings
        self.BlindSpotLeft = None        # 0 = clear, 1 = vehicle in blind spot, 2 = vehicle in blind spot + warning (turn signal on)
        self.BlindSpotRight = None       # same as BlindSpotLeft
        self.LaneDeparture = None        # "NONE", "LEFT_WARNING", "RIGHT_WARNING", "LEFT_WARNING_SEVERE", "RIGHT_WARNING_SEVERE"
        self.SideCollision = None        # "NONE", "WARN_LEFT", "WARN_RIGHT", "WARN_LEFT_RIGHT"
        self.ForwardCollision = None     # True during a forward collision warning

        self.msg_count = 0
        self.msg_bad = 0
        self.msg_last = None
        self.hz = 0


# vi: modeline tabstop=8 expandtab shiftwidth=4 softtabstop=4 syntax=python
