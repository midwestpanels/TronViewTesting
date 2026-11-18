# TronView Text Data Points

Every value below can be typed into a **Text** or **Text Segments** object in the screen editor.
Wrap the data point in single curly braces. Anything outside the braces is shown as typed.

```
IAS {airData[0].IAS:kts} KT   ALT {airData[0].Alt%0.0f} FT
```

## Formatting

| Example | Result |
|---|---|
| `{airData[0].IAS}` | Raw value |
| `{airData[0].IAS%0.0f}` | Python format spec (0 decimals) |
| `{airData[0].IAS:0.1f}` | Same, using `:` (1 decimal) |
| `{airData[0].IAS:kts}` | Convert unit, default precision |
| `{airData[0].IAS:kts:1}` | Convert unit, 1 decimal place |
| `{airData[0].IAS:kts%0.1f}` | Convert unit, `%` precision style |

Unit conversions:

| Specifier | Converts | Default |
|---|---|---|
| `:kts` | mph → knots | 0 decimals |
| `:kph` | mph → km/h | 0 decimals |
| `:mph` | mph (no change) | 0 decimals |
| `:km` | miles → kilometers | 1 decimal |
| `:nm` | miles → nautical miles | 1 decimal |
| `:ft` | feet (no change) | 1 decimal |
| `:m` | feet → meters | 1 decimal |
| `:c` | °F → °C | 1 decimal |
| `:f` | °F (no change) | 1 decimal |
| `:10th` | Round to nearest 10 | 0 decimals |

## Notes

- `[0]` is the first source of that type, `[1]` the second, and so on. For example, `imuData[1]` is a second AHRS.
  The index only exists if an input for that type is configured, or the text shows `Err:list index out of range`.
- A value that hasn't been received yet is `None`. A conversion or number format on `None` shows an `Err:` message.
- Items ending in `()` are functions. Include the parentheses.
- Lists like `EGT` are read by index: `{engineData[0].EGT[0]}`.
- `{self}` dumps every top-level value. `{gpsData[0]}` dumps every value in that object. Both are handy for debugging.

---

## Air data — `airData[n]`

Speeds are in **mph**, altitudes in **feet**, temperature in **°F**.

| Data point | Description |
|---|---|
| `{airData[0].IAS}` | Indicated airspeed (mph) |
| `{airData[0].TAS}` | True airspeed (mph) |
| `{airData[0].Alt}` | Altitude (ft) |
| `{airData[0].Alt_agl}` | Altitude above ground level (ft) |
| `{airData[0].Alt_pres}` | Pressure altitude (ft) |
| `{airData[0].Alt_baro}` | Barometric altitude (ft) |
| `{airData[0].Alt_da}` | Density altitude (ft) |
| `{airData[0].AOA}` | Angle of attack (0–100 %) |
| `{airData[0].Baro}` | Altimeter setting (inHg) |
| `{airData[0].Baro_diff}` | Baro pressure difference (inHg) |
| `{airData[0].VSI}` | Vertical speed (ft/min) |
| `{airData[0].OAT}` | Outside air temperature (°F) |
| `{airData[0].Wind_speed}` | Wind speed (mph) |
| `{airData[0].Wind_dir}` | Wind direction (degrees) |
| `{airData[0].Wind_dir_corr}` | Wind direction relative to aircraft heading |
| `{airData[0].Mag_decl}` | Magnetic declination |
| `{airData[0].sys_time_string}` | System time string from the source |
| `{airData[0].inputSrcName}` | Name of the input feeding this data |
| `{airData[0].msg_count}` | Messages received |
| `{airData[0].msg_bad}` | Bad messages received |

## Attitude / IMU — `imuData[n]`

| Data point | Description |
|---|---|
| `{imuData[0].pitch}` | Pitch (degrees) |
| `{imuData[0].roll}` | Roll (degrees) |
| `{imuData[0].yaw}` | Yaw (degrees, −180 to 180) |
| `{imuData[0].mag_head}` | Magnetic heading (degrees) |
| `{imuData[0].turn_rate}` | Turn rate (tenths of a degree/sec) |
| `{imuData[0].slip_skid}` | Slip/skid (−99 to +99, −99 = full right) |
| `{imuData[0].vert_G}` | Vertical G load |
| `{imuData[0].cali_mag}` | Magnetometer calibration status |
| `{imuData[0].cali_accel}` | Accelerometer calibration status |
| `{imuData[0].cali_gyro}` | Gyro calibration status |
| `{imuData[0].home_pitch}` | Pitch "home" offset (camera/IMU zero) |
| `{imuData[0].home_roll}` | Roll "home" offset |
| `{imuData[0].home_yaw}` | Yaw "home" offset |
| `{imuData[0].org_pitch}` | Raw pitch before home offset |
| `{imuData[0].org_roll}` | Raw roll before home offset |
| `{imuData[0].org_yaw}` | Raw yaw before home offset |
| `{imuData[0].name}` | IMU name |
| `{imuData[0].id}` | IMU id |
| `{imuData[0].hz}` | Update rate (Hz) |
| `{imuData[0].inputSrcName}` | Name of the input feeding this data |
| `{imuData[0].msg_count}` | Messages received |
| `{imuData[0].msg_bad}` | Bad messages received |

## GPS — `gpsData[n]`

| Data point | Description |
|---|---|
| `{gpsData[0].Lat}` | Latitude (decimal degrees) |
| `{gpsData[0].Lon}` | Longitude (decimal degrees) |
| `{gpsData[0].Alt}` | GPS altitude (ft MSL) |
| `{gpsData[0].AltPressure}` | Pressure altitude, if the GPS reports it (ft) |
| `{gpsData[0].GndSpeed}` | Ground speed (mph) |
| `{gpsData[0].GndTrack}` | True ground track (degrees) |
| `{gpsData[0].Mag_Decl}` | Magnetic variation |
| `{gpsData[0].GPSTime_string}` | GPS time |
| `{gpsData[0].GPSDate_string}` | GPS date |
| `{gpsData[0].SatsTracked}` | Satellites tracked |
| `{gpsData[0].SatsVisible}` | Satellites visible |
| `{gpsData[0].GPSStatus}` | Fix status number (0–5) |
| `{gpsData[0].get_status_string()}` | Fix status text ("3D fix", "Acquiring", …) |
| `{gpsData[0].GPSWAAS}` | WAAS active (True/False) |
| `{gpsData[0].Accuracy}` | Accuracy (0 = none, 1 = 2D, 2 = 3D) |
| `{gpsData[0].EWVelDir}` / `{gpsData[0].EWVelmag}` | East/West velocity direction / m/s |
| `{gpsData[0].NSVelDir}` / `{gpsData[0].NSVelmag}` | North/South velocity direction / m/s |
| `{gpsData[0].VVelDir}` / `{gpsData[0].VVelmag}` | Vertical velocity direction / m/s |
| `{gpsData[0].Source}` | GPS source name |
| `{gpsData[0].inputSrcName}` | Name of the input feeding this data |
| `{gpsData[0].msg_count}` | Messages received |

## Engine — `engineData[n]`

| Data point | Description |
|---|---|
| `{engineData[0].RPM}` | RPM |
| `{engineData[0].ManPress}` | Manifold pressure |
| `{engineData[0].OilPress}` | Oil pressure (PSI) |
| `{engineData[0].OilPress2}` | Oil pressure #2 (PSI) |
| `{engineData[0].OilTemp}` | Oil temperature (°F) |
| `{engineData[0].OilTemp2}` | Oil temperature #2 (°F) |
| `{engineData[0].CoolantTemp}` | Coolant temperature (°F) |
| `{engineData[0].FuelFlow}` | Fuel flow (GPH) |
| `{engineData[0].FuelFlow2}` | Fuel flow #2 (GPH) |
| `{engineData[0].FuelPress}` | Fuel pressure (PSI) |
| `{engineData[0].EGT[0]}` … `{engineData[0].EGT[7]}` | EGT per cylinder (°F) |
| `{engineData[0].CHT[0]}` … `{engineData[0].CHT[7]}` | CHT per cylinder (°F) |
| `{engineData[0].NumberOfCylinders}` | Number of cylinders |
| `{engineData[0].volts1}` | Bus voltage 1 |
| `{engineData[0].volts2}` | Bus voltage 2 |
| `{engineData[0].amps}` | Amps |
| `{engineData[0].hobbs_time}` | Hobbs time |
| `{engineData[0].tach_time}` | Tach time |
| `{engineData[0].name}` | Engine name |

## Fuel — `fuelData[n]`

| Data point | Description |
|---|---|
| `{fuelData[0].FuelLevels[0]}` … `{fuelData[0].FuelLevels[3]}` | Tank levels |
| `{fuelData[0].FuelRemain}` | Fuel remaining |

## Navigation / Autopilot — `navData[n]`

| Data point | Description |
|---|---|
| `{navData[0].NavStatus}` | Nav status |
| `{navData[0].SourceDesc}` | Nav source description |
| `{navData[0].HSISource}` | HSI source |
| `{navData[0].VNAVSource}` | VNAV source |
| `{navData[0].HSINeedle}` | HSI needle / course |
| `{navData[0].HSIRoseHeading}` | HSI rose heading |
| `{navData[0].HSIHorzDev}` | HSI horizontal deviation |
| `{navData[0].HSIVertDev}` | HSI vertical deviation |
| `{navData[0].ILSDev}` | ILS deviation |
| `{navData[0].GSDev}` | Glideslope deviation |
| `{navData[0].GLSHoriz}` | GLS horizontal |
| `{navData[0].GLSVert}` | GLS vertical |
| `{navData[0].HeadBug}` | Heading bug |
| `{navData[0].AltBug}` | Altitude bug |
| `{navData[0].ASIBug}` | Airspeed bug |
| `{navData[0].VSBug}` | Vertical speed bug |
| `{navData[0].WPName}` | Active waypoint name |
| `{navData[0].WPDist}` | Distance to waypoint |
| `{navData[0].WPTrack}` | Track to waypoint |
| `{navData[0].WPLat}` | Waypoint latitude |
| `{navData[0].WPLon}` | Waypoint longitude |
| `{navData[0].AP}` | Autopilot on (1) / off (0) |
| `{navData[0].AP_RollForce}` / `AP_RollPos` / `AP_RollSlip` | Autopilot roll servo |
| `{navData[0].AP_PitchForce}` / `AP_PitchPos` / `AP_PitchSlip` | Autopilot pitch servo |
| `{navData[0].AP_YawForce}` / `AP_YawPos` / `AP_YawSlip` | Autopilot yaw servo |
| `{navData[0].XPDR_Code}` | Transponder squawk code |
| `{navData[0].XPDR_Status}` | Transponder status |
| `{navData[0].XPDR_Reply}` | Transponder reply |
| `{navData[0].XPDR_Ident}` | Transponder ident |

## Traffic — `targetData[n]`

| Data point | Description |
|---|---|
| `{targetData[0].count}` | Number of traffic targets |
| `{targetData[0].buoyCount}` | Number of buoy targets |
| `{targetData[0].lcl_time_string}` | Local time string |
| `{targetData[0].src_lat}` / `{targetData[0].src_lon}` | Own-ship position used for traffic |
| `{targetData[0].src_alt}` | Own-ship altitude used for traffic (ft) |
| `{targetData[0].src_gndspeed}` | Own-ship ground speed (mph) |
| `{targetData[0].src_gndtrack}` | Own-ship ground track |
| `{targetData[0].get_all_messages_as_text()}` | Recent text messages (Meshtastic) |
| `{targetData[0].meshtastic_node_name}` | Local Meshtastic node name |

Individual targets, by list position (`targets[0]`, `targets[1]`, …).
Use `getNearestTarget()` or `get_selected_target()` in place of `targets[0]` to show the nearest or selected target:

| Data point | Description |
|---|---|
| `{targetData[0].targets[0].callsign}` | Callsign |
| `{targetData[0].targets[0].flightNumber}` | Flight number |
| `{targetData[0].targets[0].address}` | ICAO address / Meshtastic node id |
| `{targetData[0].targets[0].get_cat_name()}` | Aircraft category ("Light", "Heavy", …) |
| `{targetData[0].targets[0].alt}` | Altitude (ft MSL) |
| `{targetData[0].targets[0].altDiff}` | Altitude difference from own-ship (ft) |
| `{targetData[0].targets[0].dist}` | Distance from own-ship (miles) |
| `{targetData[0].targets[0].brng}` | Bearing from own-ship (degrees) |
| `{targetData[0].targets[0].track}` | Track (degrees) |
| `{targetData[0].targets[0].speed}` | Speed (**already in knots**; don't add `:kts`) |
| `{targetData[0].targets[0].vspeed}` | Vertical speed (ft/min) |
| `{targetData[0].targets[0].lat}` / `{targetData[0].targets[0].lon}` | Position |
| `{targetData[0].targets[0].age}` | Seconds since last heard |
| `{targetData[0].getNearestTarget().callsign}` | Nearest target's callsign (within 15 miles) |
| `{targetData[0].get_selected_target().callsign}` | Selected target's callsign |

## Analog inputs — `analogData[n]`

| Data point | Description |
|---|---|
| `{analogData[0].Name}` | Analog input name |
| `{analogData[0].Data[0]}` … `{analogData[0].Data[7]}` | Channel values |
| `{analogData[0].Min}` / `{analogData[0].Max}` | Configured min / max |

## Vehicle (car) — `vehicleData[n]`

From the `tesla_wifi` input. Speeds are in **mph** (use `:kph` to convert). A value is `None` until the car sends it.

| Data point | Description |
|---|---|
| `{vehicleData[0].Speed}` | Vehicle speed (mph) |
| `{vehicleData[0].SpeedDisplayed}` | Speed exactly as the car's screen shows it, in `SpeedUnits` |
| `{vehicleData[0].SpeedUnits}` | `mph` or `kph` (the car's display setting) |
| `{vehicleData[0].Gear}` | `P`, `R`, `N`, `D` |
| `{vehicleData[0].AccelPedal}` | Accelerator pedal (0–100 %) |
| `{vehicleData[0].BrakePedal}` | `True` while braking |
| `{vehicleData[0].SteeringAngle}` | Steering wheel angle (deg) |
| `{vehicleData[0].BlinkerLeft}` / `{vehicleData[0].BlinkerRight}` | `True` while that turn signal is lit |
| `{vehicleData[0].AutopilotStateName}` | `DISABLED`, `UNAVAILABLE`, `AVAILABLE`, `ACTIVE_NOMINAL`, `ACTIVE_RESTRICTED`, `ACTIVE_NAV`, `ACTIVE_FSD`, `ABORTING`, `ABORTED`, `FAULT` |
| `{vehicleData[0].AutopilotState}` | Same as a number (3–6 = active) |
| `{vehicleData[0].AutopilotActive}` | `True` while Autosteer / Nav / FSD is steering |
| `{vehicleData[0].AutopilotHandsOn}` | Hands-on request: `NOT_REQD`, `REQD_DETECTED`, `REQD_NOT_DETECTED`, `REQD_VISUAL`, `REQD_CHIME_1`, … |
| `{vehicleData[0].CruiseState}` | `UNAVAILABLE`, `STANDBY`, `ENABLED`, `STANDSTILL`, `OVERRIDE`, `FAULT`, … |
| `{vehicleData[0].CruiseActive}` | `True` while cruise (TACC) is controlling speed |
| `{vehicleData[0].CruiseSetSpeed}` | Cruise set speed (mph) |
| `{vehicleData[0].SpeedLimit}` | Detected speed limit, in `SpeedUnits` |
| `{vehicleData[0].AutoLaneChangeState}` | Auto lane change state number |
| `{vehicleData[0].BlindSpotLeft}` / `{vehicleData[0].BlindSpotRight}` | 0 = clear, 1 = vehicle in blind spot, 2 = vehicle in blind spot and warning |
| `{vehicleData[0].LaneDeparture}` | `NONE`, `LEFT_WARNING`, `RIGHT_WARNING`, `LEFT_WARNING_SEVERE`, `RIGHT_WARNING_SEVERE` |
| `{vehicleData[0].SideCollision}` | `NONE`, `WARN_LEFT`, `WARN_RIGHT`, `WARN_LEFT_RIGHT` |
| `{vehicleData[0].ForwardCollision}` | `True` during a forward collision warning |
| `{vehicleData[0].connected}` | `True` while CAN data is arriving |
| `{vehicleData[0].hz}` | Decoded frames per second |

## System — `internalData`

Not a list, so there's no `[0]`.

| Data point | Description |
|---|---|
| `{internalData.Temp}` | CPU temperature |
| `{internalData.LoadAvg}` | System load average |
| `{internalData.MemFree}` | Free memory |
| `{internalData.OS}` / `{internalData.OSVer}` | Operating system / version |
| `{internalData.Hardware}` | Hardware |
| `{internalData.PythonVer}` | Python version |
| `{internalData.GraphicEngine2d}` / `{internalData.GraphicEngine2dVer}` | 2D graphics engine / version |
| `{internalData.GraphicEngine3d}` / `{internalData.GraphicEngine3dVer}` | 3D graphics engine / version |
| `{sys_time_string}` | System time string |
