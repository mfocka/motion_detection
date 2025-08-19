# Motion Detection Library — Orientation Change Detection with MotionFX

## Overview

Robust, long‑running motion/orientation detection for an outdoor, mostly static ISM330DHCX sensor using ST MotionFX. The goal is to detect abrupt orientation changes (azimuth/yaw and altitude/pitch) within seconds, while rejecting drift and wind‑induced oscillations, and to persist calibration and the reference orientation across power cycles.

Key capabilities:
- Abrupt change detection with validation and hysteresis
- WDS device orientation support via a fixed frame transform
- Drift and vibration rejection (filtering + time series heuristics)
- EEPROM persistence (gbias, thresholds, validation windows, reference quaternion)
- Detailed console logging for analysis and field debug

Relevant spec: UM2220 — ST MotionFX Sensor Fusion Library (see References).

---

## System Description

- Hardware: ISM330DHCX (accelerometer + gyroscope)
- Placement: Outdoor, on a 1 m pole; device axes are in WDS orientation (West x, Down y, South z)
- Sampling rate: 104 Hz (accelerometer and gyroscope)
- Environment: Mostly static; wind and occasional abrupt rotations possible

---

## Detection Requirements

- Azimuth threshold: 10°
- Altitude threshold: 5°
- Condition: Report an event if orientation exceeds thresholds within a few seconds and then remains stable for at least n minutes (typically > 60 min)
- Ignore: Slow drift (bias, long‑term azimuth instability) and small, short oscillations due to wind/vibrations

---

## Coordinate Frames and WDS Conversion

MotionFX outputs orientation in its internal board frame. Our device uses WDS axes:
- x_device = West = −East
- y_device = Down = Down
- z_device = South = −North

If MotionFX is configured for NED (x=N, y=E, z=D), the matrix that maps device‑frame vectors to NED is:

```
R_ned_from_wds = [  0  0 -1 ;  // N from WDS: z_device = South → −N
                    -1  0  0 ;  // E from WDS: x_device = West → −E
                     0  1  0 ]  // D from WDS: y_device = Down →  D
```

Use a fixed quaternion `q_ned_from_wds` derived from `R_ned_from_wds` to convert IMU samples and/or fused orientation to a consistent global frame. Apply the transform in a single place to avoid double rotations. For quaternions, compute the delta to the reference as:

```
q_delta = q_reference^{-1} ⊗ q_current
```

Extract yaw (azimuth) and pitch (altitude) from `q_delta`. Roll is ignored for event decisions but can be logged.

---

## Algorithm (Drift‑Resistant Orientation Change)

1) Sensor fusion
- Feed 104 Hz accel + gyro to MotionFX. Persist and restore gyro bias (gbias).

2) Reference orientation
- Maintain a persistent `q_reference`. On first install or after explicit recalibration, set `q_reference = q_current` once stable.

3) Delta orientation
- Compute `q_delta = inv(q_reference) ⊗ q_current` and convert to yaw/pitch (`azimuth`, `altitude`).

4) Filtering and detection
- Low‑pass filter (LPF) on yaw/pitch to remove sensor noise.
- High‑pass/derivative gate to detect rapid changes (suppresses slow drift).
- Peak/hold: if either |Δazimuth| ≥ 10° or |Δaltitude| ≥ 5° within T_detect (e.g., 2 s), enter Validation.

5) Validation and hysteresis
- Require stability for T_validate (e.g., 5–10 s): variance of yaw/pitch below small thresholds, gyro magnitude below quiet limit.
- If validated, emit Event immediately. Start Hold state to ensure the new pose is not transient.
- Hold: require continuous stability for T_hold = n minutes/hours before committing `q_reference = q_current` and persisting to EEPROM. If instability resumes, revert to Monitoring.

6) Drift defenses
- HPF/derivative gate blocks very low‑frequency motion (drift).
- Azimuth bounding: because Ax and Ay are never both zero (sensor is tilted), compute yaw with gravity‑constrained projection to reduce heading drift.
- Periodically re‑level small pitch drift only when gyro is quiet and no event is pending.

---

## State Machine

States
- Init → Calibration → Idle → Monitoring → Validation → Event → HoldCommit → Monitoring
- Error is reachable from any state.

Transitions (summary)
- Init: load EEPROM (gbias, thresholds, windows, `q_reference`), configure MotionFX
- Calibration: estimate gbias (stationary), capture initial `q_reference`
- Idle: low activity; escalate to Monitoring on movement or timer
- Monitoring: compute filtered yaw/pitch deltas; if threshold within T_detect → Validation
- Validation: require stability for T_validate; pass → Event, fail → Monitoring
- Event: log once; begin HoldCommit timer
- HoldCommit: require stability for T_hold; pass → persist `q_reference`, return to Monitoring; fail → Monitoring
- Error: log, wait for reset or recalibrate

---

## Filtering and Time‑Series Heuristics

- LPF: first‑order IIR on yaw/pitch (fc ≈ 1–2 Hz) to reduce noise.
- HPF or derivative threshold on yaw/pitch (fc ≈ 0.05–0.1 Hz) to reject drift.
- Gyro quiet check: |ω| RMS < ω_quiet (e.g., 0.5–1.0 °/s) for stability decisions.
- Short‑term variance window (0.5–2 s) to confirm stability in Validation/Hold.
- Spike rejector: ignore single‑sample outliers using median‑of‑3 or Hampel filter.

---

## Persistence (EEPROM)

Persist the following structure via `saveToEEPROM` and restore on boot:
- gbias (3×float)
- q_reference (4×float, normalized)
- thresholds: azimuth_deg, altitude_deg
- windows: T_detect (s), T_validate (s), T_hold (min)
- firmware data version (uint16_t) for compatibility

Commit policy
- Write sparingly (upon HoldCommit success or explicit SaveState) to minimize wear.

---

## Public API (C/C++)

```cpp
// Initialization / lifecycle
void MotionDetection_Init();              // Load EEPROM, configure MotionFX
void MotionDetection_Calibrate();         // Stationary bias calib + set q_reference
void MotionDetection_SaveState();         // Force persist to EEPROM
void MotionDetection_GetStateEEPROM();    // Load persisted state

// Data ingress (called by accelerometer thread at 104 Hz)
void MotionDetection_Update(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            uint32_t timestamp);

// Configuration
void MotionDetection_set_thresholds(float altitude_deg, float azimuth_deg);
void MotionDetection_set_windows(float t_detect_s, float t_validate_s, float t_hold_min);

// Diagnostics (optional)
void MotionDetection_GetDeltas(float* azimuth_deg, float* altitude_deg);
void MotionDetection_GetState(char* state_str_buf, uint32_t buf_len);
```

Integration contract
- An external accelerometer thread owns device I/O and calls `MotionDetection_Update(...)` at 104 Hz with a monotonically increasing `timestamp`.
- A global `console` is available for logging. A global `eeprom` exposes `saveToEEPROM()`.

---

## Logging (Console)

Use `console->printOutput()` and `console->printOutputWOTime()` only. Avoid advanced format specifiers.

Examples
```cpp
console->printOutputWOTime("%s\n", "MotionDetection Initialized");
console->printOutput("State %s | dAz %f deg | dAlt %f deg\n", stateStr, dAz, dAlt);
console->printOutput("Event | az %f | alt %f | t %u\n", dAz, dAlt, timestamp);
```

Recommended periodic logs
- State transitions (from, to)
- Threshold crossings with timestamps
- Validation decisions (variance, gyro RMS)
- EEPROM loads/saves and versions

---

## Calibration and Startup

Cold start
1) Init → try EEPROM; if version mismatch, fall back to Calibration
2) Calibration: require gyro quiet; run gbias estimation; set `q_reference` once yaw/pitch are stable
3) Persist state; enter Idle/Monitoring

Recalibration command
- Resets gbias and re‑levels `q_reference` when stationary. Wired to host command handler.

---

## Edge Cases and Robustness

- Power loss between Event and HoldCommit: event logged, but reference not updated; on reboot, delta remains large and will quickly re‑validate. This is acceptable and visible in logs.
- Long‑term azimuth drift: constrained yaw from gravity projection (Ax, Ay ≠ 0) and HPF gate minimize false events.
- Wind‑induced oscillations: fail Validation due to sustained variance and gyro not quiet.

---

## Performance Targets

- CPU: light; most work in MotionFX + simple IIRs
- RAM: small buffers for short windows (≤ 2 s at 104 Hz)
- EEPROM: write only on state commits or operator request

---

## Repository Layout (proposed)

Note: only implement `motion_detection` and `testing`. Everything else is already implemented.

```
/src
├─ /app
│  ├─ globals.cpp                // console, eeprom, motion detection instance
│  ├─ eeprom.cpp                 // thin wrappers to save/load blobs
│  └─ accelerometer_thread.cpp   // sensor I/O, calls MotionDetection_Update
└─ /motion_detection
   ├─ motion_detection.h         // API, enums, configuration
   ├─ motion_detection.cpp       // state machine, detection logic
   ├─ quaternion.cpp             // quaternion math, frame transforms
   ├─ filters.cpp                // LPF/HPF, variance, spike rejector
   ├─ storage.cpp                // EEPROM serialization/deserialization
   └─ logging.cpp                // console helpers and formatting

/testing
├─ console_reader.py             // serial capture
├─ motion_analysis.py            // parse logs → CSV, compute stats
└─ motion_analysis.ipynb         // exploratory analysis
```

Branching
- main: stable releases
- dev: integration branch
- feature/motion_detection, feature/filters, feature/storage: focused workstreams with PRs into dev

---

## Configuration (defaults)

- thresholds: azimuth=10°, altitude=5°
- windows: T_detect=2 s, T_validate=5 s, T_hold=60 min
- filters: LPF_fc=1.5 Hz, HPF_fc=0.08 Hz, gyro_quiet=0.8 °/s RMS

All are persisted and can be changed via console commands.

---

## Example Usage

```cpp
// Startup
MotionDetection_Init();

// Optional: override defaults
MotionDetection_set_thresholds(5.0f, 10.0f);
MotionDetection_set_windows(2.0f, 6.0f, 120.0f);

// In accelerometer thread @104 Hz
MotionDetection_Update(ax, ay, az, gx, gy, gz, timestamp);

// On operator command
MotionDetection_Calibrate();
MotionDetection_SaveState();
```

---

## Build and Integration Notes

- Include MotionFX library per UM2220 and initialize with 104 Hz configuration.
- Ensure consistent unit conventions: accel in g, gyro in dps or rad/s as required by MotionFX.
- Apply WDS→NED transform consistently (either pre‑fusion on raw data or post‑fusion on orientation, not both).
- Timestamps must be monotonic; use data‑poll index at 104 Hz if no RTC is available.

---

## Testing and Analysis Toolkit

- Record console output continuously; rotate logs daily.
- Use `/testing/motion_analysis.py` to convert logs to CSV and compute:
  - Δazimuth/Δaltitude time series, validation windows, variance, gyro RMS
  - False‑positive/negative rates under wind vs. true moves
- Notebooks (`motion_analysis.ipynb`) provide spectral views and parameter tuning guidance.

---

## References

- ST MotionFX Sensor Fusion Library — UM2220 (see link below)
- ISM330DHCX datasheet and application notes

UM2220: https://www.st.com/resource/en/user_manual/um2220-getting-started-with-motionfx-sensor-fusion-library-in-xcubemems1-expansion-for-stm32cube-stmicroelectronics.pdf

---

## Future Work

- Adaptive thresholds based on wind/vibration level
- ML‑assisted classification (vibration vs. true pose change)
- Remote command interface (thresholds, windows, calibration)