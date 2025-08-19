### Motion Detection Library — Report

#### What was done
- Implemented a robust C++ library under `src/motion_detection/` for detecting abrupt orientation changes (azimuth and altitude) using MotionFX outputs.
- Added WDS→NED fixed-frame transform to align device axes with MotionFX NED configuration.
- Built a state machine with states: Init, Calibration, Idle, Monitoring, Validation, Event, HoldCommit, Error.
- Implemented filters and time-series heuristics: LPF, HPF gate, rolling variance, gyro RMS quiet check.
- Added EEPROM persistence of `gbias`, `q_reference`, thresholds, and timing windows with versioning.
- Provided console logging hooks using `console->printOutput()` and `console->printOutputWOTime()`.

#### Why it works
- Detects rapid orientation changes by combining LPF for noise suppression with HPF/derivative-like gating to reject slow drift.
- Requires short stability in Validation to ensure the change is real, then a long Hold window to ensure the new pose is persistent before updating the reference and writing EEPROM.
- Gyro quiet and variance constraints mitigate wind/vibration false positives.
- Gravity-constrained yaw heuristic reduces heading drift when pitch is small but non-zero (Ax, Ay ≠ 0 per installation).

#### What was tested (intended)
- Functional flow with synthetic inputs: threshold crossings, stability windows, and persistence behavior across sessions.
- EEPROM versioning paths: load success/failure; commit behavior on HoldCommit pass.
- WDS→NED transform correctness against known axis rotations.

#### State machine diagram
```
Init -> Calibration -> Idle -> Monitoring -> Validation -> Event -> HoldCommit -> Monitoring
                 ^                                                     |
                 |-----------------------------------------------------|
Error is reachable from any state.
```

Transitions (summary)
- Init: load EEPROM (gbias, thresholds, windows, q_reference), init MotionFX, → Idle
- Calibration: wait for gyro quiet, set gbias, capture q_reference, → Monitoring
- Idle: low activity; escalate to Monitoring immediately (or via motion/timer)
- Monitoring: compute filtered yaw/pitch deltas; if threshold within T_detect → Validation
- Validation: require stability for T_validate; pass → Event, fail → Monitoring
- Event: emit once; → HoldCommit
- HoldCommit: require stability for T_hold; pass → persist q_reference and return to Monitoring; fail → Monitoring
- Error: on any unrecoverable fault

#### Architecture diagram
```
Accelerometer thread @104 Hz
  ├─ WDS→NED transform (fixed quaternion)
  ├─ MotionFX update (104 Hz config)
  └─ MotionDetection_Update
       ├─ Delta quaternion: q_delta = inv(q_reference) ⊗ q_current
       ├─ Extract yaw/pitch (deg), gravity-constrained yaw heuristic
       ├─ Filters: LPF on yaw/pitch; HPF gate for drift rejection
       ├─ Stats: rolling variance, gyro RMS
       └─ State machine: Monitoring → Validation → Event → HoldCommit

EEPROM persistence
  ├─ thresholds: azimuth_deg, altitude_deg
  ├─ windows: T_detect (s), T_validate (s), T_hold (min)
  ├─ gbias (3 floats)
  └─ q_reference (normalized quaternion)

Console logging
  ├─ State transitions
  ├─ Threshold crossings
  ├─ Validation decisions (variance, gyro RMS)
  └─ EEPROM load/save status
```

#### Physics and drift/vibration considerations
- Drift: HPF/derivative gate suppresses very low-frequency trends; reference only updates after long stability (HoldCommit), avoiding chasing drift.
- Wind/vibration: Validation and HoldCommit require low variance and gyro quiet; oscillatory motion fails stability checks.
- Azimuth drift: constrained by gravity when pitch ≠ 0; heuristic scales yaw depending on pitch magnitude to damp heading wander.

#### Configuration (defaults)
- thresholds: azimuth=10°, altitude=5°
- windows: T_detect=2 s, T_validate=5 s, T_hold=60 min
- filters: LPF_fc=1.5 Hz, HPF_fc=0.08 Hz, gyro_quiet=0.8 °/s RMS

#### Public API (C/C++)
```cpp
// Initialization / lifecycle
void MotionDetection_Init();
void MotionDetection_Calibrate();
void MotionDetection_SaveState();
void MotionDetection_GetStateEEPROM();

// Data ingress (called @104 Hz)
void MotionDetection_Update(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            uint32_t timestamp);

// Configuration
void MotionDetection_set_thresholds(float altitude_deg, float azimuth_deg);
void MotionDetection_set_windows(float t_detect_s, float t_validate_s, float t_hold_min);

// Diagnostics
void MotionDetection_GetDeltas(float* azimuth_deg, float* altitude_deg);
void MotionDetection_GetState(char* state_str_buf, uint32_t buf_len);
```

#### Branching plan
- dev: integration branch
- feature/motion_detection: state machine and detection logic
- feature/filters: filtering and time-series heuristics
- feature/storage: EEPROM persistence and versioning
- feature/analysis: testing scripts and log analysis

#### Next steps
- Integrate actual MotionFX wrapper calls; verify units (g, dps/rad/s) per UM2220.
- Tune LPF/HPF cutoffs and thresholds based on field data; add spike rejector (median-of-3/Hampel).
- Add analysis scripts to parse console logs and compute false-positive/negative rates.

### Motion Detection Library — Report

#### What was done
- Implemented a robust C++ library under `src/motion_detection/` for detecting abrupt orientation changes (azimuth and altitude) using MotionFX outputs.
- Added WDS→NED fixed-frame transform to align device axes with MotionFX NED configuration.
- Built a state machine with states: Init, Calibration, Idle, Monitoring, Validation, Event, HoldCommit, Error.
- Implemented filters and time-series heuristics: LPF, HPF gate, rolling variance, gyro RMS quiet check.
- Added EEPROM persistence of `gbias`, `q_reference`, thresholds, and timing windows with versioning.
- Provided extensive console logging hooks using `console->printOutput()` / `console->printOutputWOTime()`.

#### Why it works
- Detects rapid orientation changes by combining LPF for noise suppression with HPF/derivative-like gating to reject slow drift.
- Requires short stability in Validation to ensure the change is real, then a long Hold window to ensure the new pose is persistent before updating the reference and writing EEPROM.
- Gyro quiet and variance constraints mitigate wind/vibration false positives.
- Gravity-constrained yaw heuristic reduces heading drift when pitch is small but non-zero (Ax, Ay ≠ 0 per installation).

#### What was tested (intended)
- Functional flow with synthetic inputs: threshold crossings, stability windows, and persistence behavior across sessions.
- EEPROM versioning paths: load success/failure; commit behavior on HoldCommit pass.
- WDS→NED transform correctness against known axis rotations.

#### State machine diagram
```
Init -> Calibration -> Idle -> Monitoring -> Validation -> Event -> HoldCommit -> Monitoring
                 ^                                                     |
                 |-----------------------------------------------------|
Error is reachable from any state.
```

Transitions (summary)
- Init: load EEPROM (gbias, thresholds, windows, q_reference), init MotionFX, → Idle
- Calibration: wait for gyro quiet, set gbias, capture q_reference, → Monitoring
- Idle: low activity; escalate to Monitoring immediately (or via motion/timer)
- Monitoring: compute filtered yaw/pitch deltas; if threshold within T_detect → Validation
- Validation: require stability for T_validate; pass → Event, fail → Monitoring
- Event: emit once; → HoldCommit
- HoldCommit: require stability for T_hold; pass → persist q_reference and return to Monitoring; fail → Monitoring
- Error: on any unrecoverable fault

#### Architecture diagram
```
Accelerometer thread @104 Hz
  ├─ WDS→NED transform (fixed quaternion)
  ├─ MotionFX update (104 Hz config)
  └─ MotionDetection_Update
       ├─ Delta quaternion: q_delta = inv(q_reference) ⊗ q_current
       ├─ Extract yaw/pitch (deg), gravity-constrained yaw heuristic
       ├─ Filters: LPF on yaw/pitch; HPF gate for drift rejection
       ├─ Stats: rolling variance, gyro RMS
       └─ State machine: Monitoring → Validation → Event → HoldCommit

EEPROM persistence
  ├─ thresholds: azimuth_deg, altitude_deg
  ├─ windows: T_detect (s), T_validate (s), T_hold (min)
  ├─ gbias (3 floats)
  └─ q_reference (normalized quaternion)

Console logging
  ├─ State transitions
  ├─ Threshold crossings
  ├─ Validation decisions (variance, gyro RMS)
  └─ EEPROM load/save status
```

#### Physics and drift/vibration considerations
- Drift: HPF/derivative gate suppresses very low-frequency trends; reference only updates after long stability (HoldCommit), avoiding chasing drift.
- Wind/vibration: Validation and HoldCommit require low variance and gyro quiet; oscillatory motion fails stability checks.
- Azimuth drift: constrained by gravity when pitch ≠ 0; heuristic scales yaw depending on pitch magnitude to damp heading wander.

#### Configuration (defaults)
- thresholds: azimuth=10°, altitude=5°
- windows: T_detect=2 s, T_validate=5 s, T_hold=60 min
- filters: LPF_fc=1.5 Hz, HPF_fc=0.08 Hz, gyro_quiet=0.8 °/s RMS

#### Public API (C/C++)
```cpp
// Initialization / lifecycle
void MotionDetection_Init();
void MotionDetection_Calibrate();
void MotionDetection_SaveState();
void MotionDetection_GetStateEEPROM();

// Data ingress (called @104 Hz)
void MotionDetection_Update(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            uint32_t timestamp);

// Configuration
void MotionDetection_set_thresholds(float altitude_deg, float azimuth_deg);
void MotionDetection_set_windows(float t_detect_s, float t_validate_s, float t_hold_min);

// Diagnostics
void MotionDetection_GetDeltas(float* azimuth_deg, float* altitude_deg);
void MotionDetection_GetState(char* state_str_buf, uint32_t buf_len);
```

#### Branching plan
- dev: integration branch
- feature/motion_detection: state machine and detection logic
- feature/filters: filtering and time-series heuristics
- feature/storage: EEPROM persistence and versioning

#### Next steps
- Integrate actual MotionFX wrapper calls; verify units (g, dps/rad/s) per UM2220.
- Tune LPF/HPF cutoffs and thresholds based on field data; add spike rejector (median-of-3/Hampel).
- Add analysis scripts to parse console logs and compute false-positive/negative rates.


