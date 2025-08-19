# Motion Detection Library – Orientation Change Detection with MotionFX

## Overview

This project implements a **robust motion detection application** using the STMicroelectronics **ISM330DHCX accelerometer and gyroscope** in combination with the **MotionFX Sensor Fusion Library**.

The goal is to **detect abrupt orientation changes** (azimuth and altitude) while ignoring drift and small oscillations caused by environmental disturbances such as wind, vibrations, or long-term sensor bias.

The system must be **robust enough to run for multiple days**, save and restore calibration across power cycles, and log events for diagnostics.

---

## System Description

* **Hardware:** ISM330DHCX (accelerometer + gyroscope)
* **Placement:** Sensor mounted on a 1m outdoor pole, in **WDS orientation** (West x-axis, Down y-axis, South z-axis).
* **Sampling rate:** 104 Hz.
* **Environment:** Mostly static; subject to wind and potential abrupt changes (e.g., earthquakes, forced rotations).

---

## Detection Requirements

* **Azimuth threshold:** 10°
* **Altitude threshold:** 5°
* **Condition:** Event reported if an abrupt orientation change **exceeds thresholds within a few seconds** and remains stable for at least `n` minutes (will mostly be >60 minutes).
* **Ignore:**

  * Drift (sensor bias, long-term azimuth instability).
  * Small oscillations from wind/vibrations.

---

## MotionFX Usage

* MotionFX provides sensor fusion (accelerometer + gyroscope → orientation quaternion).
* WDS orientation is **not natively supported**; a **custom conversion function** is required to align MotionFX output with our coordinate system.
* System must store and restore:

  * **Gyroscope bias (gbias)**
  * **Orientation reference quaternion**
  * **Thresholds**
  * **Validation time window**

Stored data is saved to **EEPROM** via `saveToEEPROM()`.

---

## State Machine

The library operates as a **state machine**:

1. **Initialization**

   * Configure MotionFX.
   * Load calibration and reference orientation from EEPROM.

2. **Calibration**

   * Estimate sensor bias (gyro offset, accelerometer tilt reference).
   * Wait until stable.

3. **Idle**

   * Low activity; waiting for significant changes.

4. **Monitoring**

   * Process sensor fusion at 104 Hz.
   * Track azimuth/altitude changes relative to reference.
   * Apply filters to suppress drift and oscillations.

5. **Validation**

   * Confirm if detected orientation exceeds threshold for a **minimum duration** (e.g., 2–5 seconds).
   * Reject transient events (wind gusts, short shocks).

6. **Event**

   * Log abrupt orientation change.
   * Update reference orientation.
   * Store new orientation in EEPROM.

7. **Error**

   * Entered if calibration or sensor fusion fails.
   * Requires reset or recalibration.

---

## Filtering and Drift Handling

* **Low-pass filter** to smooth sensor noise.
* **High-pass filter** to detect abrupt changes.
* **Wave/time series analysis** to distinguish:

  * **Drift** (slow bias, low frequency, ignored).
  * **Abrupt events** (fast change, high frequency, validated).
* Physics consideration:

  * Since `Ax` and `Ay` are never both zero (pole tilt assumption), azimuth drift can be bounded.
  * Long-term gyroscope drift is corrected by MotionFX fusion.

---

## Software Architecture

### File Structure

```
/src
├── /app
├──── globals.cpp # Implemented - stores shared classes like `console`, `eeprom`, ... and our motion detection class.
├──── eeprom.cpp # Implemented - stores data that we need after shut downs. Only read/write here once or twice a day.
├──── accelerometer_thread.cpp # Implemented - sets up everything for the accelerometer, initializes our library, sends updates of the data (acc + gyro + timestamp) and collects intterupts/events from ISM330.
├──── /motion_detection # Not implemented (TODO)
        ├── motion_detection.cpp     # Core state machine, orientation detection
        ├── motion_detection.h       # Public API
        ├── quaternion.cpp           # Quaternion math, orientation conversions
        ├── filters.cpp              # Low-pass / high-pass filters
        ├── storage.cpp              # EEPROM save/load functions
        ├── logging.cpp              # Wrappers for console logging
├──── /testing # Not implemented (TODO)
        ├── console_reader.py # read all data from console (COM4)
        ├── motion_analysis.py # collect all console into .txt and store clean data in csvs for analysis later on.
        ├── motion_analysis.ipynb # analyse the csvs for statistical patterns, frequencies, movements, gbias, ... and to compare with other runs.
```

### Public API (motion\_detection.h)

```cpp
// Initialize system (loads calibration + reference from EEPROM)
void MotionDetection_Init();

// Process one sensor update (accel + gyro + timestamp)
void MotionDetection_Update(float ax, float ay, float az,
                            float gx, float gy, float gz,
                            uint32_t timestamp);
// Set threshold calibration sequence
void MotionDetection_set_thresholds(float altitude_threshold, float azimuth_threshold);
// Trigger calibration sequence
void MotionDetection_Calibrate();

// Force-save current state to EEPROM
void MotionDetection_SaveState();

// Collect from EEPROM on restart
void MotionDetection_GetStateEEPROM();
```

---

## Logging

The system provides detailed logs using `console->printOutput()` and `console->printOutputWOTime()`.

Examples:

```cpp
console->printOutput("Orientation Change in state: Azimuth %f, Altitude %f, State: ", az, alt);
console->printOutputWOTime("%s\n", "MotionDetection Initialized"); // Note: it is better to create new line for this like this.
```

---

## EEPROM Storage

Persistent values:

* `gbias` (gyro bias calibration).
* `reference_quaternion`.
* `thresholds` (azimuth, altitude).
* `validation_time`.

This allows restart with orientation continuity.

---

## Example Use Case

1. Sensor is static for hours → No event.
2. Earthquake occurs → Sensor pole tilts abruptly by 50° azimuth.
3. System validates change (>10° azimuth within 2s, stable after).
4. Event logged + new reference orientation saved.
5. System resumes monitoring with updated baseline.

---

## Future Work

* Implement **adaptive thresholds** (different noise levels in calm vs. windy conditions).
* Explore **machine learning classifiers** for vibration vs. true movement separation.
* Add **remote command interface** (console-based commands for recalibration, threshold updates).

---

## References

* [STMicroelectronics MotionFX Sensor Fusion Library – User Manual (UM2220)](https://www.st.com/resource/en/user_manual/um2220-getting-started-with-motionfx-sensor-fusion-library-in-xcubemems1-expansion-for-stm32cube-stmicroelectronics.pdf)
* ISM330DHCX Datasheet