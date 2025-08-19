// Motion Detection Library — Orientation Change Detection with MotionFX
// Public API and core types

#pragma once

#include <stdint.h>

// Forward declarations for external integration points
struct Console { void printOutput(const char* fmt, ...); void printOutputWOTime(const char* fmt, ...); };
extern Console* console;

// External EEPROM helpers provided by the platform
extern bool saveToEEPROM(const void* data, uint32_t sizeBytes);
extern bool loadFromEEPROM(void* data, uint32_t sizeBytes);

namespace motion_detection {

// Units: angles in degrees unless otherwise stated; time in seconds

struct GyroBias { float gx; float gy; float gz; };

enum class DetectorState : uint8_t {
    Init = 0,
    Calibration,
    Idle,
    Monitoring,
    Validation,
    Event,
    HoldCommit,
    Error
};

struct ThresholdsDeg {
    float altitude; // pitch
    float azimuth;  // yaw
};

struct WindowsSec {
    float t_detect_s;
    float t_validate_s;
    float t_hold_min; // minutes, but stored as seconds multiplier for convenience in impl
};

// Initialization / lifecycle
void MotionDetection_Init();              // Load EEPROM, configure MotionFX (via wrapper), set initial state
void MotionDetection_Calibrate();         // Start/perform stationary bias calibration and set reference orientation
void MotionDetection_SaveState();         // Force persist to EEPROM
void MotionDetection_GetStateEEPROM();    // Attempt to load persisted state (logs outcome)

// Data ingress (called by accelerometer thread at 104 Hz)
void MotionDetection_Update(float ax_g, float ay_g, float az_g,
                            float gx_dps, float gy_dps, float gz_dps,
                            uint32_t timestampTicks);

// Configuration
void MotionDetection_set_thresholds(float altitude_deg, float azimuth_deg);
void MotionDetection_set_windows(float t_detect_s, float t_validate_s, float t_hold_min);

// Diagnostics (optional)
void MotionDetection_GetDeltas(float* azimuth_deg, float* altitude_deg);
void MotionDetection_GetState(char* state_str_buf, uint32_t buf_len);

} // namespace motion_detection


