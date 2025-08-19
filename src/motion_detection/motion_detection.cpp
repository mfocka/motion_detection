#include "motion_detection.h"

#include <math.h>
#include <string.h>

// Local headers (implemented within this library)
#include "quaternion.h"
#include "filters.h"
#include "storage.h"
#include "logging.h"

// MotionFX wrapper (stubbed here; platform must link actual library per UM2220)
namespace motionfx_wrapper {
struct Gbias { float gx; float gy; float gz; };
struct Orientation { float qw; float qx; float qy; float qz; };
void init_104hz();
void set_gbias(const Gbias& b);
Gbias get_gbias();
Orientation update(float ax_g, float ay_g, float az_g,
                   float gx_dps, float gy_dps, float gz_dps);
}

namespace motion_detection {

using namespace quat_math;

// Constants
static const float kLpfCutoffHz = 1.5f;
static const float kHpfCutoffHz = 0.08f;
static const float kGyroQuietRmsDps = 0.8f;
static const float kSampleRateHz = 104.0f;
static const float kDt = 1.0f / kSampleRateHz;

// WDS (West, Down, South) to NED fixed transform quaternion
// Derived from matrix R = [ 0 0 -1 ; -1 0 0 ; 0 1 0 ]
static const Quaternion kQ_ned_from_wds = quaternionFromRotationMatrix(
    0.0f, 0.0f, -1.0f,
    -1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f);

// State container
struct DetectorContext {
    DetectorState state;

    // Config
    ThresholdsDeg thresholds;
    WindowsSec windows;

    // Persistence
    GyroBias gbias;
    Quaternion q_reference; // NED frame

    // Filters and buffers
    FirstOrderLowPass lpf_yaw;
    FirstOrderLowPass lpf_pitch;
    FirstOrderHighPass hpf_yaw;
    FirstOrderHighPass hpf_pitch;
    RmsAccumulator gyro_rms;
    RollingVariance yaw_var;
    RollingVariance pitch_var;

    // Derived
    Quaternion q_current; // NED
    Quaternion q_delta;   // reference^{-1} ⊗ current
    float yaw_deg;        // delta yaw
    float pitch_deg;      // delta pitch

    // Timers
    float detect_window_s;
    float validate_timer_s;
    float hold_timer_s;

    // Misc
    bool threshold_crossed;
    bool validation_ok;
    bool event_emitted;
};

static DetectorContext g_ctx;

// Helpers
static void setState(DetectorState s) {
    DetectorState prev = g_ctx.state;
    g_ctx.state = s;
    const char* prevStr = logging::stateToStr(prev);
    const char* nextStr = logging::stateToStr(s);
    console->printOutput("State %s -> %s\n", prevStr, nextStr);
}

static void resetFilters() {
    g_ctx.lpf_yaw.reset(kLpfCutoffHz, kSampleRateHz);
    g_ctx.lpf_pitch.reset(kLpfCutoffHz, kSampleRateHz);
    g_ctx.hpf_yaw.reset(kHpfCutoffHz, kSampleRateHz);
    g_ctx.hpf_pitch.reset(kHpfCutoffHz, kSampleRateHz);
    g_ctx.gyro_rms.reset( (uint32_t)(kSampleRateHz * 1.0f) );
    g_ctx.yaw_var.reset( (uint32_t)(kSampleRateHz * 1.0f) );
    g_ctx.pitch_var.reset( (uint32_t)(kSampleRateHz * 1.0f) );
}

static void computeDeltasFromOrientation(const Quaternion& q_ned_current) {
    g_ctx.q_current = q_ned_current;
    Quaternion q_ref_inv = conjugate(normalize(g_ctx.q_reference));
    g_ctx.q_delta = multiply(q_ref_inv, normalize(q_ned_current));

    // Extract yaw (azimuth) and pitch (altitude) from delta
    float yaw_rad, pitch_rad, roll_rad;
    eulerFromQuaternionYawPitchRoll(g_ctx.q_delta, yaw_rad, pitch_rad, roll_rad);

    // Apply basic physics hint: constrain yaw using gravity projection to reduce drift
    // This is implemented inside filters::constrainYawWithGravity in filters.h/cpp
    constrainYawWithGravity(yaw_rad, pitch_rad);

    g_ctx.yaw_deg = yaw_rad * 180.0f / 3.1415926535f;
    g_ctx.pitch_deg = pitch_rad * 180.0f / 3.1415926535f;
}

static void processDetectionGates(float gx, float gy, float gz) {
    // Filtering
    float yaw_lpf = g_ctx.lpf_yaw.step(g_ctx.yaw_deg);
    float pitch_lpf = g_ctx.lpf_pitch.step(g_ctx.pitch_deg);
    float yaw_hpf = g_ctx.hpf_yaw.step(yaw_lpf);
    float pitch_hpf = g_ctx.hpf_pitch.step(pitch_lpf);

    // Time-series stats
    g_ctx.yaw_var.push(yaw_lpf);
    g_ctx.pitch_var.push(pitch_lpf);
    g_ctx.gyro_rms.push3(gx, gy, gz);

    // Threshold within detect window
    if (fabsf(yaw_lpf) >= g_ctx.thresholds.azimuth || fabsf(pitch_lpf) >= g_ctx.thresholds.altitude) {
        g_ctx.threshold_crossed = true;
        g_ctx.detect_window_s = 0.0f; // reset window timer upon crossing
        console->printOutput("ThresholdCross | yaw %f | pitch %f\n", yaw_lpf, pitch_lpf);
    } else {
        g_ctx.detect_window_s += kDt;
        if (g_ctx.detect_window_s > g_ctx.windows.t_detect_s) {
            g_ctx.threshold_crossed = false;
        }
    }

    // Drift rejection using HPF/derivative gate
    const float drift_gate = 0.2f; // deg/s equivalent on filtered signals
    bool fast_change = (fabsf(yaw_hpf) > drift_gate) || (fabsf(pitch_hpf) > drift_gate);
    if (!fast_change && !g_ctx.threshold_crossed) {
        // Likely drift; do not arm validation
        return;
    }

    if (g_ctx.state == DetectorState::Monitoring && g_ctx.threshold_crossed) {
        setState(DetectorState::Validation);
        g_ctx.validate_timer_s = 0.0f;
        g_ctx.validation_ok = false;
    }
}

static void runValidationAndHold() {
    // Stability checks
    const float var_thresh = 0.5f; // deg^2
    const float gyro_quiet = kGyroQuietRmsDps;
    bool variance_ok = (g_ctx.yaw_var.variance() < var_thresh) && (g_ctx.pitch_var.variance() < var_thresh);
    bool gyro_ok = (g_ctx.gyro_rms.rms() < gyro_quiet);

    if (g_ctx.state == DetectorState::Validation) {
        g_ctx.validate_timer_s += kDt;
        if (variance_ok && gyro_ok) {
            if (g_ctx.validate_timer_s >= g_ctx.windows.t_validate_s) {
                g_ctx.validation_ok = true;
                setState(DetectorState::Event);
                g_ctx.event_emitted = false;
                g_ctx.hold_timer_s = 0.0f;
            }
        } else {
            // Reset timer if instability returns
            g_ctx.validate_timer_s = 0.0f;
        }
    }

    if (g_ctx.state == DetectorState::Event) {
        if (!g_ctx.event_emitted) {
            console->printOutput("Event | yaw %f | pitch %f\n", g_ctx.yaw_deg, g_ctx.pitch_deg);
            g_ctx.event_emitted = true;
            setState(DetectorState::HoldCommit);
        }
    }

    if (g_ctx.state == DetectorState::HoldCommit) {
        // Require sustained stability before committing new reference
        bool variance_ok_hold = variance_ok;
        bool gyro_ok_hold = gyro_ok;
        if (variance_ok_hold && gyro_ok_hold) {
            g_ctx.hold_timer_s += kDt;
            if (g_ctx.hold_timer_s >= (g_ctx.windows.t_hold_min * 60.0f)) {
                // Commit new reference orientation
                g_ctx.q_reference = g_ctx.q_current;
                StorageBlob blob;
                memset(&blob, 0, sizeof(blob));
                blob.version = storage::kStorageVersion;
                blob.thresholds = g_ctx.thresholds;
                blob.windows = g_ctx.windows;
                blob.gbias = { g_ctx.gbias.gx, g_ctx.gbias.gy, g_ctx.gbias.gz };
                blob.q_reference = g_ctx.q_reference;
                storage::save(blob);
                console->printOutputWOTime("%s\n", "HoldCommit persisted");
                setState(DetectorState::Monitoring);
            }
        } else {
            // Instability → revert
            setState(DetectorState::Monitoring);
        }
    }
}

// Public API
void MotionDetection_Init() {
    memset(&g_ctx, 0, sizeof(g_ctx));
    g_ctx.state = DetectorState::Init;
    g_ctx.thresholds = {5.0f, 10.0f};
    g_ctx.windows = {2.0f, 5.0f, 60.0f};
    resetFilters();

    // Load persisted state
    StorageBlob blob;
    if (storage::load(blob)) {
        g_ctx.thresholds = blob.thresholds;
        g_ctx.windows = blob.windows;
        g_ctx.gbias = { blob.gbias.gx, blob.gbias.gy, blob.gbias.gz };
        g_ctx.q_reference = normalize(blob.q_reference);
        console->printOutputWOTime("%s\n", "EEPROM loaded");
    } else {
        // Default reference is identity (no rotation)
        g_ctx.q_reference = Quaternion{1.0f, 0.0f, 0.0f, 0.0f};
        console->printOutputWOTime("%s\n", "EEPROM load failed");
    }

    // Init MotionFX
    motionfx_wrapper::init_104hz();
    motionfx_wrapper::set_gbias(motionfx_wrapper::Gbias{ g_ctx.gbias.gx, g_ctx.gbias.gy, g_ctx.gbias.gz });
    setState(DetectorState::Idle);
}

void MotionDetection_Calibrate() {
    setState(DetectorState::Calibration);
    // Simple calibration policy: when gyro is quiet for several seconds, capture gbias and reference
    g_ctx.gyro_rms.reset((uint32_t)(kSampleRateHz * 2.0f));
    // The actual update loop will drive the calibration finish when quiet
}

void MotionDetection_SaveState() {
    StorageBlob blob;
    memset(&blob, 0, sizeof(blob));
    blob.version = storage::kStorageVersion;
    blob.thresholds = g_ctx.thresholds;
    blob.windows = g_ctx.windows;
    blob.gbias = g_ctx.gbias;
    blob.q_reference = g_ctx.q_reference;
    storage::save(blob);
    console->printOutputWOTime("%s\n", "State saved");
}

void MotionDetection_GetStateEEPROM() {
    StorageBlob blob;
    if (storage::load(blob)) {
        console->printOutput("EEPROM v %u | az %f | alt %f\n", (unsigned)blob.version, blob.thresholds.azimuth, blob.thresholds.altitude);
    } else {
        console->printOutputWOTime("%s\n", "EEPROM empty");
    }
}

void MotionDetection_Update(float ax_g, float ay_g, float az_g,
                            float gx_dps, float gy_dps, float gz_dps,
                            uint32_t /*timestampTicks*/) {
    // Transform raw IMU from WDS to NED before feeding to MotionFX
    // For accel, rotate vector; for gyro, rotate as angular rate (same quaternion rotation)
    Vector3 a_wds{ax_g, ay_g, az_g};
    Vector3 g_wds{gx_dps, gy_dps, gz_dps};
    Vector3 a_ned = rotateVector(kQ_ned_from_wds, a_wds);
    Vector3 g_ned = rotateVector(kQ_ned_from_wds, g_wds);

    // Feed to MotionFX
    motionfx_wrapper::Orientation o = motionfx_wrapper::update(a_ned.x, a_ned.y, a_ned.z, g_ned.x, g_ned.y, g_ned.z);
    Quaternion q_board{o.qw, o.qx, o.qy, o.qz};

    // If MotionFX outputs in board frame aligned with NED config, we already fed NED vectors.
    // So orientation is in NED as well. If not, an additional fixed rotation could be applied here.
    Quaternion q_ned_current = normalize(q_board);

    if (g_ctx.state == DetectorState::Calibration) {
        // Accumulate gyro RMS and when quiet, capture gbias and reference
        g_ctx.gyro_rms.push3(g_ned.x, g_ned.y, g_ned.z);
        if (g_ctx.gyro_rms.sampleCount() > (uint32_t)(kSampleRateHz * 2.0f) && g_ctx.gyro_rms.rms() < kGyroQuietRmsDps) {
            {
                auto b = motionfx_wrapper::get_gbias();
                g_ctx.gbias = GyroBias{ b.gx, b.gy, b.gz };
            }
            g_ctx.q_reference = q_ned_current;
            console->printOutputWOTime("%s\n", "Calibration done");
            setState(DetectorState::Monitoring);
        }
        return;
    }

    // Normal monitoring pipeline
    computeDeltasFromOrientation(q_ned_current);
    processDetectionGates(g_ned.x, g_ned.y, g_ned.z);
    runValidationAndHold();
}

void MotionDetection_set_thresholds(float altitude_deg, float azimuth_deg) {
    g_ctx.thresholds.altitude = altitude_deg;
    g_ctx.thresholds.azimuth = azimuth_deg;
    console->printOutput("Cfg thresholds | az %f | alt %f\n", g_ctx.thresholds.azimuth, g_ctx.thresholds.altitude);
}

void MotionDetection_set_windows(float t_detect_s, float t_validate_s, float t_hold_min) {
    g_ctx.windows.t_detect_s = t_detect_s;
    g_ctx.windows.t_validate_s = t_validate_s;
    g_ctx.windows.t_hold_min = t_hold_min;
    console->printOutput("Cfg windows | detect %f | validate %f | hold_min %f\n", t_detect_s, t_validate_s, t_hold_min);
}

void MotionDetection_GetDeltas(float* azimuth_deg, float* altitude_deg) {
    if (azimuth_deg) *azimuth_deg = g_ctx.yaw_deg;
    if (altitude_deg) *altitude_deg = g_ctx.pitch_deg;
}

void MotionDetection_GetState(char* state_str_buf, uint32_t buf_len) {
    const char* s = logging::stateToStr(g_ctx.state);
    if (state_str_buf && buf_len > 0) {
        // simple safe copy
        uint32_t i = 0;
        for (; s[i] != '\0' && i + 1 < buf_len; ++i) state_str_buf[i] = s[i];
        state_str_buf[i] = '\0';
    }
}

} // namespace motion_detection


