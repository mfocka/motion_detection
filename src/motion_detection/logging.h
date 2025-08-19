#pragma once

#include "motion_detection.h"

namespace motion_detection { namespace logging {

inline const char* stateToStr(DetectorState s) {
    switch (s) {
        case DetectorState::Init: return "Init";
        case DetectorState::Calibration: return "Calibration";
        case DetectorState::Idle: return "Idle";
        case DetectorState::Monitoring: return "Monitoring";
        case DetectorState::Validation: return "Validation";
        case DetectorState::Event: return "Event";
        case DetectorState::HoldCommit: return "HoldCommit";
        case DetectorState::Error: return "Error";
        default: return "Unknown";
    }
}

}} // namespace motion_detection::logging


