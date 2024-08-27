#pragma once

#include "ConfigTypes.h"
#include "common/foc_utils.h"

namespace drvconstants {
    using namespace BlueDrive;
    constexpr float kADCGain = -64.0f/7;
    constexpr float kShuntOhms = 0.003f;
    constexpr float kNominalBusVoltage = 24;
    constexpr float kPWMFreqHz = 15000;
    constexpr float kHFI_V = 4;
    constexpr float kHFI_Gain_Vel = 5.0f * _2PI;
    constexpr float kHFI_Gain_Pos = 750.0f * _2PI;
    constexpr float kHFI_AngDeltaDiscardThresh = 0.1f;
    constexpr float kCurrentSenseAlignEffort_V = 4;

    constexpr HFIMotorData kTmotorG60{14, 4.6f, 25.0f, 1987e-6f, 1474e-6f}; // TODO:
    constexpr HFIMotorData kTmotorG80{21, 1.64, 30.0f,707e-6f, 540e-6f}; // TODO:
    constexpr HFIMotorData kRandomDroneOutrunner{2, 0.0026, 2300, 7.465e-6f, 6.51e-6f}; // TODO:
}

namespace config {
    using namespace BlueDrive;

    constexpr HFIMotorData currentMotor = drvconstants::kTmotorG60;
    constexpr int canID = 1; // TODO: Make unique
}
