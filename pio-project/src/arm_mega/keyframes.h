// pio-project/src/arm_mega/keyframes.h
// ----------------------------------------------------------------------
// PLACEHOLDER — all poses set to HOME. The firmware compiles fine but
// triggering the pickup sequence will just sit at home and dwell.
//
// To populate with real poses:
//     cd pio-project
//     python3 scripts/record_keyframes.py
//
// The recorder OVERWRITES this file. Re-run it any time you want to
// recapture (e.g. you moved the bin, swapped the object, etc.).
// ----------------------------------------------------------------------
#pragma once
#include <stdint.h>

struct Pose {
    uint8_t base;
    uint8_t shoulder;
    uint8_t elbow;
    uint8_t claw;
};

// Home pose — must match LIM_*.initial in main.cpp.
static constexpr Pose KF_HOME { 74, 68, 55, 21 };

static constexpr Pose KF_APPROACH     { 142, 30, 113,  21 };
static constexpr Pose KF_GRASP_DOWN   { 142, 30, 113,  21 };
static constexpr Pose KF_GRASP_CLOSE  { 142, 30, 113,  21 };
static constexpr Pose KF_LIFT         { 142, 30, 113,  21 };
static constexpr Pose KF_DROP_OVER    { 142, 30, 113,  21 };
static constexpr Pose KF_DROP_RELEASE { 142, 30, 113,  21 };

static constexpr Pose PICKUP_SEQUENCE[] = {
    KF_HOME,
    KF_APPROACH,
    KF_GRASP_DOWN,
    KF_GRASP_CLOSE,
    KF_LIFT,
    KF_DROP_OVER,
    KF_DROP_RELEASE,
    KF_HOME,
};
static constexpr uint8_t PICKUP_SEQUENCE_LEN =
    sizeof(PICKUP_SEQUENCE) / sizeof(PICKUP_SEQUENCE[0]);
