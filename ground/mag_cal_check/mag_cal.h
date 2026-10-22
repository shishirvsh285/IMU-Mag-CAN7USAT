// PLACEHOLDER: no calibration (output = raw). Run mag_calibrate.py, it overwrites this file.
#pragma once
const float MAG_HARD[3]    = {0.0f, 0.0f, 0.0f};
const float MAG_SOFT[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
const float MAG_FIELD_MG   = 0.0f;
#define MAG_CAL_STAMP "NONE (placeholder, run mag_calibrate.py)"
