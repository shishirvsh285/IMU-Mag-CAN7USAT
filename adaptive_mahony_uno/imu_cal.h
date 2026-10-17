// PLACEHOLDER: replace with the imu_cal.h written by full_calibration.py
#pragma once
const float GYRO_BIAS_DPS[3] = {0.0f, 0.0f, 0.0f};
const float ACC_OFFSET_G[3]  = {0.0f, 0.0f, 0.0f};
const float ACC_GAIN[3]      = {1.0f, 1.0f, 1.0f};
const float MAG_HARD_MG[3]   = {0.0f, 0.0f, 0.0f};
const float MAG_SOFT[3][3]   = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
const int   MAG_MAP[3][3]    = {{1, 0, 0}, {0, -1, 0}, {0, 0, 1}};
#define USE_MAG 0            // 1 once the mag is calibrated and the map is CLEAR
const float MAG_DIP_DEG = 26.0f;
const float MAG_FIELD_MG = 306.0f;
