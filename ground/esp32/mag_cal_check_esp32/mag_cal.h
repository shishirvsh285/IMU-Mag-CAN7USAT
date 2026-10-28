// Written by mag_calibrate.py 2026-10-22 21:42:12
// m_cal = MAG_SOFT * (m_raw - MAG_HARD), mG, IIS2MDC axes
#pragma once
const float MAG_HARD[3]    = {-206.409832f, 288.737747f, -437.838697f};
const float MAG_SOFT[3][3] = {{0.924919f, 0.012008f, 0.005965f}, {0.012008f, 1.044602f, -0.020555f}, {0.005965f, -0.020555f, 1.041246f}};
const float MAG_FIELD_MG   = 265.2f;
#define MAG_CAL_STAMP "2026-10-22 21:42:12"
