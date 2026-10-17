// Adaptive Mahony on the Uno: BMI270 + IIS2MDC -> roll, pitch, yaw at 50 Hz.
// Prints:  roll,pitch,yaw,kp,wa,us      (deg, deg, deg, accel gain 1/s, accel trust 0..1, filter time us)
// Folder needs: bmi270_config.h, AdaptiveMahony.h, imu_cal.h (copy the one full_calibration.py made;
// the one here is a placeholder with no calibration).
// Keep the board still for the first ~2 s after reset.

#include <Wire.h>
#include "bmi270_config.h"
#include "imu_cal.h"
#include "AdaptiveMahony.h"

AdaptiveMahony filt;

const uint8_t BMI = 0x68;   // BMI270, SDO = GND
const uint8_t MAG = 0x1E;   // IIS2MDC

bool wr(uint8_t dev, uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool rd(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(dev, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

int rd1(uint8_t dev, uint8_t reg)
{
  uint8_t v;
  return rd(dev, reg, &v, 1) ? v : -1;
}

bool bmiInit()
{
  wr(BMI, 0x7E, 0xB6); delay(5);          // soft reset
  rd1(BMI, 0x00);                         // dummy read
  if (rd1(BMI, 0x00) != 0x24) return false;
  wr(BMI, 0x7C, 0x00); delay(1);          // adv power save off
  wr(BMI, 0x59, 0x00);                    // prepare config load
  for (uint16_t i = 0; i < sizeof(bmi270_config_file); i += 16)
  {
    uint16_t word = i / 2;
    Wire.beginTransmission(BMI);
    Wire.write(0x5B); Wire.write(word & 0x0F); Wire.write(word >> 4);
    Wire.endTransmission();
    Wire.beginTransmission(BMI);
    Wire.write(0x5E);
    for (uint8_t k = 0; k < 16; k++) Wire.write(pgm_read_byte(&bmi270_config_file[i + k]));
    if (Wire.endTransmission() != 0) return false;
  }
  wr(BMI, 0x59, 0x01);                    // complete config load
  int st = 0;
  for (int t = 0; t < 10 && (st & 0x0F) != 1; t++) { delay(5); st = rd1(BMI, 0x21); }
  if ((st & 0x0F) != 1) return false;
  wr(BMI, 0x7D, 0x0E);                    // acc + gyr + temp on
  wr(BMI, 0x40, 0xA8);                    // acc 100 Hz
  wr(BMI, 0x42, 0xE9);                    // gyr 200 Hz
  wr(BMI, 0x7C, 0x02);                    // power save off
  delay(50);
  return true;
}

bool magInit()
{
  if (rd1(MAG, 0x4F) != 0x40) return false;
  wr(MAG, 0x60, 0x20); delay(20);         // soft reset
  wr(MAG, 0x60, 0x88);                    // temp comp, high-res, 50 Hz, continuous
  wr(MAG, 0x61, 0x03);                    // offset cancellation + LPF
  wr(MAG, 0x62, 0x10);                    // BDU
  delay(50);
  return true;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);
  delay(20);
  bool a = bmiInit();
  bool m = magInit();
  if (!a || !m)
  {
    while (1)
    {
      Serial.print(F("INIT FAILED  BMI270=")); Serial.print(a);
      Serial.print(F("  IIS2MDC=")); Serial.println(m);
      delay(1000);
    }
  }
  filt.useDip = USE_MAG;            // dip / |B| tests only when the mag is calibrated
  filt.dipDeg = MAG_DIP_DEG;
  filt.magField = USE_MAG ? MAG_FIELD_MG : 0;
  Serial.println(F("roll,pitch,yaw,kp,wa,us"));
}

unsigned long lastUs = 0;

void loop()
{
  int st = rd1(MAG, 0x67);                      // pace on the mag 50 Hz data-ready
  if (st < 0 || !(st & 0x08)) return;
  uint8_t mb[6];
  for (uint8_t i = 0; i < 6; i++) { int v = rd1(MAG, 0x68 + i); if (v < 0) return; mb[i] = v; }
  uint8_t b[12];
  if (!rd(BMI, 0x0C, b, 12)) return;

  unsigned long now = micros();
  float dt = lastUs ? (now - lastUs) * 1e-6f : 0.02f;
  lastUs = now;
  if (dt > 0.1f) dt = 0.1f;

  float acc[3], gyr[3], raw[3], mag[3];
  for (int i = 0; i < 3; i++)
  {
    int16_t ra = (int16_t)(b[2 * i + 1] << 8 | b[2 * i]);
    int16_t rg = (int16_t)(b[2 * i + 7] << 8 | b[2 * i + 6]);
    int16_t rm = (int16_t)(mb[2 * i + 1] << 8 | mb[2 * i]);
    acc[i] = (ra / 4096.0f - ACC_OFFSET_G[i]) * ACC_GAIN[i];               // g
    gyr[i] = (rg / 16.4f - GYRO_BIAS_DPS[i]) * AdaptiveMahony::DEG;        // rad/s
    raw[i] = rm * 1.5f - MAG_HARD_MG[i];                                    // mG
  }
  for (int i = 0; i < 3; i++)                                               // m = MAP * W * (raw - hard)
  {
    float s = 0;
    for (int j = 0; j < 3; j++)
    {
      float w = 0;
      for (int k = 0; k < 3; k++) w += MAG_SOFT[j][k] * raw[k];
      s += MAG_MAP[i][j] * w;
    }
    mag[i] = s;
  }

  unsigned long t0 = micros();
  filt.update(gyr[0], gyr[1], gyr[2], acc[0], acc[1], acc[2], dt, USE_MAG ? mag : nullptr);
  unsigned long us = micros() - t0;

  float r, p, y;
  filt.euler(r, p, y);
  Serial.print(r, 2); Serial.print(',');
  Serial.print(p, 2); Serial.print(',');
  Serial.print(y, 2); Serial.print(',');
  Serial.print(filt.kp, 3); Serial.print(',');
  Serial.print(filt.wa, 3); Serial.print(',');
  Serial.println(us);
}
