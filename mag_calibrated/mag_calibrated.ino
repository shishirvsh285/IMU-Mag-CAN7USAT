// BMI270 + IIS2MDC with the MotionCal magnetometer calibration applied on the Uno
// Calibration from MotionCal (2026-10-07 22:44):
//   offset V = (8.24, 3.71, 5.88) uT = (82.4, 37.1, 58.8) mG
//   calibrated = invW * (raw - V)        (same formula as MotionCal's apply_calibration)
// Prints at ~50 Hz on COM5, two lines per sample:
//   mag mG: cx,cy,cz          calibrated field in mG (for our Python windows)
//   Raw:ax,ay,az,gx,gy,gz,cx,cy,cz   MotionCal format, mag = calibrated, 1 count = 1 mG = 0.1 uT
// MotionCal units (imuread.h): accel 8192 counts/g, gyro 16 counts/(deg/s), mag 0.1 uT/count (= 1 mG/count)
// Axes are each chip's own axes, unchanged. Keep bmi270_config.h in this folder.

#include <Wire.h>
#include "bmi270_config.h"

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

// ---- magnetometer calibration (from MotionCal) ----
const float MAG_V[3] = {82.4, 37.1, 58.8};          // hard-iron offset, mG
const float MAG_INVW[3][3] = {{ 0.825, -0.100, -0.039},
                              {-0.100,  1.135,  0.003},
                              {-0.039,  0.003,  1.081}};  // soft-iron inverse matrix

long clamp16(long v) { return v > 32767 ? 32767 : (v < -32768 ? -32768 : v); }

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);                  // 100 kHz: safer with jumper wires
  Wire.setWireTimeout(3000, true);        // if the I2C bus locks up (wires knocked), reset it instead of hanging forever
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
}

void loop()
{
  // pace on the magnetometer's 50 Hz data-ready
  int st = rd1(MAG, 0x67);
  if (st < 0 || !(st & 0x08)) return;

  uint8_t mb[6];
  for (uint8_t i = 0; i < 6; i++) { int v = rd1(MAG, 0x68 + i); if (v < 0) return; mb[i] = v; }
  uint8_t b[12];
  if (!rd(BMI, 0x0C, b, 12)) return;

  int16_t r[9];
  for (int i = 0; i < 6; i++) r[i] = (int16_t)(b[2 * i + 1] << 8 | b[2 * i]);
  for (int i = 0; i < 3; i++) r[6 + i] = (int16_t)(mb[2 * i + 1] << 8 | mb[2 * i]);

  long out[9];
  for (int i = 0; i < 3; i++) out[i] = clamp16((long)r[i] * 2);                       // 4096 -> 8192 counts/g
  for (int i = 3; i < 6; i++) out[i] = clamp16(lround(r[i] * (16.0 / 16.4)));          // 16.4 -> 16 counts/dps
  // raw mag in mG, minus offset, times invW
  float d[3], c[3];
  for (int i = 0; i < 3; i++) d[i] = r[6 + i] * 1.5 - MAG_V[i];
  for (int i = 0; i < 3; i++) c[i] = MAG_INVW[i][0] * d[0] + MAG_INVW[i][1] * d[1] + MAG_INVW[i][2] * d[2];
  for (int i = 0; i < 3; i++) out[6 + i] = clamp16(lround(c[i]));

  Serial.print(F("mag mG: "));
  Serial.print(c[0], 1); Serial.print(' ');
  Serial.print(c[1], 1); Serial.print(' ');
  Serial.print(c[2], 1); Serial.print(F("\r\n"));

  Serial.print(F("Raw:"));
  for (int i = 0; i < 9; i++) { Serial.print(out[i]); if (i < 8) Serial.print(','); }
  Serial.print(F("\r\n"));   // MotionCal ends a line on CR (13), so CR LF is required
}
