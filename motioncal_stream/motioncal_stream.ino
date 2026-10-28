// BMI270 + IIS2MDC -> PJRC MotionCal stream
// Prints at ~50 Hz:  Raw:ax,ay,az,gx,gy,gz,mx,my,mz
// MotionCal units (imuread.h): accel 8192 counts/g, gyro 16 counts/(deg/s), mag 0.1 uT/count (= 1 mG/count)
// Axes are each chip's own axes, unchanged. Keep bmi270_config.h in this folder.

#include <Wire.h>
#include "bmi270_config.h"

uint8_t BMI = 0x68;         // BMI270: 0x68 if SDO = GND, 0x69 if SDO = 3.3 V. Found at start-up.
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

int8_t casZX = 0;                         // read from the chip in bmiInit()

bool bmiInit()
{
  // use whichever BMI270 address answers (boards differ in how SDO is wired)
  for (uint8_t a = 0x68; a <= 0x69; a++)
  {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { BMI = a; break; }
  }
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
  wr(BMI, 0x40, 0xA7);                    // acc 50 Hz, normal, filter_perf (matches the 50 Hz read rate)
  wr(BMI, 0x42, 0xE7);                    // gyr 50 Hz, normal, noise+filter perf (no aliasing at 50 Hz)
  wr(BMI, 0x7C, 0x02);                    // power save off
  delay(50);
  // Gyro cross-axis post-processing factor (datasheet 4.6.10): x -= factor_zx * z / 2^9
  wr(BMI, 0x2F, 0x00);                    // FEAT_PAGE = 0
  uint8_t cb[2];
  if (rd(BMI, 0x3C, cb, 2)) casZX = (int8_t)((cb[0] & 0x7F) << 1) >> 1;   // GYR_CAS.factor_zx, 7-bit signed
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

long clamp16(long v) { return v > 32767 ? 32767 : (v < -32768 ? -32768 : v); }

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);                  // 400 kHz (fast mode, both chips allow it): needed for a full 50 Hz
  Wire.setWireTimeout(3000, true);        // a stuck bus resets itself instead of hanging the Uno
  delay(20);
  // retry until both chips start, instead of giving up forever
  for (uint8_t tries = 1; ; tries++)
  {
    bool a = bmiInit();
    bool m = magInit();
    if (a && m) break;
    Serial.print(F("INIT FAILED  BMI270=")); Serial.print(a);
    Serial.print(F("  IIS2MDC=")); Serial.print(m);
    Serial.print(F("  (try ")); Serial.print(tries); Serial.println(F(", retrying)"));
    Wire.end(); delay(500); Wire.begin(); Wire.setClock(400000); Wire.setWireTimeout(3000, true);
  }
  Serial.print(F("# BMI270 at 0x")); Serial.println(BMI, HEX);
  Serial.print(F("# GYR_CAS factor_zx = ")); Serial.println(casZX);
}

unsigned long lastSample = 0;

void loop()
{
  if (millis() - lastSample > 1000)       // no data for 1 s: re-init both chips
  {
    Serial.println(F("# no data for 1 s, re-initialising"));
    bmiInit(); magInit();
    lastSample = millis();
  }
  // pace on the magnetometer's 50 Hz data-ready
  int st = rd1(MAG, 0x67);
  if (st < 0 || !(st & 0x08)) return;

  uint8_t mb[6];
  if (!rd(MAG, 0x68 | 0x80, mb, 6)) return;    // OUTX_L..OUTZ_H in one burst (MSB of address = auto-increment, datasheet 6.1.1)
  uint8_t b[12];
  if (!rd(BMI, 0x0C, b, 12)) return;

  int16_t r[9];
  for (int i = 0; i < 6; i++) r[i] = (int16_t)(b[2 * i + 1] << 8 | b[2 * i]);
  for (int i = 0; i < 3; i++) r[6 + i] = (int16_t)(mb[2 * i + 1] << 8 | mb[2 * i]);

  long out[9];
  for (int i = 0; i < 3; i++) out[i] = (long)r[i] * 2;     // 4096 -> 8192 counts/g, NOT clamped: keeps up to +-8 g
  float g[3] = {r[3] - casZX * (float)r[5] / 512.0f, (float)r[4], (float)r[5]};       // CAS correction on X
  for (int i = 0; i < 3; i++) out[3 + i] = clamp16(lround(g[i] * (16.0 / 16.384)));   // 16.384 -> 16 counts/dps
  for (int i = 6; i < 9; i++) out[i] = clamp16(lround(r[i] * 1.5));                    // 1.5 mG/LSB -> 1 mG = 0.1 uT

  lastSample = millis();
  Serial.print(F("Raw:"));
  for (int i = 0; i < 9; i++) { Serial.print(out[i]); if (i < 8) Serial.print(','); }
  Serial.print(F("\r\n"));   // MotionCal ends a line on CR (13), so CR LF is required
}
