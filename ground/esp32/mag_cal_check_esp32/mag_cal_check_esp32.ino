// mag_cal_check (ESP32 version) : like motioncal_stream, but the magnetometer is CALIBRATED on the Uno with the
// numbers mag_calibrate.py wrote into mag_cal.h (same folder):   m_cal = MAG_SOFT * (m_raw - MAG_HARD)
// Open PJRC MotionCal on this stream: a good calibration shows a sphere centred on zero, MotionCal's
// own "Magnetic Offset" close to 0 and its fit error small.
// Prints at ~50 Hz:  Raw:ax,ay,az,gx,gy,gz,mx,my,mz   (accel 8192/g, gyro 16/(deg/s), mag 1 mG = 0.1 uT)
// The first line after reset says which calibration is loaded.
//
// ESP32 DevKit V1 wiring (sensor header -> ESP32):
//   pin 1 I2C_SCK -> D22 (SCL)    pin 2 GND -> GND
//   pin 3 I2C_SDA -> D21 (SDA)    pin 4 3.3V -> 3V3   (never VIN / 5V)
// Arduino IDE board: "ESP32 Dev Module". Serial 115200.
// Uses mag_cal.h in this folder (mag_calibrate.py writes it here too).

#include <Wire.h>

const int PIN_SDA = 21;     // D21
const int PIN_SCL = 22;     // D22
#include "bmi270_config.h"
#include "mag_cal.h"

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
  if (Wire.requestFrom((uint16_t)dev, (size_t)len) != len) return false;
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
  delay(200);                             // let the USB-serial settle after reset
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);                  // 400 kHz (fast mode, both chips allow it): needed for a full 50 Hz
  Wire.setTimeOut(20);                    // ms: a stuck bus gives up instead of hanging
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
    Wire.end(); delay(500); Wire.begin(PIN_SDA, PIN_SCL); Wire.setClock(400000); Wire.setTimeOut(20);
  }
  Serial.print(F("# BMI270 at 0x")); Serial.println(BMI, HEX);
  Serial.print(F("# GYR_CAS factor_zx = ")); Serial.println(casZX);
  Serial.print(F("# MAG CAL ")); Serial.print(F(MAG_CAL_STAMP));
  Serial.print(F("  field ")); Serial.print(MAG_FIELD_MG, 1); Serial.println(F(" mG"));
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
  float mr[3] = {r[6] * 1.5f - MAG_HARD[0], r[7] * 1.5f - MAG_HARD[1], r[8] * 1.5f - MAG_HARD[2]};   // mG, hard iron off
  for (int i = 0; i < 3; i++)                                                            // soft iron
    out[6 + i] = clamp16(lround(MAG_SOFT[i][0] * mr[0] + MAG_SOFT[i][1] * mr[1] + MAG_SOFT[i][2] * mr[2]));

  static unsigned long lastTag = 0;
  if (millis() - lastTag > 2000)          // repeat: the ESP32 may not reset when a port opens
  {
    lastTag = millis();
    Serial.print(F("# MAG CAL ")); Serial.print(F(MAG_CAL_STAMP));
    Serial.print(F("  field ")); Serial.print(MAG_FIELD_MG, 1); Serial.println(F(" mG"));
  }
  lastSample = millis();
  Serial.print(F("Raw:"));
  for (int i = 0; i < 9; i++) { Serial.print(out[i]); if (i < 8) Serial.print(','); }
  Serial.print(F("\r\n"));   // MotionCal ends a line on CR (13), so CR LF is required
}
