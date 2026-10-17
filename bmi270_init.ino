// BMI270 initialisation with the Bosch config file (datasheet section 4.4)
// Arduino Uno, BMI270 at 0x68 (SDO = GND)
// Keep bmi270_config.h in the same folder as this .ino

#include <Wire.h>
#include "bmi270_config.h"

const uint8_t BMI_ADDR = 0x68;

bool writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(BMI_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
  Wire.beginTransmission(BMI_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BMI_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

int readReg(uint8_t reg)
{
  uint8_t v;
  return readRegs(reg, &v, 1) ? v : -1;
}

bool bmi270Init()
{
  // 0. Soft reset so INIT_CTRL=1 is only ever written once per reset
  writeReg(0x7E, 0xB6);                 // CMD = softreset
  delay(5);
  readReg(0x00);                        // dummy read

  int id = readReg(0x00);
  Serial.print(F("CHIP_ID = 0x")); Serial.println(id, HEX);
  if (id != 0x24) return false;

  // 1. Disable advanced power save, wait >= 450 us
  writeReg(0x7C, 0x00);                 // PWR_CONF
  delay(1);

  // 2. Prepare config load
  writeReg(0x59, 0x00);                 // INIT_CTRL = 0

  // 3. Burst write the 8192-byte config to INIT_DATA (0x5E).
  //    Uno Wire buffer is 32 bytes, so send 16-byte chunks.
  //    INIT_ADDR_0/1 hold the start address in words (bytes / 2).
  const uint16_t CHUNK = 16;
  for (uint16_t i = 0; i < sizeof(bmi270_config_file); i += CHUNK)
  {
    uint16_t word = i / 2;
    Wire.beginTransmission(BMI_ADDR);
    Wire.write(0x5B);                   // INIT_ADDR_0, auto-increments to 0x5C
    Wire.write(word & 0x0F);            // bits 0..3
    Wire.write(word >> 4);              // bits 4..11
    Wire.endTransmission();

    Wire.beginTransmission(BMI_ADDR);
    Wire.write(0x5E);                   // INIT_DATA
    for (uint16_t k = 0; k < CHUNK; k++)
      Wire.write(pgm_read_byte(&bmi270_config_file[i + k]));
    if (Wire.endTransmission() != 0)
    {
      Serial.print(F("Upload failed at byte ")); Serial.println(i);
      return false;
    }
  }

  // 4. Complete config load
  writeReg(0x59, 0x01);                 // INIT_CTRL = 1

  // 5. Wait for INTERNAL_STATUS.message = 0x01 (max 20 ms)
  int st = 0;
  for (int t = 0; t < 10; t++)
  {
    delay(5);
    st = readReg(0x21);
    if ((st & 0x0F) == 0x01) break;
  }
  Serial.print(F("INTERNAL_STATUS = 0x")); Serial.println(st, HEX);
  if ((st & 0x0F) != 0x01) return false;

  // 6. Performance mode (datasheet example): acc + gyr + temp on
  writeReg(0x7D, 0x0E);                 // PWR_CTRL
  writeReg(0x40, 0xA8);                 // ACC_CONF: 100 Hz, normal, filter_perf
  writeReg(0x42, 0xE9);                 // GYR_CONF: 200 Hz, normal, noise+filter perf
  writeReg(0x7C, 0x02);                 // PWR_CONF: adv_power_save off
  delay(50);
  return true;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(10);

  if (bmi270Init())
    Serial.println(F("BMI270 init OK"));
  else
  {
    Serial.println(F("BMI270 init FAILED"));
    while (1) {}
  }
}

void loop()
{
  // Quick proof of life: default ranges are +-8 g (4096 LSB/g) and +-2000 dps (16.4 LSB/dps)
  uint8_t b[12];
  if (!readRegs(0x0C, b, 12)) { Serial.println(F("read error")); delay(500); return; }

  int16_t v[6];
  for (int i = 0; i < 6; i++) v[i] = (int16_t)(b[2 * i + 1] << 8 | b[2 * i]);

  Serial.print(F("acc g: "));
  for (int i = 0; i < 3; i++) { Serial.print(v[i] / 4096.0, 3); Serial.print(' '); }
  Serial.print(F("  gyr dps: "));
  for (int i = 3; i < 6; i++) { Serial.print(v[i] / 16.4, 2); Serial.print(' '); }
  Serial.println();
  delay(200);
}
