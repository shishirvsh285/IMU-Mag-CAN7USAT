// IIS2MDC magnetometer: init + continuous read (datasheet DocID030986 Rev 1)
// Arduino Uno, IIS2MDC at 0x1E, 3.3 V supply
// Prints:  mag mG: <x> <y> <z>     (milligauss, 1.5 mG/LSB)

#include <Wire.h>

const uint8_t MAG_ADDR    = 0x1E;
const uint8_t WHO_AM_I    = 0x4F;  // should read 0x40
const uint8_t CFG_REG_A   = 0x60;
const uint8_t CFG_REG_B   = 0x61;
const uint8_t CFG_REG_C   = 0x62;
const uint8_t STATUS_REG  = 0x67;  // bit3 Zyxda = new X,Y,Z set ready
const uint8_t OUTX_L_REG  = 0x68;  // X L/H, Y L/H, Z L/H up to 0x6D

const float MG_PER_LSB = 1.5;

bool writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

int readReg(uint8_t reg)
{
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MAG_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

bool magInit()
{
  int id = readReg(WHO_AM_I);
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(id, HEX);
  if (id != 0x40) return false;

  writeReg(CFG_REG_A, 0x20);   // SOFT_RST: reset config and user registers
  delay(20);

  // CFG_REG_A: COMP_TEMP_EN=1 (datasheet: must be 1), high-resolution,
  //            ODR = 50 Hz (ODR1:0 = 10), continuous mode (MD = 00)
  writeReg(CFG_REG_A, 0x88);
  // CFG_REG_B: OFF_CANC=1 (offset cancellation, offset spec is +-60 mG with it), LPF=1 (BW = ODR/4)
  writeReg(CFG_REG_B, 0x03);
  // CFG_REG_C: BDU=1 so low and high bytes always come from the same sample
  writeReg(CFG_REG_C, 0x10);

  delay(50);
  Serial.print(F("CFG A/B/C = 0x"));
  Serial.print(readReg(CFG_REG_A), HEX); Serial.print(F(" 0x"));
  Serial.print(readReg(CFG_REG_B), HEX); Serial.print(F(" 0x"));
  Serial.println(readReg(CFG_REG_C), HEX);
  return true;
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  delay(20);

  if (magInit())
    Serial.println(F("IIS2MDC init OK"));
  else
  {
    Serial.println(F("IIS2MDC init FAILED"));
    while (1) {}
  }
}

void loop()
{
  // wait for a new X,Y,Z set
  int st = readReg(STATUS_REG);
  if (st < 0 || !(st & 0x08)) return;

  // read the 6 output bytes one at a time (with BDU on, each L/H pair stays matched)
  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++)
  {
    int v = readReg(OUTX_L_REG + i);
    if (v < 0) return;
    b[i] = v;
  }
  int16_t x = (int16_t)(b[1] << 8 | b[0]);
  int16_t y = (int16_t)(b[3] << 8 | b[2]);
  int16_t z = (int16_t)(b[5] << 8 | b[4]);

  Serial.print(F("mag mG: "));
  Serial.print(x * MG_PER_LSB, 1); Serial.print(' ');
  Serial.print(y * MG_PER_LSB, 1); Serial.print(' ');
  Serial.println(z * MG_PER_LSB, 1);
}
