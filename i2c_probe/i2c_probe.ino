// I2C probe: scans the bus and reads the chip IDs once a second
#include <Wire.h>
void setup() { Serial.begin(115200); Wire.begin(); Wire.setClock(100000); Wire.setWireTimeout(3000, true); }
int rd1(uint8_t dev, uint8_t reg) {
  Wire.beginTransmission(dev); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(dev, (uint8_t)1) != 1) return -1;
  return Wire.read();
}
void loop() {
  Serial.print(F("found:"));
  for (uint8_t a = 1; a < 127; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) { Serial.print(F(" 0x")); Serial.print(a, HEX); } }
  if (Wire.getWireTimeoutFlag()) { Serial.print(F("  [I2C TIMEOUT: bus stuck]")); Wire.clearWireTimeoutFlag(); }
  Serial.print(F("   BMI270 id=0x")); Serial.print(rd1(0x68, 0x00), HEX);
  Serial.print(F(" (want 0x24)   IIS2MDC id=0x")); Serial.print(rd1(0x1E, 0x4F), HEX);
  Serial.println(F(" (want 0x40)"));
  delay(1000);
}
