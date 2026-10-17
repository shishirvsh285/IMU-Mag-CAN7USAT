# i2c_probe

Wiring check. Once a second it scans the I2C bus and reads both chip IDs.

```
found: 0x1E 0x68   BMI270 id=0x24 (want 0x24)   IIS2MDC id=0x40 (want 0x40)
```

- Both addresses found and both IDs right: wiring is fine.
- `[I2C TIMEOUT: bus stuck]`: SDA or SCL is loose, shorted, or pulled the wrong way.
- An ID of `0xFFFFFFFF` (-1): that chip does not answer. Check its power (3.3 V) and address pin.

Run this first whenever another sketch prints nothing.
