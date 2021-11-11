/*
Comunicação I2C para com o sensor MPU 
Endereço padrão MPU = 0x69
AD0 = HIGH 
*/

#include <Arduino.h>
#include <Wire.h>
#include "I2Cyangui.h"

uint8_t comunication :: i2cWritey(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) 
{
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWritey failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

/*
uint8_t i2cWritey2(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWritey(registerAddress, &data, 1, sendStop); // Returns 0 on success
}
*/

uint8_t comunication :: i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) 
{
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
