/*
Comunicação I2C para com o sensor MPU 
Endereço padrão MPU = 0x69
AD0 = HIGH 
*/

#ifndef _I2Cyangui_H
#define _I2Cyangui_H


#include <Arduino.h>
#include <Wire.h>

const uint8_t IMUAddress = 0x69; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication


class comunication {
  public:
  const uint8_t IMUAddress = 0x69; // AD0 is logic low on the PCB
  const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

  uint8_t i2cWritey(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
  uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

};

#endif