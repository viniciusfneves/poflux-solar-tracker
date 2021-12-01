#ifndef IMU_HPP
#define IMU_HPP

#include <Wire.h>

class IMU {
   private:
    int _IMUAddress;
    unsigned int _I2C_timeout;
    IMUData& _data;

   public:
    IMU(int IMUAddress, IMUData& data, int I2C_timeout = 1000) {
        _IMUAddress = IMUAddress;
        _I2C_timeout = I2C_timeout;
        _data = data;
    }

    int readMPU() {
        Wire.beginTransmission(_IMUAddress);
        Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
        Wire.endTransmission(false);
        //Solicita os dados do sensor
        Wire.requestFrom(_IMUAddress, 14, true);
        //Armazena o valor dos sensores nas variaveis correspondentes
        _data.AcX = Wire.read() << 8 | Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
        _data.AcY = Wire.read() << 8 | Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        _data.AcZ = Wire.read() << 8 | Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        _data.Tmp = Wire.read() << 8 | Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
        _data.GyX = Wire.read() << 8 | Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        _data.GyY = Wire.read() << 8 | Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        _data.GyZ = Wire.read() << 8 | Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    }
};

struct IMUData {
    int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
};

#endif  // IMU_HPP