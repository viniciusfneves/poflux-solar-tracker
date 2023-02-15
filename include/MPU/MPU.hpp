#pragma once

#include <Kalman.h>
#include <Wire.h>
#include <math.h>

#include <configurations/configurations.hpp>
#include <debugLED/debugLED.hpp>

#define RAW_TO_G 16384.
#define RAW_TO_RAD_PER_SECOND 131 * 0.01745
#define RAW_TO_DEGREES_PER_SECOND 131.
#define RAW_TO_CELSIUS 340.
#define SECONDS_TO_RECONNECT 1
#define MPU_PWR_CTRL_PIN 15

Kalman kalmanX;  // Create the Kalman instances
Kalman kalmanY;

// Estutura para armazenar os dados do MPU
struct MPUData {
    bool   isTrusted = false;
    double AcX, AcY, AcZ, Tmp, GyXRate, GyYRate, GyZRate;
    double roll, pitch;
    double kalAngleX, kalAngleY;  // Ângulo usando filtro de Kalman
};

class MPU6050_Solar {
   private:
    int           _mpuAddress;
    unsigned long _timer = 0;

    void _debugI2CResponse(const byte errorCode) {
        switch (errorCode) {
            case 0:
                Serial.printf("MPU: Sucesso!");
                break;
            case 1:
                Serial.printf(
                    "MPU ERROR: Mensagem grande demais para o buffer");
                break;
            case 2:
                Serial.printf(
                    "MPU ERROR: NACK recebido no endereço de transmissão");
                break;
            case 3:
                Serial.printf(
                    "MPU ERROR: NACK recebido ao transmitir a mensagem");
                break;
            case 4:
                Serial.printf("MPU ERROR: Erro genérico...");
                break;

            default:
                Serial.printf("MPU ERROR: %02d - Código de erro desconhecido",
                              errorCode);
                break;
        }
    }

    void _handleErrors() {
        configs.mode == Mode::Halt;
        updateLEDState(LEDState::solving_error);
        reset();
    }

    bool _init() {
        digitalWrite(MPU_PWR_CTRL_PIN, HIGH);
        delay(200);
        byte response;
        // -- Configurações gerais -- //
        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x19);  // Registro SMPLRT_DIV
        Wire.write(0);
        response = Wire.endTransmission();
        if (response != 0) return false;

        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x1A);  // Registro CONFIG
        Wire.write(0);
        response = Wire.endTransmission();
        if (response != 0) return false;

        // -- Configuração do Gyro -- //
        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x1B);  // Registro de configuração do Gyro
        Wire.write(0);     // Configura o Full Scale Range para + ou - 250 graus
                           // por segundo
        response = Wire.endTransmission();
        if (response != 0) return false;

        // -- Configuração do Acelerômetro -- //
        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x1C);  // Registro de configuração do Accel
        Wire.write(0);     // Configura o Full Scale Range para + ou - 2gs
        response = Wire.endTransmission();
        if (response != 0) return false;

        // Ativa o MPU
        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x6B);  // Registro PWR_MGMT_1
        Wire.write(0);     // Ativa o MPU-6050
        response = Wire.endTransmission();
        if (response != 0) return false;

        //! READS THE MPU 50 TIMES. THIS STABILIZES THE FINAL OUTPUT VALUE
        for (size_t i = 0; i < 50; i++) {
            delay(10);
            readMPU();
        }
        _timer = micros();
        kalmanX.setAngle(data.roll);  // Set starting angle
        kalmanY.setAngle(data.pitch);

        updateLEDState(LEDState::running);

        return true;
    }

   public:
    MPU6050_Solar(int MPUAddress) { _mpuAddress = MPUAddress; }

    MPUData data;

    // --    Configura o MPU    -- //
    void init() {
        pinMode(MPU_PWR_CTRL_PIN, OUTPUT);
        digitalWrite(MPU_PWR_CTRL_PIN, LOW);
        delay(200);
        if (!_init()) {
            updateLEDState(LEDState::error);
            Serial.printf("\nErro de conexão com a IMU...");
            delay(30000);
            ESP.restart();
        }
    }

    void reset() {
        while (!_init()) {
            digitalWrite(MPU_PWR_CTRL_PIN, LOW);
            delay(200);
        }
    }

    void readMPU() {
        try {
            Wire.beginTransmission(_mpuAddress);
            Wire.write(0x3B);  // Começa a leitura abaixo no endereço do
                               // registrador 0x3B (ACCEL_XOUT_H)
            byte response = Wire.endTransmission(false);
            if (response != 0) throw(response);

            // Solicita os dados do sensor
            byte responseLenght = Wire.requestFrom(_mpuAddress, 14);
            if (responseLenght == 14) {
                updateLEDState(LEDState::running);
                data.isTrusted = true;
            } else
                throw "MPU ERROR: Falha na leitura do MPU";

            // Armazena o valor dos registradores em um array
            int16_t mpuRawData[7];

            for (int i = 0; i < 7; i++) {
                mpuRawData[i] = Wire.read() << 8 | Wire.read();
            }
            // Delta time
            double dt = (micros() - _timer) / 1000000.;
            _timer    = micros();

            // Converte as unidades dos dados recebidos para as correspondentes
            // Aceleração  = G
            // Temperatura = Celsius
            // Gyro        = Rad/s
            data.AcX     = (double)mpuRawData[0] / RAW_TO_G;
            data.AcY     = (double)mpuRawData[1] / RAW_TO_G;
            data.AcZ     = (double)mpuRawData[2] / RAW_TO_G;
            data.Tmp     = (double)mpuRawData[3] / RAW_TO_CELSIUS + 35;
            data.GyXRate = (double)mpuRawData[4] / RAW_TO_DEGREES_PER_SECOND;
            data.GyYRate = (double)mpuRawData[5] / RAW_TO_DEGREES_PER_SECOND;
            data.GyZRate = (double)mpuRawData[6] / RAW_TO_DEGREES_PER_SECOND;

            data.roll  = atan2(data.AcY, data.AcZ) * RAD_TO_DEG;
            data.pitch = atan(-data.AcX /
                              sqrt(data.AcY * data.AcY + data.AcZ * data.AcZ)) *
                         RAD_TO_DEG;

            // This fixes the transition problem when the accelerometer angle
            // jumps between -180 and 180 degrees
            if ((data.roll < -90 && data.kalAngleX > 90) ||
                (data.roll > 90 && data.kalAngleX < -90)) {
                kalmanX.setAngle(data.roll);
            } else
                // Angle using a Kalman filter
                data.kalAngleX = kalmanX.getAngle(data.roll, data.GyXRate, dt);

            if (abs(data.kalAngleX) > 90)
                data.GyYRate =
                    -data.GyYRate;  // Invert rate, so it fits the restriced
                                    // accelerometer reading
            data.kalAngleY = kalmanY.getAngle(data.pitch, data.GyYRate, dt);

            //! ROUND TO 1 DECIMAL PLACE
            data.kalAngleX = round(data.kalAngleX * 10) / 10.;
            data.kalAngleY = round(data.kalAngleY * 10) / 10.;

        } catch (const byte e) {
            _debugI2CResponse(e);
            data.isTrusted = false;
            _handleErrors();
        } catch (const char* e) {
            Serial.println(e);
            data.isTrusted = false;
            _handleErrors();
        }
    }
};

MPU6050_Solar mpu(0x69);