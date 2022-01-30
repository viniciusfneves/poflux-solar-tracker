#pragma once

#include <Wire.h>
#include <math.h>

#include <debugLED/debugLED.hpp>

#define RAW_TO_G 16384.
#define RAW_TO_RAD_PER_SECOND 131 * 0.01745
#define RAW_TO_DEGREES_PER_SECOND 131.
#define RAW_TO_CELSIUS 340.
#define SECONDS_TO_RECONNECT 1

//Estutura para armazenar os dados do MPU
struct MPUData {
    double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
    double roll, pitch;
    bool isTrusted = false;
};

class MPU6050_Solar {
   private:
    int _mpuAddress;

    void _debugI2CResponse(const byte errorCode) {
        switch (errorCode) {
            case 0:
                Serial.printf("MPU: Sucesso!");
                break;
            case 1:
                Serial.printf("MPU ERROR: Mensagem grande demais para o buffer");
                break;
            case 2:
                Serial.printf("MPU ERROR: NACK recebido no endereço de transmissão");
                break;
            case 3:
                Serial.printf("MPU ERROR: NACK recebido ao transmitir a mensagem");
                break;
            case 4:
                Serial.printf("MPU ERROR: Erro genérico...");
                break;

            default:
                Serial.printf("MPU ERROR: %02d - Código de erro desconhecido", errorCode);
                break;
        }
    }

    void _handleErrors() {
        updateLEDState(LEDState::solving_error);
        reset();
    }

    void
    _init() {
        byte response;
        // -- Configurações gerais -- //
        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x19);  // Registro SMPLRT_DIV
        Wire.write(0);
        response = Wire.endTransmission();
        if (response != 0)
            throw(response);

        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x1A);  // Registro CONFIG
        Wire.write(0);
        response = Wire.endTransmission();
        if (response != 0)
            throw(response);

        // -- Configuração do Gyro -- //
        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x1B);  // Registro de configuração do Gyro
        Wire.write(0);     // Configura o Full Scale Range para + ou - 250 graus por segundo
        response = Wire.endTransmission();
        if (response != 0)
            throw(response);

        // -- Configuração do Acelerômetro -- //
        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x1C);  // Registro de configuração do Accel
        Wire.write(0);     // Configura o Full Scale Range para + ou - 2g graus por segundo
        response = Wire.endTransmission();
        if (response != 0)
            throw(response);

        // Ativa o MPU
        Wire.beginTransmission(_mpuAddress);
        Wire.write(0x6B);  // Registro PWR_MGMT_1
        Wire.write(0);     // Ativa o MPU-6050
        response = Wire.endTransmission();
        if (response != 0)
            throw(response);

        updateLEDState(LEDState::running);
        delay(100);  // Aguarda o MPU estabilizar a leitura
    }

   public:
    MPU6050_Solar(int MPUAddress) {
        _mpuAddress = MPUAddress;
    }

    // --    Configura o MPU    -- //
    void init() {
        try {
            _init();
        } catch (const byte e) {
            updateLEDState(LEDState::error);
            for (int cont = 0; cont < SECONDS_TO_RECONNECT * 2; cont++) {
                delay(500);
                Serial.print(".");
            }
            Serial.printf("\nReconenctando ao MPU...");
            init();
        }
    }

    void reset() {
        updateLEDState(LEDState::solving_error);
        try {
            _init();
        } catch (const byte e) {
            Serial.printf("\nFalha no MPU -> Reiniciando conexão...");
            delay(200);
            reset();
        }
    }

    void readMPU(MPUData& _data) {
        try {
            Wire.beginTransmission(_mpuAddress);
            Wire.write(0x3B);  // Começa a leitura abaixo no endereço do registrador 0x3B (ACCEL_XOUT_H)
            byte response = Wire.endTransmission(false);
            if (response != 0)
                throw(response);

            //Solicita os dados do sensor
            byte responseLenght = Wire.requestFrom(_mpuAddress, 14);
            if (responseLenght == 14)
                _data.isTrusted = true;
            else
                throw "MPU ERROR: Falha na leitura do MPU";

            //Armazena o valor dos registradores em um array
            int16_t mpuRawData[7];

            for (int i = 0; i < 7; i++) {
                mpuRawData[i] = Wire.read() << 8 | Wire.read();
            }

            // Converte as unidades dos dados recebidos para as correspondentes
            // Aceleração  = G
            // Temperatura = Celsius
            // Gyro        = Rad/s
            _data.AcX = (float)mpuRawData[0] / RAW_TO_G;
            _data.AcY = (float)mpuRawData[1] / RAW_TO_G;
            _data.AcZ = (float)mpuRawData[2] / RAW_TO_G;
            _data.Tmp = (float)mpuRawData[3] / RAW_TO_CELSIUS + 35;
            _data.GyX = (float)mpuRawData[4] / RAW_TO_DEGREES_PER_SECOND;
            _data.GyY = (float)mpuRawData[5] / RAW_TO_DEGREES_PER_SECOND;
            _data.GyZ = (float)mpuRawData[6] / RAW_TO_DEGREES_PER_SECOND;

            int sigZ = _data.AcZ < 0 ? -1 : 1;

            _data.roll = RAD_TO_DEG * atan2(_data.AcY, sigZ * sqrt(_data.AcZ * _data.AcZ + _data.AcX * _data.AcX));
            _data.pitch = -RAD_TO_DEG * atan2(_data.AcX, sqrt(_data.AcZ * _data.AcZ + _data.AcY * _data.AcY));
            updateLEDState(LEDState::running);

            //-- DEBUG --//

#ifdef DEBUG_MPU
            // Serial.printf(" | X Gs: %02.1f | Y Gs: %02.1f | Z Gs: %02.1f", _data.AcX, _data.AcY, _data.AcZ);
            Serial.printf(" | Roll: %04.1f", _data.roll);
            //Serial.printf(" | Pitch: %04.1f", _data.pitch);
#endif
        } catch (const byte e) {
            _debugI2CResponse(e);
            _data.isTrusted = false;
            _handleErrors();
        } catch (const char* e) {
            Serial.println(e);
            _data.isTrusted = false;
            _handleErrors();
        }
    }
};

MPUData mpuData;
MPU6050_Solar mpu(0x69);