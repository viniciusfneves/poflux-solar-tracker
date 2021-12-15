#ifndef MPU_HPP
#define MPU_HPP

#include <Wire.h>
#include <math.h>

//Estutura para armazenar os dados do MPU
struct MPUData {
    double AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
    double roll, pitch, yaw;
};

class MPU6050_Solar {
   private:
    int _MPUAddress;
    int _mpuErrorCounter = 0;
    const unsigned int _RAW_TO_G = 16384;
    const double _RAW_TO_RAD_PER_SECOND = 131 * 0.01745;
    const int _RAW_TO_DEGREES_PER_SECOND = 131;
    const int _RAW_TO_CELSIUS = 340;

    void _debugI2CResponse(byte errorCode) {
        Serial.print(" | ");
        switch (errorCode) {
            case 0:
                Serial.print("MPU: Sucesso!");
                break;
            case 1:
                Serial.print("MPU ERROR: 1 - Mensagem grande demais para o buffer");
                break;
            case 2:
                Serial.print("MPU ERROR: 2 - NACK recebido no endereço de transmissão");
                break;
            case 3:
                Serial.print("MPU ERROR: 3 - NACK recebido ao transmitir a mensagem");
                break;
            case 4:
                Serial.print("MPU ERROR: 4 - Erro genérico...");
                break;

            default:
                Serial.print("MPU ERROR: ");
                Serial.print(errorCode);
                Serial.print(" - Código de erro desconhecido");
                break;
        }
    }

   public:
    MPU6050_Solar(int MPUAddress) {
        _MPUAddress = MPUAddress;
    }

    void init() {
        // --    Configura o MPU    -- //
        // -- Configuração do Gyro -- //
        try {
            Wire.beginTransmission(_MPUAddress);
            Wire.write(0x1B);  // Registro de configuração do Gyro
            Wire.write(0x00);  // Configura o Full Scale Range para + ou - 250 graus por segundo
            byte response = Wire.endTransmission();
            if (response != 0)
                throw(response);

            // -- Configuração do Acelerômetro -- //
            Wire.beginTransmission(_MPUAddress);
            Wire.write(0x1C);  // Registro de configuração do Gyro
            Wire.write(0x00);  // Configura o Full Scale Range para + ou - 2g graus por segundo
            response = Wire.endTransmission();
            if (response != 0)
                throw(response);

            // Ativa o MPU
            Wire.beginTransmission(_MPUAddress);
            Wire.write(0x6B);  // Registro PWR_MGMT_1
            Wire.write(0);     // definido como zero (ativa o MPU-6050)
            response = Wire.endTransmission();
            if (response != 0)
                throw(response);

        } catch (byte e) {
            const int secondsToRestart = 5;
            Serial.printf("\n\n\nMPU não pôde ser encontrado ou calibrado corretamente. Tentando reiniciar em %d segundos\n", secondsToRestart);
            for (int cont = 0; cont < secondsToRestart * 2; cont++) {
                delay(500);
                Serial.print(".");
            }
            Serial.print("\nReiniciando aplicação!");
            ESP.restart();
        }
        // Aguarda o MPU estabilizar a leitura
        delay(500);
    }

    void readMPU(MPUData& _data) {
        try {
            Wire.beginTransmission(_MPUAddress);
            Wire.write(0x3B);  // Começa a leitura abaixo no endereço do registrador 0x3B (ACCEL_XOUT_H)
            byte response = Wire.endTransmission();
            if (response != 0)
                throw(response);

            //Solicita os dados do sensor
            byte responseLenght = Wire.requestFrom(_MPUAddress, 14);
            if (responseLenght == 14)
                _mpuErrorCounter = 0;

            //Armazena o valore cru dos registradores do sensor nas variaveis correspondentes
            _data.AcX = Wire.read() << 8 | Wire.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
            _data.AcY = Wire.read() << 8 | Wire.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
            _data.AcZ = Wire.read() << 8 | Wire.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
            _data.Tmp = Wire.read() << 8 | Wire.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
            _data.GyX = Wire.read() << 8 | Wire.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
            _data.GyY = Wire.read() << 8 | Wire.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
            _data.GyZ = Wire.read() << 8 | Wire.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

            // Converte as unidades dos dados recebidos para as correspondentes
            // Aceleração  = Gs
            // Temperatura = Celsius
            // Gyro        = Rad/s
            _data.AcX /= _RAW_TO_G;
            _data.AcY /= _RAW_TO_G;
            _data.AcZ /= _RAW_TO_G;
            _data.Tmp = _data.Tmp / _RAW_TO_CELSIUS + 35;
            _data.GyX /= _RAW_TO_RAD_PER_SECOND;
            _data.GyY /= _RAW_TO_RAD_PER_SECOND;
            _data.GyZ /= _RAW_TO_RAD_PER_SECOND;

            // GAMBIARRA PARA DADOS NÃO ESPERADOS ENVIADOS PELO SENSOR
            if (_data.AcY > 2) {
                _data.AcY -= 4;
                _data.roll = RAD_TO_DEG * (atan2(-_data.AcZ, _data.AcY) + PI / 2);
            }

            else
                _data.roll = RAD_TO_DEG * (atan2(-_data.AcZ, _data.AcY) + PI / 2);

                //_data.pitch = RAD_TO_DEG * (atan2(-_data.AcZ, _data.AcX) + PI / 2);

                //-- DEBUG --//

#ifdef DEBUG_MPU
            // Serial.print(" | X Gs: ");
            // Serial.printf("%05.3f", _data.AcX);
            // Serial.print(" | Y Gs: ");
            // Serial.printf("%05.3f", _data.AcY);
            // Serial.print(" | Z Gs: ");
            // Serial.printf("%5.3f", _data.AcZ);
            Serial.print(" | MPU Roll: ");
            Serial.printf("%5.3f", _data.roll);
            // Serial.print(" | MPU Pitch: ");
            // Serial.printf("%5.3f", _data.pitch);
#endif
        } catch (byte e) {
            // Caso já tenham havido 200 tentativas consecutivas de leitura sem sucesso: tenta reiniciar o ESP
            if (_mpuErrorCounter >= 200) {
                Serial.print("\n//---- FALHA NA LEITURA DO MPU ----//");
                ESP.restart();
            }
            _mpuErrorCounter++;
            _debugI2CResponse(e);
            delay(50);  // Esse delay é necessário, caso contrário o BUS I2C do ESP crasha com a excessão
        }
    }
};

#endif  // MPU_HPP