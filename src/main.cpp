// analogWrite to digitalWrite em main.cpp e Motor_Comands.cpp (comentado com 0-255)
#include <Arduino.h>
#include <Wire.h>
#include <analogWrite.h>

//----------------------------RTC settings----------------------------------------//
#include <RtcDS3231.h>
#include <RtcDateTime.h>

RtcDS3231<TwoWire> rtc(Wire);  //Criação do objeto do tipo DS3231
RtcDateTime RTC_Data;          //Criação do objeto do tipo RTCDateTime
                               //RTC em seu endereço padrão 0x68

//----------------------------Kalman settings-------------------------------------//
#include <Kalman.h>     // Source: https://github.com/TKJElectronics/KalmanFilter
#define RESTRICT_PITCH  // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX;  //Criação de objeto
Kalman kalmanY;  //Criação de objeto

//----------------------------IMU settings---------------------------------------//
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double gyroXangle, gyroYangle;  // Angle calculate using the gyro only
double compAngleX, compAngleY;  // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14];  // Buffer for I2C data
uint8_t dataI2c;

//----------------------------Timerlord settings-------------------------------------//
#include <Time.h>
float const LONGITUDE = -43.2311486;
float const LATITUDE = -22.8613427;
int const TIMEZONE = -3;
SunLight Sun_Time;

//----------------------------I2c Comunication---------------------------------------//
#include <I2Cyangui.h>
comunication com;

//----------------------------Driver Settings---------------------------------------//

#include <Motor_Comands.h>
#define LPWM 4     //lpwm
#define RPWM 2     //rpwm
#define ENABLE 19  //pwm enable

Driver_Setup Motor;

//-------------------------------Limits-------------------------------//

int Threshold_Max;
int Threshold_Min;
int Max_Angle_Limit;
int Min_Angle_Limit;

//----------------------------PID Settings----------------------------//

#include <PID.h>
int Setpoint, Input, Output;  //Define Variables we'll be connecting to
PID_Control PID_Calculator;   //PID object
int Erro;

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::://

void setup() {
    //----------------------------I2C settings----------------------------------//

    Serial.begin(115200);
    Wire.begin();

    //----------------------------Driver Configurations----------------------------//
    pinMode(LPWM, OUTPUT);
    pinMode(RPWM, OUTPUT);
    pinMode(ENABLE, OUTPUT);

    //---------------------------------RTC settings-----------------------------//
    rtc.Begin();  //Inicialização do RTC DS3231
    RtcDateTime time = RtcDateTime(__DATE__, __TIME__);
    rtc.SetDateTime(time);  //Configurando valores iniciais do RTC DS3231

    Wire.setClock(400000UL);  // Set I2C frequency to 400kHz

    //frequency between 10kHz-400kHz

    i2cData[0] = 7;     // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
    i2cData[1] = 0x00;  // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
    i2cData[2] = 0x00;  // Set Gyro Full Scale Range to ±250deg/s
    i2cData[3] = 0x00;  // Set Accelerometer Full Scale Range to ±2g

    while (com.i2cWritey(0x19, i2cData, 4, false))  // Write to all four registers at once
        ;
    dataI2c = 0x01;
    while (com.i2cWritey(0x6B, &dataI2c, 1, true))  // PLL with X axis gyroscope reference and disable sleep mode
        ;

    while (com.i2cRead(0x75, i2cData, 1, true))
        ;
    if (i2cData[0] != 0x68) {  // Read "WHO_AM_I" register //0x69 for AD0 High default of solar sensor project
        Serial.print(F("Error reading sensor"));
        while (1)
            ;
    }

    delay(100);  // Wait for sensor to stabilize
    //----------------------------------------------------------------------//

    //---------------------------kalman----------------------------------//
    // Set kalman and gyro starting angle //
    while (com.i2cRead(0x3B, i2cData, 6, true))
        ;
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
#ifdef RESTRICT_PITCH                              // Eq. 25 and 26
    double roll = atan2(accY, accZ) * RAD_TO_DEG;  //atan2 outputs the value of -π to π (radians. It is then converted from radians to degrees
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    kalmanX.setAngle(roll);  // Set starting angle
    kalmanY.setAngle(pitch);
    gyroXangle = roll;
    gyroYangle = pitch;
    compAngleX = roll;
    compAngleY = pitch;

    timer = micros();
}

//------------------------------------------------------------------------//
//Atualiza os valores de data e hora instantâneos lidos pelo RTC
//Imprime no monitor Serial dados do RTC
void CallRTC(RtcDateTime &RTC_Data) {
    RTC_Data = rtc.GetDateTime();  //Atribuindo valores instantâneos de data e hora à instância data e hora

    Serial.print(RTC_Data.Day());  //Imprimindo o Dia
    Serial.print("-");
    Serial.print(RTC_Data.Month());  //Imprimindo o Mês
    Serial.print("-");
    Serial.print(RTC_Data.Year());  //Imprimindo o Ano
    Serial.print("  ");
    Serial.print(RTC_Data.Hour());  //Imprimindo a Hora
    Serial.print(":");
    Serial.print(RTC_Data.Minute());  //Imprimindo o Minuto
    Serial.print(":");
    Serial.print(RTC_Data.Second());  //Imprimindo o Segundo
    Serial.print("  ");
}

//------------------------------------------------------------------------//

void Motor_Direction(int erro, int PWM, int input) {
    Threshold_Max = 3;
    Threshold_Min = -Threshold_Max;
    Max_Angle_Limit = 85;
    Min_Angle_Limit = -Max_Angle_Limit;

    if (90 > input * -1 > -90 && erro < Threshold_Min)  //Girar no sentido horário
    {
        analogWrite(ENABLE, PWM);  //0-255
        digitalWrite(LPWM, LOW);
        digitalWrite(RPWM, HIGH);
        Serial.print(" | Motor: Horário");
    } else if (-90 < input < 90 && erro > Threshold_Max)  //Girar no sentido antihorário
    {
        analogWrite(ENABLE, PWM);  //0-255
        digitalWrite(LPWM, HIGH);
        digitalWrite(RPWM, LOW);
        Serial.print(" | Motor: Anti-Horário");
    } else if (Threshold_Min < erro < Threshold_Max)  //Não Girar
    {
        analogWrite(ENABLE, PWM);  //0-255
        digitalWrite(LPWM, LOW);
        digitalWrite(RPWM, LOW);
        Serial.print(" | Motor: Parado");
    }
}
//------------------------------------------------------------------------//

int mapeamento(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//------------------------------------------------------------------------//
//Lê os bytes recebidos pela comunicação Master-Slave
void Erro_Read(int Setpoint, int Input) {
    //Setpoint = 0; //test angle, discomment to work properly
    Setpoint = constrain(Setpoint, -80, 80);

    Input = constrain(Input, -80, 80);

    Erro = Setpoint - Input;

    Output = PID_Calculator.PID(abs(Erro), true, Threshold_Max);

    Output = mapeamento(Output, -216, 216, 110, 230);  //mudar valores para variáveis

#ifdef DEBUG
    Serial.print("Setpoint: ");
    Serial.print(Setpoint);
    Serial.print(" | Erro: ");
    Serial.print(Erro);
#endif

    Motor_Direction(Erro, Output, Input);
}
//------------------------------------------------------------------------//

//::::::::::::::::::::::::::::::::::::LOOP:::::::::::::::::::::::::::::::::::::::://
void loop() {
    //----------------------------------------------------------------------//
    // Calling RTC before for record of time //
    CallRTC(RTC_Data);  //Buscando Dados RTC
    //----------------------------------------------------------------------//

    //----------------------------------------------------------------------//
    // Calculating Sun parameters  //

    Sun_Time.Sun_Range(LONGITUDE, LATITUDE, TIMEZONE);
    double Sun_Setpoint = Sun_Time.Sun_Position(RTC_Data.Second(), RTC_Data.Minute(), RTC_Data.Hour(), RTC_Data.Day(), RTC_Data.Month(), RTC_Data.Year());

    //---------------------------------MPU_Read-------------------------------------//

    while (com.i2cRead(0x3B, i2cData, 14, true))
        ;  // Update all the values //
    accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
    accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
    accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
    tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
    gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
    gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
    gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
    ;

    double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees

#ifdef RESTRICT_PITCH  // Eq. 25 and 26
    double roll = atan2(accY, accZ) * RAD_TO_DEG;
    double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else  // Eq. 28 and 29
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    double gyroXrate = gyroX / 131.0;  // Convert to deg/s
    double gyroYrate = gyroY / 131.0;  // Convert to deg/s

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
        kalmanX.setAngle(roll);
        compAngleX = roll;
        kalAngleX = roll;
        gyroXangle = roll;
    } else
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter

    if (abs(kalAngleX) > 90)
        gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
        kalmanY.setAngle(pitch);
        compAngleY = pitch;
        kalAngleY = pitch;
        gyroYangle = pitch;
    } else
        kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);  // Calculate the angle using a Kalman filter

    if (abs(kalAngleY) > 90)
        gyroXrate = -gyroXrate;                         // Invert rate, so it fits the restriced accelerometer reading
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter
#endif

    gyroXangle += gyroXrate * dt;  // Calculate gyro angle without any filter
    gyroYangle += gyroYrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll;  // Calculate the angle using a Complimentary filter
    compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

    if (gyroXangle < -180 || gyroXangle > 180)  // Reset the gyro angle when it has drifted too much
        gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
        gyroYangle = kalAngleY;

    //----------------------------------------------------------------------//

    Erro_Read(Sun_Setpoint, kalAngleX);

    //-------------------------------------------------------------------------//

    Serial.println("");
}