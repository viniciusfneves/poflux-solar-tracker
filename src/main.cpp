#include <Arduino.h>
#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter
#include <PID.h>
#include <RtcDS3231.h>
#include <RtcDateTime.h>
#include <Time.h>
#include <Wire.h>
#include <analogWrite.h>

#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <motor/motor.hpp>

//----------------------------RTC settings----------------------------------------//

RtcDS3231<TwoWire> rtc(Wire);              //Criação do objeto do tipo DS3231
RtcDateTime RTC_Data(__DATE__, __TIME__);  //Criação do objeto do tipo RTCDateTime iniciando com tempo do sistema

//----------------------------Kalman settings-------------------------------------//

#define RESTRICT_PITCH  // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX;  //Criação de objeto
Kalman kalmanY;  //Criação de objeto

//----------------------------IMU settings---------------------------------------//

MPUData MPU_Data;
MPU6050_Solar mpu(0x69);
double gyroXangle, gyroYangle;
double compAngleX, compAngleY;  // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter
uint32_t timer;

//----------------------------Timerlord settings-------------------------------------//

float const LONGITUDE = -43.2311486;
float const LATITUDE = -22.8613427;
int const TIMEZONE = -3;
SunLight Sun_Time;

//----------------------------Driver Settings---------------------------------------//

#define LPWM 4     //lpwm
#define RPWM 2     //rpwm
#define ENABLE 19  //pwm enable
Motor motor(ENABLE, LPWM, RPWM);

//-------------------------------Limits-------------------------------//

int Threshold_Max;
int Threshold_Min;
int Max_Angle_Limit;
int Min_Angle_Limit;

//----------------------------PID Settings----------------------------//
PID_Controller pid(1, 0.3, 0.4);
//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::://

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif

    //----------------------------I2C settings----------------------------------//
    Wire.begin();
    Wire.setClock(400000UL);  // Set I2C frequency to 400kHz (frequency between 10kHz-400kHz)

    //----------------------------Motor Configurations----------------------------//
    motor.init();

    //---------------------------------RTC settings-----------------------------//
    rtc.Begin();                //Inicialização do RTC DS3231
    rtc.SetDateTime(RTC_Data);  //Configurando valores iniciais do RTC DS3231

    //---------------------------------MPU settings-----------------------------//
    mpu.init();             // Configura e inicia o MPU
    mpu.readMPU(MPU_Data);  // realiza a primeira leitura do MPU para preencher os dados do MPUData

    //---------------------------Kalman----------------------------------//
    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26

    kalmanX.setAngle(MPU_Data.roll);  // Set starting angle
    //kalmanY.setAngle(pitch);
    gyroXangle = MPU_Data.roll;
    //gyroYangle = pitch;
    compAngleX = MPU_Data.roll;
    //compAngleY = pitch;

    timer = micros();
}

//------------------------------------------------------------------------//
//Atualiza os valores de data e hora instantâneos lidos pelo RTC
//Imprime no monitor Serial dados do RTC
void callRTC(RtcDateTime &RTC_Data) {
    RTC_Data = rtc.GetDateTime();  //Atribuindo valores instantâneos de data e hora à instância data e hora

#ifdef DEBUG_RTC
    Serial.print(RTC_Data.Day());
    Serial.print("-");
    Serial.print(RTC_Data.Month());
    Serial.print("-");
    Serial.print(RTC_Data.Year());
    Serial.print("  ");
    Serial.print(RTC_Data.Hour());
    Serial.print(":");
    Serial.print(RTC_Data.Minute());
    Serial.print(":");
    Serial.print(RTC_Data.Second());
    Serial.print("  ");
#endif
}

//------------------------------------------------------------------------//

void commandMotor(int PWM) {
    if (PWM == 0)
        motor.stop();
    else if (PWM < 0)
        motor.rotateClockwise(abs(PWM));
    else
        motor.rotateCounterClockwise(abs(PWM));
}

//------------------------------------------------------------------------//
//Lê os bytes recebidos pela comunicação Master-Slave
void adjustLens(int currentPosition, int targetPosition) {
    //Setpoint = 0; //test angle, discomment to work properly
    targetPosition = constrain(targetPosition, -80, 80);

    int output = pid.calculateOutput(currentPosition, targetPosition);

    // Map da saída do PID para valores de PWM dos motores
    output = map(output, -55, 55, -255, 255);

    commandMotor(output);
#ifdef DEBUG_ERRO
    Serial.print(" | Setpoint: ");
    Serial.printf("%02d", targetPosition);
    Serial.print(" | Input: ");
    Serial.printf("%02d", currentPosition);
    Serial.print(" | Output: ");
    Serial.printf("%03d", output);
#endif
}

void loop() {
    callRTC(RTC_Data);      // Atualiza RTC
    mpu.readMPU(MPU_Data);  // Atualiza MPU

    // Calculating Sun parameters  //

    Sun_Time.Sun_Range(LONGITUDE, LATITUDE, TIMEZONE);
    double Sun_Setpoint = Sun_Time.Sun_Position(RTC_Data.Second(), RTC_Data.Minute(), RTC_Data.Hour(), RTC_Data.Day(), RTC_Data.Month(), RTC_Data.Year());

    //double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
    //timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees

    //double gyroXrate = MPU_Data.GyX / 131.0;  // Convert to deg/s
    //double gyroYrate = MPU_Data.GyY / 131.0;  // Convert to deg/s

    // #ifdef RESTRICT_PITCH
    //     // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    //     if ((MPU_Data.roll < -90 && kalAngleX > 90) || (MPU_Data.roll > 90 && kalAngleX < -90)) {
    //         kalmanX.setAngle(MPU_Data.roll);
    //         compAngleX = MPU_Data.roll;
    //         kalAngleX = MPU_Data.roll;
    //         gyroXangle = MPU_Data.roll;
    //     } else
    //         kalAngleX = kalmanX.getAngle(MPU_Data.roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter

    //         //if (abs(kalAngleX) > 90)
    //         //gyroYrate = -gyroYrate;  // Invert rate, so it fits the restriced accelerometer reading
    //         //kalAngleY = kalmanY.getAngle(MPU_Data.pitch, gyroYrate, dt);
    // #else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    // if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    //     kalmanY.setAngle(pitch);
    //     compAngleY = pitch;
    //     kalAngleY = pitch;
    //     gyroYangle = pitch;
    // } else
    //     kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);  // Calculate the angle using a Kalman filter

    // if (abs(kalAngleY) > 90)
    //     gyroXrate = -gyroXrate;                         // Invert rate, so it fits the restriced accelerometer reading
    // kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);  // Calculate the angle using a Kalman filter
    // #endif

    // gyroXangle += gyroXrate * dt;  // Calculate gyro angle without any filter
    // //gyroYangle += gyroYrate * dt;
    // //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    // //gyroYangle += kalmanY.getRate() * dt;

    // compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * MPU_Data.roll;  // Calculate the angle using a Complimentary filter
    // //compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * MPU_Data.pitch;

    // if (gyroXangle < -180 || gyroXangle > 180)  // Reset the gyro angle when it has drifted too much
    //     gyroXangle = kalAngleX;
    // //if (gyroYangle < -180 || gyroYangle > 180)
    // //gyroYangle = kalAngleY;

    //----------------------------------------------------------------------//

    adjustLens(MPU_Data.roll, Sun_Setpoint);

#ifdef DEBUG
    Serial.println("");
#endif
}