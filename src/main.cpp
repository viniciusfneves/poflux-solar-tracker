#include <Arduino.h>
#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter
#include <PID.h>
#include <RtcDS3231.h>
#include <RtcDateTime.h>
#include <Time.h>
#include <Wire.h>
#include <analogWrite.h>
#include <math.h>

#include <MPU/MPU.hpp>
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
MPU6050_Solar mpu(0x69, MPU_Data);
double gyroXangle, gyroYangle;
double compAngleX, compAngleY;  // Calculated angle using a complementary filter
double kalAngleX, kalAngleY;    // Calculated angle using a Kalman filter
uint32_t timer;
uint8_t i2cData[14];  // Buffer for I2C data
uint8_t dataI2c;

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

int Setpoint, Input, Output;  //Define Variables we'll be connecting to
PID_Control PID_Calculator;   //PID object
int Erro;

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
    mpu.init();     // Configura e inicia o MPU
    mpu.readMPU();  // realiza a primeira leitura do MPU para preencher os dados do MPUData

    //---------------------------Kalman----------------------------------//
// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
#ifdef RESTRICT_PITCH                                              // Eq. 25 and 26
    double roll = atan2(MPU_Data.AcY, MPU_Data.AcZ) * RAD_TO_DEG;  //atan2 outputs the value of -π to π (radians. It is then converted from radians to degrees
    double pitch = atan(-MPU_Data.AcX / sqrt(pow(MPU_Data.AcY, 2) + pow(MPU_Data.AcZ, 2))) * RAD_TO_DEG;
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
void callRTC(RtcDateTime &RTC_Data) {
    RTC_Data = rtc.GetDateTime();  //Atribuindo valores instantâneos de data e hora à instância data e hora

#ifdef DEBUG_RTC
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
#endif
}

//------------------------------------------------------------------------//

void Motor_Direction(int erro, int PWM, int input) {
    Threshold_Max = 3;
    Threshold_Min = -Threshold_Max;
    Max_Angle_Limit = 85;
    Min_Angle_Limit = -Max_Angle_Limit;

    if (90 > input * -1 > -90 && erro < Threshold_Min) {
        motor.rotateClockwise(PWM);
    } else if (-90 < input < 90 && erro > Threshold_Max) {
        motor.rotateCounterClockwise(PWM);
    } else if (Threshold_Min < erro < Threshold_Max) {
        motor.stop();
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

    Output = PID_Calculator.PID(abs(Erro), Threshold_Max);

    Output = mapeamento(Output, -216, 216, 110, 230);  //mudar valores para variáveis

#ifdef DEBUG_MPU
    Serial.print(" | Setpoint: ");
    Serial.print(Setpoint);
    Serial.print(" | Erro: ");
    Serial.print(Erro);
#endif

    Motor_Direction(Erro, Output, Input);
}

void loop() {
    callRTC(RTC_Data);  // Atualiza RTC
    mpu.readMPU();      // Atualiza MPU

    // Calculating Sun parameters  //

    Sun_Time.Sun_Range(LONGITUDE, LATITUDE, TIMEZONE);
    double Sun_Setpoint = Sun_Time.Sun_Position(RTC_Data.Second(), RTC_Data.Minute(), RTC_Data.Hour(), RTC_Data.Day(), RTC_Data.Month(), RTC_Data.Year());

    double dt = (double)(micros() - timer) / 1000000;  // Calculate delta time
    timer = micros();

    // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
    // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
    // It is then converted from radians to degrees

#ifdef RESTRICT_PITCH                                              // Eq. 25 and 26
    double roll = atan2(MPU_Data.AcY, MPU_Data.AcZ) * RAD_TO_DEG;  //atan2 outputs the value of -π to π (radians. It is then converted from radians to degrees
    double pitch = atan(-MPU_Data.AcX / sqrt(pow(MPU_Data.AcY, 2) + pow(MPU_Data.AcZ, 2))) * RAD_TO_DEG;
#else  // Eq. 28 and 29
    double roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
    double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

    double gyroXrate = MPU_Data.GyX / 131.0;  // Convert to deg/s
    double gyroYrate = MPU_Data.GyY / 131.0;  // Convert to deg/s

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

#ifdef DEBUG
    Serial.println("");
#endif
}