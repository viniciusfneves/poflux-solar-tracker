#include <Arduino.h>
//#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter
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

//----------------------------IMU settings---------------------------------------//

MPUData MPU_Data;
MPU6050_Solar mpu(0x69);

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

//----------------------------PID Settings----------------------------//

PID_Controller pid(1, 0.3, 0.4);

//::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::://

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif

    // I2C settings //
    Wire.begin();
    Wire.setClock(400000UL);  // Set I2C frequency to 400kHz (frequency between 10kHz-400kHz)

    // Motor Settings//
    motor.init();

    // RTC settings//
    rtc.Begin();                //Inicialização do RTC DS3231
    rtc.SetDateTime(RTC_Data);  //Configurando valores iniciais do RTC DS3231

    // TimeLord Settings //
    Sun_Time.Sun_Range(LONGITUDE, LATITUDE, TIMEZONE);

    //---------------------------------MPU settings-----------------------------//
    mpu.init();             // Configura e inicia o MPU
    mpu.readMPU(MPU_Data);  // realiza a primeira leitura do MPU para preencher os dados do MPUData
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
// Comanda o motor de ajuste da lente
// int PWM -> Potência do motor e sentido de rotação |->[-255,255]
void commandMotor(int PWM) {
    if (PWM == 0)
        motor.stop();
    else if (PWM < 0)
        motor.rotateClockwise(abs(PWM));
    else
        motor.rotateCounterClockwise(abs(PWM));
}

//------------------------------------------------------------------------//
// Comanda o ajuste do ângulo da lente
// int targetPosition -> ângulo desejado da lente
// int currentePosition [OPTIONAL] -> ângulo atual da lente
void adjustLens(int targetPosition, int currentPosition = MPU_Data.roll) {
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

    double Sun_Setpoint = Sun_Time.Sun_Position(RTC_Data.Second(), RTC_Data.Minute(), RTC_Data.Hour(), RTC_Data.Day(), RTC_Data.Month(), RTC_Data.Year());

    adjustLens(Sun_Setpoint);

#ifdef DEBUG
    Serial.println("");
#endif
}