#include <Arduino.h>
//#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Wire.h>
#include <analogWrite.h>

#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <TimeController/TimeController.hpp>
#include <filters/moving_average.hpp>
#include <motor/motor.hpp>

//---------------------------- RTC settings ----------------------------//

TimeController time_info(0x68);

//---------------------------- IMU settings ----------------------------//

MPUData MPU_Data;
MPU6050_Solar mpu(0x69);
MovingAverage average;

//---------------------------- Driver Settings ----------------------------//

#define LPWM 4     //lpwm
#define RPWM 2     //rpwm
#define ENABLE 19  //pwm enable
Motor motor(ENABLE, LPWM, RPWM);

//---------------------------- PID Settings ----------------------------//

PID_Controller pid(1, 0.3, 0.4);

//::::::::::::::::::::::::::::::::: SETUP ::::::::::::::::::::::::::::::::://

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif

    //-------- I2C --------//
    Wire.begin();
    Wire.setClock(400000U);  // Set I2C frequency to 400kHz (frequency between 10kHz-400kHz)

    //-------- Sensors --------//
    time_info.init();
    motor.init();
    mpu.init();                              // Configura e inicia o MPU
    mpu.readMPU(MPU_Data);                   // realiza a primeira leitura do MPU para preencher os dados do MPUData
    average.setInitialValue(MPU_Data.roll);  // Seta o valor inicial no filtro de mediaMovel
}

// Aciona o driver de motor
void commandMotor(int PWM) {
    if (PWM == 0)
        motor.stop();
    else if (PWM < 0)
        motor.rotateClockwise(abs(PWM));
    else
        motor.rotateCounterClockwise(abs(PWM));
}

// Comanda o ajuste do ângulo da lente
// int targetPosition -> ângulo desejado da lente
// int currentePosition [OPTIONAL] -> ângulo atual da lente
void adjustLens(int targetPosition, int currentPosition = average.filter(MPU_Data.roll)) {
    targetPosition = constrain(targetPosition, -82, 82);
    currentPosition = constrain(currentPosition, -85, 85);

    int output = pid.calculateOutput(currentPosition, targetPosition);

    commandMotor(output);
#ifdef DEBUG_ERRO
    Serial.print(" | Setpoint: ");
    Serial.printf("%02d", targetPosition);
    Serial.print(" | Input: ");
    Serial.printf("%02d", currentPosition);
#endif
}

void loop() {
    time_info.CallRTC();
    mpu.readMPU(MPU_Data);

    adjustLens(time_info.sunPosition());

#ifdef DEBUG
    Serial.println("");
#endif
}