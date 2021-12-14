#include <Arduino.h>
//#include <Kalman.h>  // Source: https://github.com/TKJElectronics/KalmanFilter
#include <Wire.h>
#include <analogWrite.h>

#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <motor/motor.hpp>
#include <TimeController/TimeController.hpp>

//----------------------------RTC settings----------------------------------------//

TimeController time_info(0x68);

//----------------------------IMU settings---------------------------------------//

MPUData MPU_Data;
MPU6050_Solar mpu(0x69);

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

    //----------------------------RTC Configurations----------------------------//

    time_info.init();
    
    //----------------------------Motor Configurations----------------------------//
    motor.init();

    //---------------------------------MPU settings-----------------------------//
    mpu.init();             // Configura e inicia o MPU
    mpu.readMPU(MPU_Data);  // realiza a primeira leitura do MPU para preencher os dados do MPUData
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
    
void adjustLens(int currentPosition, int targetPosition = 0) { //Lê os bytes recebidos pela comunicação Master-Slave
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

    adjustLens(MPU_Data.roll, time_info.sun_position()); // Atualiza a hora e retorna a posição do sol (setpoint)

    mpu.readMPU(MPU_Data);  // Atualiza MPU

#ifdef DEBUG
    Serial.println("");
#endif
}