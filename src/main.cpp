#include <Arduino.h>
#include <Wire.h>

#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <TimeController/TimeController.hpp>
#include <WiFi/HTTPServer/http_server.hpp>
#include <WiFi/Websocket/websocket.hpp>
#include <WiFi/wifi.hpp>
#include <configurations/configurations.hpp>
#include <debugLED/debugLED.hpp>
#include <motor/motor.hpp>
#include <tracking_file/tracking_file_handler.hpp>

#define DAMPING_ANGLE 30.
#define MAX_DAMPING_FACTOR .6
#define DUMPING_STRECH 1.

void setup() {
    // LEDs de DEBUG
    initLEDs();
    updateLEDState(LEDState::configuring);

    //-------- I2C --------//
    Wire.begin();

    //-------- Sensores --------//
    timeInfo.init();
    mpu.init();
    motor.init();

    //---------WIFI---------//
    connectToWifiNetwork();
    startHTTPServer();
    startWSS();

    // Se as configurações forem concluídas com sucesso, atualiza o estado do
    // programa nos LEDs de DEBUG
    updateLEDState(LEDState::running);
}

// Comanda o ajuste do ângulo da lente
// int targetPosition -> ângulo desejado da lente
// int currentePosition [OPCIONAL] -> ângulo atual da lente
void adjustLens(int targetPosition  = timeInfo.sunPosition(),
                int currentPosition = mpu.data.kalAngleX) {
    if (configs.mode == Mode::Halt) {
        pid.reset();
        motor.command(0);
        return;
    }
    if (configs.mode == Mode::Manual)
        targetPosition = configs.manualSetpoint;
    else if (configs.mode == Mode::Cicle)
        targetPosition = timeInfo.ciclePosition(currentPosition);

    currentPosition = constrain(currentPosition, -180, 180);
    targetPosition  = constrain(targetPosition, -75, 75);

    double dampingFactor;
    int    output = pid.calculateOutput(currentPosition, targetPosition);

    if (output > 0) {
        if (currentPosition <= 0) {
            output = output * MAX_DAMPING_FACTOR;
            Serial.println(
                "DAMPING STATE: MAX - output > 0 & currentPosition <=0");
        } else if (currentPosition < DAMPING_ANGLE) {
            dampingFactor =
                constrain(abs(currentPosition) * DUMPING_STRECH / DAMPING_ANGLE,
                          MAX_DAMPING_FACTOR, 1.);
            output = output * dampingFactor;
            Serial.print("DAMPING STATE: ");
            Serial.print(dampingFactor);
            Serial.println(" - output > 0 & currentPosition < DAMPING_ANGLE");
        }
    } else {
        if (currentPosition >= 0) {
            output = output * MAX_DAMPING_FACTOR;
        } else if (currentPosition > -DAMPING_ANGLE) {
            dampingFactor =
                constrain(abs(currentPosition) * DUMPING_STRECH / DAMPING_ANGLE,
                          MAX_DAMPING_FACTOR, 1.);
            output = output * dampingFactor;
        }
    }
    motor.command(output);
}

void loop() {
    runDataLogger();
    timeInfo.callRTC();
    mpu.readMPU();

    adjustLens();
}