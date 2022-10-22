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
    if (configs.mode == Mode::Manual)
        targetPosition = configs.manualSetpoint;
    else if (configs.mode == Mode::Cicle)
        targetPosition = timeInfo.ciclePosition(currentPosition);

    currentPosition = constrain(currentPosition, -180, 180);
    targetPosition  = constrain(targetPosition, -75, 75);

    int output = pid.calculateOutput(currentPosition, targetPosition);

    motor.command(output);
}

void loop() {
    runDataLogger();
    timeInfo.callRTC();
    mpu.readMPU();

    adjustLens();
}