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

#define SSID "lif2"
#define PASSWORD "fotonica"

void setup() {
    updateLEDState(LEDState::configuring);

    // LEDs de DEBUG
    initLEDs();

    //---------WIFI---------//
    wifiConnect(SSID, PASSWORD);
    startHTTPServer();
    startWSS();

    //-------- I2C --------//
    Wire.begin();

    //-------- Sensores --------//
    timeInfo.init();
    motor.init();
    mpu.init();

    // Se as configurações forem concluídas com sucesso, atualiza o estado do programa nos LEDs de DEBUG
    updateLEDState(LEDState::running);
}

// Comanda o ajuste do ângulo da lente
// int targetPosition -> ângulo desejado da lente
// int currentePosition [OPCIONAL] -> ângulo atual da lente
void adjustLens(int targetPosition, int currentPosition = mpu.data.kalAngleX) {
    if (configs.mode == Mode::Manual)
        targetPosition = configs.manualSetpoint;
    currentPosition = constrain(currentPosition, -85, 85);
    targetPosition  = constrain(targetPosition, -80, 80);

    int output = pid.calculateOutput(currentPosition, targetPosition);

    motor.command(output);
}

void loop() {
    timeInfo.callRTC();
    mpu.readMPU();

    adjustLens(timeInfo.sunPosition());
}