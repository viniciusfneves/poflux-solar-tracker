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
#include <tracking/tracking_handler.hpp>

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
    mpu.init();
    motor.init();

    // Se as configurações forem concluídas com sucesso, atualiza o estado do programa nos LEDs de DEBUG
    updateLEDState(LEDState::running);
}

// Comanda o ajuste do ângulo da lente
// int targetPosition -> ângulo desejado da lente
// int currentePosition [OPCIONAL] -> ângulo atual da lente
void adjustLens(int targetPosition, int currentPosition = mpu.data.kalAngleX) {
    if (configs.mode == Mode::Manual)
        targetPosition = configs.manualSetpoint;
    currentPosition = constrain(currentPosition, -180, 180);
    targetPosition  = constrain(targetPosition, -75, 75);

    int output = pid.calculateOutput(currentPosition, targetPosition);

    motor.command(output);
}

int64_t ESPtimestamp     = 0;
int64_t lastESPtimestamp = 0;

void loop() {
    ESPtimestamp = esp_timer_get_time() / 1000;
    if (ESPtimestamp - lastESPtimestamp >= 10000) {
        writeDataToTrackingFile(dateTime.Epoch32Time(), timeInfo.sunPosition(), mpu.data.kalAngleX);
        lastESPtimestamp = ESPtimestamp;
    }
    timeInfo.callRTC();
    mpu.readMPU();

    adjustLens(timeInfo.sunPosition());
}