#include <Arduino.h>
#include <Wire.h>

#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <TimeController/TimeController.hpp>
#include <WiFi/HTTPServer/http_server.hpp>
#include <WiFi/Websocket/websocket.hpp>
#include <WiFi/wifi.hpp>
#include <configurations/configurations.hpp>
#include <datalogger/datalogger.hpp>
#include <debugLED/debugLED.hpp>
#include <motor/motor.hpp>

void setup() {
    Serial.begin(115200);
    Serial.println("Inicializando...");
    //-------- LEDS DE DEBUG --------//
    debugLED.updateState(LEDState::configuring);

    //-------- MEMÓRIA --------//
    LittleFS.begin(true);

    //-------- I2C --------//
    Wire.begin();

    Serial.println("Configurando sensores...");
    //-------- SENSORES --------//
    timeInfo.init();
    mpu.init();
    motor.init();

    //--------- WIFI ---------//
    connectToWifiNetwork();
    startHTTPServer();
    startWSS();

    Serial.println("Iniciando datalogger...");
    //--------- DATALOGGER ---------//
    xTaskCreate(dataloggerTask, "DATALOGGER", 3096, NULL, 5, NULL);

    // Se as configurações forem concluídas com sucesso, seta os leds para
    // running
    Serial.println("Finalizando configurações...");
    debugLED.updateState(LEDState::running);
    Serial.println("Rodando...");
}

// Comanda o ajuste do ângulo da lente
// int targetPosition -> ângulo desejado da lente
// int currentePosition [OPCIONAL] -> ângulo atual da lente
void adjustLens(double targetPosition,
                double currentPosition = mpu.data.kalAngleX) {
    currentPosition = constrain(currentPosition, -180, 180);
    targetPosition  = constrain(targetPosition, -75, 75);

    int output = pid.calculateOutput(currentPosition, targetPosition);

    motor.command(output);
}

void loop() {
    //* ATUALIZA A LEITURA DOS SENSORES
    timeInfo.callRTC();
    // mpu.readMPU();
    mpu.data.isTrusted = true;
    if (mpu.data.isTrusted) {
        //* VERIFICA O MODO DE OPERAÇÃO DO RASTREADOR
        switch (configs.mode) {
            case Mode::Halt:
                pid.reset();
                motor.stop();
                break;
            case Mode::Auto:
                // Seta o objetivo de ângulo da lente para a posição do sol
                adjustLens(timeInfo.sunPosition());
                break;
            case Mode::Manual:
                // Seta o objetivo de ângulo da lente para a posição comandada
                // pelo operador manualmente
                adjustLens(configs.manualSetpoint);
                break;
        }

        debugLED.updateState(LEDState::running);

    } else {
        debugLED.updateState(LEDState::error);
        Mode activeMode = configs.mode;
        configs.changeMode(Mode::Halt);
        if (mpu.reset()) {
            configs.changeMode(activeMode);
        }
    }
}