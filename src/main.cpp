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

#define MAX_LENS_ANGLE 45
void setup() {
    Serial.begin(115200);
    Serial.println("Inicializando...");
    //-------- LEDS DE DEBUG --------//
    debugLED.changeState(LEDState::configuring);
    debugLED.updateState();

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

    debugLED.changeState(LEDState::running);
    debugLED.updateState();

    Serial.println("Rodando...");
}

double presentationAngleGenerator() {
    static double  _angle       = 0;
    static int16_t _rot         = 0;
    static int64_t _last_change = 0;

    if (_rot >= 360) _rot = 0;
    _rot++;

    // Stretching sine wave along the horizontal axis and changing the amplitude of the
    // wave for the desired angle amplitude
    _angle = MAX_LENS_ANGLE * sin(_rot * PI / 180);
    return _angle;
}

// Comanda o ajuste do ângulo da lente
// int targetPosition -> ângulo desejado da lente
// int currentePosition [OPCIONAL] -> ângulo atual da lente
void adjustLens(double targetPosition, double currentPosition = mpu.data.kalAngleX) {
    currentPosition = constrain(currentPosition, -180, 180);
    targetPosition  = constrain(targetPosition, -MAX_LENS_ANGLE, MAX_LENS_ANGLE);

    int output = pid.calculateOutput(currentPosition, targetPosition);

    motor.command(output);
}

void loop() {
    static int64_t timeController = 0;

    //* ATUALIZA A LEITURA DOS SENSORES
    timeInfo.callRTC();
    mpu.readMPU();

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
            case Mode::Presentation:
                // Seta o objetivo de ângulo da lente para a posição gerada pela
                // função de apresentação
                double presentation_angle = presentationAngleGenerator();
                adjustLens(presentation_angle);
                break;
        }

        debugLED.changeState(LEDState::running);

    } else {
        debugLED.changeState(LEDState::error);
        debugLED.updateState();
        Mode activeMode = configs.mode;
        configs.changeMode(Mode::Halt);
        if (mpu.reset()) {
            configs.changeMode(activeMode);
        }
    }

    if (millis() - timeController > 500) {
        debugLED.updateState();
        timeController = millis();
    }

    delay(10);
}