#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <Wire.h>
#include <analogWrite.h>

#include <HTTPServer/http_server.hpp>
#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <TimeController/TimeController.hpp>
#include <configurations/configurations.hpp>
#include <debugLED/debugLED.hpp>
#include <filters/moving_average.hpp>
#include <motor/motor.hpp>

#define SSID "rede"
#define PASSWORD "senha"

void setup() {
#ifdef DEBUG
    Serial.begin(115200);
#endif
    // LEDs de DEBUG
    initLEDs();

    //---------WIFI---------//
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    Serial.print("Conectando à rede WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(200);
    }
    Serial.print("Connected: ");
    Serial.println(WiFi.localIP());

    MDNS.begin("lif");
    initHTTPServer();

    //-------- I2C --------//
    Wire.begin();

    //-------- Sensores --------//
    timeInfo.init();
    motor.init();
    mpu.init();
    mpu.readMPU(mpuData);                  // realiza a primeira leitura do MPU para preencher os dados do MPUData
    filter.setInitialValue(mpuData.roll);  // Seta o valor inicial no filtro de média movel

    // Se as configurações forem concluídas com sucesso, atualiza o estado do programa nos LEDs de DEBUG
    updateLEDState(LEDState::running);
}

// Comanda o ajuste do ângulo da lente
// int targetPosition -> ângulo desejado da lente
// int currentePosition [OPCIONAL] -> ângulo atual da lente
void adjustLens(int targetPosition, int currentPosition = filter.getAverage(mpuData.roll)) {
    currentPosition = constrain(currentPosition, -85, 85);
    targetPosition = constrain(targetPosition, -82, 82);
    if (configs.mode == Mode::Manual)
        targetPosition = configs.manualSetpoint;

    int output = pid.calculateOutput(currentPosition, targetPosition);

    motor.commandMotor(output);
}

void loop() {
    timeInfo.callRTC();
    mpu.readMPU(mpuData);

    adjustLens(timeInfo.sunPosition());

#ifdef DEBUG
    Serial.println("");
#endif
}