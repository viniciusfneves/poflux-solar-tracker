#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <Wire.h>
#include <analogWrite.h>

#include <HTTPServer/http_server.hpp>
#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <TimeController/TimeController.hpp>
#include <filters/moving_average.hpp>
#include <motor/motor.hpp>

//---------------------------- WIFi ----------------------------//

#define SSID "rede"
#define PASSWORD "senha"

void setup() {
    //---------WIFI---------//
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);

    Serial.println(WiFi.localIP());
    initHTTPServer();

#ifdef DEBUG
    Serial.begin(115200);
    MDNS.begin("lif");
#endif

    //-------- I2C --------//
    Wire.begin();

    //-------- Sensors --------//
    timeInfo.init();
    motor.init();
    mpu.init();                            // Configura e inicia o MPU
    mpu.readMPU(mpuData);                  // realiza a primeira leitura do MPU para preencher os dados do MPUData
    filter.setInitialValue(mpuData.roll);  // Seta o valor inicial no filtro de média movel
}

// Comanda o ajuste do ângulo da lente
// int targetPosition -> ângulo desejado da lente
// int currentePosition [OPTIONAL] -> ângulo atual da lente
void adjustLens(int targetPosition, int currentPosition = filter.getAverage(mpuData.roll)) {
#ifdef TEST_SETPOINT
    targetPosition = SETPOINT;
#else
    targetPosition = constrain(targetPosition, -82, 82);
#endif
    currentPosition = constrain(currentPosition, -85, 85);

    int output = pid.calculateOutput(currentPosition, targetPosition);

    motor.commandMotor(output);
}

void loop() {
    timeInfo.callRTC();
    mpu.readMPU(mpuData);

    if (mpuData.isTrusted)
        adjustLens(timeInfo.sunPosition());
    else
        motor.commandMotor(0);

#ifdef DEBUG
    Serial.println("");
#endif
}