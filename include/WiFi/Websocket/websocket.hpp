#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>

#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <TimeController/TimeController.hpp>
#include <configurations/configurations.hpp>
#include <motor/motor.hpp>

WebSocketsServer wss(81);  // Configura o serviço do WebSockets para a porta 81

void broadcastLUXInfo(uint8_t interval) {
    static uint32_t lastBroadcastTimestamp = 0;
    if (micros() - lastBroadcastTimestamp < interval * 1000) return;
    StaticJsonDocument<2048> json;
    String                   stringBuffer;

    json["esp_clock"] = timeInfo.datetime();
    json["rtc"]       = timeInfo.RTCdatetime();

    json["mpu"]["lensAngle"]    = mpu.data.kalAngleX;
    json["mpu"]["trustedValue"] = mpu.data.isTrusted;

    switch (configs.mode) {
        case Mode::Auto:
            json["mode"] = "auto";
            break;
        case Mode::Manual:
            json["mode"] = "manual";
            break;
        case Mode::Halt:
            json["mode"] = "halt";
            break;
        case Mode::Presentation:
            json["mode"] = "presentation";
            break;
    }

    json["manual_setpoint"]      = configs.manualSetpoint;
    json["sun_position"]         = timeInfo.sunPosition();
    json["lens_error_threshold"] = pid.getThreshold();

    json["pid_values"]["kp"]     = pid.getKp();
    json["pid_values"]["ki"]     = pid.getKi();
    json["pid_values"]["kd"]     = pid.getKd();
    json["pid_values"]["error"]  = pid.getErrorValue();
    json["pid_values"]["p"]      = pid.getPValue();
    json["pid_values"]["i"]      = pid.getIValue();
    json["pid_values"]["d"]      = pid.getDValue();
    json["pid_values"]["output"] = pid.getOutputValue();

    json["motor"] = motor.power();

    serializeJson(json, stringBuffer);
    wss.broadcastTXT(stringBuffer);
    lastBroadcastTimestamp = micros();
}

void handleWSServer(void *_) {
    for (;;) {
        wss.loop();
        broadcastLUXInfo(configs.broadcastInterval);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void startWSS() {
    wss.begin();
    xTaskCreate(handleWSServer, "WS_Handler", 1024 * 4, NULL, 1, NULL);
}