#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include <time.h>

#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <TimeController/TimeController.hpp>
#include <configurations/configurations.hpp>
#include <motor/motor.hpp>

WebSocketsServer wss(81);  // Configura o serviço do WebSockets para a porta 81

void handleWSData(String message) {
    StaticJsonDocument<1024> jsonM;
    deserializeJson(jsonM, message);
    if (jsonM.containsKey("command")) {
        if (strcmp(jsonM["command"], "reset") == 0) {
            ESP.restart();
        }
    }
    if (jsonM.containsKey("mode")) {
        if (strcmp(jsonM["mode"], "auto") == 0) {
            configs.mode = Mode::Auto;
        }
        if (strcmp(jsonM["mode"], "manual") == 0) {
            configs.mode = Mode::Manual;
        }
        if (strcmp(jsonM["mode"], "halt") == 0) {
            configs.mode = Mode::Halt;
        }
    }
    if (jsonM.containsKey("manual_setpoint")) {
        configs.manualSetpoint = jsonM["manual_setpoint"].as<int8_t>();
    }
    if (jsonM.containsKey("adjust")) {
        if (jsonM["adjust"].containsKey("kp")) {
            pid.setKp(jsonM["adjust"]["kp"].as<double>());
        }
        if (jsonM["adjust"].containsKey("ki")) {
            pid.setKi(jsonM["adjust"]["ki"].as<double>());
        }
        if (jsonM["adjust"].containsKey("kd")) {
            pid.setKd(jsonM["adjust"]["kd"].as<double>());
        }
        if (jsonM["adjust"].containsKey("error_threshold")) {
            pid.setThreshold(jsonM["adjust"]["error_threshold"].as<double>());
        }
        if (jsonM["adjust"].containsKey("rtc")) {
            // {'adjust':{'rtc':Epoch64Time}}
            int64_t timestamp = jsonM["adjust"]["rtc"];
            xSemaphoreTake(RTCSemaphore, portMAX_DELAY);
            dateTime.InitWithEpoch64Time(timestamp);
            rtc.SetDateTime(dateTime);
            xSemaphoreGive(RTCSemaphore);
            timeval _date;
            _date.tv_sec  = timestamp;
            _date.tv_usec = 0;
            settimeofday(&_date, NULL);
        }
    }
}

void handleWSEvent(uint8_t client_id, WStype_t type, uint8_t *payload,
                   size_t length) {
    switch (type) {
        // Caso a mensagem seja do tipo text
        case WStype_TEXT:
            handleWSData((String)((char *)payload));
            break;
        default:
            break;
    }
}

void broadcastLUXInfo(uint8_t interval) {
    static uint32_t lastBroadcastTimestamp = 0;
    if (micros() - lastBroadcastTimestamp < interval * 1000) return;
    StaticJsonDocument<1024> json;
    String                   stringBuffer;

    json["ESPClock"] = time(NULL);
    json["RTC"]      = dateTime.Epoch64Time();

    json["MPU"]["lensAngle"]    = mpu.data.kalAngleX;
    json["MPU"]["trustedValue"] = mpu.data.isTrusted;

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
    }

    json["manualSetpoint"]       = configs.manualSetpoint;
    json["cicleSetpoint"]        = timeInfo.getCicleSetpoint();
    json["sunPosition"]          = timeInfo.sunPosition();
    json["lens_error_threshold"] = pid.getThreshold();

    json["PID_values"]["kp"]     = pid.getKp();
    json["PID_values"]["ki"]     = pid.getKi();
    json["PID_values"]["kd"]     = pid.getKd();
    json["PID_values"]["p"]      = pid.getInstantP();
    json["PID_values"]["i"]      = pid.getInstantI();
    json["PID_values"]["d"]      = pid.getInstantD();
    json["PID_values"]["output"] = pid.getOutput();

    json["motor"] = motor.data.pwm;

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
    wss.onEvent(handleWSEvent);
    xTaskCreate(handleWSServer, "WS_Handler", 1024 * 3, NULL, 1, NULL);
}