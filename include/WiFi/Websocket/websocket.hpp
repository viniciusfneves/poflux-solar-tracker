#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>

#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <TimeController/TimeController.hpp>
#include <configurations/configurations.hpp>
#include <ftp_client/ftp_client.hpp>

WebSocketsServer wss(81);  // Configura o serviço do WebSockets para a porta 81

void handleWSData(String message) {
    StaticJsonDocument<128> jsonM;
    deserializeJson(jsonM, message);
    if (jsonM.containsKey("command")) {
        if (strcmp(jsonM["command"], "reset") == 0) {
            ESP.restart();
        }
        if (strcmp(jsonM["command"], "lif_upload") == 0) {
            upload_tracking_file_to_chronos();
        }
    }
    if (jsonM.containsKey("mode")) {
        if (strcmp(jsonM["mode"], "auto") == 0) {
            configs.mode = Mode::Auto;
        }
        if (strcmp(jsonM["mode"], "manual") == 0) {
            configs.mode = Mode::Manual;
        }
    }
    if (jsonM.containsKey("manual_setpoint")) {
        configs.manualSetpoint = (int)jsonM["manual_setpoint"];
    }
    if (jsonM.containsKey("adjust")) {
        if (jsonM["adjust"].containsKey("kp")) {
            pid.setKp(jsonM["adjust"]["kp"]);
        }
        if (jsonM["adjust"].containsKey("ki")) {
            pid.setKi(jsonM["adjust"]["ki"]);
        }
        if (jsonM["adjust"].containsKey("kd")) {
            pid.setKd(jsonM["adjust"]["kd"]);
        }
        if (jsonM["adjust"].containsKey("error_threshold")) {
            pid.setThreshold(jsonM["adjust"]["error_threshold"]);
        }
        if (jsonM["adjust"].containsKey("rtc")) {
            // {'adjust':{'rtc':{'date':"Mar 25 2022",'time':"01:50:07"}}}
            const char *date = jsonM["adjust"]["rtc"]["date"];
            const char *time = jsonM["adjust"]["rtc"]["time"];
            xSemaphoreTake(RTCSemaphore, portMAX_DELAY);
            dateTime = RtcDateTime(date, time);
            rtc.SetDateTime(dateTime);
            xSemaphoreGive(RTCSemaphore);
        }
    }
}

void handleWSEvent(uint8_t client_id, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        // Caso haja um erro no recebimento de uma mensagem do cliente
        case WStype_ERROR:
            break;
        // Caso uma nova conexão seja aceita
        case WStype_CONNECTED:
            break;
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
    if (micros() - lastBroadcastTimestamp < interval * 1000)
        return;
    StaticJsonDocument<1024> json;
    String                   stringBuffer;

    // TODO = Transformar a informação para horário EPOCH
    // TODO: E fazer a conversão no javascript do front-end
    json["RTC"]["day"]    = dateTime.Day();
    json["RTC"]["month"]  = dateTime.Month();
    json["RTC"]["year"]   = dateTime.Year();
    json["RTC"]["hour"]   = dateTime.Hour();
    json["RTC"]["minute"] = dateTime.Minute();
    json["RTC"]["second"] = dateTime.Second();

    json["MPU"]["lensAngle"]    = mpu.data.kalAngleX;
    json["MPU"]["trustedValue"] = mpu.data.isTrusted;

    switch (configs.mode) {
        case Mode::Auto:
            json["mode"] = "auto";
            break;
        case Mode::Manual:
            json["mode"] = "manual";
            break;
    }

    json["manualSetpoint"]       = configs.manualSetpoint;
    json["sunPosition"]          = timeInfo.sunPosition();
    json["lens_error_threshold"] = pid.getThreshold();

    json["PID_values"]["kp"]     = pid.getKp();
    json["PID_values"]["ki"]     = pid.getKi();
    json["PID_values"]["kd"]     = pid.getKd();
    json["PID_values"]["p"]      = pid.getInstantP();
    json["PID_values"]["i"]      = pid.getInstantI();
    json["PID_values"]["d"]      = pid.getInstantD();
    json["PID_values"]["output"] = pid.getOutput();

    json["motor"]["pwm"]       = motor.data.pwm;
    json["motor"]["direction"] = motor.data.direction;

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