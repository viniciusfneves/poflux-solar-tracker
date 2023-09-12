#pragma once

#include <ArduinoJson.h>

#include <MPU/MPU.hpp>
#include <PID/PID_Controller.hpp>
#include <TimeController/TimeController.hpp>
#include <motor/motor.hpp>

enum class Mode { Auto, Manual, Presentation, Halt };

struct Configs {
    Mode     mode              = Mode::Halt;
    double   manualSetpoint    = 0;    // graus
    uint16_t broadcastInterval = 250;  // ms

    void changeMode(Mode _mode) {
        mode = _mode;

        //! FAIL SAFE
        // Se por algum motivo o modo halt for ativado, o comando é
        // imediatamente enviado aos motores, não necessitando da execução de
        // outras funções no código
        if (_mode == Mode::Halt) motor.command(0);
    }

    void configWithJSON(DynamicJsonDocument jsonM) {
        if (jsonM.containsKey("command")) {
            if (strcmp(jsonM["command"], "reset") == 0) {
                ESP.restart();
            }
        }

        if (jsonM.containsKey("mode")) {
            if (strcmp(jsonM["mode"], "auto") == 0) {
                changeMode(Mode::Auto);
            }
            if (strcmp(jsonM["mode"], "manual") == 0) {
                changeMode(Mode::Manual);
            }
            if (strcmp(jsonM["mode"], "halt") == 0) {
                changeMode(Mode::Halt);
            }
            if (strcmp(jsonM["mode"], "presentation") == 0) {
                changeMode(Mode::Presentation);
            }
        }

        if (jsonM.containsKey("manual_setpoint")) {
            manualSetpoint = jsonM["manual_setpoint"].as<int8_t>();
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
                timeInfo.setDatetime(jsonM["adjust"]["rtc"].as<int64_t>());
            }
        }
    }
};

Configs configs;
