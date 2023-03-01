#pragma once

enum class Mode { Auto, Manual, Halt };

struct Configs {
    Mode    mode              = Mode::Halt;
    int8_t  manualSetpoint    = 0;    // graus
    uint8_t broadcastInterval = 100;  // ms
};

Configs configs;