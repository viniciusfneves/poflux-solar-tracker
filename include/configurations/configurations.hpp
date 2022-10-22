#pragma once

enum class Mode { Auto, Manual, Cicle, Halt };

struct Configs {
    Mode    mode              = Mode::Auto;
    int8_t  manualSetpoint    = 0;    // graus
    uint8_t broadcastInterval = 100;  // ms

    Mode   runCicle  = Mode::Auto;
    int8_t cicleTime = 5;
};

Configs configs;