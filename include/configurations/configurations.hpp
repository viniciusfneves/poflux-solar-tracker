#pragma once

enum class Mode {
    Auto,
    Manual
};

struct Configs {
    Mode    mode              = Mode::Auto;
    int8_t  manualSetpoint    = 0;    // graus
    uint8_t broadcastInterval = 100;  // ms
};

Configs configs;