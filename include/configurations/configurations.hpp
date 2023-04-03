#pragma once

#include <motor/motor.hpp>

enum class Mode { Auto, Manual, Halt };

struct Configs {
    Mode    mode              = Mode::Halt;
    int8_t  manualSetpoint    = 0;    // graus
    uint8_t broadcastInterval = 100;  // ms

    void changeMode(Mode _mode) {
        mode = _mode;
        if (_mode == Mode::Halt) motor.command(0);
    }
};

Configs configs;