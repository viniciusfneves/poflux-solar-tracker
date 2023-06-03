#pragma once

#include <motor/motor.hpp>

enum class Mode { Auto, Manual, Halt };

struct Configs {
    Mode     mode              = Mode::Halt;
    double   manualSetpoint    = 0;    // graus
    uint16_t broadcastInterval = 100;  // ms

    void changeMode(Mode _mode) {
        mode = _mode;

        //! FAIL SAFE
        // Se por algum motivo o modo halt for ativado, o comando é
        // imediatamente enviado aos motores, não necessitando da execução de
        // outras funções no código
        if (_mode == Mode::Halt) motor.command(0);
    }
};

Configs configs;