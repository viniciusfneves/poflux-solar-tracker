#pragma once

enum class Mode {
    Auto,
    Manual
};

struct Configs {
    Mode mode = Mode::Auto;
    int manualSetpoint = 0;
};

Configs configs;