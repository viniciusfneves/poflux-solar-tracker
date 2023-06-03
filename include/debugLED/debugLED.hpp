#pragma once

#include <Arduino.h>

#define RUN_LED_PIN 18
#define ERROR_LED_PIN 19

enum class LEDState { off, configuring, running, error };

class LEDController {
   private:
    LEDState _state;
    uint8_t  _runLEDPin;
    uint8_t  _errorLEDPin;

   public:
    LEDController(uint8_t runLEDPin, uint8_t errorLEDPin,
                  LEDState initialState = LEDState::off) {
        _runLEDPin   = runLEDPin;
        _errorLEDPin = errorLEDPin;
        _state       = initialState;
    }

    LEDState ledState() { return _state; }

    void updateState(LEDState state) {
        _state = state;
        switch (state) {
            case LEDState::off:
                digitalWrite(_runLEDPin, LOW);
                digitalWrite(_errorLEDPin, LOW);
                break;
            case LEDState::configuring:
                digitalWrite(_runLEDPin, HIGH);
                digitalWrite(_errorLEDPin, HIGH);
                break;
            case LEDState::running:
                digitalWrite(_runLEDPin, !digitalRead(_runLEDPin));
                digitalWrite(_errorLEDPin, LOW);
                break;
            case LEDState::error:
                digitalWrite(_runLEDPin, LOW);
                digitalWrite(_errorLEDPin, HIGH);
                break;
        }
    }
};

LEDController debugLED(RUN_LED_PIN, ERROR_LED_PIN);

void ledTask(void* _) {
    for (;;) {
        static uint8_t controller = 0;

        //* This is what makes the running LED blink
        if (debugLED.ledState() == LEDState::running) {
            controller++;
            if (controller > 5) {
                debugLED.updateState(LEDState::running);
                controller = 0;
            }
        } else {
            controller = 0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
