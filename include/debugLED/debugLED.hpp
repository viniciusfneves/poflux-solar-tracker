#pragma once

#include <Arduino.h>

#define RUN_LED 26
#define ERRO_LED 27

enum class LEDState {
    configuring,
    running,
    solving_error,
    error
};

portMUX_TYPE debugKey = portMUX_INITIALIZER_UNLOCKED;

LEDState debuggingLED = LEDState::configuring;

void setLED(LEDState state) {
    switch (state) {
        case LEDState::configuring:
            digitalWrite(RUN_LED, HIGH);
            digitalWrite(ERRO_LED, HIGH);
            break;
        case LEDState::running:
            digitalWrite(RUN_LED, !digitalRead(RUN_LED));
            digitalWrite(ERRO_LED, LOW);
            break;
        case LEDState::solving_error:
            digitalWrite(RUN_LED, LOW);
            digitalWrite(ERRO_LED, !digitalRead(ERRO_LED));
            break;
        case LEDState::error:
            digitalWrite(RUN_LED, LOW);
            digitalWrite(ERRO_LED, HIGH);
            break;

        default:
            digitalWrite(RUN_LED, LOW);
            digitalWrite(ERRO_LED, HIGH);
            debuggingLED = LEDState::error;
            break;
    }
}

void updateLEDState(LEDState state) {
    portENTER_CRITICAL(&debugKey);
    debuggingLED = state;
    portEXIT_CRITICAL(&debugKey);
}

void ledHandler(void* _) {
    while (1) {
        static LEDState state;
        static int controller = 0;

        portENTER_CRITICAL(&debugKey);
        if (state == debuggingLED) {
            controller++;
        } else {
            state = debuggingLED;
            controller = 0;
        }
        portEXIT_CRITICAL(&debugKey);
        if (controller == 6)
            controller = 0;
        if (controller == 0)
            setLED(state);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void initLEDs() {
    pinMode(RUN_LED, OUTPUT);
    pinMode(ERRO_LED, OUTPUT);
    setLED(LEDState::configuring);
    xTaskCreate(ledHandler, "LED Debug", 1024, NULL, 1, NULL);
}