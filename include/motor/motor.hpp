#pragma once

#include <Arduino.h>
#include <analogWrite.h>

#define LPWM_PIN 33    // lpwm
#define RPWM_PIN 32    // rpwm
#define ENABLE_PIN 25  // pwm enable

#define MIN_OUTPUT_PWM_VALUE 165
#define MAX_OUTPUT_PWM_VALUE 255
#define pwmMap(value) map(value, 0, 255, MIN_OUTPUT_PWM_VALUE, MAX_OUTPUT_PWM_VALUE)

class Motor {
   private:
    int     _enable, _lpwm, _rpwm;
    uint8_t _pwm;

    void rotateClockwise(int PWM) {
        PWM = pwmMap(PWM);
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, HIGH);
        digitalWrite(_rpwm, LOW);
        _pwm = PWM;
    }

    void rotateCounterClockwise(int PWM) {
        PWM = pwmMap(PWM);
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, HIGH);
        _pwm = PWM;
    }

   public:
    Motor(int enable_pin, int lpwm_pin, int rpwm_pin) {
        _enable = enable_pin;
        _lpwm   = lpwm_pin;
        _rpwm   = rpwm_pin;
    }

    void init() {
        pinMode(_enable, OUTPUT);
        pinMode(_lpwm, OUTPUT);
        pinMode(_rpwm, OUTPUT);
        digitalWrite(_enable, LOW);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, LOW);
        _pwm = 0;
    }

    // Para o motor
    void stop() {
        digitalWrite(_enable, LOW);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, LOW);
        _pwm = 0;
    }

    // Aciona o driver de motor
    void command(int PWM) {
        if (PWM == 0)
            stop();
        else if (PWM < 0)
            rotateClockwise(abs(PWM));
        else
            rotateCounterClockwise(abs(PWM));
    }

    // Retorna o atual PWM aplicado no motor
    uint8_t power() { return _pwm; }
};

Motor motor(ENABLE_PIN, LPWM_PIN, RPWM_PIN);
