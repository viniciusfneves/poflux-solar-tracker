#pragma once

#include <Arduino.h>

#define LPWM 4     //lpwm
#define RPWM 2     //rpwm
#define ENABLE 19  //pwm enable

#define MIN_OUTPUT_PWM 180
#define MAX_OUTPUT_PWM 255
#define pwmMap(value) map(value, 1, 255, MIN_OUTPUT_PWM, MAX_OUTPUT_PWM)

class Motor {
   private:
    int _enable, _lpwm, _rpwm;

    void rotateClockwise(int PWM) {
        PWM = pwmMap(PWM);
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, HIGH);
        digitalWrite(_rpwm, LOW);
    }

    void rotateCounterClockwise(int PWM) {
        PWM = pwmMap(PWM);
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, HIGH);
    }
    void stop() {
        digitalWrite(_enable, LOW);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, LOW);
    }

   public:
    Motor(int enable_pin, int lpwm_pin, int rpwm_pin) {
        _enable = enable_pin;
        _lpwm = lpwm_pin;
        _rpwm = rpwm_pin;
    }

    void init() {
        pinMode(_enable, OUTPUT);
        pinMode(_lpwm, OUTPUT);
        pinMode(_rpwm, OUTPUT);
        digitalWrite(_enable, LOW);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, LOW);
    }

    // Aciona o driver de motor
    void commandMotor(int PWM) {
        if (PWM == 0) {
            stop();
#ifdef DEBUG_MOTOR
            Serial.print(" | Motor: Parado");
#endif
        } else if (PWM < 0) {
            rotateClockwise(abs(PWM));
#ifdef DEBUG_MOTOR
            Serial.print(" | Motor: Horário");
            Serial.print(" | PWM: ");
            Serial.printf("%03d", PWM);
#endif
        } else {
            rotateCounterClockwise(abs(PWM));
#ifdef DEBUG_MOTOR
            Serial.print(" | Motor: Anti-Horário");
            Serial.print(" | PWM: ");
            Serial.printf("%03d", PWM);
#endif
        }
    }
};

Motor motor(ENABLE, LPWM, RPWM);
