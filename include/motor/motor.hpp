#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <Arduino.h>

class Motor {
   private:
    int _enable, _lpwm, _rpwm;

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

    void rotateClockwise(int PWM) {
        PWM = map(PWM, 1, 255, 180, 255);
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, HIGH);
        digitalWrite(_rpwm, LOW);

        //-- DEBUG --//
#ifdef DEBUG_MOTOR
        Serial.print(" | Motor: Horário");
        Serial.print(" | PWM: ");
        Serial.printf("%03d", PWM);
#endif
    }

    void rotateCounterClockwise(int PWM) {
        PWM = map(PWM, 1, 255, 180, 255);
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, HIGH);

        //-- DEBUG --//
#ifdef DEBUG_MOTOR
        Serial.print(" | Motor: Anti-Horário");
        Serial.print(" | PWM: ");
        Serial.printf("%03d", PWM);
#endif
    }

    void stop() {
        digitalWrite(_enable, LOW);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, LOW);

        //-- DEBUG --//
#ifdef DEBUG_MOTOR
        Serial.print(" | Motor: Parado");
        Serial.print(" | PWM: ");
        Serial.printf("%03d", 0);
#endif
    }
};

#endif  // MOTORS_HPP