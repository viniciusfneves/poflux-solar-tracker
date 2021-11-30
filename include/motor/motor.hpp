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
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, HIGH);
        digitalWrite(_rpwm, LOW);
    }
    void rotateCounterClockwise(int PWM) {
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, HIGH);
    }
    void stop() {
        digitalWrite(_enable, LOW);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, LOW);
    }
};

#endif  // MOTORS_HPP