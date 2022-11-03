#pragma once

#include <Arduino.h>
#include <analogWrite.h>

#include <configurations/configurations.hpp>

#define LPWM 33    // lpwm
#define RPWM 32    // rpwm
#define ENABLE 25  // pwm enable

#define MIN_OUTPUT_PWM 170
#define MAX_OUTPUT_PWM 235
#define pwmMap(value) map(value, 1, 255, MIN_OUTPUT_PWM, MAX_OUTPUT_PWM)

struct MotorData {
    uint8_t pwm;
    String  direction;

    void setMotorState(uint8_t _pwm, String _direction) {
        pwm       = _pwm;
        direction = _direction;
    }
};

class Motor {
   private:
    int _enable, _lpwm, _rpwm;

    void rotateClockwise(int PWM) {
        PWM = pwmMap(PWM);
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, HIGH);
        digitalWrite(_rpwm, LOW);
        data.setMotorState(PWM, "AH");
    }

    void rotateCounterClockwise(int PWM) {
        PWM = pwmMap(PWM);
        analogWrite(_enable, PWM);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, HIGH);
        data.setMotorState(PWM, "H");
    }
    void stop() {
        digitalWrite(_enable, LOW);
        digitalWrite(_lpwm, LOW);
        digitalWrite(_rpwm, LOW);
        data.setMotorState(0, "P");
    }

   public:
    MotorData data;

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
        data.setMotorState(0, "P");
    }

    // Aciona o driver de motor
    void command(int PWM) {
        if (configs.mode == Mode::Halt) {
            stop();
            return;
        }
        if (PWM == 0)
            stop();
        else if (PWM < 0)
            rotateClockwise(abs(PWM));
        else
            rotateCounterClockwise(abs(PWM));
    }
};

Motor motor(ENABLE, LPWM, RPWM);
