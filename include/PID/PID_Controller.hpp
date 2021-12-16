#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <Arduino.h>

#define TIME_T0_STABILIZE 350  // ms

class PID_Controller {
   private:
    double _kp, _ki, _kd;
    int _outputLimit;
    int _integrativeLimit;
    double _threshold;
    double _lastError = 0.;
    double _lastIntegrativeValue = 0.;
    unsigned long _lastRun = 0;
    unsigned long _lastUnstableTimestamp = 0;

   public:
    // O limite integrativo controlado pela variável integrativeLimit deve ser passado em porcentagem
    // Esse valor definirá qual porcentagem máxima de participação no output do controlador a constrante integrativa terá
    PID_Controller(double kp, double ki, double kd, double threshold = 1.7, int integrativeLimitPercentage = 68) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _threshold = threshold;
        _outputLimit = 45 * (_kp + _ki * integrativeLimitPercentage);
        _integrativeLimit = _outputLimit * integrativeLimitPercentage / 100;
    }

    int calculateOutput(double actualState, double target = 0, unsigned long time = millis()) {
        double error = actualState - target;

        if (abs(error) < _threshold) {
            if (_lastUnstableTimestamp + TIME_T0_STABILIZE > time)
                this->reset();

            _lastRun = time;
            _lastError = 0.;
            return 0;
        }

        int dt = time - _lastRun;
        double P = _kp * error;
        double I = _lastIntegrativeValue + (_ki * error * dt / 1000.0);
        double D = _kd * ((error - _lastError) / dt);
        I = constrain(I, -_integrativeLimit, _integrativeLimit);
        //error = (KFE * error) + ((1 - KFE) * lastError);  //Filtra a variacao do erro para a derivada

        _lastUnstableTimestamp = time;
        _lastRun = time;
        _lastIntegrativeValue = I;
        _lastError = error;

        int output = static_cast<int>(P + I + D);
        output = constrain(output, -_outputLimit, _outputLimit);
        output = map(output, -_outputLimit, _outputLimit, -255, 255);

#ifdef DEBUG_PID
        Serial.print(" | Input: ");
        Serial.printf("%02.1f", actualState);
        Serial.print(" | Setpoint: ");
        Serial.printf("%02.1f", target);
        Serial.print(" | P: ");
        Serial.printf("%.3f", P);
        Serial.print(" | I: ");
        Serial.printf("%.3f", I);
        Serial.print(" | D: ");
        Serial.printf("%.3f", D);
        Serial.print(" | OUTPUT: ");
        Serial.printf("%03d", output);
#endif
        return output;
    }

    void reset() {
        _lastIntegrativeValue = 0.;
    }
};

#endif  // PID_CONTROLLER_HPP