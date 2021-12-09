#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <Arduino.h>

class PID_Controller {
   private:
    double _kp, _ki, _kd;
    double _lastError = 0.;
    double _lastIntegrativeValue = 0.;
    long _lastRun;
    int _outputLimit;
    int _integrativeLimit;
    double _threshold;
    int _stabilityCounter = 0;

   public:
    PID_Controller(double kp, double ki, double kd, double threshold = 1.7, int outputLimit = 255, int integrativeLimit = 35) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _threshold = threshold;
        _outputLimit = outputLimit;
        _integrativeLimit = integrativeLimit;
    }

    int calculateOutput(double error, double target = 0, unsigned long time = millis()) {
        error -= target;

        if (abs(error) < _threshold) {
            _stabilityCounter++;
            if (_stabilityCounter > 250) {
                this->reset();
                _stabilityCounter = 0;  // Caso um RESET na constante integrativa seja necessário, descomentar
            }
            _lastRun = time;
            _lastError = 0.;
            return 0;
        }
        _stabilityCounter = 0;

        int dt = time - _lastRun;
        double P = _kp * error;
        double I = _lastIntegrativeValue + (_ki * error * dt / 1000.0);
        double D = _kd * ((error - _lastError) / dt);
        I = constrain(I, -_integrativeLimit, _integrativeLimit);
        //error = (KFE * error) + ((1 - KFE) * lastError);  //Filtra a variacao do erro para a derivada

        _lastRun = time;
        _lastIntegrativeValue = I;
        _lastError = error;

        int output = static_cast<int>(P + I + D);
#ifdef DEBUG_PID
        Serial.print(" | P: ");
        Serial.printf("%5.3f", P);
        Serial.print(" | I: ");
        Serial.printf("%5.3f", I);
        Serial.print(" | D: ");
        Serial.printf("%5.3f", D);
#endif
        return constrain(output, -_outputLimit, _outputLimit);
    }

    void reset() {
        _lastIntegrativeValue = 0.;
    }
};

#endif  // PID_CONTROLLER_HPP