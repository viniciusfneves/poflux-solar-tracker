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

   public:
    PID_Controller(double kp, double ki, double kd, double threshold = 1.7, int outputLimit = 255, int integrativeLimit = 200) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
        _threshold = threshold;
        _outputLimit = outputLimit;
        _integrativeLimit = integrativeLimit;
    }

    int calculateOutput(double error, unsigned long time = millis()) {
        if (abs(error) < _threshold) {
            this->reset();
            return 0;
        }

        int dt = time - _lastRun;
        _lastRun = time;

        double P = _kp * error;
        double I = _lastIntegrativeValue + (_ki * error * dt / 1000.0);
        I = constrain(I, -_integrativeLimit, _integrativeLimit);
        double D = _kd * ((error - _lastError) / dt);
        //error = (KFE * error) + ((1 - KFE) * lastError);  //Filtra a variacao do erro para a derivada

        _lastIntegrativeValue = I;

        int output = static_cast<int>(P + I + D);

        return constrain(output, -_outputLimit, _outputLimit);
    }

    void reset() {
        _lastIntegrativeValue = 0.;
    }
};

#endif  // PID_CONTROLLER_HPP