#pragma once

#include <Arduino.h>

#define TIME_T0_STABILIZE 62000  // us

class PID_Controller {
   private:
    double  _kp, _ki, _kd;
    double  _p, _i, _d, _output;
    int     _outputLimit = 100;
    int     _integrativeLimit;
    double  _integrativeLimitPercentage;
    double  _threshold;
    double  _lastError             = 0.;
    double  _lastIntegrativeValue  = 0.;
    int64_t _lastRun               = 0UL;
    int64_t _lastUnstableTimestamp = 0UL;

   public:
    // O limite integrativo controlado pela variável integrativeLimit deve ser
    // passado em porcentagem. Esse valor definirá qual porcentagem máxima de
    // participação no output do controlador a constrante integrativa terá
    PID_Controller(double kp, double ki, double kd, double threshold = 1.7,
                   int integrativeLimitPercentage = 90) {
        _integrativeLimitPercentage = integrativeLimitPercentage / 100.;
        _threshold                  = threshold;
        _kp                         = kp;
        _ki                         = ki;
        _kd                         = kd;
        _integrativeLimit = (int)(_outputLimit * _integrativeLimitPercentage);
    }

    void setKp(double kp) { _kp = kp; }
    void setKi(double ki) { _ki = ki; }
    void setKd(double kd) { _kd = kd; }
    void setThreshold(double threshold) { _threshold = threshold; }

    double getKp() { return _kp; }
    double getKi() { return _ki; }
    double getKd() { return _kd; }
    double getThreshold() { return _threshold; }

    double getInstantP() { return _p; }
    double getInstantI() { return _i; }
    double getInstantD() { return _d; }
    int    getOutput() { return (int)_output; }

    int calculateOutput(double actualState, double target = 0.,
                        double  stateVariation = 0.,
                        int64_t time           = esp_timer_get_time()) {
        double error = actualState - target;
        double dt    = (time - _lastRun) / 1000000.;

        if (abs(error) < _threshold) {
            if (_lastUnstableTimestamp + TIME_T0_STABILIZE > time) reset();
            _p         = 0;
            _i         = 0;
            _d         = stateVariation;
            _lastRun   = time;
            _lastError = 0.;
            _output    = 0;
            return 0;
        }

        _p = _kp * error;
        _i = _lastIntegrativeValue + (_ki * error * dt);
        _d = stateVariation;
        _i = constrain(_i, -_integrativeLimit, _integrativeLimit);
        // Filtra a variacao do erro para a derivada
        // error = (KFE * error) + ((1 - KFE) * lastError);

        _lastUnstableTimestamp = time;
        _lastRun               = time;
        _lastIntegrativeValue  = _i;
        _lastError             = error;

        _output = _p + _i + _d;
        _output = constrain(_output, -_outputLimit, _outputLimit);
        _output = map(_output, -_outputLimit, _outputLimit, -255, 255);

        return (int)_output;
    }

    void reset() {
        _lastIntegrativeValue = 0.;
        _i                    = 0;
    }
};

PID_Controller pid(0.7, 0.3, 0.40);