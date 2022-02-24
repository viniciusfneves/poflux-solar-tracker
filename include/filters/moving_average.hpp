#pragma once

#define SAMPLES 50

class MovingAverage {
   private:
    double _samples[SAMPLES];

    double _sumSamples() {
        double sum = 0;
        for (int x : _samples) {
            sum += x;
        }
        return sum;
    }

    void _addSample(double newSample) {
        for (int i = SAMPLES - 1; i > 0; i--) {
            _samples[i] = _samples[i - 1];
        }
        _samples[0] = newSample;
    }

   public:
    double getAverage(double sample) {
        _addSample(sample);
        return _sumSamples() / SAMPLES;
    }

    void setInitialValue(double value) {
        for (int i = 0; i < SAMPLES; i++) {
            _samples[i] = value;
        }
    }
};

MovingAverage filter;