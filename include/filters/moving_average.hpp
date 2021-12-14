#ifndef MOVING_AVERAGE_HPP
#define MOVING_AVERAGE_HPP

#include <vector>

using std::vector;

class MovingAverage {
   private:
    int _numberOfSamples;
    vector<double> _samples;

    double sumSamples() {
        double sum = 0;
        for (int x : _samples) {
            sum += x;
        }
        return sum;
    }

    void addSample(double newSample) {
        for (int i = _numberOfSamples; i > 0; i--) {
            _samples[i] = _samples[i - 1];
        }
        _samples[0] = newSample;
    }

   public:
    MovingAverage(int samples = 10) {
        _numberOfSamples = samples;
    };

    double filter(double sample) {
        this->addSample(sample);
        return this->sumSamples() / _numberOfSamples;
    }
};

#endif  // MOVING_AVERAGE_HPP