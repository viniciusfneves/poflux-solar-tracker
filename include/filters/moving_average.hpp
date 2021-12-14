#ifndef MOVING_AVERAGE_HPP
#define MOVING_AVERAGE_HPP

#define SAMPLES 25

struct MovingAverage {
    double _samples[SAMPLES];

    double sumSamples() {
        double sum = 0;
        for (int x : _samples) {
            sum += x;
        }
        return sum;
    }

    void addSample(double newSample) {
        for (int i = SAMPLES - 1; i > 0; i--) {
            _samples[i] = _samples[i - 1];
        }
        _samples[0] = newSample;
    }

    double filter(double sample) {
        addSample(sample);
        return sumSamples() / SAMPLES;
    }
};

#endif  // MOVING_AVERAGE_HPP