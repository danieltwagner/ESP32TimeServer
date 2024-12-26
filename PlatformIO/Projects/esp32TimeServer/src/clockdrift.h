#ifndef CLOCKDRIFT_H
#define CLOCKDRIFT_H

#include <Arduino.h>

class DriftCalculator {
private:
    static const int MAX_SAMPLES = 6;
    
    struct Sample {
        int64_t expected_us;
        int64_t measured_us;
    };
    
    Sample samples[MAX_SAMPLES];
    int sample_count = 0;
    int write_idx = 0;
    
    // Running statistics for linear regression
    int64_t sum_x = 0;     // Sum of x (expected times)
    int64_t sum_y = 0;     // Sum of y (error: measured - expected)
    int64_t sum_xy = 0;    // Sum of x*y
    int64_t sum_xx = 0;    // Sum of x*x
    
public:
    // Add a new measurement
    void addSample(int64_t measured_us, int64_t error_us);
    bool hasMaxSamples();
    double getDrift();
};

#endif // CLOCKDRIFT_H