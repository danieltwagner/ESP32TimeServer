#include "clockdrift.h"

void DriftCalculator::addSample(int64_t measured_us, int64_t error_us) {
    // If we're at max samples, subtract the oldest sample
    if (sample_count == MAX_SAMPLES) {
        Sample& old = samples[write_idx];
        sum_y -= old.measured_us - old.expected_us;
    } else {
        sample_count++;
    }
    
    // Store new sample - reconstruct expected from measured and error
    int64_t expected_us = measured_us - error_us;
    samples[write_idx] = {expected_us, measured_us};
    
    // Update running sum of errors
    sum_y += error_us;
    
    write_idx = (write_idx + 1) % MAX_SAMPLES;
}

double DriftCalculator::getDrift() {
    if (sample_count < 1) return 0.0;
    
    // Calculate average error rate
    double avg_error = static_cast<double>(sum_y) / sample_count;
    
    // Use the most recent expected time as reference
    int64_t latest_expected = samples[(write_idx + MAX_SAMPLES - 1) % MAX_SAMPLES].expected_us;
    return (avg_error / latest_expected);
}

bool DriftCalculator::hasMaxSamples() {
    return sample_count == MAX_SAMPLES;
}
