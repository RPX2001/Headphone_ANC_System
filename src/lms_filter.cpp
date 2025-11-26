#include "lms_filter.h"
#include <string.h>

LMSFilter::LMSFilter() : buf_idx(0) {}

void LMSFilter::begin() {
    memset(weights, 0, sizeof(weights));
    memset(x_buffer, 0, sizeof(x_buffer));
    buf_idx = 0;
}

void LMSFilter::reset() {
    memset(x_buffer, 0, sizeof(x_buffer));
    buf_idx = 0;
}

// Optimized dot product with loop unrolling
inline float LMSFilter::dotProduct() {
    float sum = 0.0f;
    int idx = buf_idx;
    
    // Unroll 4x for speed
    for (int i = 0; i < FILTER_TAPS; i += 4) {
        sum += weights[i] * x_buffer[idx];
        idx = (idx - 1) & (FILTER_TAPS - 1);
        
        sum += weights[i+1] * x_buffer[idx];
        idx = (idx - 1) & (FILTER_TAPS - 1);
        
        sum += weights[i+2] * x_buffer[idx];
        idx = (idx - 1) & (FILTER_TAPS - 1);
        
        sum += weights[i+3] * x_buffer[idx];
        idx = (idx - 1) & (FILTER_TAPS - 1);
    }
    
    return sum;
}

// Optimized weight update with loop unrolling
inline void LMSFilter::updateWeights(float error) {
    float mu_error = LMS_MU * error;
    int idx = buf_idx;
    
    // Unroll 4x
    for (int i = 0; i < FILTER_TAPS; i += 4) {
        weights[i] += mu_error * x_buffer[idx];
        idx = (idx - 1) & (FILTER_TAPS - 1);
        
        weights[i+1] += mu_error * x_buffer[idx];
        idx = (idx - 1) & (FILTER_TAPS - 1);
        
        weights[i+2] += mu_error * x_buffer[idx];
        idx = (idx - 1) & (FILTER_TAPS - 1);
        
        weights[i+3] += mu_error * x_buffer[idx];
        idx = (idx - 1) & (FILTER_TAPS - 1);
    }
}

float LMSFilter::process(float primary, float error) {
    // Add new sample to ring buffer
    x_buffer[buf_idx] = primary;
    
    // Calculate filter output
    float output = dotProduct();
    
    // Update weights
    updateWeights(error);
    
    // Advance buffer index
    buf_idx = (buf_idx + 1) & (FILTER_TAPS - 1);
    
    return output;
}
