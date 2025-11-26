#ifndef LMS_FILTER_H
#define LMS_FILTER_H

#include "anc_config.h"

class LMSFilter {
public:
    LMSFilter();
    
    // Initialize filter
    void begin();
    
    // Process one sample - highly optimized
    float process(float primary, float error);
    
    // Reset filter state
    void reset();

private:
    // Filter coefficients
    float weights[FILTER_TAPS];
    
    // Input delay line (ring buffer)
    float x_buffer[FILTER_TAPS];
    int buf_idx;
    
    // Optimized dot product
    inline float dotProduct();
    
    // Optimized weight update
    inline void updateWeights(float error);
};

#endif // LMS_FILTER_H
