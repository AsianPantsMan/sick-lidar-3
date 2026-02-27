#ifndef BUTTERWORTH_FILTER_H
#define BUTTERWORTH_FILTER_H

#include <stdint.h>

#define BUTTERWORTH_ORDER 4

typedef struct {
    // Filter state variables (delay line)
    float z[BUTTERWORTH_ORDER];

    // Numerator coefficients (b0, b1, ..., b4)
    float b[BUTTERWORTH_ORDER+1];

    // Denominator coefficients (a1, a2, ..., a4)
    float a[BUTTERWORTH_ORDER+1];

} Butterworth_Filter_t;

// Function Prototypes
void Butterworth_Init(Butterworth_Filter_t* filter);
float Butterworth_Apply(Butterworth_Filter_t* filter, float input);

#endif // BUTTERWORTH_FILTER_H
