#ifndef CONVERSION_H
#define CONVERSION_H

#include "softfloat.h"

// Conversion functions for float16_t
float float16_to_float(float16_t value);
float16_t float_to_float16(float value);

// Conversion functions for float32_t
float float32_to_float(float32_t value);
float32_t float_to_float32(float value);

// Conversion functions for float64_t
double float64_to_double(float64_t value);
float64_t double_to_float64(double value);

// Conversion functions for float128_t
_Float128 float128_to__Float128(float128_t value);
float128_t _Float128_to_float128(_Float128 value);

#endif // CONVERSION_H
