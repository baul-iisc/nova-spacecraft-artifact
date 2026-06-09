#include "conversion.h"

// Conversion functions for float16_t
float float16_to_float(float16_t value) {
    float32_t temp = f16_to_f32(value);
    return *(float*)&temp;
}

float16_t float_to_float16(float value) {
    float32_t temp = *(float32_t*)&value;
    return f32_to_f16(temp);
}

// Conversion functions for float32_t
float float32_to_float(float32_t value) {
    return *(float*)&value;
}

float32_t float_to_float32(float value) {
    return *(float32_t*)&value;
}

// Conversion functions for float64_t
double float64_to_double(float64_t value) {
    return *(double*)&value;
}

float64_t double_to_float64(double value) {
    return *(float64_t*)&value;
}

// Conversion functions for float128_t
_Float128 float128_to__Float128(float128_t value) {
    return *(_Float128*)&value;
}

float128_t _Float128_to_float128(_Float128 value) {
    return *(float128_t*)&value;
}
