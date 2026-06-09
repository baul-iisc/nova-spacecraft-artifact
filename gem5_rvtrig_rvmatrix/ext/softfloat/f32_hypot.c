#include <math.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float32_t f32_hypot(float32_t a, float32_t b) {
    float a_float = float32_to_float(a);
    float b_float = float32_to_float(b);
    float result = hypotf(a_float, b_float);
    return float_to_float32(result);
}
