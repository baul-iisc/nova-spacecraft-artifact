#include <math.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float32_t f32_atan2(float32_t y, float32_t x) {
    float y_float = float32_to_float(y);
    float x_float = float32_to_float(x);
    float result = atan2f(y_float, x_float);
    return float_to_float32(result);
}

