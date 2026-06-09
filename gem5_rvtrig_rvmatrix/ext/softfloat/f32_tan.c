#include <math.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float32_t f32_tan(float32_t a) {
    float a_float = float32_to_float(a);
    float result = tanf(a_float);
    return float_to_float32(result);
}

