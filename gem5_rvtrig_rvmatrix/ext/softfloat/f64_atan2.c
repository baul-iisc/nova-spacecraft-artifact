#include <math.h>
#include <stdint.h>
#include "softfloat.h"
#include "conversion.h"

float64_t f64_atan2(float64_t y, float64_t x) {
    double y_double = float64_to_double(y);
    double x_double = float64_to_double(x);
    double result = atan2(y_double, x_double);
    return double_to_float64(result);
}

