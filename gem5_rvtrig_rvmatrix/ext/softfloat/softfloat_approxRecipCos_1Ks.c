/*============================================================================

This C source file is part of the SoftFloat IEEE Floating-Point Arithmetic
Package, Release 3d, by John R. Hauser.

=============================================================================*/

#include <stdint.h>
#include "platform.h"
#include "primitives.h"
#include "primitives.h"
#include "primitives_trig.h"
const uint16_t softfloat_approxRecipCos_1k0s[16] = {
    0xB504, 0xB4C9, 0xB48F, 0xB455, 0xB41B, 0xB3E1, 0xB3A7, 0xB36D,
    0xB333, 0xB2F9, 0xB2BF, 0xB285, 0xB24B, 0xB211, 0xB1D7, 0xB19D
};
const uint16_t softfloat_approxRecipCos_1k1s[16] = {
    0x5A82, 0x5A7D, 0x5A78, 0x5A73, 0x5A6E, 0x5A69, 0x5A64, 0x5A5F,
    0x5A5A, 0x5A55, 0x5A50, 0x5A4B, 0x5A46, 0x5A41, 0x5A3C, 0x5A37
};
