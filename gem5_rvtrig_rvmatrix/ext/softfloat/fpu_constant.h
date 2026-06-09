#ifndef FPU_CONSTANT_H
#define FPU_CONSTANT_H

#include <stdint.h>

//////////////////////////////
// PI, PI/2, PI/4 constants
//////////////////////////////

#define FLOATX80_PI_EXP  (0x4000)

// 128-bit PI fraction
#ifdef HIGH_PRECISION
#define FLOAT_PI_HI (U64(0xc90fdaa22168c234))
#define FLOAT_PI_LO (U64(0xc4c6628b80dc1cd1))
#else
#define FLOAT_PI_HI (U64(0xc90fdaa22168c234))
#define FLOAT_PI_LO (U64(0xC000000000000000))
#endif

#define FLOATX80_PI2_EXP  (0x3FFF)
#define FLOATX80_PI4_EXP  (0x3FFE)

//////////////////////////////
// 3PI/4 constant
//////////////////////////////

#define FLOATX80_3PI4_EXP (0x4000)

// 128-bit 3PI/4 fraction
#ifdef HIGH_PRECISION
#define FLOAT_3PI4_HI (U64(0x96cbe3f9990e91a7))
#define FLOAT_3PI4_LO (U64(0x9394c9e8a0a5159c))
#else
#define FLOAT_3PI4_HI (U64(0x96cbe3f9990e91a7))
#define FLOAT_3PI4_LO (U64(0x9000000000000000))
#endif

//////////////////////////////
// 1/LN2 constant
//////////////////////////////

#define FLOAT_LN2INV_EXP  (0x3FFF)

// 128-bit 1/LN2 fraction
#ifdef HIGH_PRECISION
#define FLOAT_LN2INV_HI (U64(0xb8aa3b295c17f0bb))
#define FLOAT_LN2INV_LO (U64(0x0000000000000000))
#else
#define FLOAT_LN2INV_HI (U64(0xb8aa3b295c17f0bb))
#define FLOAT_LN2INV_LO (U64(0x0000000000000000))
#endif

#define FLOAT_PI ((float128_t){ .v = { FLOAT_PI_HI, FLOAT_PI_LO } })
#define FLOAT_3PI4 ((float128_t){ .v = { FLOAT_3PI4_HI, FLOAT_3PI4_LO } })
#define FLOAT_LN2INV ((float128_t){ .v = { FLOAT_LN2INV_HI, FLOAT_LN2INV_LO } })

#endif // FPU_CONSTANT_H