#ifndef TRIG_FUNCTIONS_H
#define TRIG_FUNCTIONS_H

#include <stdint.h>
#include "platform.h"
#include "fpu_constant.h" // Include the constants from fpu_constant.h

// Function declarations
void shortShift128Left(uint64_t aSig1, uint64_t aSig0, int count, uint64_t *zSig1, uint64_t *zSig0);
uint64_t estimateDiv128To64(uint64_t aSig1, uint64_t aSig0, uint64_t bSig);
void mul128By64To192(uint64_t aSig0, uint64_t aSig1, uint64_t bSig, uint64_t *zSig0, uint64_t *zSig1, uint64_t *zSig2);
void sub128(uint64_t aSig1, uint64_t aSig0, uint64_t bSig1, uint64_t bSig0, uint64_t *zSig1, uint64_t *zSig0);
uint64_t reduce_trig_arg(int expDiff, int *zSign, uint64_t *aSig0, uint64_t *aSig1);

static uint64_t argument_reduction_kernel(uint64_t aSig0, int exp, uint64_t *zSig0, uint64_t *zSig1) {
    uint64_t term0, term1, term2;
    uint64_t aSig1 = 0;

    shortShift128Left(aSig1, aSig0, exp, &aSig1, &aSig0);
    uint64_t q = estimateDiv128To64(aSig1, aSig0, FLOAT_PI_HI);
    mul128By64To192(FLOAT_PI_HI, FLOAT_PI_LO, q, &term0, &term1, &term2);
    sub128(aSig1, aSig0, term0, term1, zSig1, zSig0);
    return q;
}

#endif // TRIG_FUNCTIONS_H
