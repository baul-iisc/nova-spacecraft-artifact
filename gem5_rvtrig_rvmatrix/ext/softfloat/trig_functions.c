#include "trig_functions.h"

void shortShift128Left(uint64_t aSig1, uint64_t aSig0, int count, uint64_t *zSig1, uint64_t *zSig0) {
    if (count == 0) {
        *zSig1 = aSig1;
        *zSig0 = aSig0;
    } else if (count < 64) {
        *zSig1 = (aSig1 << count) | (aSig0 >> (64 - count));
        *zSig0 = aSig0 << count;
    } else {
        *zSig1 = aSig0 << (count - 64);
        *zSig0 = 0;
    }
}

uint64_t estimateDiv128To64(uint64_t aSig1, uint64_t aSig0, uint64_t bSig) {
    if (aSig1 >= bSig) return UINT64_MAX;
    uint64_t bSig0 = bSig >> 32;
    uint64_t bSig1 = bSig & 0xFFFFFFFF;
    uint64_t q = aSig1 / bSig0;
    uint64_t r = aSig1 % bSig0;
    uint64_t term0 = q * bSig1;
    uint64_t term1 = (r << 32) | (aSig0 >> 32);
    if (term0 > term1) {
        q--;
        term1 += bSig;
        if (term0 > term1) q--;
    }
    return q;
}

void mul128By64To192(uint64_t aSig0, uint64_t aSig1, uint64_t bSig, uint64_t *zSig0, uint64_t *zSig1, uint64_t *zSig2) {
    uint64_t z0, z1, z2, z3;
    uint64_t more1, more2;

    z0 = (uint64_t)(uint32_t)aSig0 * (uint64_t)(uint32_t)bSig;
    z1 = ((uint64_t)(uint32_t)aSig0 * (bSig >> 32)) + (z0 >> 32);
    z2 = ((aSig0 >> 32) * (uint64_t)(uint32_t)bSig) + (z1 >> 32);
    z3 = ((aSig0 >> 32) * (bSig >> 32)) + (z2 >> 32);

    z1 = (z1 & 0xFFFFFFFF) | (z2 << 32);
    z2 = (z2 >> 32) | (z3 << 32);

    more1 = (uint64_t)(uint32_t)aSig1 * (uint64_t)(uint32_t)bSig;
    more2 = ((uint64_t)(uint32_t)aSig1 * (bSig >> 32)) + (more1 >> 32);
    z2 += more1;
    z3 = ((aSig1 >> 32) * (uint64_t)(uint32_t)bSig) + (more2 >> 32) + (z2 >> 32);

    *zSig0 = z0;
    *zSig1 = z1;
    *zSig2 = z3;
}

void sub128(uint64_t aSig1, uint64_t aSig0, uint64_t bSig1, uint64_t bSig0, uint64_t *zSig1, uint64_t *zSig0) {
    uint64_t diff0 = aSig0 - bSig0;
    uint64_t diff1 = aSig1 - bSig1 - (diff0 > aSig0);
    *zSig1 = diff1;
    *zSig0 = diff0;
}

uint64_t reduce_trig_arg(int expDiff, int *zSign, uint64_t *aSig0, uint64_t *aSig1) {
    uint64_t zSig0, zSig1;
    uint64_t q = argument_reduction_kernel(*aSig0, expDiff, &zSig0, &zSig1);
    if (q & 1) {
        *zSign = !(*zSign);
    }
    *aSig0 = zSig0;
    *aSig1 = zSig1;
    return q;
}
