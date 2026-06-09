#ifndef SPACECRAFT_SW_INV3_H
#define SPACECRAFT_SW_INV3_H

#include <math.h>

static inline void sw_inv3(const double A[9], double Ainv[9])
{
    const double a11 = A[0], a12 = A[1], a13 = A[2];
    const double a21 = A[3], a22 = A[4], a23 = A[5];
    const double a31 = A[6], a32 = A[7], a33 = A[8];

    const double det =
        a11 * (a22 * a33 - a23 * a32) -
        a12 * (a21 * a33 - a23 * a31) +
        a13 * (a21 * a32 - a22 * a31);

    if (fabs(det) < 1e-12) {
        // Singular: return identity.
        for (int k = 0; k < 9; k++)
            Ainv[k] = (k == 0 || k == 4 || k == 8) ? 1.0 : 0.0;
        return;
    }

    const double invDet = 1.0 / det;
    // A^{-1} = adj(A)/det; adj is transpose of cofactor matrix
    Ainv[0] = (a22 * a33 - a23 * a32) * invDet;
    Ainv[1] = (a13 * a32 - a12 * a33) * invDet;
    Ainv[2] = (a12 * a23 - a13 * a22) * invDet;
    Ainv[3] = (a23 * a31 - a21 * a33) * invDet;
    Ainv[4] = (a11 * a33 - a13 * a31) * invDet;
    Ainv[5] = (a13 * a21 - a11 * a23) * invDet;
    Ainv[6] = (a21 * a32 - a22 * a31) * invDet;
    Ainv[7] = (a12 * a31 - a11 * a32) * invDet;
    Ainv[8] = (a11 * a22 - a12 * a21) * invDet;
}

#endif

