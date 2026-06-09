/*
 * SPDX-License-Identifier: Apache-2.0
 * Copyright 2025 the authors of IEEE TC paper TC-2025-09-0830
 * ("Characterizing and Accelerating Spacecraft Onboard Workloads
 *  on RISC-V Platform").  Licensed under the Apache License,
 * Version 2.0; see LICENSE at the root of this repository.
 *
 * Synthetic workload derived for academic research from
 * production spacecraft onboard flight software characteristics.
 */
/* Custom clock() that avoids rdcycle CSR (not available in Spike pk) */
#include <time.h>
clock_t clock(void) {
    return (clock_t)0;
}
