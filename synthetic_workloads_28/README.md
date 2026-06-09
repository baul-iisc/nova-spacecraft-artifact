# Synthetic Spacecraft Workloads — IEEE TC Paper TC-2025-09-0830

**Paper:** "Characterizing and Accelerating Spacecraft Onboard Workloads on RISC-V Platform"  
**Journal:** IEEE Transactions on Computers

Synthetic benchmark suite derived from production spacecraft onboard flight software characteristics.
Algorithmic structure, BCE distributions, matrix dimensions, and memory-access behaviour are preserved.
Original ISRO flight software and mission datasets are **not** included.

---

## Directory Layout

```
synthetic_workloads_28/
├── flight_compliance.h          # Shared flight-compliance shim (included by all workloads)
├── sw_inv3.h                    # Software-inverse helper (used by HW-mat variants)
├── build_all_binaries.zsh       # Cross-compile all 28×5 variants
├── run_gem5_simulations.zsh     # Run gem5 for all built binaries
├── verify_gem5_mv9_paper.sh     # Reproduce paper numbers (28×4 RISC-V = 112 runs)
├── verify_gem5_mv9.sh           # Full verification (all 145 binaries)
├── WORKLOAD_LIST.md             # Per-workload BCE breakdown and simulation parameters
├── 01_Navigation/               # 5 source files (SW + 4 HW variants)
│   ├── spacecraft_navigation.c
│   ├── spacecraft_navigation_hw.c
│   ├── spacecraft_navigation_hw_trig.c
│   ├── spacecraft_navigation_hw_mat.c
│   ├── spacecraft_navigation_hw_mat_systolic.c
│   └── clock_stub.c             # clock_gettime stub for bare-metal builds
├── 02_Rendezvous/ ... 27_FDIR/  # Same 5-file pattern per workload
└── 28_AI/                       # Multi-file workload (8 .c + 1 .h + Makefile)
```

Each workload directory contains up to 5 source variants:

| Source suffix | Accelerators used | Paper configuration |
|---------------|-------------------|---------------------|
| `spacecraft_<name>.c` | None | Baseline RISC-V (SW only) |
| `spacecraft_<name>_hw.c` | CORDIC + 3×3 systolic | Both accelerators |
| `spacecraft_<name>_hw_trig.c` | CORDIC trigonometric unit | Trig-only |
| `spacecraft_<name>_hw_mat.c` | Matrix accelerator (MMACDF) | Mat-only |
| `spacecraft_<name>_hw_mat_systolic.c` | 3×3 systolic array (SYS_MMACD) | Systolic mat-only |

---

## Prerequisites

### 1 — NOVA RISC-V Toolchain

The HW variants use custom ISA extensions (`fsin.d`, `fcos.d`, `fatan2.d`, `sys.mmacd`, `mlds`, …)
that require the patched RISC-V GNU toolchain.

```bash
cd ../riscv_gnu_toolchain_patches
bash build_toolchain.sh $HOME/riscv-nova
export PATH=$HOME/riscv-nova/bin:$PATH
```

Build time: ~40–60 minutes.  
Dependencies: `autoconf automake texinfo flex bison libgmp-dev libmpfr-dev libmpc-dev`

`build_toolchain.sh` checks out the official upstream RISC-V GNU toolchain, applies
the NOVA patches from `../riscv_gnu_toolchain_patches/patches/`, and builds it into
the prefix you pass. No separate repository needs to be cloned.

Verify NOVA instructions are recognised:
```bash
echo 'void f(void){ __asm__ volatile("fsin.d fa0,fa0"); }' | \
    riscv64-unknown-elf-gcc -x c - -c -o /dev/null && echo "NOVA toolchain OK"
```

### 2 — gem5 with NOVA ISA Extensions

```bash
cd ../gem5_rvtrig_rvmatrix
pip3 install -r requirements.txt    # Python deps (pyfiglet, six, …)
scons build/RISCV/gem5.opt -j$(nproc)
```

gem5 version: v24.0-based with NOVA RISC-V modifications.
See `../gem5_rvtrig_rvmatrix/BUILD_IEEE_TC.txt` for full build flags and simulation parameters.

**Build time:** ~20–30 minutes.

---

## Step 1 — Compile All Workloads

```bash
cd synthetic_workloads_28/
export RISCV_GCC=$HOME/riscv-nova/bin/riscv64-unknown-elf-gcc
export GEM5_RISCV_DIR=../gem5_rvtrig_rvmatrix
zsh build_all_binaries.zsh
```

This produces per-workload binaries in each subdirectory:
```
01_Navigation/spacecraft_navigation.riscv           # SW baseline
01_Navigation/spacecraft_navigation_hw.riscv        # Both accelerators
01_Navigation/spacecraft_navigation_hw_trig.riscv   # Trig-only
01_Navigation/spacecraft_navigation_hw_mat_systolic.riscv
...
```

**Compile flags:**
```
RISC-V:  -O3 -march=rv64gc -mabi=lp64d -fno-common -static -DFLIGHT_HOST_IO
ARM:     -O3 -march=armv7-a -mfpu=neon -mfloat-abi=hard -static -DFLIGHT_HOST_IO
```

**Estimated time:** 5–10 minutes for all 145 binaries.

---

## Step 2 — Run gem5 Simulations

```bash
cd synthetic_workloads_28/
export GEM5_RISCV_DIR=../gem5_rvtrig_rvmatrix
export GEM5_IEEE_TC_ARTIFACT=1      # enables single-instance accelerator model (required)
zsh run_gem5_simulations.zsh
```

gem5 simulation parameters matching the paper (Table II):

| Parameter | Value |
|-----------|-------|
| CPU model | `TimingSimpleCPU` |
| Clock frequency | 500 MHz |
| L1-I cache | 8 kB, 2-way, tag/data latency 2/2 |
| L1-D cache | 16 kB, 2-way, tag/data latency 4/4 |
| L2 cache | 64 kB |
| MSHRs | 2 per L1 |
| Branch predictor | `BiModeBP` |
| Memory range | 64 GB |
| Prefetcher | None |

Results are written to `m5out-<workload>-<variant>/stats.txt` (one directory per run).

**Estimated time:** 4–8 hours for all 112 RISC-V runs.

---

## Step 3 — Reproduce Paper Numbers

```bash
cd synthetic_workloads_28/
bash verify_gem5_mv9_paper.sh
```

The script runs 112 gem5 simulations (28 workloads × 4 RISC-V variants: SW, HW, HW_TRIG,
HW_MAT_SYSTOLIC), extracts execution times from `stats.txt`, and compares against the
paper's Table III values with ±5% tolerance.

Expected results (representative workloads):

| Workload | SW→HW Speedup | Geomean check |
|----------|---------------|---------------|
| Navigation | ~3.1× | |
| ADCS | ~9.3× (peak) | |
| OdnP | ~3.6× | |
| SNS | ~4.4× | |
| **All 18 reported** | | **3.13× geomean** |

---

## Step 4 — McPAT Power Analysis

After gem5 simulations, run power modelling:

```bash
cd ../mcpat/source
make -j$(nproc)
cd ../scripts
python3 spacecraft_soc_power_mcpat.py
```

Expected result: **37.2% energy reduction** (HW both vs SW baseline, geomean 18 workloads).  
Technology node: 22 nm, 500 MHz. XML template: `../mcpat/xml/riscv_spacecraft_22nm_500MHz.xml`.

---

## Environment Variables Summary

| Variable | Purpose | Example |
|----------|---------|---------|
| `RISCV_GCC` | Path to NOVA riscv64-unknown-elf-gcc | `$HOME/riscv-nova/bin/riscv64-unknown-elf-gcc` |
| `GEM5_RISCV_DIR` | Root of gem5_rvtrig_rvmatrix tree | `../gem5_rvtrig_rvmatrix` |
| `GEM5_IEEE_TC_ARTIFACT` | Use paper accelerator model | Set to `1` |
| `GEM5_RESULTS_DIR` | Override output directory | `/scratch/results` (optional) |

---

## Troubleshooting

**`fsin.d: unrecognised instruction`**  
→ The standard toolchain does not support NOVA ISA. Rebuild using `../riscv_gnu_toolchain_patches/build_toolchain.sh`.

**gem5 segfaults or traps on HW variant at runtime**  
→ Binary was compiled with a non-NOVA toolchain. It will compile but trap on the custom instruction. Recompile with the NOVA toolchain (verify with the one-liner above).

**`stats.txt` is empty or missing**  
→ Check `m5out-.../simout` for errors. Ensure `GEM5_IEEE_TC_ARTIFACT=1` is exported before running gem5.

**`28_AI` compilation fails**  
→ `28_AI/` uses its own `Makefile`; `build_all_binaries.zsh` invokes it automatically. Ensure `make` is on your PATH.

**`clock_stub.c: No such file`**  
→ `clock_stub.c` is only in `01_Navigation/` and linked specifically for that workload. It is not needed by the other 27 workloads.

---

## Provenance

Synthetic workloads derived from real ISRO spacecraft flight software:
- **Algorithmic structure** (computational kernels, data-flow, loop structure) preserved
- **BCE distributions** (trig/matrix/FFT/KalmanFilter ratios) match production software profiles
- **Matrix dimensions** match actual onboard attitude/navigation algorithm sizes (3×3 to 24×24)
- **Original ISRO flight software and mission data are NOT included**

All 144 source files carry Apache 2.0 SPDX headers. See `../LICENSE`.

Full BCE breakdown and per-workload simulation step counts: see `WORKLOAD_LIST.md`.
