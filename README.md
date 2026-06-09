# IEEE TC Artifact Bundle — TC-2025-09-0830

**Paper:** "Characterizing and Accelerating Spacecraft Onboard Workloads on RISC-V Platform"  
**Journal:** IEEE Transactions on Computers  
**DOI / submission ID:** TC-2025-09-0830  

Complete reproducibility artifact for the paper. All five components are included in this bundle.

---

## Bundle Contents

| Package | What is inside | Details |
|---------|---------------|---------|
| `synthetic_workloads_28/` | 144 `.c` source files (28 workloads × 5 variants) + build/simulation scripts + headers | See `synthetic_workloads_28/README.md` |
| `gem5_rvtrig_rvmatrix/` | gem5 v24.0 with NOVA ISA extensions (RVTrig + RVMatrix) | See `BUILD_IEEE_TC.txt` |
| `fpga_rtl_and_impl/` | SystemVerilog RTL, Vivado flow, timing/utilisation reports (VU9P @ 100 MHz, WNS +0.165 ns) | See `fpga_rtl_and_impl/README.md` |
| `riscv_gnu_toolchain_patches/` | Patched binutils + Spike sources for NOVA custom instructions + build script | See `riscv_gnu_toolchain_patches/NOVA_CUSTOM_ISA_EXTENSIONS_README.md` |
| `mcpat/` | McPAT v0.8 source + CACTI + per-accelerator XML templates + analysis scripts | See `mcpat/scripts/` |

---

## Quick Start (end-to-end reproduction)

### Step 1 — Build the NOVA RISC-V toolchain

```bash
cd riscv_gnu_toolchain_patches
bash build_toolchain.sh $HOME/riscv-nova      # ~40–60 min
export PATH=$HOME/riscv-nova/bin:$PATH
```

Requires: `autoconf automake texinfo flex bison libgmp-dev libmpfr-dev libmpc-dev`

### Step 2 — Build gem5

```bash
cd gem5_rvtrig_rvmatrix
pip3 install -r requirements.txt
scons build/RISCV/gem5.opt -j$(nproc)         # ~20–30 min
```

### Step 3 — Compile all 28 workloads

```bash
cd synthetic_workloads_28
export RISCV_GCC=$HOME/riscv-nova/bin/riscv64-unknown-elf-gcc
export GEM5_RISCV_DIR=../gem5_rvtrig_rvmatrix
zsh build_all_binaries.zsh                    # ~5–10 min, produces 145 binaries
```

### Step 4 — Run gem5 simulations

```bash
export GEM5_IEEE_TC_ARTIFACT=1               # use paper accelerator model
zsh run_gem5_simulations.zsh                 # ~4–8 hours for all 112 RISC-V runs
```

### Step 5 — Verify against paper numbers

```bash
bash verify_gem5_mv9_paper.sh
# Expected: 3.13× geomean speedup, peak 9.29× on ADCS
```

### Step 6 — McPAT power analysis (optional)

```bash
cd ../mcpat/source && make -j$(nproc)
cd ../scripts
python3 spacecraft_soc_power_mcpat.py
# Expected: 37.2% energy reduction
```

### Step 7 — FPGA re-synthesis (optional, requires Vivado 2024.2)

```bash
cd ../fpga_rtl_and_impl/vivado
make full    # target: xcvu9p-flga2104-2-e @ 100 MHz
# Pre-built reports already in vivado/reports/
```

---

## Key Paper Results

| Metric | Value |
|--------|-------|
| Geomean speedup (18 workloads, both accels) | **3.13×** |
| Peak speedup (ADCS, both accels) | **9.29×** |
| Energy reduction (geomean, 22 nm McPAT) | **37.2%** |
| FPGA timing closure @ 100 MHz | WNS = **+0.165 ns** (VU9P) |
| FPGA utilization (LUTs) | 181,533 (15.36% of VU9P) |

---

## Self-Contained Artifact

This repository is the complete, authoritative artifact for the paper. Everything
needed to reproduce the results is included here — the gem5 source with the NOVA
ISA extensions (`gem5_rvtrig_rvmatrix/`), the RISC-V toolchain patches and build
script (`riscv_gnu_toolchain_patches/`), the FPGA RTL and reports
(`fpga_rtl_and_impl/`), the McPAT models (`mcpat/`), and all 28 workload sources
(`synthetic_workloads_28/`). No external repositories need to be cloned.

The RISC-V GNU toolchain itself is built from the official upstream source with the
patches in `riscv_gnu_toolchain_patches/` applied; see that folder's instructions.

---

## Provenance and Licensing

- Synthetic workloads are derived from real ISRO spacecraft onboard flight software characteristics.  
- Original ISRO flight software and mission datasets are **not** included.  
- Mission-sensitive operational details are sanitised.  
- All source files are licensed under **Apache 2.0**. See `LICENSE`.

See `MANIFEST.md` for component versions and FPGA result checksums.  
See `synthetic_workloads_28/README.md` for full compilation and simulation instructions.
