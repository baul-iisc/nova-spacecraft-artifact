# gem5 with NOVA RISC-V ISA Extensions (RVTrig + RVMatrix)

This is the gem5 simulator used in the IEEE TC paper *"Characterizing and
Accelerating Spacecraft Onboard Workloads on RISC-V Platform"* (TC-2025-09-0830).

It extends gem5 (v24.0-based) with two domain-specific RISC-V ISA extensions:

- **RVTrig** — CORDIC double-precision trigonometric instructions
  (`fsin.d`, `fcos.d`, `fatan2.d`, `fasin.d`, `facos.d`, `ftan.d`).
- **RVMatrix** — a 3×3 output-stationary systolic-array matrix unit
  (`mldS`, `msdS`, `sys.mmacd`, `mzero`, `minv`).

The custom decode models add `TimingSimpleCPU` latency stalls and shared
functional-unit contention so that hardware-accelerated runs are modelled
faithfully (not just functionally).

See `BUILD_IEEE_TC.txt` for the exact build flags and the full list of
simulation parameters used in the paper.

---

## Build

```bash
pip3 install -r requirements.txt          # Python dependencies
scons build/RISCV/gem5.opt -j$(nproc)     # ~20-30 min
```

## Single-core SE-mode run

The paper runs every workload in System-call Emulation (SE) mode, representing
bare-metal execution on a spacecraft processor (no OS overhead).

```bash
export GEM5_IEEE_TC_ARTIFACT=1            # paper accelerator model (single blocking instance per unit)

./build/RISCV/gem5.opt --outdir=m5out-run \
    configs/learning_gem5/part1/riscv_modified_configs.py \
    /path/to/spacecraft_workload.riscv \
    --l1i_size=8kB --l1d_size=16kB --l2_size=64kB \
    --system_mem_range=64GB --clock_freq=500MHz \
    --branch_pred=BiModeBP --prefetcher=None \
    --cpu_type=TimingSimpleCPU \
    --l1i_assoc=2 --l1d_assoc=2 \
    --l1i_tag_latency=2 --l1d_tag_latency=4 \
    --l1i_data_latency=2 --l1d_data_latency=4 \
    --l1i_response_latency=2 --l1d_response_latency=4 \
    --l1i_mshrs=2 --l1d_mshrs=2 \
    --l1i_tgts_per_mshr=8 --l1d_tgts_per_mshr=8
```

Results are written to `m5out-run/stats.txt`.

To compile the workload binaries and run all 28 workloads in batch, use the
scripts in `../synthetic_workloads_28/` (see that folder's README).

## Where the NOVA changes live

| Component | Path |
|-----------|------|
| Matrix instruction semantics | `src/arch/riscv/insts/matrix.cc`, `matrix.hh` |
| Matrix register file | `src/arch/riscv/regs/matrix.hh` |
| Matrix ISA decode/formats/templates | `src/arch/riscv/isa/.../matrix*.isa` |
| Trig + matrix decode entries | `src/arch/riscv/isa/decoder.isa` |

## CPU models for verification

- **`AtomicSimpleCPU`** — fast functional ISA/memory check; does *not* model
  accelerator latency or contention. Sanity pass only.
- **`TimingSimpleCPU`** — what the paper uses; models L1/L2/memory timing plus
  blocked stalls on the shared trig/matrix functional units. Required to trust
  hardware-variant results.

---

## Provenance

Based on gem5 v24.0 (https://www.gem5.org). The NOVA RISC-V extensions are the
contribution of this work and are released under the same license as gem5
(see `LICENSE`).
