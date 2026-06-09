# Octa-Core RV64IMAFDCV Space Processor V7 — Virtex UltraScale+ VU9P

## Overview

A fault-tolerant octa-core **64-bit RISC-V** space processor with hardware floating-point and vector extensions, migrated to Virtex UltraScale+ for improved timing and resources.

**Author:** Chandraboul, IISc  
**Target:** Xilinx Virtex UltraScale+ VU9P (xcvu9p-flga2104-2-e)  
**Space-Grade Part:** XQRVU9P (radiation-tolerant)  
**Clock:** 125 MHz  
**ISA:** RV64IMAFDCV  

---

## Migration Rationale (VU095 → VU9P)

The design failed timing closure on VU095 at 100 MHz (WNS = -6.7 ns). Root causes:
- UltraScale (20nm) fabric speed limitations at the target frequency
- Critical path delay exceeding the 10 ns clock period
- Routing congestion on the VU095 die with the full SoC

The VU9P (Virtex UltraScale+, 16nm FinFET+) resolves these issues:

| Parameter | VU095 (old) | VU9P (new) | Improvement |
|-----------|-------------|------------|-------------|
| **Process** | 20nm UltraScale | 16nm UltraScale+ | ~25% faster fabric |
| **CLB LUTs** | 537,600 | 1,182,240 | 2.2× |
| **Flip-Flops** | 1,075,200 | 2,364,480 | 2.2× |
| **Block RAM (36Kb)** | 1,728 | 2,160 | 1.25× |
| **UltraRAM** | None | 960 (270 Mb) | New resource |
| **DSP Slices** | 768 | 6,840 | 8.9× |
| **Transceivers** | GTH 16.3G | GTY 32.75G | 2× bandwidth |
| **Max Clock (fabric)** | ~80-100 MHz | ~125-150 MHz | +25-50% |
| **Rad Tolerance** | XQRVU095 | XQRVU9P | Both space-qualified |
| **Package** | FFVB1760 | FLGA2104 | More I/O |

---

## Architecture

Identical to V6 (VU095) — the RTL is shared via symlink:

- 8 × RV64IMAFDCV cores in 4 DLS (Dual-Lockstep) pairs
- Integrated FP64 FPU (IEEE 754, FADD/FSUB/FMUL/FDIV/FSQRT/FMA)
- RVV 1.0 vector coprocessor (VLEN=256, 4 lanes)
- Per-core CORDIC + 3×3 Systolic Array via PCPI
- L1 I$ (32KB×8) + L1 D$ (32KB×8) + L2 (2MB shared) with SECDED ECC
- MOESI coherence over snoop bus
- Peripherals: SpaceWire×4, TSN Ethernet×4, CXL×4, JESD204B×2, SPI×4, UART, JTAG
- TinyML subsystem (32 PEs), CCSDS compression

---

## Quick Start

### Build (Full Synth + Impl)
```bash
cd vivado
make full       # Combined synthesis + implementation
```

### Build (Step by Step)
```bash
cd vivado
make synth      # Synthesis only
make impl       # Synthesis → Implementation
make bit        # Synthesis → Implementation → Bitstream
```

### Re-run Implementation Only (from synth checkpoint)
```bash
cd vivado
make impl_only
```

### Clean
```bash
cd vivado
make clean
```

---

## Directory Structure

```
fpga_vu9p_v7_full/
├── rtl/              → symlink to ../fpga_vu095_v6_full/rtl/
├── sim/              → simulation testbenches
├── vivado/
│   ├── constraints/
│   │   ├── timing_vu9p.xdc    # 125 MHz clock constraint
│   │   └── pins_vu9p.xdc      # FLGA2104 pin assignments
│   ├── scripts/
│   │   ├── synth_pcpi_soc.tcl # Full synth+impl flow
│   │   ├── run_synth.tcl      # Synthesis only
│   │   ├── run_impl.tcl       # Implementation only (from synth)
│   │   ├── run_bit.tcl        # Bitstream generation
│   │   └── impl_only.tcl      # P&R only (from synth DCP)
│   ├── build/                  # Checkpoints and outputs
│   ├── reports/                # Utilization, timing, power
│   └── Makefile
├── docs/
└── README.md
```

---

## Expected Resource Utilization

Based on VU095 post-implementation results (design uses ~109K LUTs):

| Resource | Used | VU9P Available | Est. Util% |
|----------|------|---------------|------------|
| CLB LUTs | ~109,243 | 1,182,240 | ~9.2% |
| CLB Registers | ~77,500 | 2,364,480 | ~3.3% |
| Block RAM | ~121 | 2,160 | ~5.6% |
| DSP Slices | 0 | 6,840 | 0% |
| I/O | ~146 | ~832 | ~17.6% |

The very low utilization on VU9P means:
- Minimal routing congestion → better timing
- Vivado has maximum freedom for placement → better WNS
- Room for future expansion (more cores, wider vectors, etc.)

---

## Synthesis Strategy

The VU9P build uses performance-optimized directives (different from VU095's area-optimized approach):

| Stage | VU095 Directive | VU9P Directive | Rationale |
|-------|----------------|----------------|-----------|
| Synth | AreaOptimized_high | PerformanceOptimized | Prioritize speed over area |
| Opt | ExploreWithRemap | ExploreWithRemap | (same) |
| Place | SpreadLogic_high | ExtraTimingOpt | Timing-focused on SSI device |
| Route | AlternateCLBRouting | AggressiveExplore | Explore more routing solutions |
| Phys Opt | AggressiveExplore | AggressiveExplore | (same) |

Additional features enabled:
- `-retiming` : Register balancing across pipeline stages
- `-flatten_hierarchy rebuilt` : Allow cross-module optimization
