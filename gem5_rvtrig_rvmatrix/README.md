# NOVA Processor - Multicore Spacecraft DSE (Design Space Exploration)

## PhD Research: Area-Constrained Accelerator Optimization for Multicore RISC-V Spacecraft Processors

This project uses gem5 to simulate a multicore RISC-V spacecraft processor with shared hardware accelerators (trigonometric CORDIC, matrix multiplication, VPU, NPU, GPU). The goal is to find the **knee point** of the performance-area curve -- the optimal core-accelerator mix that maximizes performance within a fixed silicon area budget.

---

## Table of Contents

1. [Project Structure](#1-project-structure)
2. [The 21 Spacecraft Workloads](#2-the-21-spacecraft-workloads)
3. [Source Files and Headers](#3-source-files-and-headers)
4. [Compilation](#4-compilation)
5. [gem5 Build](#5-gem5-build)
6. [gem5 Configuration](#6-gem5-configuration)
7. [Running Simulations](#7-running-simulations)
8. [Monitoring Progress](#8-monitoring-progress)
9. [Checking for Failures](#9-checking-for-failures)
10. [Contention Tracking & Analysis](#10-contention-tracking--analysis)
11. [Key gem5 Modifications](#11-key-gem5-modifications)
12. [Analysis Scripts](#12-analysis-scripts)
13. [Important Design Decisions](#13-important-design-decisions)
14. [Troubleshooting](#14-troubleshooting)
15. [V2 Comprehensive Workloads (Docking, On-Orbit Servicing, Cloud Detection)](#15-v2-comprehensive-workloads-docking-on-orbit-servicing-cloud-detection)

---

## 1. Project Structure

```
gem5-multicore21-spacecraft-dse/
├── build/RISCV/gem5.opt                          # Built gem5 binary (RISC-V)
├── nova_timing_dse_fixed.py                      # gem5 SE-mode config script
├── run_dse_simulations_forkjoin.sh               # Master simulation launch script (21 workloads)
├── run_v2_workloads.sh                           # V2 simulation launch script (3 workloads)
├── monitor_v2.sh                                 # V2 simulation monitoring script
├── Summary_of_PhD_Status_Review_Meeting.md       # Professor's guidance
├── src/cpu/simple/
│   ├── timing.cc                                 # MODIFIED: contention model + suspendContext fix
│   └── timing.hh                                 # MODIFIED: multi-instance SharedFUState
├── new_spacecraft_workloads_fork_join/            # Source code (fork-join fixed)
│   ├── compile_all_workloads_fork_join.sh        # Compilation script (21 workloads)
│   ├── compile_v2_workloads.sh                   # Compilation script (3 V2 workloads)
│   ├── fix_multi_site.py                         # Fork-join transformation tool
│   ├── binaries_new2/                            # Compiled RISC-V binaries (incl. V2)
│   ├── riscv-matrix-updated2_mv_3x3.h            # Matrix accelerator header
│   ├── nova_cordic_accel.h                        # CORDIC trig accelerator header
│   ├── nova_space_gpu.h                           # GPU accelerator header
│   ├── nova_vpu_accel.h                           # VPU accelerator header
│   ├── nova_npu_accel.h                           # NPU accelerator header
│   ├── accel_regs.h                               # Accelerator register definitions
│   ├── nova_accel_regs.h                          # NOVA accelerator registers
│   ├── thread_safe_rand.h                         # Thread-safe random number generator
│   └── spacecraft-*-workload/                     # 21 + 3 V2 workload source directories
├── simulation_results_forkjoin/                   # Simulation output (21 workloads)
│   ├── {workload}_{phase}_c{cores}_t{trig}_m{mat}/
│   │   ├── stats.txt                              # gem5 statistics
│   │   └── config.ini                             # gem5 configuration dump
│   └── logs/
│       └── {workload}_{phase}_c{cores}_t{trig}_m{mat}.log  # Simulation log + NOVA stats
└── simulation_results_v2/                         # Simulation output (3 V2 workloads)
    ├── {workload}_v2_{phase}_c{cores}_t{trig}_m{mat}/
    │   ├── stats.txt
    │   └── config.ini
    ├── v2_launch.log                              # V2 launch log
    └── logs/
        └── {workload}_v2_{phase}_c{cores}_t{trig}_m{mat}.log
```

---

## 2. The 21 Spacecraft Workloads

| # | Workload Name | Binary Prefix | Description |
|---|--------------|---------------|-------------|
| 1 | `adcs` | `adcs_st`, `adcs_t{1,2,4,8}` | Attitude Determination & Control System |
| 2 | `ai_autonomous_nav` | `ai_autonomous_nav_st`, `..._t{1,2,4,8}` | AI-based Autonomous Navigation |
| 3 | `anomaly_detection` | `anomaly_detection_st`, `..._t{1,2,4,8}` | Spacecraft Anomaly Detection |
| 4 | `cloud_detection` | `cloud_detection_st`, `..._t{1,2,4,8}` | Cloud Detection (Earth observation) |
| 5 | `collision_avoidance` | `collision_avoidance_st`, `..._t{1,2,4,8}` | Orbital Collision Avoidance |
| 6 | `docking` | `docking_st`, `..._t{1,2,4,8}` | Spacecraft Docking Algorithm |
| 7 | `ds_navigation` | `ds_navigation_st`, `..._t{1,2,4,8}` | Deep Space Navigation |
| 8 | `formation_control` | `formation_control_st`, `..._t{1,2,4,8}` | Satellite Formation Control |
| 9 | `hyperspectral` | `hyperspectral_st`, `..._t{1,2,4,8}` | Hyperspectral Image Compression |
| 10 | `lambic_guidance` | `lambic_guidance_st`, `..._t{1,2,4,8}` | Lambert Guidance Algorithm |
| 11 | `landing` | `landing_st`, `..._t{1,2,4,8}` | Spacecraft Landing Algorithm |
| 12 | `mars_helicopter` | `mars_helicopter_st`, `..._t{1,2,4,8}` | Mars Helicopter Control |
| 13 | `onorbit_servicing` | `onorbit_servicing_st`, `..._t{1,2,4,8}` | On-Orbit Servicing Operations |
| 14 | `orbit_determination` | `orbit_determination_st`, `..._t{1,2,4,8}` | Orbit Determination |
| 15 | `path_planning` | `path_planning_st`, `..._t{1,2,4,8}` | Rover Path Planning |
| 16 | `robotic_arm` | `robotic_arm_st`, `..._t{1,2,4,8}` | Spacecraft Robotic Arm Control |
| 17 | `sensor_fusion` | `sensor_fusion_st`, `..._t{1,2,4,8}` | Multi-Sensor Fusion (EKF) |
| 18 | `slam` | `slam_st`, `..._t{1,2,4,8}` | Rover SLAM (Simultaneous Localization & Mapping) |
| 19 | `sns_imaging` | `sns_imaging_st`, `..._t{1,2,4,8}` | SNS Imaging Algorithm |
| 20 | `terrain_visualization` | `terrain_visualization_st`, `..._t{1,2,4,8}` | Terrain Visualization (GPU-accelerated) |
| 21 | `vision_nav` | `vision_nav_st`, `..._t{1,2,4,8}` | Vision-Based Navigation |

### Binary Naming Convention

- `{workload}_st` -- Single-threaded baseline
- `{workload}_t1` -- Multi-threaded, 1 thread (for validation)
- `{workload}_t2` -- Multi-threaded, 2 threads
- `{workload}_t4` -- Multi-threaded, 4 threads
- `{workload}_t8` -- Multi-threaded, 8 threads

All binaries are statically-linked RISC-V 64-bit ELF executables located in:
```
new_spacecraft_workloads_fork_join/binaries_new2/
```

---

## 3. Source Files and Headers

### Shared Headers (in `new_spacecraft_workloads_fork_join/`)

| Header | Purpose |
|--------|---------|
| `riscv-matrix-updated2_mv_3x3.h` | Custom RISC-V matrix extension instructions (3x3 double precision) |
| `nova_cordic_accel.h` | CORDIC trigonometric accelerator (hw_sin, hw_cos, hw_tan, hw_atan, hw_atan2, hw_asin, hw_acos) |
| `nova_space_gpu.h` | GPU accelerator instructions |
| `nova_vpu_accel.h` | Vector Processing Unit instructions |
| `nova_npu_accel.h` | Neural Processing Unit instructions |
| `accel_regs.h` | Accelerator register definitions |
| `nova_accel_regs.h` | NOVA-specific accelerator registers |
| `thread_safe_rand.h` | Thread-safe random number generation for multi-threaded workloads |

### Source Directories

Each workload has its own directory under `new_spacecraft_workloads_fork_join/`:

| Directory | ST Source | MT Source (fork-join) |
|-----------|-----------|----------------------|
| `spacecraft-adcs-workload/` | `spacecraft-adcs-algorithm-new.c` | `spacecraft-adcs-algorithm-multithreaded.c` |
| `spacecraft-ai-autonomous-navigation-workload/` | `*_hw_all.c` | `*_hw_all_mt_proper.c` |
| `spacecraft-anomaly-detection-workload/` | `*_hw_matrix.c` | `*_hw_matrix_mt.c` |
| `spacecraft-cloud-detection-workload/` | `*_hw_matrix.c` | `*-multithread.c` |
| `spacecraft-collision-avoidance-workload/` | `*_hw_all.c` | `*_mt_proper.c` |
| `spacecraft-docking-workload/` | `*_hw_all.c` | `*_hw_all_mt_proper.c` |
| `spacecraft-ds-navigation-workload/` | `*_hw_all.c` | `*_hw_all_mt_proper.c` |
| `spacecraft-formation-control-workload/` | `*_hw_all_3x3.c` | `*_mt_proper.c` |
| `spacecraft-hyperspectral-workload/` | `*_hw_all_3x3.c` | `*_mt_proper.c` |
| `spacecraft-lambic-guidance-workload/` | `*_hw_all_3x3.c` | `*_hw_all_3x3_MT.c` |
| `spacecraft-landing-workload/` | `*_hw_all_3x3.c` | `*_mt_proper.c` |
| `mars-helicopter-workload/` | `*_hw_all.c` | `*_hw_all_mt_proper.c` |
| `spacecraft-onorbit-servicing-workload/` | `*_hw_all.c` | `*_hw_all_mt.c` |
| `spacecraft-orbit-determination-workload/` | `*_hw_all.c` | `*-mt-proper.c` |
| `space-rover-path-planning-workload/` | `*_hw_all_3x3.c` | `*_mt_proper.c` |
| `spacecraft-robotic-arm-workload/` | `*_hw_all_3x3.c` | `*_mt_proper.c` |
| `spacecraft-sensor-fusion-workload/` | `*_hw_all_3x3.c` | `*_mt_proper.c` |
| `space-rover-SLAM-workload/` | `*_hw_all_3x3.c` | `*_hw_all_3x3_MT.c` |
| `spacecraft-sns-imaging-workload/` | `*_hw_all_3x3.c` | `*_mt_proper.c` |
| `spacecraft-terrain-visualization-workload/` | `*_hw_gpu.c` | `*_hw_gpu_mt.c` |
| `spacecraft-vision-nav-workload/` | `*_hw_all_3x3.c` | `*_mt_proper.c` |

---

## 4. Compilation

### Prerequisites

```bash
# RISC-V cross-compiler must be in PATH
which riscv64-unknown-linux-gnu-gcc
# Should output: /path/to/riscv64-unknown-linux-gnu-gcc
```

### Compile All Workloads

```bash
cd /data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/new_spacecraft_workloads_fork_join

# Run the compilation script
bash compile_all_workloads_fork_join.sh 2>&1 | tee compile_log.txt
```

### Compile a Single Workload Manually

```bash
CC=riscv64-unknown-linux-gnu-gcc
BASE_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/new_spacecraft_workloads_fork_join"
BIN_DIR="$BASE_DIR/binaries_new2"
CFLAGS="-O3 -static -march=rv64gc -I${BASE_DIR} -I.."
LDFLAGS="-lm -lpthread"

# Example: compile ds_navigation ST
$CC $CFLAGS -o $BIN_DIR/ds_navigation_st \
    spacecraft-ds-navigation-workload/spacecraft-ds-nav-algorithm-new1_hw_all.c \
    $LDFLAGS

# Example: compile ds_navigation MT for 4 threads
$CC $CFLAGS -DNUM_THREADS=4 -o $BIN_DIR/ds_navigation_t4 \
    spacecraft-ds-navigation-workload/spacecraft-ds-nav-algorithm-new1_hw_all_mt_proper.c \
    $LDFLAGS
```

### Verify Binaries

```bash
# List all compiled binaries
ls -la binaries_new2/

# Check binary count (expect 105 = 21 workloads x 5 variants)
ls binaries_new2/ | wc -l

# Verify a binary is RISC-V
file binaries_new2/ds_navigation_st
# Expected: ELF 64-bit LSB executable, UCB RISC-V, ...
```

---

## 5. gem5 Build

### Build gem5 from Source

```bash
cd /data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse

# Build (uses all available cores)
scons build/RISCV/gem5.opt -j$(nproc)
```

### Rebuild After Modifying gem5 Source

```bash
# Only recompiles changed files + relinks
scons build/RISCV/gem5.opt -j$(nproc)
```

### Key Modified Files

| File | Modification |
|------|-------------|
| `src/cpu/simple/timing.hh` | `SharedFUState` struct: added `instanceBusyUntil[16]` array for multi-instance tracking |
| `src/cpu/simple/timing.cc` | `tryAcquireSharedFU()`: N-instance contention model; `suspendContext()`: removed overly strict assertion |

---

## 6. gem5 Configuration

### Configuration Script

**File:** `nova_timing_dse_fixed.py`

| Parameter | Value | Description |
|-----------|-------|-------------|
| CPU model | `TimingSimpleCPU` | Timing-accurate simple CPU (SE mode) |
| Clock | `1 GHz` | System clock frequency |
| Memory | `32 GB DDR4_2400_16x4` | Large to prevent pthread allocation failures |
| ISA | RISC-V 64-bit | Target architecture |
| Caches | None (direct bus) | Memory goes through `SystemXBar` to `MemCtrl` |

### Environment Variables (Set Automatically)

| Variable | Default | Description |
|----------|---------|-------------|
| `NOVA_NUM_TRIG_ACCELS` | 1 | Number of trigonometric accelerator instances |
| `NOVA_NUM_MAT_ACCELS` | 1 | Number of matrix accelerator instances |
| `NOVA_TRIG_LATENCY` | 32 cycles | Latency per trig operation |
| `NOVA_MAT_LATENCY` | 27 cycles | Latency per matrix operation |

### Accelerator Latencies (Hardcoded in timing.cc)

| Accelerator | Latency (cycles) | OpClasses |
|-------------|------------------|-----------|
| Trig (CORDIC) | 32 | `FloatSin`, `FloatCos`, `FloatTan`, `FloataSin`, `FloataCos`, `FloataTan`, `FloataTan2` |
| Matrix | 27 | `Matrix`, `MatrixMov`, `MatrixOP` |
| VPU | 100 | `SimdMisc`, `SimdMult`, `SimdMultAcc`, `SimdAdd`, `SimdShift`, `SimdSqrt`, etc. |
| NPU | 200 | `SimdFloatMultAcc`, `SimdFloatMult`, `SimdFloatMisc` |
| GPU | 150 | `SimdFloatAdd`, `SimdFloatCmp`, `SimdFloatCvt`, `SimdFloatAlu`, etc. |

### Run a Single Simulation Manually

```bash
GEM5="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/build/RISCV/gem5.opt"
CONFIG="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/nova_timing_dse_fixed.py"
BIN_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/new_spacecraft_workloads_fork_join/binaries_new2"

# Example: ds_navigation, 4 cores, 2 trig accels, 1 mat accel
$GEM5 --outdir=my_output_dir $CONFIG \
    --num-cores=4 \
    --num-trig=2 \
    --num-mat=1 \
    --num-threads=4 \
    --workload=$BIN_DIR/ds_navigation_t4 \
    --iterations=100
```

---

## 7. Running Simulations

### Full DSE Sweep (798 Simulations)

```bash
cd /data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse

# Launch all simulations (runs up to 32 in parallel)
nohup bash run_dse_simulations_forkjoin.sh > /tmp/dse_launch.log 2>&1 &
echo "Launch PID: $!"
```

### Simulation Phases

| Phase | Description | Configs | Sims |
|-------|-------------|---------|------|
| 1 | ST baselines (1 core, 1T+1M) | 1 per workload | 21 |
| 2 | Core scalability (1,2,4,8 cores, 1T+1M) | 4 per workload | 84 |
| 3 | 4-core accelerator mix DSE (trig={1..4} x mat={1..4}, skip 1,1) | 15 per workload | 315 |
| 4 | 8-core accelerator mix DSE (trig={1..4} x mat={1..4}, skip 1,1) | 15 per workload | 315 |
| 5 | 2-core accelerator mix (trig={1,2} x mat={1,2}, skip 1,1) | 3 per workload | 63 |
| **Total** | | | **798** |

### Output Naming Convention

```
simulation_results_forkjoin/
├── {workload}_st_c1_t1_m1/          # Phase 1: ST baseline
├── {workload}_scale_c{1,2,4,8}_t1_m1/  # Phase 2: Core scaling
├── {workload}_4c_t{T}_m{M}/        # Phase 3: 4-core DSE
├── {workload}_8c_t{T}_m{M}/        # Phase 4: 8-core DSE
├── {workload}_2c_t{T}_m{M}/        # Phase 5: 2-core DSE
└── logs/
    └── {same_name}.log              # Simulation log + NOVA contention stats
```

---

## 8. Monitoring Progress

### Quick Status Check

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"

echo "Completed: $(find $OUTPUT_DIR -name stats.txt | wc -l) / 798"
echo "Running: $(pgrep -c -f gem5.opt 2>/dev/null || echo 0)"
```

### Live Monitoring (Auto-refresh Every 60s)

```bash
watch -n 60 'OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"; echo "Completed: $(find $OUTPUT_DIR -name stats.txt | wc -l) / 798 | Running: $(pgrep -c -f gem5.opt 2>/dev/null || echo 0)"'
```

### Phase-by-Phase Breakdown

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"

echo "Phase 1 (ST baselines):   $(ls -d $OUTPUT_DIR/*_st_*/stats.txt 2>/dev/null | wc -l) / 21"
echo "Phase 2 (Core scaling):   $(ls -d $OUTPUT_DIR/*_scale_*/stats.txt 2>/dev/null | wc -l) / 84"
echo "Phase 3 (4-core DSE):     $(ls -d $OUTPUT_DIR/*_4c_*/stats.txt 2>/dev/null | wc -l) / 315"
echo "Phase 4 (8-core DSE):     $(ls -d $OUTPUT_DIR/*_8c_*/stats.txt 2>/dev/null | wc -l) / 315"
echo "Phase 5 (2-core DSE):     $(ls -d $OUTPUT_DIR/*_2c_*/stats.txt 2>/dev/null | wc -l) / 63"
```

### Per-Workload 4-Core DSE Completion

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"

for wl in adcs ai_autonomous_nav anomaly_detection cloud_detection collision_avoidance docking ds_navigation formation_control hyperspectral lambic_guidance landing mars_helicopter onorbit_servicing orbit_determination path_planning robotic_arm sensor_fusion slam sns_imaging terrain_visualization vision_nav; do
    count=$(ls -d $OUTPUT_DIR/${wl}_4c_*/stats.txt 2>/dev/null | wc -l)
    printf "  %-25s %2d/15\n" "$wl" "$count"
done
```

---

## 9. Checking for Failures

### Find All Failed Simulations

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"

for log in $OUTPUT_DIR/logs/*.log; do
    if grep -q "Assertion.*failed\|Page table fault\|panic" "$log" 2>/dev/null; then
        echo "FAILED: $(basename $log .log)"
    fi
done
```

### Check Launch Script for Aborts

```bash
grep "Aborted" /tmp/dse_launch.log 2>/dev/null
```

### Common Failure Modes and Fixes

| Error | Cause | Fix |
|-------|-------|-----|
| `Assertion '_status == BaseSimpleCPU::Running' failed` | Thread exit during memory stall | Fixed in `timing.cc:suspendContext()` -- removed strict assertion |
| `Page table fault` | Stack overflow in main thread | Use `static __thread` for large arrays in worker functions |
| `pthread allocation failure` | Insufficient simulated memory | Config uses 32GB memory |

---

## 10. Contention Tracking & Analysis

### How It Works

The NOVA v3.1 contention model in `src/cpu/simple/timing.cc`:
1. Each accelerator type (trig, mat, VPU, NPU, GPU) has N independent instances
2. When a core issues an accelerator instruction, it acquires the earliest-free instance
3. If all instances are busy, the core **blocks** until one frees up
4. Statistics are printed at simulation end in the log file

### View Contention Stats for a Single Simulation

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"

# Example: ds_navigation 4-core, 1 trig, 1 mat
grep "NOVA\|TrigAccel\|MatAccel\|Total stall" $OUTPUT_DIR/logs/ds_navigation_scale_c4_t1_m1.log
```

**Example output:**
```
NOVA v3.1: TrigAccels=1 (lat=32), MatAccels=1 (lat=27), ...
TrigAccel (1 instances): requests=1163608, queued=40695 (3.5%), avgWait=19.4 cycles, ...
MatAccel (1 instances): requests=853301, queued=9507 (1.1%), avgWait=15.1 cycles, ...
Total stall ticks from contention: 935051000
```

### Compare Contention Across Accelerator Configurations

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"

# Compare a workload across all 4-core accelerator mixes
WORKLOAD="adcs"  # Change this to any workload name

echo "Config           simSec      TrigQ       MatQ     StallTicks"
for config in scale_c4_t1_m1 4c_t2_m1 4c_t3_m1 4c_t4_m1 4c_t1_m2 4c_t1_m3 4c_t1_m4 4c_t2_m2 4c_t3_m3 4c_t4_m4; do
    stats="$OUTPUT_DIR/${WORKLOAD}_${config}/stats.txt"
    log="$OUTPUT_DIR/logs/${WORKLOAD}_${config}.log"
    if [ -f "$stats" ]; then
        secs=$(grep "simSeconds" "$stats" | awk '{print $2}')
        trig_q=$(grep "TrigAccel" "$log" 2>/dev/null | tail -1 | grep -oP 'queued=\K[0-9]+')
        mat_q=$(grep "MatAccel" "$log" 2>/dev/null | tail -1 | grep -oP 'queued=\K[0-9]+')
        stall=$(grep "Total stall ticks from contention" "$log" 2>/dev/null | tail -1 | grep -oP '[0-9]+$')
        printf "%-16s %10s %10s %10s %14s\n" "$config" "$secs" "${trig_q:-0}" "${mat_q:-0}" "${stall:-0}"
    fi
done
```

### Full Contention Sweep for ALL Workloads

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"

for wl in adcs ai_autonomous_nav anomaly_detection cloud_detection collision_avoidance docking ds_navigation formation_control hyperspectral lambic_guidance landing mars_helicopter onorbit_servicing orbit_determination path_planning robotic_arm sensor_fusion slam sns_imaging terrain_visualization vision_nav; do
    base="$OUTPUT_DIR/${wl}_scale_c4_t1_m1/stats.txt"
    if [ ! -f "$base" ]; then continue; fi
    
    echo "--- $wl (4-core) ---"
    printf "  %-16s %10s %8s %8s %14s\n" "Config" "simSec" "TrigQ" "MatQ" "StallTicks"
    for config in scale_c4_t1_m1 4c_t2_m1 4c_t3_m1 4c_t4_m1 4c_t1_m2 4c_t1_m3 4c_t1_m4 4c_t2_m2 4c_t3_m3 4c_t4_m4; do
        stats="$OUTPUT_DIR/${wl}_${config}/stats.txt"
        log="$OUTPUT_DIR/logs/${wl}_${config}.log"
        if [ -f "$stats" ]; then
            secs=$(grep "simSeconds" "$stats" | awk '{print $2}')
            trig_q=$(grep "TrigAccel" "$log" 2>/dev/null | tail -1 | grep -oP 'queued=\K[0-9]+')
            mat_q=$(grep "MatAccel" "$log" 2>/dev/null | tail -1 | grep -oP 'queued=\K[0-9]+')
            stall=$(grep "Total stall ticks from contention" "$log" 2>/dev/null | tail -1 | grep -oP '[0-9]+$')
            printf "  %-16s %10s %8s %8s %14s\n" "$config" "$secs" "${trig_q:-0}" "${mat_q:-0}" "${stall:-0}"
        fi
    done
    echo ""
done
```

### Core Scalability Analysis

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"

printf "%-25s %10s %10s %10s %10s %8s %8s %8s\n" "Workload" "ST(s)" "2c(s)" "4c(s)" "8c(s)" "Sp-2c" "Sp-4c" "Sp-8c"
echo "-------------------------------------------------------------------------------------------------------"

for wl in adcs ai_autonomous_nav anomaly_detection cloud_detection collision_avoidance docking ds_navigation formation_control hyperspectral lambic_guidance landing mars_helicopter onorbit_servicing orbit_determination path_planning robotic_arm sensor_fusion slam sns_imaging terrain_visualization vision_nav; do
    st=$(grep "simSeconds" "$OUTPUT_DIR/${wl}_st_c1_t1_m1/stats.txt" 2>/dev/null | awk '{print $2}')
    t2=$(grep "simSeconds" "$OUTPUT_DIR/${wl}_scale_c2_t1_m1/stats.txt" 2>/dev/null | awk '{print $2}')
    t4=$(grep "simSeconds" "$OUTPUT_DIR/${wl}_scale_c4_t1_m1/stats.txt" 2>/dev/null | awk '{print $2}')
    t8=$(grep "simSeconds" "$OUTPUT_DIR/${wl}_scale_c8_t1_m1/stats.txt" 2>/dev/null | awk '{print $2}')
    
    if [ -n "$st" ]; then
        sp2=$(echo "$st $t2" | awk '{if($2>0) printf "%.2fx",$1/$2; else print "N/A"}')
        sp4=$(echo "$st $t4" | awk '{if($2>0) printf "%.2fx",$1/$2; else print "N/A"}')
        sp8=$(echo "$st $t8" | awk '{if($2>0) printf "%.2fx",$1/$2; else print "N/A"}')
        printf "%-25s %10s %10s %10s %10s %8s %8s %8s\n" "$wl" "${st:-N/A}" "${t2:-N/A}" "${t4:-N/A}" "${t8:-N/A}" "$sp2" "$sp4" "$sp8"
    fi
done
```

### Extract Per-CPU Instruction Counts (Verify Fork-Join)

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"
WORKLOAD="ds_navigation"  # Change this

for cores in 1 2 4 8; do
    stats="$OUTPUT_DIR/${WORKLOAD}_scale_c${cores}_t1_m1/stats.txt"
    if [ -f "$stats" ]; then
        echo "=== $WORKLOAD ${cores}-core ==="
        grep "commitStats0.numInsts" "$stats" | head -${cores}
    fi
done
```

---

## 11. Key gem5 Modifications

### A. Multi-Instance Shared Accelerator Contention Model (NOVA v3.1)

**File:** `src/cpu/simple/timing.hh` -- `SharedFUState` struct

**Before (buggy):** Single `busyUntil` timestamp per accelerator type -- contention was identical regardless of `numAccels`.

**After (fixed):** Array of `instanceBusyUntil[MAX_ACCEL_INSTANCES]` timestamps, one per physical accelerator instance.

```cpp
struct SharedFUState {
    static constexpr int MAX_ACCEL_INSTANCES = 16;
    Tick instanceBusyUntil[MAX_ACCEL_INSTANCES] = {};  // Per-instance busy tracking
    int numInstances = 1;        // Number of physical instances
    uint64_t totalRequests = 0;
    uint64_t queuedRequests = 0; // Requests that had to wait
    uint64_t totalWaitCycles = 0;
    uint64_t totalWaitTicks = 0;
    uint64_t totalStallTicks = 0;
    // Contention period tracking
    Tick contentionRangeStart = 0;
    Tick contentionRangeEnd = 0;
    uint64_t totalContentionTicks = 0;
};
```

**File:** `src/cpu/simple/timing.cc` -- `tryAcquireSharedFU()`

The function now:
1. Finds the instance that will be free earliest: `min(instanceBusyUntil[0..numAccels-1])`
2. If `earliestFreeTime <= now`: acquires it with zero contention wait (only operation latency)
3. If `earliestFreeTime > now`: all instances busy, must wait (contention + operation latency)

### B. suspendContext Assertion Fix

**File:** `src/cpu/simple/timing.cc` -- `suspendContext()`

**Before:** `assert(_status == BaseSimpleCPU::Running);` -- caused abort when thread exits during memory stall.

**After:** Removed the assertion. Properly handles suspension from any CPU state (`DcacheWaitResponse`, `DTBWaitResponse`, etc.) and cleans up shared-FU stall state.

---

## 12. Analysis Scripts

### Comprehensive Knee-Point Analysis (Python)

Save as `analyze_dse.py` and run with `python3 analyze_dse.py`:

```python
#!/usr/bin/env python3
"""Accelerator Contention & Knee-Point Analysis"""
import os

OUTDIR = "/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"

WORKLOADS = [
    "adcs", "ai_autonomous_nav", "anomaly_detection", "cloud_detection",
    "collision_avoidance", "docking", "ds_navigation", "formation_control",
    "hyperspectral", "lambic_guidance", "landing", "mars_helicopter",
    "onorbit_servicing", "orbit_determination", "path_planning", "robotic_arm",
    "sensor_fusion", "slam", "sns_imaging", "terrain_visualization", "vision_nav"
]

def get_sim_seconds(statsfile):
    try:
        with open(statsfile) as f:
            for line in f:
                if "simSeconds" in line:
                    return float(line.split()[1])
    except:
        return None

# Core Scalability
print("=" * 100)
print("  CORE SCALABILITY (1 -> 2 -> 4 -> 8 cores)")
print("=" * 100)
print(f"{'Workload':<25} {'ST(s)':>10} {'2c(s)':>10} {'4c(s)':>10} {'8c(s)':>10} {'Sp-2c':>7} {'Sp-4c':>7} {'Sp-8c':>7}")

for wl in WORKLOADS:
    st = get_sim_seconds(os.path.join(OUTDIR, f"{wl}_st_c1_t1_m1", "stats.txt"))
    t2 = get_sim_seconds(os.path.join(OUTDIR, f"{wl}_scale_c2_t1_m1", "stats.txt"))
    t4 = get_sim_seconds(os.path.join(OUTDIR, f"{wl}_scale_c4_t1_m1", "stats.txt"))
    t8 = get_sim_seconds(os.path.join(OUTDIR, f"{wl}_scale_c8_t1_m1", "stats.txt"))
    if st:
        sp2 = f"{st/t2:.2f}x" if t2 else "N/A"
        sp4 = f"{st/t4:.2f}x" if t4 else "N/A"
        sp8 = f"{st/t8:.2f}x" if t8 else "N/A"
        print(f"{wl:<25} {st:>10.3f} {t2 or 'N/A':>10} {t4 or 'N/A':>10} {t8 or 'N/A':>10} {sp2:>7} {sp4:>7} {sp8:>7}")

# 4-Core Accelerator DSE
print("\n" + "=" * 100)
print("  4-CORE ACCELERATOR MIX DSE (Performance Matrix)")
print("=" * 100)

for wl in WORKLOADS:
    st = get_sim_seconds(os.path.join(OUTDIR, f"{wl}_st_c1_t1_m1", "stats.txt"))
    configs = {}
    for trig in range(1, 5):
        for mat in range(1, 5):
            name = f"{wl}_scale_c4_t1_m1" if (trig == 1 and mat == 1) else f"{wl}_4c_t{trig}_m{mat}"
            t = get_sim_seconds(os.path.join(OUTDIR, name, "stats.txt"))
            if t: configs[(trig, mat)] = t
    if not configs or not st: continue

    print(f"\n--- {wl.upper()} (ST={st:.3f}s) ---")
    print(f"  {'':>8}", end="")
    for m in range(1, 5): print(f"  mat={m:>6}", end="")
    print()
    for trig in range(1, 5):
        print(f"  trig={trig:<4}", end="")
        for mat in range(1, 5):
            t = configs.get((trig, mat))
            if t:
                sp = st / t
                print(f"  {sp:>7.2f}x", end="")
            else:
                print(f"  {'N/A':>8}", end="")
        print()
```

---

## 13. Important Design Decisions

### Fork-Join Threading Model

All multi-threaded workloads use a **fork-join** pattern where:
- The **main thread on cpu0 runs thread 0's work directly** (not idle)
- Threads 1 through N-1 are created via `pthread_create` on cpu1 through cpu(N-1)
- All N cores actively compute the parallel workload
- Total work = 100% (N threads doing 1/N each = full work)

This was changed from the original model where the main thread was idle and only N-1 workers ran.

### Area-Constrained Optimization (Professor's Guidance)

The correct research question is NOT "How many accelerators do I need?" but rather:

> "Given a fixed area budget X, what combination of cores and accelerators maximizes overall performance?"

Key principles:
1. Some contention and non-zero queueing is acceptable if performance impact is negligible (<2-3%)
2. Adding accelerators beyond the **knee point** yields marginal gains not worth the area cost
3. Trade-offs across accelerator types must be evaluated (e.g., fewer trig + more matrix)
4. Applications must first be evaluated for scalability -- if they saturate at 4 cores, 8-core designs aren't useful

### Area Model (Simple)

For the DSE analysis, the area model assumes:
- 1 trig accelerator = 1 area unit
- 1 matrix accelerator = 1 area unit
- Total accelerator area = num_trig + num_mat

---

## 14. Troubleshooting

### Kill All Running Simulations

```bash
# Kill the launch script first (it respawns gem5 processes)
pkill -f "run_dse_simulations"
sleep 2

# Then kill all gem5 processes
pkill -9 -f gem5.opt
sleep 3

# Verify
echo "Remaining: $(pgrep -c -f gem5.opt 2>/dev/null || echo 0)"
```

### Re-run Only Failed Simulations

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin"
GEM5="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/build/RISCV/gem5.opt"
CONFIG="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/nova_timing_dse_fixed.py"
BIN_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/new_spacecraft_workloads_fork_join/binaries_new2"

# Find and re-run failed simulations
for log in $OUTPUT_DIR/logs/*.log; do
    if grep -q "Assertion.*failed\|Page table fault\|panic" "$log" 2>/dev/null; then
        name=$(basename "$log" .log)
        echo "Re-running: $name"
        # Extract parameters from the name and re-launch
        # (parse workload, cores, trig, mat from name)
    fi
done
```

### Check if gem5 Binary is Up to Date

```bash
ls -la /data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/build/RISCV/gem5.opt
# Check modification timestamp matches your last build
```

### Disk Space Check

```bash
du -sh /data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_forkjoin/
# Each simulation produces ~500KB-2MB of stats + config
# 798 simulations ~ 400MB-1.6GB total
```

---

## 15. V2 Comprehensive Workloads (Docking, On-Orbit Servicing, Cloud Detection)

Three workloads were rewritten as computationally heavy V2 versions to ensure proper multicore scaling and meaningful accelerator contention study.

### V2 Source Files

| Workload | Directory | ST Source | MT Source |
|----------|-----------|-----------|-----------|
| Docking V2 | `new_spacecraft_workloads_fork_join/spacecraft-docking-workload/` | `spacecraft-docking-v2_hw_all.c` | `spacecraft-docking-v2_hw_all_mt.c` |
| On-Orbit Servicing V2 | `new_spacecraft_workloads_fork_join/spacecraft-onorbit-servicing-workload/` | `onorbit-servicing-v2_hw_all.c` | `onorbit-servicing-v2_hw_all_mt.c` |
| Cloud Detection V2 | `new_spacecraft_workloads_fork_join/spacecraft-cloud-detection-workload/` | `cloud-detection-v2_hw_all.c` | `cloud-detection-v2_hw_all_mt.c` |

### V2 Binaries

All V2 binaries are in `new_spacecraft_workloads_fork_join/binaries_new2/` with `_v2_` suffix:

| Binary | Description |
|--------|-------------|
| `docking_v2_st` | Docking V2, single-threaded |
| `docking_v2_t{1,2,4,8}` | Docking V2, multi-threaded (1/2/4/8 threads) |
| `onorbit_servicing_v2_st` | On-Orbit Servicing V2, single-threaded |
| `onorbit_servicing_v2_t{1,2,4,8}` | On-Orbit Servicing V2, multi-threaded |
| `cloud_detection_v2_st` | Cloud Detection V2, single-threaded |
| `cloud_detection_v2_t{1,2,4,8}` | Cloud Detection V2, multi-threaded |

Total: 15 binaries (3 workloads x 5 variants).

### V2 Compilation

```bash
cd /data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/new_spacecraft_workloads_fork_join

# Compile all 3 V2 workloads (ST + MT for 1,2,4,8 threads)
bash compile_v2_workloads.sh
```

**Compilation script:** `new_spacecraft_workloads_fork_join/compile_v2_workloads.sh`

Manual compilation example:

```bash
CC=riscv64-unknown-linux-gnu-gcc
BASE_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/new_spacecraft_workloads_fork_join"
BIN_DIR="$BASE_DIR/binaries_new2"
CFLAGS="-O3 -static -march=rv64gc -I${BASE_DIR} -I.."
LDFLAGS="-lm -lpthread"

# Docking V2 ST
$CC $CFLAGS -o $BIN_DIR/docking_v2_st \
    spacecraft-docking-workload/spacecraft-docking-v2_hw_all.c $LDFLAGS

# Docking V2 MT 4 threads
$CC $CFLAGS -DNUM_THREADS=4 -o $BIN_DIR/docking_v2_t4 \
    spacecraft-docking-workload/spacecraft-docking-v2_hw_all_mt.c $LDFLAGS
```

### V2 Simulation Script

```bash
cd /data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse

# Launch all V2 simulations (~114 total, runs up to 48 in parallel)
nohup bash run_v2_workloads.sh > simulation_results_v2/v2_run_output.log 2>&1 &
```

**Launch script:** `run_v2_workloads.sh`

V2 simulation phases:

| Phase | Description | Sims |
|-------|-------------|------|
| 1 | ST baselines (1 core, 1T+1M) | 3 |
| 2 | Core scalability (1,2,4,8 cores, 1T+1M) | 12 |
| 3 | 4-core accelerator mix DSE (trig={1..4} x mat={1..4}, skip 1,1) | 45 |
| 4 | 8-core accelerator mix DSE (trig={1..4} x mat={1..4}, skip 1,1) | 45 |
| 5 | 2-core accelerator mix (trig={1,2} x mat={1,2}, skip 1,1) | 9 |
| **Total** | | **114** |

### V2 Simulation Results

Results are stored in a separate directory from the original 21 workloads:

```
simulation_results_v2/
├── {workload}_v2_st_c1_t1_m1/          # Phase 1: ST baseline
├── {workload}_v2_scale_c{1,2,4,8}_t1_m1/  # Phase 2: Core scaling
├── {workload}_v2_4c_t{T}_m{M}/        # Phase 3: 4-core DSE
├── {workload}_v2_8c_t{T}_m{M}/        # Phase 4: 8-core DSE
├── {workload}_v2_2c_t{T}_m{M}/        # Phase 5: 2-core DSE
├── v2_launch.log                        # Launch log
└── logs/
    └── {same_name}.log                  # Simulation log + NOVA contention stats
```

Where `{workload}` is one of: `docking_v2`, `onorbit_servicing_v2`, `cloud_detection_v2`.

### V2 Monitoring

```bash
# Monitor V2 simulation progress
bash /data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/monitor_v2.sh
```

**Monitoring script:** `monitor_v2.sh`

Quick status check:

```bash
OUTPUT_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-multicore21-spacecraft-dse/simulation_results_v2"

echo "Completed: $(find $OUTPUT_DIR -name stats.txt | wc -l) / 114"
echo "Running: $(pgrep -c -f gem5.opt 2>/dev/null || echo 0)"
```

---

## Quick Reference Card

```bash
# === COMPILE ===
cd new_spacecraft_workloads_fork_join && bash compile_all_workloads_fork_join.sh

# === BUILD GEM5 ===
scons build/RISCV/gem5.opt -j$(nproc)

# === RUN ALL SIMS ===
nohup bash run_dse_simulations_forkjoin.sh > /tmp/dse_launch.log 2>&1 &

# === MONITOR ===
watch -n 60 'echo "Done: $(find simulation_results_forkjoin -name stats.txt | wc -l)/798 | Running: $(pgrep -c -f gem5.opt 2>/dev/null)"'

# === CHECK FAILURES ===
for log in simulation_results_forkjoin/logs/*.log; do grep -q "failed\|fault\|panic" "$log" && echo "FAIL: $(basename $log)"; done

# === KILL ALL ===
pkill -f run_dse_simulations; sleep 2; pkill -9 -f gem5.opt

# === CONTENTION CHECK ===
grep "TrigAccel\|MatAccel\|Total stall" simulation_results_forkjoin/logs/adcs_scale_c4_t1_m1.log
```
