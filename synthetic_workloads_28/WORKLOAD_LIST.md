# 28 Spacecraft Workloads — IEEE TC Paper TC-2025-09-0830

Synthetic benchmark suite derived from production spacecraft onboard flight software characteristics.
Algorithmic structure, BCE distributions, matrix dimensions, and memory-access behaviour are preserved.
Original ISRO flight software and mission datasets are **not** included.

## Per-Workload Variants (5 source files each)

| Variant | Source suffix | Description |
|---------|---------------|-------------|
| SW baseline | `spacecraft_<name>.c` | Software-only (no custom ISA) |
| HW (Trig+Mat) | `spacecraft_<name>_hw.c` | CORDIC + matrix accelerators |
| HW Trig-only | `spacecraft_<name>_hw_trig.c` | CORDIC trigonometric accelerator |
| HW Mat-only | `spacecraft_<name>_hw_mat.c` | Optimised matrix accelerator |
| HW Mat Systolic | `spacecraft_<name>_hw_mat_systolic.c` | 3×3 systolic array (SYS_MMACD) |

## Complete Workload List (28)

| # | Directory | Workload | Category | Dominant BCEs |
|---|-----------|----------|----------|---------------|
| 01 | `01_Navigation` | Navigation | Nav & Positioning | Matrix(26%), KalF(16%), Trig(16%) |
| 02 | `02_Rendezvous` | Rendezvous | Nav & Positioning | Matrix(38%), Vector(14%), OrbMech(12%) |
| 03 | `03_VBN` | VBN | Nav & Positioning | Matrix(28%), ImgProc(18%), Vector(12%) |
| 04 | `04_SPS` | SPS | Nav & Positioning | Trig(24%), KalF(23%), OrbMech(15%) |
| 05 | `05_TRN` | TRN | Nav & Positioning | ImgProc(34%), Matrix(27%), Trig(17%) |
| 06 | `06_STAR` | STAR | Nav & Positioning | Matrix(29%), Trig(18%), KalF(12%) |
| 07 | `07_RoverSLAM` | RoverSLAM | Nav & Positioning | Matrix(18%), ImgProc(17%), KalF(17%) |
| 08 | `08_OdnP` | OdnP | Nav & Positioning | Matrix(38%), Trig(22%), KalF(11%) |
| 09 | `09_CloudDet` | CloudDet | Remote Sensing | ImgProc(31%), Matrix(23%), Spectral(21%) |
| 10 | `10_SNS` | SNS | Remote Sensing | Trig(31%), StatePred(13%), Matrix(13%) |
| 11 | `11_SciInstrument` | SciInstrument | Remote Sensing | Matrix(41%), FFT(16%), Vector(13%) |
| 12 | `12_SpaceWeather` | SpaceWeather | Remote Sensing | Matrix(39%), FFT(16%), Trig(14%) |
| 13 | `13_SAR` | SAR | Remote Sensing | Trig(25%), Matrix(22%), FFT(14%) |
| 14 | `14_HyperSpectral` | HyperSpectral | Remote Sensing | Wavelet(34%), FFT(21%), Matrix(17%) |
| 15 | `15_SenFusion` | SenFusion | Remote Sensing | Matrix(32%), KalF(20%), StatePred(14%) |
| 16 | `16_ADCS` | ADCS | Control & Guidance | Trig(36%), Matrix(27%), Quat(11%) |
| 17 | `17_Guidance` | Guidance | Control & Guidance | Matrix(33%), NumInteg(20%), StatePred(15%) |
| 18 | `18_OptLanding` | OptLanding | Control & Guidance | Matrix(28%), TrajecGen(15%), Opt(13%) |
| 19 | `19_FormationFLY` | FormationFLY | Control & Guidance | Matrix(25%), Trig(16%), OrbMech(12%) |
| 20 | `20_RoboticARM` | RoboticARM | Control & Guidance | Matrix(29%), Trig(16%), NumInteg(12%) |
| 21 | `21_RoverPathPlan` | RoverPathPlan | Control & Guidance | ImgProc(22%), TerrainEval(16%), Vector(15%) |
| 22 | `22_MissionMgmt` | MissionMgmt | Mission Mgmt & Comm | Matrix(38%), FFT(16%), Trig(14%) |
| 23 | `23_SatComm` | SatComm | Mission Mgmt & Comm | Matrix(33%), FFT(14%), Vector(14%) |
| 24 | `24_CCSDS122` | CCSDS122 | Mission Mgmt & Comm | Vector(22%), FFT(21%), OrbMech(18%) |
| 25 | `25_ISL` | ISL | Mission Mgmt & Comm | Matrix(36%), FFT(14%), OrbMech(12%) |
| 26 | `26_AnomalyDet` | AnomalyDet | Autonomous Ops | NN(34%), BLAS(30%), Matrix(16%) |
| 27 | `27_FDIR` | FDIR | Autonomous Ops | NN(36%), BLAS(29%), Matrix(17%) |
| 28 | `28_AI` | AI | Autonomous Ops | NN(33%), BLAS(21%), Matrix(16%) |

## Simulation Parameters (per workload)

| # | Workload | Loop Constant | Steps | Max Matrix |
|---|----------|---------------|-------|------------|
| 1 | Navigation | SIM_TIMESTEPS | 1,000 | 15×15 |
| 2 | Rendezvous | MAX_TIMESTEPS | 4,000 | 9×9 |
| 3 | VBN | Hardcoded loop | 5 | 18×18 |
| 4 | SPS | steps | 36,000 | 15×15 |
| 5 | TRN | MAX_TIMESTEPS | 200 | 9×9 |
| 6 | STAR | NUM_TIMESTEPS | 100 | 18×18 |
| 7 | RoverSLAM | num_steps | 500 | dyn ≤2003 |
| 8 | OdnP | NUM_TIMESTEPS | 200 | 16×16 |
| 9 | CloudDet | NUM_TIMESTEPS | 25 | 9×9 |
| 10 | SNS | iterations | 10 | 9×9 |
| 11 | SciInstrument | Hardcoded | 120 | 18×18 |
| 12 | SpaceWeather | NUM_TIMESTEPS | 50 | 18×18 |
| 13 | SAR | Single-pass | — | 12×12 |
| 14 | HyperSpectral | Single-pass | — | 24×24 |
| 15 | SenFusion | NUM_MONTE_CARLO | 100 | 18×18 |
| 16 | ADCS | num_timesteps | 100 | 12×12 |
| 17 | Guidance | TIMESTEPS | 50 | 15×15 |
| 18 | OptLanding | SIMULATION_STEPS | 5,000 | 15×15 |
| 19 | FormationFLY | timesteps | 100 | 18×18 |
| 20 | RoboticARM | duration/TIME_STEP | 200 | 18×18 |
| 21 | RoverPathPlan | max_steps | 1,000 | 18×18 |
| 22 | MissionMgmt | NUM_TIMESTEPS | 50 | 18×18 |
| 23 | SatComm | TIMESTEPS | 100 | 16×16 |
| 24 | CCSDS122 | Single-pass | — | DWT block=8 |
| 25 | ISL | NUM_TIMESTEPS | 50 | 18×18 |
| 26 | AnomalyDet | Hardcoded | 100 | 9×9 |
| 27 | FDIR | num_steps | 1,000 | 15×15 |
| 28 | AI | Hardcoded | 10 | 15×15 |

## Scripts in This Package

| Script | Purpose |
|--------|---------|
| `build_all_binaries.zsh` | Cross-compile all variants (requires NOVA toolchain + workload sources) |
| `run_gem5_simulations.zsh` | Run gem5 simulations for all built binaries |
| `verify_gem5_mv9_paper.sh` | Paper verification (28×4 RISC-V variants = 112 runs) |
| `verify_gem5_mv9.sh` | Full verification (all 145 binaries) |

## Build Flags

```
-O3 -march=rv64gc -mabi=lp64d -fno-common -static
```

Toolchain: `riscv64-unknown-elf-gcc` (GCC 13.2.0, branch `nova-custom-isa`)
