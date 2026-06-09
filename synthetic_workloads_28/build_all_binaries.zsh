#!/bin/zsh

# ═══════════════════════════════════════════════════════════════════════════════
# Build script for all 28 spacecraft benchmark workloads — SYSTOLIC ARRAY variant
# Builds: SW (RISC-V + ARM + ARM-SIMD), HW/HW_TRIG/HW_MAT_SYSTOLIC (RISC-V)
# HW_MAT uses SYS_MMACD (systolic array, C += A × B) instead of MMACDF
#
# Usage: zsh build_all_binaries.zsh
# ═══════════════════════════════════════════════════════════════════════════════

set -e

# Source zsh configuration
if [ -f ~/.zshrc ]; then
    source ~/.zshrc
fi

# ─── Directories ───────────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKLOAD_DIR="$SCRIPT_DIR"

# ─── Toolchains ────────────────────────────────────────────────────────────────
# Override via environment if needed:
#   RISCV_GCC=/path/to/riscv64-unknown-elf-gcc
#   ARM_GCC=/path/to/arm-linux-gnueabihf-gcc
#
# HW variants need the NOVA-patched toolchain (fsin.d, sys.mmacd, mlds, …).
# A generic /usr/local riscv64-unknown-elf-gcc will compile SW only and fail HW.

toolchain_supports_nova_isa() {
    local gcc="$1"
    [[ -x "$gcc" ]] || return 1
    local bindir="${gcc:h}"
    local tmp
    tmp="$(mktemp /tmp/nova_isa_test.XXXXXX.c)"
    print 'void f(void){ __asm__ volatile("fsin.d fa0,fa0"); }' > "$tmp"
    PATH="${bindir}:${PATH}" "$gcc" -x c "$tmp" -c -o /dev/null 2>/dev/null
    local rc=$?
    rm -f "$tmp"
    return $rc
}

find_riscv_gcc() {
    local -a candidates
    if [[ -n "${RISCV_GCC:-}" ]]; then
        candidates+=("$RISCV_GCC")
    fi
    candidates+=(
        /data1/home/chandraboul/development/riscv-spike/bin/riscv64-unknown-elf-gcc
        /data/home/chandraboul/development/riscv-spike/bin/riscv64-unknown-elf-gcc
        "$HOME/development/riscv-spike/bin/riscv64-unknown-elf-gcc"
    )
    if command -v riscv64-unknown-elf-gcc >/dev/null 2>&1; then
        candidates+=("$(command -v riscv64-unknown-elf-gcc)")
    fi
    local cand seen=""
    for cand in "${candidates[@]}"; do
        [[ -n "$cand" && -x "$cand" ]] || continue
        if [[ " $seen " == *" $cand "* ]]; then
            continue
        fi
        seen="$seen $cand"
        if toolchain_supports_nova_isa "$cand"; then
            print -r -- "$cand"
            return 0
        fi
    done
    return 1
}

find_arm_gcc() {
    local -a candidates
    if [[ -n "${ARM_GCC:-}" ]]; then
        candidates+=("$ARM_GCC")
    fi
    local linaro_root="${GCC_LINARO_PATH:-$HOME/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf}"
    candidates+=(
        "${linaro_root}/bin/arm-linux-gnueabihf-gcc"
        "$HOME/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc"
    )
    if command -v arm-linux-gnueabihf-gcc >/dev/null 2>&1; then
        candidates+=("$(command -v arm-linux-gnueabihf-gcc)")
    fi
    local cand seen=""
    for cand in "${candidates[@]}"; do
        [[ -n "$cand" && -x "$cand" ]] || continue
        if [[ " $seen " == *" $cand "* ]]; then
            continue
        fi
        seen="$seen $cand"
        print -r -- "$cand"
        return 0
    done
    return 1
}

RISCV_GCC="$(find_riscv_gcc)" || RISCV_GCC=""
ARM_GCC="$(find_arm_gcc)" || ARM_GCC=""

# Ensure gcc invokes the matching binutils (not a stale /usr/local assembler).
if [[ -n "$RISCV_GCC" ]]; then
    export PATH="${RISCV_GCC:h}:${PATH}"
fi

# Common flags
# gem5 / FPGA / host replay builds need stdio-backed file I/O for image
# and telemetry capture.  Flight builds omit -DFLIGHT_HOST_IO and the
# FLIGHT_F* wrappers in flight_compliance.h fall back to no-ops, so the
# same source compiles cleanly for an embedded RTOS target.
#
# -ffile-prefix-map rewrites absolute build-host paths (which would
# otherwise appear in __FILE__ expansions inside assert() and similar
# macros) so that the produced ELFs do not leak the build user's home.
HOST_IO_FLAG="-DFLIGHT_HOST_IO"
# -ffile-prefix-map is GCC 8+. RISC-V toolchain is typically newer; Linaro ARM 7.5 rejects it.
PREFIX_MAP_FLAGS="-ffile-prefix-map=${HOME}=. -ffile-prefix-map=$(pwd)=. -no-canonical-prefixes"
RISCV_FLAGS="-O3 -march=rv64gc -mabi=lp64d -fno-common -static ${HOST_IO_FLAG} ${PREFIX_MAP_FLAGS}"
ARM_PREFIX_MAP_FLAGS="${PREFIX_MAP_FLAGS}"
if ! "$ARM_GCC" -Werror -ffile-prefix-map=x=y -c -x c /dev/null -o /dev/null 2>/dev/null; then
    ARM_PREFIX_MAP_FLAGS="-no-canonical-prefixes"
fi
ARM_FLAGS="-static -O3 -mcpu=cortex-a9 -marm ${HOST_IO_FLAG} ${ARM_PREFIX_MAP_FLAGS}"
ARM_SIMD_FLAGS="-static -O3 -mcpu=cortex-a9 -mfpu=neon -mfloat-abi=hard -ftree-vectorize -ffast-math -marm ${HOST_IO_FLAG} ${ARM_PREFIX_MAP_FLAGS}"

# Strip binaries after build to remove debug info and any residual
# toolchain paths embedded by the linker (e.g. crt object paths).
RISCV_STRIP="${RISCV_GCC%/*}/riscv64-unknown-elf-strip"
ARM_STRIP="${ARM_GCC%/*}/arm-linux-gnueabihf-strip"

if [[ ! -x "$RISCV_GCC" ]]; then
    echo "ERROR: NOVA-capable RISC-V compiler not found."
    echo "       HW builds need riscv-spike with custom ISA support (fsin.d, sys.mmacd, …)."
    echo "       Set RISCV_GCC or install under one of:"
    echo "         /data/home/chandraboul/development/riscv-spike/bin/"
    echo "         /data1/home/chandraboul/development/riscv-spike/bin/"
    exit 1
fi
if [[ ! -x "$ARM_GCC" ]]; then
    echo "ERROR: ARM compiler not found."
    echo "       Set ARM_GCC or install Linaro arm-linux-gnueabihf-gcc, e.g.:"
    echo "         $HOME/gcc-linaro-7.5.0-2019.12-x86_64_arm-linux-gnueabihf/bin/"
    exit 1
fi

echo "Using RISC-V toolchain: $RISCV_GCC"
echo "Using ARM toolchain:    $ARM_GCC"
echo ""

# ─── Workload definitions ─────────────────────────────────────────────────────
# Format: DIR_NAME:SW_BASENAME:HW_BASENAME
WORKLOADS=(
    "01_Navigation:spacecraft_navigation:spacecraft_navigation_hw"
    "02_Rendezvous:spacecraft_rendezvous:spacecraft_rendezvous_hw"
    "03_VBN:spacecraft_vbn:spacecraft_vbn_hw"
    "04_SPS:spacecraft_sps:spacecraft_sps_hw"
    "05_TRN:spacecraft_trn:spacecraft_trn_hw"
    "06_STAR:spacecraft_star:spacecraft_star_hw"
    "07_RoverSLAM:spacecraft_rover_slam:spacecraft_rover_slam_hw"
    "08_OdnP:spacecraft_odnp:spacecraft_odnp_hw"
    "09_CloudDet:spacecraft_cloud_detection:spacecraft_cloud_detection_hw"
    "10_SNS:spacecraft_sns:spacecraft_sns_hw"
    "11_SciInstrument:spacecraft_sci_instrument:spacecraft_sci_instrument_hw"
    "12_SpaceWeather:spacecraft_space_weather:spacecraft_space_weather_hw"
    "13_SAR:spacecraft_sar:spacecraft_sar_hw"
    "14_HyperSpectral:spacecraft_hyperspectral:spacecraft_hyperspectral_hw"
    "15_SenFusion:spacecraft_sensor_fusion:spacecraft_sensor_fusion_hw"
    "16_ADCS:spacecraft_adcs:spacecraft_adcs_hw"
    "17_Guidance:spacecraft_guidance:spacecraft_guidance_hw"
    "18_OptLanding:spacecraft_opt_landing:spacecraft_opt_landing_hw"
    "19_FormationFLY:spacecraft_formation_fly:spacecraft_formation_fly_hw"
    "20_RoboticARM:spacecraft_robotic_arm:spacecraft_robotic_arm_hw"
    "21_RoverPathPlan:spacecraft_rover_path_plan:spacecraft_rover_path_plan_hw"
    "22_MissionMgmt:spacecraft_mission_mgmt:spacecraft_mission_mgmt_hw"
    "23_SatComm:spacecraft_sat_comm:spacecraft_sat_comm_hw"
    "24_CCSDS122:spacecraft_ccsds122:spacecraft_ccsds122_hw"
    "25_ISL:spacecraft_isl:spacecraft_isl_hw"
    "26_AnomalyDet:spacecraft_anomaly_det:spacecraft_anomaly_det_hw"
    "27_FDIR:spacecraft_fdir:spacecraft_fdir_hw"
    "28_AI:spacecraft_ai:spacecraft_ai_hw"
)

# ─── Counters ──────────────────────────────────────────────────────────────────
PASS=0
FAIL=0
TOTAL=0

build_one() {
    local compiler=$1
    local flags=$2
    local src=$3
    local out=$4
    local label=$5
    local include_dir=$6

    TOTAL=$((TOTAL+1))
    local inc_flag=""
    [[ -n "$include_dir" ]] && inc_flag="-I${include_dir}"

    if $compiler $=flags $inc_flag "$src" -o "$out" -lm 2>/dev/null; then
        echo "  PASS  $label -> $(basename $out)"
        PASS=$((PASS+1))
        strip_binary "$compiler" "$out"
    else
        echo "  FAIL  $label -> $(basename $out)"
        $compiler $=flags $inc_flag "$src" -o "$out" -lm 2>&1 | head -5
        FAIL=$((FAIL+1))
    fi
}

build_multifile() {
    local compiler=$1
    local flags=$2
    local srcs=$3
    local out=$4
    local label=$5
    local include_dir=$6

    TOTAL=$((TOTAL+1))
    local inc_flag=""
    [[ -n "$include_dir" ]] && inc_flag="-I${include_dir}"

    if $compiler $=flags $inc_flag $=srcs -o "$out" -lm 2>/dev/null; then
        echo "  PASS  $label -> $(basename $out)"
        PASS=$((PASS+1))
        strip_binary "$compiler" "$out"
    else
        echo "  FAIL  $label -> $(basename $out)"
        $compiler $=flags $inc_flag $=srcs -o "$out" -lm 2>&1 | head -5
        FAIL=$((FAIL+1))
    fi
}

# Pick the right strip and post-process the binary so the produced
# ELF carries no debug info and no residual build-host paths.
strip_binary() {
    local compiler=$1
    local elf=$2
    local strip_tool=""
    case "$compiler" in
        *riscv64-unknown-elf-gcc*) strip_tool="$RISCV_STRIP" ;;
        *arm-linux-gnueabihf-gcc*) strip_tool="$ARM_STRIP" ;;
    esac
    if [[ -n "$strip_tool" && -x "$strip_tool" && -f "$elf" ]]; then
        "$strip_tool" --strip-all "$elf" 2>/dev/null || true
    fi
}

echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║  Building 28 Spacecraft Benchmarks (SW+HW/TRIG/MAT_SYSTOLIC) RV+ARM║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo ""

for entry in "${WORKLOADS[@]}"; do
    IFS=':' read -r dir_name sw_base hw_base <<< "$entry"

    dir="${WORKLOAD_DIR}/${dir_name}"
    sw_src="${dir}/${sw_base}.c"
    hw_src="${dir}/${hw_base}.c"

    echo "── ${dir_name} ──────────────────────────────────────────"

    # 1) SW RISC-V
    #    28_AI is multi-file: compile all .c sources together
    if [[ "$dir_name" == "28_AI" ]]; then
        ai_srcs="${dir}/spacecraft_ai_main.c ${dir}/spacecraft_ai_math.c ${dir}/spacecraft_ai_weights.c ${dir}/spacecraft_ai_workloads.c"
        build_multifile "$RISCV_GCC" "$RISCV_FLAGS" "$ai_srcs" "${dir}/${sw_base}.riscv" "SW RISC-V (multi-file)" "$dir"
    elif [[ -f "$sw_src" ]]; then
        build_one "$RISCV_GCC" "$RISCV_FLAGS" "$sw_src" "${dir}/${sw_base}.riscv" "SW RISC-V" "$dir"
    else
        echo "  SKIP  SW source not found: $sw_src"
    fi

    # 2) HW RISC-V (trig + matrix)
    if [[ -f "$hw_src" ]]; then
        build_one "$RISCV_GCC" "$RISCV_FLAGS" "$hw_src" "${dir}/${hw_base}.riscv" "HW RISC-V" "$dir"
    else
        echo "  SKIP  HW source not found: $hw_src"
    fi

    # 3) HW_TRIG RISC-V (trig-only CORDIC)
    hw_trig_src="${dir}/${sw_base}_hw_trig.c"
    if [[ -f "$hw_trig_src" ]]; then
        build_one "$RISCV_GCC" "$RISCV_FLAGS" "$hw_trig_src" "${dir}/${sw_base}_hw_trig.riscv" "HW_TRIG RISC-V" "$dir"
    else
        echo "  SKIP  HW_TRIG source not found: $hw_trig_src"
    fi

    # 4) HW_MAT_SYSTOLIC RISC-V (matrix-only AME — SYS_MMACD systolic array)
    hw_mat_non_systolic_src="${dir}/${sw_base}_hw_mat.c"
    if [[ -f "$hw_mat_non_systolic_src" ]]; then
        build_one "$RISCV_GCC" "$RISCV_FLAGS" "$hw_mat_non_systolic_src" "${dir}/${sw_base}_hw_mat.riscv" "HW_MAT RISC-V" "$dir"
    else
        echo "  SKIP  HW_MAT source not found: $hw_mat_non_systolic_src"
    fi

    # 5) HW_MAT_SYSTOLIC RISC-V (matrix-only AME — SYS_MMACD systolic array)
    hw_mat_src="${dir}/${sw_base}_hw_mat_systolic.c"
    if [[ -f "$hw_mat_src" ]]; then
        build_one "$RISCV_GCC" "$RISCV_FLAGS" "$hw_mat_src" "${dir}/${sw_base}_hw_mat_systolic.riscv" "HW_MAT_SYSTOLIC RISC-V" "$dir"
    else
        echo "  SKIP  HW_MAT_SYSTOLIC source not found: $hw_mat_src"
    fi

    # 6) SW ARM
    #    28_AI is multi-file: compile all .c sources together
    if [[ "$dir_name" == "28_AI" ]]; then
        ai_srcs="${dir}/spacecraft_ai_main.c ${dir}/spacecraft_ai_math.c ${dir}/spacecraft_ai_weights.c ${dir}/spacecraft_ai_workloads.c"
        build_multifile "$ARM_GCC" "$ARM_FLAGS" "$ai_srcs" "${dir}/${sw_base}.arm" "SW ARM (multi-file)" "$dir"
    elif [[ -f "$sw_src" ]]; then
        build_one "$ARM_GCC" "$ARM_FLAGS" "$sw_src" "${dir}/${sw_base}.arm" "SW ARM" "$dir"
    fi

    # 7) SW ARM SIMD (NEON)
    if [[ "$dir_name" == "28_AI" ]]; then
        ai_srcs="${dir}/spacecraft_ai_main.c ${dir}/spacecraft_ai_math.c ${dir}/spacecraft_ai_weights.c ${dir}/spacecraft_ai_workloads.c"
        build_multifile "$ARM_GCC" "$ARM_SIMD_FLAGS" "$ai_srcs" "${dir}/${sw_base}.armsimd" "SW ARM-SIMD (multi-file)" "$dir"
    elif [[ -f "$sw_src" ]]; then
        build_one "$ARM_GCC" "$ARM_SIMD_FLAGS" "$sw_src" "${dir}/${sw_base}.armsimd" "SW ARM-SIMD" "$dir"
    fi

    echo ""
done

echo "════════════════════════════════════════════════════════════════════════"
echo "  Build Summary:  PASS=$PASS  FAIL=$FAIL  TOTAL=$TOTAL"
echo "════════════════════════════════════════════════════════════════════════"
