#!/bin/zsh

# ═══════════════════════════════════════════════════════════════════════════════
# gem5 Simulation Runner for 28 Spacecraft Benchmarks — SYSTOLIC ARRAY variant
#
# Runs SW-RISC-V, HW-RISC-V, HW_TRIG-RISC-V, HW_MAT_SYSTOLIC-RISC-V, SW-ARM,
# and SW-ARM-SIMD simulations
# HW_MAT_SYSTOLIC uses SYS_MMACD (systolic array, C += A × B) instead of MMACDF
# - Up to 48 simulations in parallel
# - Memory guard: kills heaviest gem5 process when system memory >= 90%
#
# RISC-V gem5: gem5-nova-fix1 (NOVA tree)
# ARM    gem5: gem5-arm
#
# Usage: zsh run_spacecraft_simulations_systolic_matmul.zsh
# ═══════════════════════════════════════════════════════════════════════════════

if [[ -o interactive ]]; then
    set -o monitor  # enable job control for interactive shells only
fi

# Source zsh configuration
if [ -f ~/.zshrc ]; then
    source ~/.zshrc
fi

# ─── gem5 Directories ─────────────────────────────────────────────────────────
# For portability, override these via environment variables:
#   GEM5_RISCV_DIR=/path/to/gem5-nova-fix1
#   GEM5_ARM_DIR=/path/to/gem5-arm
GEM5_RISCV_DIR="${GEM5_RISCV_DIR:-$(cd "$(dirname "$0")/../gem5_rvtrig_rvmatrix" && pwd)}"
GEM5_ARM_DIR="${GEM5_ARM_DIR:-/data1/home/chandraboul/development/gem5-arm}"

GEM5_RISCV_BIN="${GEM5_RISCV_DIR}/build/RISCV/gem5.opt"
GEM5_ARM_BIN="${GEM5_ARM_DIR}/build/ARM/gem5.opt"

CONFIG_RISCV="${GEM5_RISCV_DIR}/configs/learning_gem5/part1/riscv_modified_configs.py"
CONFIG_ARM="${GEM5_ARM_DIR}/configs/learning_gem5/part1/arm_modified_configs.py"

# ─── Workload / Stats ─────────────────────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKLOAD_DIR="$SCRIPT_DIR"
# One directory for all outputs: set GEM5_RESULTS_DIR to reuse/repair an existing tree (non-empty stats.txt skipped).
if [[ -n "${GEM5_RESULTS_DIR:-}" ]]; then
    mkdir -p "${GEM5_RESULTS_DIR}"
    STATS_DIR="$(cd "${GEM5_RESULTS_DIR}" && pwd)"
else
    STATS_DIR="${SCRIPT_DIR}/gem5_results_$(date +'%Y-%m-%d_%H-%M-%S')"
fi

# ─── Parallelism & Memory Guard ───────────────────────────────────────────────
MAX_PARALLEL="${MAX_PARALLEL:-128}"
MEM_THRESHOLD=90          # percent — start killing when usage >= this
MEM_CHECK_INTERVAL=15     # seconds between memory checks

# ─── gem5 Simulation Parameters ───────────────────────────────────────────────
SIM_PARAMS=(
    "--l1i_size=8kB"
    "--l1d_size=16kB"
    "--l2_size=64kB"
    "--system_mem_range=64GB"
    "--clock_freq=500MHz"
    "--branch_pred=BiModeBP"
    "--prefetcher=None"
    "--cpu_type=TimingSimpleCPU"
    "--l1i_assoc=2"
    "--l1d_assoc=2"
    "--l1i_tag_latency=2"
    "--l1d_tag_latency=4"
    "--l1i_data_latency=2"
    "--l1d_data_latency=4"
    "--l1i_response_latency=2"
    "--l1d_response_latency=4"
    "--l1i_mshrs=2"
    "--l1d_mshrs=2"
    "--l1i_tgts_per_mshr=8"
    "--l1d_tgts_per_mshr=8"
)
SIM_PARAMS_STR="${SIM_PARAMS[*]}"

# ─── Workload Table ───────────────────────────────────────────────────────────
# DIR_NAME:SW_BASENAME:HW_BASENAME
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

# ─── Create directories ───────────────────────────────────────────────────────
mkdir -p "${STATS_DIR}/logs"

# ─── Tracking arrays ──────────────────────────────────────────────────────────
typeset -A PID_TO_NAME       # PID -> human-readable name
typeset -a ALL_PIDS          # all launched PIDs (for memory guard name lookup)
LAUNCHED=0
COMPLETED=0
KILLED=0
ERRORS=0

# ─── Slot-file semaphore ──────────────────────────────────────────────────────
# Each running job owns one file in SLOT_DIR.  wait_for_slot counts the files.
# An EXIT trap in every subshell removes the file immediately on completion or
# kill, so slots open the instant a simulation finishes — not in batches.
SLOT_DIR=$(mktemp -d "${TMPDIR:-/tmp}/gem5_slots_XXXXXX")

# ═══════════════════════════════════════════════════════════════════════════════
#  Helper functions
# ═══════════════════════════════════════════════════════════════════════════════

# Percent of RAM that is NOT readily available: (MemTotal−MemAvailable)/MemTotal×100
# Uses /proc/meminfo (kB). Safer than parsing `free` across procps versions.
# Set GEM5_MEM_GUARD=0 to disable SIGKILL of heavy gem5.opt (for debugging only).
get_mem_usage_pct() {
    awk '/^MemTotal:/ {t=$2} /^MemAvailable:/ {a=$2} END {
        if (t > 0 && a != "") printf "%d", int((t - a) * 100 / t + 0.5)
        else print "0"
    }' /proc/meminfo
}

kill_heaviest_gem5() {
    # Find the gem5.opt process using the most RSS and kill it
    local heaviest_pid heaviest_rss heaviest_name

    heaviest_pid=$(ps -eo pid,rss,comm --no-headers \
        | awk '$3 == "gem5.opt" { print $1, $2 }' \
        | sort -k2 -n -r \
        | head -1 \
        | awk '{print $1}')

    if [[ -n "$heaviest_pid" ]]; then
        heaviest_rss=$(ps -o rss= -p "$heaviest_pid" 2>/dev/null)
        heaviest_name="${PID_TO_NAME[$heaviest_pid]:-unknown}"
        local rss_gb=$(( heaviest_rss / 1048576.0 ))
        echo "[MEM-GUARD] $(date '+%H:%M:%S') Killing PID $heaviest_pid ($heaviest_name) — RSS ${rss_gb} GB"
        kill -9 "$heaviest_pid" 2>/dev/null
        KILLED=$((KILLED+1))
        echo "[MEM-GUARD] Killed $heaviest_name (PID $heaviest_pid, RSS ~${rss_gb} GB)" >> "${STATS_DIR}/logs/memory_guard.log"
        return 0
    fi
    return 1
}

wait_for_slot() {
    # Block until fewer than MAX_PARALLEL simulations are running AND memory
    # is below threshold.  Uses SLOT_DIR file count so slots open the instant
    # any simulation finishes (EXIT trap removes the file), not in batches.
    while true; do
        # Count slot files owned by currently running jobs
        local running
        running=$(ls -1 "$SLOT_DIR" 2>/dev/null | wc -l)

        # Memory guard
        local mem_pct
        mem_pct=$(get_mem_usage_pct)
        if [[ "${GEM5_MEM_GUARD:-1}" != "0" ]] && (( mem_pct >= MEM_THRESHOLD )); then
            echo "[MEM-GUARD] $(date '+%H:%M:%S') Memory at ${mem_pct}% (>= ${MEM_THRESHOLD}%). Killing heaviest gem5..."
            kill_heaviest_gem5
            sleep 5
            continue
        fi

        if (( running < MAX_PARALLEL )); then
            break
        fi

        sleep "$MEM_CHECK_INTERVAL"
    done
}

launch_sim() {
    local gem5_dir=$1
    local gem5_bin=$2
    local config=$3
    local binary=$4
    local outdir=$5
    local label=$6

    local log_file="${STATS_DIR}/logs/${label}.log"

    # Skip if binary doesn't exist
    if [[ ! -f "$binary" ]]; then
        echo "  SKIP  $label — binary not found: $binary"
        return
    fi

    # Skip if simulation already completed (non-empty stats.txt)
    if [[ -s "${outdir}/stats.txt" ]]; then
        echo "  DONE  $label — already completed (stats.txt exists)"
        COMPLETED=$((COMPLETED+1))
        return
    fi

    wait_for_slot

    mkdir -p "$outdir"

    # Claim a slot before forking so the count is accurate immediately
    local slot_file="${SLOT_DIR}/${label}"
    touch "$slot_file"

    (
        # Release slot the instant this subshell exits for any reason
        # (normal completion, error, or kill by memory guard)
        trap "rm -f '${slot_file}'" EXIT

        cd "$gem5_dir"
        "$gem5_bin" --outdir="$outdir" "$config" "$binary" \
            $=SIM_PARAMS_STR \
            > "$log_file" 2>&1

        local rc=$?
        if [[ $rc -eq 0 ]]; then
            echo "  OK    $label completed"
        else
            echo "  ERR   $label failed (rc=$rc) — see $log_file"
        fi
    ) &

    local pid=$!
    ALL_PIDS+=($pid)
    PID_TO_NAME[$pid]="$label"
    LAUNCHED=$((LAUNCHED+1))
    echo "  RUN   $label  (PID $pid)  [$LAUNCHED launched]"
}

# ═══════════════════════════════════════════════════════════════════════════════
#  Memory guard background watchdog
# ═══════════════════════════════════════════════════════════════════════════════

memory_watchdog() {
    while true; do
        sleep "$MEM_CHECK_INTERVAL"
        local mem_pct
        mem_pct=$(get_mem_usage_pct)
        if [[ "${GEM5_MEM_GUARD:-1}" != "0" ]] && (( mem_pct >= MEM_THRESHOLD )); then
            echo "[MEM-GUARD] $(date '+%H:%M:%S') Memory at ${mem_pct}% — killing heaviest gem5"
            kill_heaviest_gem5
            sleep 5
        fi
    done
}

# Start the watchdog in the background
memory_watchdog &
WATCHDOG_PID=$!
trap "kill $WATCHDOG_PID 2>/dev/null; wait $WATCHDOG_PID 2>/dev/null; rm -rf '$SLOT_DIR'" EXIT

# ═══════════════════════════════════════════════════════════════════════════════
#  Main — Launch all simulations
# ═══════════════════════════════════════════════════════════════════════════════

echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║  gem5 Simulations — 28 Spacecraft Benchmarks (SW+HW/TRIG/MAT_SYS) ║"
echo "║  Max parallel: ${MAX_PARALLEL}    Memory threshold: ${MEM_THRESHOLD}%                     ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo ""
echo "RISC-V gem5: $GEM5_RISCV_BIN"
echo "ARM    gem5: $GEM5_ARM_BIN"
echo "Stats  dir : $STATS_DIR"
echo ""

for entry in "${WORKLOADS[@]}"; do
    IFS=':' read -r dir_name sw_base hw_base <<< "$entry"

    dir="${WORKLOAD_DIR}/${dir_name}"

    echo "── ${dir_name} ──────────────────────────────────────────"

    # 1) SW RISC-V
    launch_sim "$GEM5_RISCV_DIR" "$GEM5_RISCV_BIN" "$CONFIG_RISCV" \
        "${dir}/${sw_base}.riscv" \
        "${STATS_DIR}/${dir_name}_sw_riscv" \
        "${dir_name}_sw_riscv"

    # 2) HW RISC-V (trig + matrix)
    launch_sim "$GEM5_RISCV_DIR" "$GEM5_RISCV_BIN" "$CONFIG_RISCV" \
        "${dir}/${hw_base}.riscv" \
        "${STATS_DIR}/${dir_name}_hw_riscv" \
        "${dir_name}_hw_riscv"

    # 3) HW_TRIG RISC-V (trig-only CORDIC)
    launch_sim "$GEM5_RISCV_DIR" "$GEM5_RISCV_BIN" "$CONFIG_RISCV" \
        "${dir}/${sw_base}_hw_trig.riscv" \
        "${STATS_DIR}/${dir_name}_hw_trig_riscv" \
        "${dir_name}_hw_trig_riscv"

    # 4) HW_MAT_SYSTOLIC RISC-V (matrix-only AME — SYS_MMACD systolic array)
    launch_sim "$GEM5_RISCV_DIR" "$GEM5_RISCV_BIN" "$CONFIG_RISCV" \
        "${dir}/${sw_base}_hw_mat_systolic.riscv" \
        "${STATS_DIR}/${dir_name}_hw_mat_systolic_riscv" \
        "${dir_name}_hw_mat_systolic_riscv"

    # 5) SW ARM
    launch_sim "$GEM5_ARM_DIR" "$GEM5_ARM_BIN" "$CONFIG_ARM" \
        "${dir}/${sw_base}.arm" \
        "${STATS_DIR}/${dir_name}_sw_arm" \
        "${dir_name}_sw_arm"

    # 6) SW ARM SIMD
    launch_sim "$GEM5_ARM_DIR" "$GEM5_ARM_BIN" "$CONFIG_ARM" \
        "${dir}/${sw_base}.armsimd" \
        "${STATS_DIR}/${dir_name}_sw_armsimd" \
        "${dir_name}_sw_armsimd"

    echo ""
done

# ═══════════════════════════════════════════════════════════════════════════════
#  Wait for all to finish
# ═══════════════════════════════════════════════════════════════════════════════

echo "════════════════════════════════════════════════════════════════════════"
echo "  All $LAUNCHED simulations launched. Waiting for completion..."
echo "════════════════════════════════════════════════════════════════════════"

# Periodically report progress while waiting
while true; do
    still_running=$(ls -1 "$SLOT_DIR" 2>/dev/null | wc -l)

    if (( still_running == 0 )); then
        break
    fi

    mem_pct=$(get_mem_usage_pct)
    echo "[STATUS] $(date '+%H:%M:%S')  Running: $still_running / $LAUNCHED   Memory: ${mem_pct}%"
    sleep 60
done

# Stop the watchdog
kill $WATCHDOG_PID 2>/dev/null
wait $WATCHDOG_PID 2>/dev/null

# ═══════════════════════════════════════════════════════════════════════════════
#  Summary
# ═══════════════════════════════════════════════════════════════════════════════

echo ""
echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║  SIMULATION COMPLETE                                               ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo ""

# Count successes and failures
SUCCESS=0
FAIL_COUNT=0
SKIP_COUNT=0
for entry in "${WORKLOADS[@]}"; do
    IFS=':' read -r dir_name sw_base hw_base <<< "$entry"
    for suffix in sw_riscv hw_riscv hw_trig_riscv hw_mat_systolic_riscv sw_arm sw_armsimd; do
        outdir="${STATS_DIR}/${dir_name}_${suffix}"
        if [[ -s "${outdir}/stats.txt" ]]; then
            SUCCESS=$((SUCCESS+1))
        elif [[ -d "${outdir}" ]]; then
            FAIL_COUNT=$((FAIL_COUNT+1))
        else
            SKIP_COUNT=$((SKIP_COUNT+1))
        fi
    done
done

echo "  Launched  : $LAUNCHED"
echo "  Succeeded : $SUCCESS  (stats.txt present)"
echo "  Failed    : $FAIL_COUNT"
echo "  Skipped   : $SKIP_COUNT"
echo "  Killed    : $KILLED  (by memory guard)"
echo ""
echo "  Results   : ${STATS_DIR}"
echo "  Logs      : ${STATS_DIR}/logs/"

# List failed simulations
if (( FAIL_COUNT > 0 )); then
    echo ""
    echo "  Failed simulations:"
    for entry in "${WORKLOADS[@]}"; do
        IFS=':' read -r dir_name sw_base hw_base <<< "$entry"
        for suffix in sw_riscv hw_riscv hw_trig_riscv hw_mat_systolic_riscv sw_arm sw_armsimd; do
            outdir="${STATS_DIR}/${dir_name}_${suffix}"
            if [[ -d "${outdir}" ]] && [[ ! -s "${outdir}/stats.txt" ]]; then
                echo "    - ${dir_name}_${suffix}"
            fi
        done
    done
fi

if [[ -f "${STATS_DIR}/logs/memory_guard.log" ]]; then
    echo ""
    echo "  Memory guard kills:"
    cat "${STATS_DIR}/logs/memory_guard.log" | sed 's/^/    /'
fi
