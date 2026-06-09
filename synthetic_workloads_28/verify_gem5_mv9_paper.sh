#!/usr/bin/env bash
# Verify the 112 RISC-V binaries used by run_gem5_simulations.zsh
# (28 workloads × SW, HW, HW_TRIG, HW_MAT_SYSTOLIC)
set -uo pipefail

SUB="$(cd "$(dirname "$0")" && pwd)"
GEM5="${GEM5:-$(cd "$(dirname "$0")/../gem5_rvtrig_rvmatrix" && pwd)/build/RISCV/gem5.opt}"
CFG="${CFG:-$(cd "$(dirname "$0")/../gem5_rvtrig_rvmatrix" && pwd)/configs/learning_gem5/part1/riscv_modified_configs.py}"
OUT="${OUT:-$SUB/gem5_mv9_paper_verify_$(date +%Y%m%d_%H%M%S)}"
PARALLEL="${PARALLEL:-6}"
SUMMARY="$OUT/summary.csv"

mkdir -p "$OUT"
echo "tag,variant,exit_code,stats_ok,simSeconds,simInsts,binary" > "$SUMMARY"

SIM_ARGS=(
  --l1i_size=8kB --l1d_size=16kB --l2_size=64kB --system_mem_range=64GB
  --clock_freq=500MHz --branch_pred=BiModeBP --prefetcher=None
  --cpu_type=TimingSimpleCPU --l1i_assoc=2 --l1d_assoc=2
  --l1i_tag_latency=2 --l1d_tag_latency=4 --l1i_data_latency=2 --l1d_data_latency=4
  --l1i_response_latency=2 --l1d_response_latency=4 --l1i_mshrs=2 --l1d_mshrs=2
  --l1i_tgts_per_mshr=8 --l1d_tgts_per_mshr=8
)

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

run_one() {
  local bin="$1" variant="$2" dir_tag="$3"
  local tag="${dir_tag}_${variant}"
  local rdir="$OUT/$tag"
  mkdir -p "$rdir"
  local timeout_s=1800
  case "$dir_tag" in
    03_VBN|07_RoverSLAM|13_SAR|23_SatComm|28_AI) timeout_s=7200 ;;
    05_TRN|15_SenFusion|20_RoboticARM) timeout_s=3600 ;;
  esac
  timeout "$timeout_s" "$GEM5" --outdir="$rdir" "$CFG" "$bin" "${SIM_ARGS[@]}" \
    >"$rdir/stdout.log" 2>"$rdir/stderr.log"
  local ec=$?
  local ok=0 sim_s="" sim_i=""
  if [[ -s "$rdir/stats.txt" ]] && grep -q '^simSeconds' "$rdir/stats.txt"; then
    ok=1
    sim_s=$(awk '/^simSeconds/{print $2; exit}' "$rdir/stats.txt")
    sim_i=$(awk '/^simInsts/{print $2; exit}' "$rdir/stats.txt")
  fi
  {
    flock -x 9
    echo "$tag,$variant,$ec,$ok,$sim_s,$sim_i,$bin" >> "$SUMMARY"
  } 9>>"$OUT/.summary.lock"
  if [[ $ok -eq 1 ]]; then
    echo "PASS $tag simSeconds=$sim_s"
  else
    echo "FAIL $tag exit=$ec"
    tail -5 "$rdir/stderr.log" 2>/dev/null || true
  fi
}

export -f run_one
export SUB GEM5 CFG OUT SUMMARY SIM_ARGS

BINS=()
for entry in "${WORKLOADS[@]}"; do
  IFS=':' read -r dir_name sw_base hw_base <<< "$entry"
  dir="$SUB/$dir_name"
  BINS+=("$dir/${sw_base}.riscv:sw_riscv:$dir_name")
  BINS+=("$dir/${hw_base}.riscv:hw_riscv:$dir_name")
  BINS+=("$dir/${sw_base}_hw_trig.riscv:hw_trig_riscv:$dir_name")
  BINS+=("$dir/${sw_base}_hw_mat_systolic.riscv:hw_mat_systolic_riscv:$dir_name")
done

echo "gem5: $GEM5"
echo "Running ${#BINS[@]} paper RISC-V binaries (parallel=$PARALLEL)..."
for item in "${BINS[@]}"; do
  IFS=':' read -r bin variant dir_tag <<< "$item"
  echo "$bin $variant $dir_tag"
done | xargs -P "$PARALLEL" -n 3 bash -c 'run_one "$1" "$2" "$3"' _ 

pass=$(awk -F, 'NR>1 && $4==1' "$SUMMARY" | wc -l)
fail=$(awk -F, 'NR>1 && $4!=1' "$SUMMARY" | wc -l)
total=$(awk -F, 'NR>1' "$SUMMARY" | wc -l)
echo ""
echo "=== PAPER RISC-V GEM5 MV9 VERIFY ==="
echo "PASS=$pass FAIL=$fail TOTAL=$total"
echo "Results: $OUT"
if [[ "$fail" -gt 0 ]]; then
  awk -F, 'NR>1 && $4!=1 {print $1, "exit="$3}' "$SUMMARY"
  exit 1
fi
