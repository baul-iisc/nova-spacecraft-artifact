#!/usr/bin/env bash
set -uo pipefail

SUB="$(cd "$(dirname "$0")" && pwd)"
GEM5="${GEM5:-$(cd "$(dirname "$0")/../gem5_rvtrig_rvmatrix" && pwd)/build/RISCV/gem5.opt}"
CFG="${CFG:-$(cd "$(dirname "$0")/../gem5_rvtrig_rvmatrix" && pwd)/configs/learning_gem5/part1/riscv_modified_configs.py}"
OUT="${OUT:-$SUB/gem5_mv9_verify_$(date +%Y%m%d_%H%M%S)}"
PARALLEL="${PARALLEL:-4}"
SUMMARY="$OUT/summary.csv"

mkdir -p "$OUT"
echo "tag,exit_code,stats_ok,simSeconds,simInsts,binary" > "$SUMMARY"

SIM_ARGS=(
  --l1i_size=8kB --l1d_size=16kB --l2_size=64kB --system_mem_range=64GB
  --clock_freq=500MHz --branch_pred=BiModeBP --prefetcher=None
  --cpu_type=TimingSimpleCPU --l1i_assoc=2 --l1d_assoc=2
  --l1i_tag_latency=2 --l1d_tag_latency=4 --l1i_data_latency=2 --l1d_data_latency=4
  --l1i_response_latency=2 --l1d_response_latency=4 --l1i_mshrs=2 --l1d_mshrs=2
  --l1i_tgts_per_mshr=8 --l1d_tgts_per_mshr=8
)

run_one() {
  local bin="$1"
  local tag="${bin#$SUB/}"
  tag="${tag//\//_}"
  tag="${tag%.riscv}"
  local rdir="$OUT/$tag"
  mkdir -p "$rdir"
  local timeout_s=600
  case "$tag" in
    *Navigation*|*VBN*|*SAR*|*HyperSpectral*) timeout_s=1200 ;;
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
    echo "$tag,$ec,$ok,$sim_s,$sim_i,$bin" >> "$SUMMARY"
  } 9>>"$OUT/.summary.lock"
  if [[ $ok -eq 1 ]]; then
    echo "PASS $tag simSeconds=$sim_s"
  else
    echo "FAIL $tag exit=$ec"
    tail -3 "$rdir/stderr.log" 2>/dev/null || true
  fi
}

export -f run_one
export SUB GEM5 CFG OUT SUMMARY
export SIM_ARGS

mapfile -t BINS < <(find "$SUB" -maxdepth 2 -name '*.riscv' | sort)
echo "gem5: $GEM5"
echo "cfg:  $CFG"
echo "Running ${#BINS[@]} RISC-V binaries (parallel=$PARALLEL)..."
printf '%s\n' "${BINS[@]}" | xargs -P "$PARALLEL" -I{} bash -c 'run_one "$@"' _ {}

pass=$(awk -F, 'NR>1 && $3==1' "$SUMMARY" | wc -l)
fail=$(awk -F, 'NR>1 && $3!=1' "$SUMMARY" | wc -l)
total=$(awk -F, 'NR>1' "$SUMMARY" | wc -l)
echo ""
echo "=== GEM5 MV9 VERIFY COMPLETE ==="
echo "PASS=$pass FAIL=$fail TOTAL=$total"
echo "Results: $OUT"
echo "Summary: $SUMMARY"
if [[ "$fail" -gt 0 ]]; then
  echo "Failures:"
  awk -F, 'NR>1 && $3!=1 {print $1, "exit="$2}' "$SUMMARY"
  exit 1
fi
