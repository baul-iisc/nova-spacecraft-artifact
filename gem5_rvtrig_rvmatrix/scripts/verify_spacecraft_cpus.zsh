#!/usr/bin/env zsh
#
# Run spacecraft RISC-V ELFs twice: AtomicSimpleCPU (fast ISA smoke) and
# TimingSimpleCPU (full timing + accelerator stall model used in the paper).
#
# Usage:
#   GEM5_BIN=/path/to/gem5.opt zsh scripts/verify_spacecraft_cpus.zsh
# Optional:
#   WORKLOAD_ROOT=.../28_AI ONLY_TIMING=1 ONLY_ATOMIC=1
#

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ARTIFACT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

GEM5_BIN="${GEM5_BIN:-/data/home/chandraboul/development/gem5-nova-fix1/build/RISCV/gem5.opt}"
CONFIG_PY="${CONFIG_PY:-${ARTIFACT_ROOT}/configs/learning_gem5/part1/riscv_modified_configs.py}"

WORKLOAD_ROOT="${WORKLOAD_ROOT:-/data/home/chandraboul/development/iiswc2026_multicore_dse/IEEE_TC_paper/spacecraft_benchmarks_28_submission/28_AI}"

# Short workload (10 iterations) — override WORKLOAD_* for another directory / glob
BINARIES=(
  spacecraft_ai.riscv
  spacecraft_ai_hw.riscv
  spacecraft_ai_hw_trig.riscv
  spacecraft_ai_hw_mat_systolic.riscv
)

RESULTS="${RESULTS:-${ARTIFACT_ROOT}/m5_verify_cpu_compare_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "${RESULTS}/logs"

if [[ ! -x "${GEM5_BIN}" ]]; then
  echo "ERROR: GEM5_BIN not executable: ${GEM5_BIN}"
  exit 1
fi
if [[ ! -f "${CONFIG_PY}" ]]; then
  echo "ERROR: CONFIG_PY not found: ${CONFIG_PY}"
  exit 1
fi

COMMON_ARGS=(
  --l1i_size=8kB --l1d_size=16kB --l2_size=64kB --system_mem_range=64GB --clock_freq=500MHz
  --branch_pred=BiModeBP --prefetcher=None
  --l1i_assoc=2 --l1d_assoc=2 --l1i_tag_latency=2 --l1d_tag_latency=4
  --l1i_data_latency=2 --l1d_data_latency=4 --l1i_response_latency=2 --l1d_response_latency=4
  --l1i_mshrs=2 --l1d_mshrs=2 --l1i_tgts_per_mshr=8 --l1d_tgts_per_mshr=8
)

CPUS_ORDER=(AtomicSimpleCPU TimingSimpleCPU)
if [[ "${ONLY_TIMING:-}" == "1" ]]; then
  CPUS_ORDER=(TimingSimpleCPU)
elif [[ "${ONLY_ATOMIC:-}" == "1" ]]; then
  CPUS_ORDER=(AtomicSimpleCPU)
fi

export GEM5_IEEE_TC_ARTIFACT="${GEM5_IEEE_TC_ARTIFACT:-1}"

echo "gem5           : ${GEM5_BIN}"
echo "config         : ${CONFIG_PY}"
echo "workloads      : ${WORKLOAD_ROOT}"
echo "results        : ${RESULTS}"
echo "GEM5_IEEE_TC_ARTIFACT=${GEM5_IEEE_TC_ARTIFACT}"
echo "CPU order      : ${CPUS_ORDER[*]}"
echo ""

for cpu in "${CPUS_ORDER[@]}"; do
  for bn in "${BINARIES[@]}"; do
    bin_path="${WORKLOAD_ROOT}/${bn}"
    label="${bn%.riscv}_${cpu:l}"
    out="${RESULTS}/${label}"
    log="${RESULTS}/logs/${label}.log"
    mkdir -p "${out}"

    if [[ ! -f "${bin_path}" ]]; then
      echo "[SKIP] missing ${bin_path}"
      continue
    fi

    echo "━━━━━━━━ ${cpu} ← ${bn} ━━━━━━━━"
    (
      cd "$(dirname "${GEM5_BIN}")/.." || cd "${ARTIFACT_ROOT}"
      exec "${GEM5_BIN}" --outdir="${out}" "${CONFIG_PY}" "${bin_path}" \
        "${COMMON_ARGS[@]}" \
        --cpu_type="${cpu}" \
    ) >"${log}" 2>&1

    if [[ ! -s "${out}/stats.txt" ]]; then
      echo "[FAIL] empty stats: ${label}"
      tail -40 "${log}"
      exit 1
    fi

    if grep -Eiq 'panic:|fatal:|unknown operand|Unhandled|illegal' "${log}"; then
      echo "[WARN] Possible fault wording in log (inspect): ${log}"
    fi

    echo "[OK]   ${label} stats $(wc -c <"${out}/stats.txt") bytes → ${out}"
    tail -3 "${log}" || true
    echo ""
  done
done

echo "Done. Outputs under ${RESULTS}"
