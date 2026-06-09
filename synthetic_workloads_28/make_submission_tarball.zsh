#!/bin/zsh

# Create a clean submission bundle (sources + scripts only).
# Excludes built binaries, logs, gem5 output trees, and internal verification artifacts.
#
# Usage:
#   zsh make_submission_tarball.zsh
#
# Output:
#   spacecraft_benchmarks_28_submission_clean_<timestamp>.tar.gz

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

OUT="spacecraft_benchmarks_28_submission_clean_$(date +'%Y-%m-%d_%H-%M-%S').tar.gz"

tar -czf "$OUT" \
  --exclude='**/*.riscv' \
  --exclude='**/*.arm' \
  --exclude='**/*.armsimd' \
  --exclude='**/*.log' \
  --exclude='**/gem5_results_*' \
  --exclude='**/gem5_verify_after_inverse_fix/**' \
  --exclude='**/logs/**' \
  --exclude='**/config.ini' \
  --exclude='**/config.json' \
  --exclude='**/stats.txt' \
  --exclude='**/stdout.log' \
  --exclude='**/prog.norm' \
  --exclude='**/citations.bib' \
  LICENSE NOTICE README.md flight_compliance.h \
  build_all_binaries.zsh run_gem5_simulations.zsh run_gem5_arm_rerun.zsh run_gem5_riscv_retry_empty_stats.zsh \
  [0-9][0-9]_*/

echo "Wrote: $SCRIPT_DIR/$OUT"

