#!/usr/bin/env bash
# Run McPAT power analysis for all SoC components
# Usage: bash scripts/run_mcpat_power.sh [--gem5-stats FILE]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GEM5_ROOT="${GEM5_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"

MCPAT_BIN="$GEM5_ROOT/ext/mcpat/mcpat"
if [[ ! -f "$MCPAT_BIN" ]]; then
    MCPAT_BIN="$GEM5_ROOT/build/mcpat/mcpat"
fi

echo "=========================================="
echo " McPAT Power Analysis"
echo "=========================================="
echo ""

if [[ ! -f "$MCPAT_BIN" ]]; then
    echo "ERROR: McPAT binary not found."
    echo "  Build: bash scripts/build_mcpat.sh"
    exit 1
fi

echo "McPAT binary: $MCPAT_BIN"
echo ""

RESULTS_DIR="$GEM5_ROOT/m5out"
mkdir -p "$RESULTS_DIR"

# Run the full SoC power breakdown
echo "--- Full SoC Power Breakdown ---"
echo ""
python3 "$GEM5_ROOT/scripts/spacecraft_soc_power_mcpat.py" \
    --mcpat-bin "$MCPAT_BIN" \
    "$@" \
    2>&1 | tee "$RESULTS_DIR/mcpat_soc_power.txt"

echo ""
echo "--- Workload-Weighted Power Model ---"
echo ""
python3 "$GEM5_ROOT/scripts/spacecraft_power_model.py" \
    "$@" \
    2>&1 | tee "$RESULTS_DIR/workload_power.txt"

echo ""
echo "Results saved to $RESULTS_DIR/"
