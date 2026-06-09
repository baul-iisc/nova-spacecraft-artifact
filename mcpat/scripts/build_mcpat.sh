#!/usr/bin/env bash
# Build McPAT from source (included in gem5 ext/mcpat/)
# Usage: bash scripts/build_mcpat.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
GEM5_ROOT="${GEM5_ROOT:-$(cd "$SCRIPT_DIR/../.." && pwd)}"
MCPAT_DIR="$GEM5_ROOT/ext/mcpat"

echo "=========================================="
echo " Building McPAT Power Estimator"
echo "=========================================="
echo "  Source:  $MCPAT_DIR"
echo ""

cd "$MCPAT_DIR"

if ! command -v g++ &>/dev/null; then
    echo "ERROR: g++ not found. Install build-essential."
    exit 1
fi

echo "Building McPAT..."
make -j"$(nproc)" 2>&1

# McPAT may build in-place or in build dir
MCPAT_BIN=""
if [[ -f "$MCPAT_DIR/mcpat" ]]; then
    MCPAT_BIN="$MCPAT_DIR/mcpat"
elif [[ -f "$GEM5_ROOT/build/mcpat/mcpat" ]]; then
    MCPAT_BIN="$GEM5_ROOT/build/mcpat/mcpat"
fi

if [[ -n "$MCPAT_BIN" ]]; then
    echo ""
    echo "SUCCESS: McPAT binary at $MCPAT_BIN"
    ls -lh "$MCPAT_BIN"
    echo ""
    echo "Custom NOVA XML templates in $MCPAT_DIR:"
    ls -1 "$MCPAT_DIR"/riscv_spacecraft_*.xml "$MCPAT_DIR"/accel_*.xml 2>/dev/null || true
else
    echo ""
    echo "FAILED: McPAT binary not found after build."
    echo "Check build output above for errors."
    exit 1
fi
