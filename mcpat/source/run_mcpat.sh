#!/usr/bin/env bash
# Run McPAT for power modeling (spacecraft RISC-V config or custom XML).
# Usage:
#   ./run_mcpat.sh [xml_file] [print_level]
# Default: riscv_spacecraft_22nm_500MHz_newfmt.xml (22nm, 500MHz, in-order, L1 8K/16K, L2 64K)
# Build: from repo root, run: make -C ext/mcpat

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_MCPAT="$REPO_ROOT/build/mcpat/mcpat"

XML="${1:-$SCRIPT_DIR/riscv_spacecraft_22nm_500MHz_newfmt.xml}"
LEVEL="${2:-2}"

if [[ ! -x "$BUILD_MCPAT" ]]; then
    echo "McPAT binary not found. Build with: make -C ext/mcpat"
    exit 1
fi
if [[ ! -f "$XML" ]]; then
    echo "XML file not found: $XML"
    exit 1
fi

echo "Running McPAT: $BUILD_MCPAT -infile $XML -print_level $LEVEL"
"$BUILD_MCPAT" -infile "$XML" -print_level "$LEVEL"
