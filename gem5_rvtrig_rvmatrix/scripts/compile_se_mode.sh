#!/bin/bash
#
# NOVA Processor - Compile Workloads for SE (System Emulator) Mode
# PhD Research: Futuristic Spacecraft Processor
#
# SE Mode: Baremetal execution - no operating system
# Uses: riscv64-unknown-elf-gcc (baremetal toolchain)
# Output: *.riscv files
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WORKLOAD_DIR="${PROJECT_DIR}/workloads"

echo "========================================"
echo "NOVA Processor - SE Mode Compilation"
echo "========================================"
echo ""
echo "Mode: System Emulator (Baremetal)"
echo "Toolchain: riscv64-unknown-elf-gcc"
echo "Output: *.riscv files"
echo ""

cd "$WORKLOAD_DIR"

# Verify toolchain
if ! make check-baremetal-toolchain 2>/dev/null; then
    echo "ERROR: Baremetal toolchain not found!"
    echo "Expected at: /data/home/chandraboul/development/riscv-spike/bin/"
    exit 1
fi

echo ""
echo "Compiling SE mode workloads..."
echo ""

make clean-baremetal 2>/dev/null || true
make baremetal

echo ""
echo "========================================"
echo "SE MODE COMPILATION COMPLETE"
echo "========================================"
echo ""
echo "Workloads ready for SE mode simulation:"
ls -la *.riscv 2>/dev/null
echo ""
echo "Run simulation with:"
echo "  ./experiments/run_se_mode_analysis.sh"
echo ""

