#!/bin/bash
#
# NOVA Processor - Compile Workloads for FS (Full System) Mode
# PhD Research: Futuristic Spacecraft Processor
#
# FS Mode: Linux userspace execution
# Uses: riscv64-linux-gnu-gcc (Linux toolchain)
# Output: *.linux files
#
# After compilation, workloads must be copied to the disk image
# for execution inside the Linux guest.
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WORKLOAD_DIR="${PROJECT_DIR}/workloads"
FS_WORKLOAD_DIR="${PROJECT_DIR}/workloads/fs_binaries"

echo "========================================"
echo "NOVA Processor - FS Mode Compilation"
echo "========================================"
echo ""
echo "Mode: Full System (Linux Userspace)"
echo "Toolchain: riscv64-linux-gnu-gcc"
echo "Output: *.linux files"
echo ""

cd "$WORKLOAD_DIR"

# Verify toolchain
echo "Checking Linux toolchain..."
if ! which riscv64-linux-gnu-gcc > /dev/null 2>&1; then
    echo "ERROR: Linux toolchain not found!"
    echo "Install with: sudo apt install gcc-riscv64-linux-gnu"
    exit 1
fi

riscv64-linux-gnu-gcc --version | head -1
echo ""

echo "Compiling FS mode workloads..."
echo ""

make clean-linux 2>/dev/null || true
make linux

# Create directory for FS binaries
mkdir -p "$FS_WORKLOAD_DIR"
cp *.linux "$FS_WORKLOAD_DIR/" 2>/dev/null || true

echo ""
echo "========================================"
echo "FS MODE COMPILATION COMPLETE"
echo "========================================"
echo ""
echo "Workloads compiled for Linux:"
ls -la *.linux 2>/dev/null
echo ""
echo "Binaries also copied to: $FS_WORKLOAD_DIR"
echo ""
echo "NEXT STEPS FOR FS MODE:"
echo "  1. Mount disk image:"
echo "     sudo mkdir -p /mnt/gem5disk"
echo "     sudo mount -o loop ~/.cache/gem5/riscv-disk-img /mnt/gem5disk"
echo ""
echo "  2. Copy workloads to disk:"
echo "     sudo cp $FS_WORKLOAD_DIR/*.linux /mnt/gem5disk/root/"
echo ""
echo "  3. Unmount disk:"
echo "     sudo umount /mnt/gem5disk"
echo ""
echo "  4. Run FS simulation:"
echo "     ./experiments/run_fs_mode_analysis.sh"
echo ""

