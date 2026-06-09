#!/usr/bin/env bash
# Build the RISC-V GNU toolchain with the NOVA custom ISA extensions.
#
# This script is self-contained: it clones the OFFICIAL upstream
# riscv-gnu-toolchain, applies the NOVA patch files shipped alongside this
# script (./patches/), and builds. No private repository is required.
#
# The NOVA changes are:
#   - binutils opcodes  -> the assembler recognises fsin.d, fcos.d, fatan2.d,
#                          mldS, msdS, sys.mmacd, etc. (this is what the
#                          workloads need, since they emit the instructions via
#                          inline assembly; GCC itself is NOT modified).
#   - Spike (optional)  -> the ISA simulator can execute the new instructions.
#                          The paper uses gem5, not Spike, so Spike is optional.
#
# VERSION NOTE (please read):
#   The patch files are full-file replacements (including the complete Spike
#   insns/ directory). They were authored against the GCC 13.2.0-era toolchain.
#   The default upstream tag below (2023.12.20) ships GCC 13.2.0 and is the
#   best-effort match. If you build against a very different upstream version,
#   binutils' riscv-opc.c / Spike's insns may have diverged and you may need to
#   reconcile the patched files with your checkout. Override the tag with
#   UPSTREAM_REF if needed.
#
# Usage: bash build_toolchain.sh [INSTALL_PREFIX] [JOBS]
#   INSTALL_PREFIX   default: $HOME/riscv-nova
#   JOBS             default: $(nproc)
#   UPSTREAM_REF     (env) upstream riscv-gnu-toolchain tag, default 2023.12.20
#   BUILD_SPIKE      (env) set to 1 to also build the Spike simulator
set -euo pipefail

INSTALL_PREFIX="${1:-$HOME/riscv-nova}"
JOBS="${2:-$(nproc)}"
UPSTREAM_REF="${UPSTREAM_REF:-2023.12.20}"
BUILD_SPIKE="${BUILD_SPIKE:-0}"

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PATCH_DIR="$SCRIPT_DIR/patches"

echo "=========================================="
echo " Building RISC-V Toolchain (NOVA ISA)"
echo "=========================================="
echo "  Install prefix:  $INSTALL_PREFIX"
echo "  Parallel jobs:   $JOBS"
echo "  Upstream tag:    $UPSTREAM_REF (override with UPSTREAM_REF=...)"
echo "  Patches from:    $PATCH_DIR"
echo "  Build Spike:     $BUILD_SPIKE (set BUILD_SPIKE=1 to enable)"
echo ""

# --- Dependencies ----------------------------------------------------------
for cmd in git autoconf automake bison flex gawk; do
    if ! command -v "$cmd" &>/dev/null; then
        echo "ERROR: '$cmd' not found."
        echo "  sudo apt-get install git autoconf automake autotools-dev bison flex gawk \\"
        echo "       texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev"
        exit 1
    fi
done

if [[ ! -f "$PATCH_DIR/binutils/riscv-opc.c" ]]; then
    echo "ERROR: NOVA patch files not found at $PATCH_DIR"
    echo "  Run this script from inside riscv_gnu_toolchain_patches/."
    exit 1
fi

# --- 1. Clone official upstream riscv-gnu-toolchain -------------------------
SRC_DIR="$(dirname "$INSTALL_PREFIX")/riscv-gnu-toolchain-upstream"
if [[ ! -d "$SRC_DIR" ]]; then
    echo "[1/4] Cloning official upstream riscv-gnu-toolchain @ $UPSTREAM_REF ..."
    git clone https://github.com/riscv-collab/riscv-gnu-toolchain.git "$SRC_DIR"
    git -C "$SRC_DIR" checkout "$UPSTREAM_REF"
else
    echo "[1/4] Upstream source already present at $SRC_DIR (reusing)"
fi
cd "$SRC_DIR"

# Populate the submodules we patch (binutils always; spike only if requested).
echo "      Initialising binutils submodule ..."
git submodule update --init binutils
if [[ "$BUILD_SPIKE" == "1" ]]; then
    echo "      Initialising spike submodule ..."
    git submodule update --init spike
fi

# --- 2. Apply NOVA patch files ---------------------------------------------
echo "[2/4] Applying NOVA patch files ..."
cp "$PATCH_DIR/binutils/riscv-opc.c" binutils/opcodes/riscv-opc.c
cp "$PATCH_DIR/binutils/riscv-opc.h" binutils/include/opcode/riscv-opc.h
echo "      binutils opcodes patched."

if [[ "$BUILD_SPIKE" == "1" ]]; then
    cp "$PATCH_DIR/spike/encoding.h" spike/riscv/encoding.h
    cp "$PATCH_DIR/spike/disasm.cc"  spike/disasm/disasm.cc
    cp "$PATCH_DIR/spike/insns/"*.h  spike/riscv/insns/
    echo "      Spike encoding/disasm/insns patched."
fi

# --- 3. Build the bare-metal (newlib) toolchain ----------------------------
echo "[3/4] Configuring and building the newlib toolchain ..."
./configure --prefix="$INSTALL_PREFIX" --with-arch=rv64gc --with-abi=lp64d
make -j"$JOBS"

# --- 4. Optionally build Spike ---------------------------------------------
if [[ "$BUILD_SPIKE" == "1" && -d spike ]]; then
    echo "[4/4] Building Spike ISA simulator ..."
    ( cd spike && mkdir -p build && cd build \
        && ../configure --prefix="$INSTALL_PREFIX" \
        && make -j"$JOBS" && make install )
else
    echo "[4/4] Skipping Spike (set BUILD_SPIKE=1 to build it)."
fi

echo ""
echo "=========================================="
echo " Toolchain installed to: $INSTALL_PREFIX"
echo "=========================================="
echo ""
echo "Add to PATH:"
echo "  export PATH=$INSTALL_PREFIX/bin:\$PATH"
echo ""
echo "Verify the NOVA instructions assemble:"
echo "  echo 'void f(void){ __asm__ volatile(\"fsin.d fa0,fa0\"); }' | \\"
echo "    $INSTALL_PREFIX/bin/riscv64-unknown-elf-gcc -x c - -c -o /dev/null && echo OK"
