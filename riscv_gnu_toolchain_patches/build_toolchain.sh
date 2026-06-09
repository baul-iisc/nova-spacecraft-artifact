#!/usr/bin/env bash
# Build RISC-V GNU toolchain with NOVA custom ISA extensions
# Usage: bash scripts/build_toolchain.sh [INSTALL_PREFIX]
set -euo pipefail

INSTALL_PREFIX="${1:-$HOME/riscv-nova}"
JOBS="${2:-$(nproc)}"

echo "=========================================="
echo " Building RISC-V Toolchain (NOVA ISA)"
echo "=========================================="
echo "  Install prefix: $INSTALL_PREFIX"
echo "  Parallel:       $JOBS jobs"
echo ""

# Check dependencies
for cmd in autoconf automake bison flex gawk; do
    if ! command -v "$cmd" &>/dev/null; then
        echo "ERROR: $cmd not found."
        echo "  sudo apt-get install autoconf automake autotools-dev bison flex gawk"
        exit 1
    fi
done

# Clone if not already present
TOOLCHAIN_DIR="$(dirname "$INSTALL_PREFIX")/riscv-gnu-toolchain-nova"
if [[ ! -d "$TOOLCHAIN_DIR" ]]; then
    echo "Cloning RISC-V GNU toolchain (nova-custom-isa branch)..."
    git clone --branch nova-custom-isa --recursive \
        https://github.com/baul-iisc/riscv-gnu-toolchain-nova.git \
        "$TOOLCHAIN_DIR"
else
    echo "Toolchain source already exists at $TOOLCHAIN_DIR"
fi

cd "$TOOLCHAIN_DIR"

# Verify branch
BRANCH=$(git rev-parse --abbrev-ref HEAD)
if [[ "$BRANCH" != "nova-custom-isa" ]]; then
    echo "WARNING: Not on nova-custom-isa branch (on $BRANCH). Switching..."
    git checkout nova-custom-isa
fi

echo ""
echo "Configuring toolchain..."
./configure --prefix="$INSTALL_PREFIX" --with-arch=rv64gc --with-abi=lp64d

echo ""
echo "Building newlib (bare-metal) toolchain..."
make -j"$JOBS"

echo ""
echo "Building Linux (glibc) toolchain..."
make linux -j"$JOBS"

echo ""
echo "Building Spike ISA simulator..."
cd "$TOOLCHAIN_DIR"
if [[ -d spike ]]; then
    cd spike
    mkdir -p build && cd build
    ../configure --prefix="$INSTALL_PREFIX"
    make -j"$JOBS"
    make install
fi

echo ""
echo "=========================================="
echo " Toolchain installed to: $INSTALL_PREFIX"
echo "=========================================="
echo ""
echo "Add to PATH:"
echo "  export PATH=$INSTALL_PREFIX/bin:\$PATH"
echo ""
echo "Verify:"
echo "  riscv64-unknown-elf-gcc --version"
echo "  riscv64-unknown-elf-objdump --version  # Should show NOVA instructions"
echo "  spike --help                           # NOVA ISA simulator"
