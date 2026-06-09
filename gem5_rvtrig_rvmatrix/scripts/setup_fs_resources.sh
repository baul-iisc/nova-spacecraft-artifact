#!/bin/bash
#
# Setup script for gem5 RISC-V Full System resources
# PhD Research: Chandraboul
#
# This script downloads the necessary kernel and disk images
# for Full System simulation.

set -e

# Configuration
RESOURCE_DIR="${HOME}/gem5-resources/riscv"
GEM5_RESOURCE_URL="http://dist.gem5.org/dist/v22-1"

echo "=============================================="
echo "gem5 RISC-V Full System Resource Setup"
echo "=============================================="
echo ""

# Create resource directory
echo "Creating resource directory: ${RESOURCE_DIR}"
mkdir -p "${RESOURCE_DIR}"
cd "${RESOURCE_DIR}"

# Function to download file if not exists
download_if_needed() {
    local url=$1
    local filename=$2
    
    if [ -f "${filename}" ]; then
        echo "  [SKIP] ${filename} already exists"
    else
        echo "  [DOWNLOAD] ${filename}..."
        wget -q --show-progress "${url}" -O "${filename}"
    fi
}

echo ""
echo "Downloading RISC-V Linux kernel with OpenSBI bootloader..."
download_if_needed \
    "${GEM5_RESOURCE_URL}/kernels/riscv/static/bootloader-vmlinux-5.10" \
    "bootloader-vmlinux-5.10"

echo ""
echo "Downloading RISC-V disk image..."
if [ -f "riscv-disk.img" ]; then
    echo "  [SKIP] riscv-disk.img already exists"
else
    download_if_needed \
        "${GEM5_RESOURCE_URL}/images/riscv/busybox/riscv-disk.img.gz" \
        "riscv-disk.img.gz"
    echo "  Extracting disk image..."
    gunzip -k "riscv-disk.img.gz"
fi

echo ""
echo "=============================================="
echo "Setup Complete!"
echo "=============================================="
echo ""
echo "Resources downloaded to: ${RESOURCE_DIR}"
echo ""
echo "Files:"
ls -lh "${RESOURCE_DIR}"
echo ""
echo "To run Full System simulation:"
echo ""
echo "  cd /data/home/chandraboul/development/gem5-matrix/gem5-nova"
echo ""
echo "  # Basic run"
echo "  build/RISCV/gem5.opt configs/example/riscv/spacecraft_fs.py \\"
echo "      --kernel=${RESOURCE_DIR}/bootloader-vmlinux-5.10 \\"
echo "      --disk-image=${RESOURCE_DIR}/riscv-disk.img \\"
echo "      --num-cpus=4"
echo ""
echo "  # With accelerators"
echo "  build/RISCV/gem5.opt configs/example/riscv/spacecraft_fs.py \\"
echo "      --kernel=${RESOURCE_DIR}/bootloader-vmlinux-5.10 \\"
echo "      --disk-image=${RESOURCE_DIR}/riscv-disk.img \\"
echo "      --num-cpus=4 \\"
echo "      --enable-compression --enable-ml"
echo ""

# Create a convenience script
cat > "${RESOURCE_DIR}/run_spacecraft_fs.sh" << 'EOF'
#!/bin/bash
# Convenience script to run spacecraft FS simulation
GEM5_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-nova"
RESOURCE_DIR="$(dirname "$0")"

cd "${GEM5_DIR}"

${GEM5_DIR}/build/RISCV/gem5.opt \
    ${GEM5_DIR}/configs/example/riscv/spacecraft_fs.py \
    --kernel=${RESOURCE_DIR}/bootloader-vmlinux-5.10 \
    --disk-image=${RESOURCE_DIR}/riscv-disk.img \
    --num-cpus=4 \
    --mem-size=512MB \
    --enable-compression \
    --enable-ml \
    "$@"
EOF
chmod +x "${RESOURCE_DIR}/run_spacecraft_fs.sh"

echo "Convenience script created: ${RESOURCE_DIR}/run_spacecraft_fs.sh"
echo ""

