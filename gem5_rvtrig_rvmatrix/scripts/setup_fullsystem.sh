#!/bin/bash
# NOVA Processor - Full System Mode Setup Script
# This script downloads and prepares everything needed for FS mode with pthreads

set -e

GEM5_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-nova"
RESOURCES_DIR="${GEM5_DIR}/resources/riscv-fs"
WORKLOAD_DIR="${GEM5_DIR}/workloads/fullsystem"

echo "================================================================================"
echo "NOVA Processor - Full System Mode Setup"
echo "================================================================================"

# Create directories
mkdir -p "$RESOURCES_DIR"
mkdir -p "$WORKLOAD_DIR"

cd "$RESOURCES_DIR"

# ============================================================================
# Option 1: Use gem5 Resources (Recommended)
# ============================================================================

echo ""
echo "Step 1: Downloading gem5 RISC-V full system resources..."
echo "-----------------------------------------------------------------------"

# Check if gem5 resources are already downloaded
if [ ! -f "${RESOURCES_DIR}/riscv-bootloader-vmlinux-5.10" ]; then
    echo "Downloading RISC-V bootloader/kernel..."
    
    # gem5 resources repository
    GEM5_RESOURCES_URL="https://resources.gem5.org/resources"
    
    # Download using gem5 resource downloader (if available) or wget
    if command -v gem5-resources &> /dev/null; then
        gem5-resources download riscv-boot-linux-5.10 --output-dir "$RESOURCES_DIR"
    else
        echo "Manual download required. See instructions below."
    fi
else
    echo "Kernel/bootloader already present."
fi

# ============================================================================
# Option 2: Build Linux Kernel from Source
# ============================================================================

build_kernel() {
    echo ""
    echo "Building Linux kernel for RISC-V..."
    echo "-----------------------------------------------------------------------"
    
    LINUX_VERSION="5.10.110"
    LINUX_DIR="${RESOURCES_DIR}/linux-${LINUX_VERSION}"
    
    if [ ! -d "$LINUX_DIR" ]; then
        echo "Downloading Linux kernel ${LINUX_VERSION}..."
        wget -q "https://cdn.kernel.org/pub/linux/kernel/v5.x/linux-${LINUX_VERSION}.tar.xz"
        tar xf "linux-${LINUX_VERSION}.tar.xz"
        rm "linux-${LINUX_VERSION}.tar.xz"
    fi
    
    cd "$LINUX_DIR"
    
    # Use RISC-V cross compiler
    export ARCH=riscv
    export CROSS_COMPILE=riscv64-unknown-linux-gnu-
    
    # Configure for gem5
    make defconfig
    
    # Enable required features for gem5
    cat >> .config << 'EOF'
CONFIG_RISCV=y
CONFIG_64BIT=y
CONFIG_SMP=y
CONFIG_NR_CPUS=16
CONFIG_PRINTK_TIME=y
CONFIG_SERIAL_8250=y
CONFIG_SERIAL_8250_CONSOLE=y
CONFIG_VIRTIO=y
CONFIG_VIRTIO_BLK=y
CONFIG_EXT4_FS=y
CONFIG_DEVTMPFS=y
CONFIG_DEVTMPFS_MOUNT=y
EOF
    
    make olddefconfig
    make -j$(nproc) vmlinux
    
    cp vmlinux "${RESOURCES_DIR}/riscv-linux-5.10-vmlinux"
    echo "Kernel built: ${RESOURCES_DIR}/riscv-linux-5.10-vmlinux"
}

# ============================================================================
# Option 3: Create Disk Image with Busybox
# ============================================================================

create_disk_image() {
    echo ""
    echo "Creating minimal RISC-V disk image with Busybox..."
    echo "-----------------------------------------------------------------------"
    
    DISK_IMG="${RESOURCES_DIR}/riscv-disk.img"
    MOUNT_DIR="${RESOURCES_DIR}/mnt"
    
    # Create disk image (1GB)
    dd if=/dev/zero of="$DISK_IMG" bs=1M count=1024
    mkfs.ext4 "$DISK_IMG"
    
    # Mount and populate
    mkdir -p "$MOUNT_DIR"
    sudo mount -o loop "$DISK_IMG" "$MOUNT_DIR"
    
    # Download and extract busybox
    BUSYBOX_VER="1.35.0"
    cd "${RESOURCES_DIR}"
    
    if [ ! -d "busybox-${BUSYBOX_VER}" ]; then
        wget -q "https://busybox.net/downloads/busybox-${BUSYBOX_VER}.tar.bz2"
        tar xf "busybox-${BUSYBOX_VER}.tar.bz2"
    fi
    
    cd "busybox-${BUSYBOX_VER}"
    
    # Configure for static build
    make ARCH=riscv CROSS_COMPILE=riscv64-unknown-linux-gnu- defconfig
    sed -i 's/# CONFIG_STATIC is not set/CONFIG_STATIC=y/' .config
    make ARCH=riscv CROSS_COMPILE=riscv64-unknown-linux-gnu- -j$(nproc)
    make ARCH=riscv CROSS_COMPILE=riscv64-unknown-linux-gnu- CONFIG_PREFIX="$MOUNT_DIR" install
    
    # Create essential directories and files
    cd "$MOUNT_DIR"
    mkdir -p dev proc sys etc/init.d home/gem5
    
    # Create init script
    cat > etc/init.d/rcS << 'EOF'
#!/bin/sh
mount -t proc proc /proc
mount -t sysfs sysfs /sys
mount -t devtmpfs devtmpfs /dev

echo "NOVA Processor - Full System Linux Booted"
echo "Running workload..."

# Run the workload if present
if [ -f /home/gem5/workload ]; then
    /home/gem5/workload
fi

# Exit simulation when done
echo "Workload complete. Exiting..."
/sbin/m5 exit
EOF
    chmod +x etc/init.d/rcS
    
    # Create inittab
    cat > etc/inittab << 'EOF'
::sysinit:/etc/init.d/rcS
::respawn:/bin/sh
EOF
    
    sudo umount "$MOUNT_DIR"
    echo "Disk image created: $DISK_IMG"
}

# ============================================================================
# Compile Pthreads Workloads for Full System
# ============================================================================

compile_fs_workloads() {
    echo ""
    echo "Compiling pthreads workloads for full system mode..."
    echo "-----------------------------------------------------------------------"
    
    cd "$WORKLOAD_DIR"
    
    # Use Linux RISC-V toolchain (not baremetal)
    RISCV_GCC="riscv64-unknown-linux-gnu-gcc"
    
    if ! command -v $RISCV_GCC &> /dev/null; then
        echo "ERROR: $RISCV_GCC not found."
        echo "Install with: apt install gcc-riscv64-linux-gnu"
        echo "Or build from source."
        return 1
    fi
    
    for src in *.c; do
        if [ -f "$src" ]; then
            base="${src%.c}"
            echo "  Compiling $src..."
            $RISCV_GCC -O2 -static -pthread -o "${base}" "$src" -lm \
                -DUSE_HW_ACCEL || \
            $RISCV_GCC -O2 -static -pthread -o "${base}" "$src" -lm
        fi
    done
    
    echo "Workloads compiled."
}

# ============================================================================
# Download Pre-built gem5 Resources (Fastest Option)
# ============================================================================

download_prebuilt() {
    echo ""
    echo "Downloading pre-built gem5 RISC-V resources..."
    echo "-----------------------------------------------------------------------"
    
    cd "$RESOURCES_DIR"
    
    # gem5 official resources
    RESOURCE_BASE="https://resources.gem5.org/resources"
    
    # Download bootloader
    if [ ! -f "riscv-bootloader-vmlinux-5.10" ]; then
        echo "Downloading bootloader..."
        wget -q "${RESOURCE_BASE}/riscv-bootloader-vmlinux-5.10" -O riscv-bootloader-vmlinux-5.10 || \
        echo "  Could not download bootloader. Will need to build."
    fi
    
    # Download kernel
    if [ ! -f "riscv-linux-5.10" ]; then
        echo "Downloading kernel..."
        wget -q "${RESOURCE_BASE}/riscv-linux-5.10" -O riscv-linux-5.10 || \
        echo "  Could not download kernel. Will need to build."
    fi
    
    # Download disk image
    if [ ! -f "riscv-disk-img" ]; then
        echo "Downloading disk image..."
        wget -q "${RESOURCE_BASE}/riscv-disk-img" -O riscv-disk-img || \
        echo "  Could not download disk. Will need to create."
    fi
}

# ============================================================================
# Print Usage Instructions
# ============================================================================

print_usage() {
    echo ""
    echo "================================================================================"
    echo "FULL SYSTEM MODE SETUP COMPLETE"
    echo "================================================================================"
    echo ""
    echo "RESOURCES DIRECTORY: $RESOURCES_DIR"
    echo ""
    echo "REQUIRED FILES:"
    echo "  - Kernel:     riscv-linux-5.10-vmlinux (or similar)"
    echo "  - Bootloader: riscv-bootloader-vmlinux-5.10 (optional)"
    echo "  - Disk Image: riscv-disk.img"
    echo ""
    echo "TO RUN FULL SYSTEM SIMULATION:"
    echo ""
    echo "  cd $GEM5_DIR"
    echo "  ./build/RISCV/gem5.opt configs/spacecraft/nova_fullsystem.py \\"
    echo "      --num-cpus=4 \\"
    echo "      --num-trig-accels=1 \\"
    echo "      --num-mat-accels=1 \\"
    echo "      --kernel=${RESOURCES_DIR}/riscv-linux-5.10-vmlinux \\"
    echo "      --disk-image=${RESOURCES_DIR}/riscv-disk.img"
    echo ""
    echo "TO COMPILE PTHREADS WORKLOADS:"
    echo ""
    echo "  # Install Linux RISC-V toolchain"
    echo "  sudo apt install gcc-riscv64-linux-gnu"
    echo ""
    echo "  # Compile workloads"
    echo "  cd ${WORKLOAD_DIR}"
    echo "  riscv64-unknown-linux-gnu-gcc -O2 -static -pthread \\"
    echo "      -o lunar_landing_pthread lunar_landing_pthread.c -lm"
    echo ""
    echo "================================================================================"
}

# ============================================================================
# Main
# ============================================================================

echo ""
echo "Choose setup option:"
echo "  1) Download pre-built gem5 resources (fastest, recommended)"
echo "  2) Build Linux kernel from source"
echo "  3) Create minimal disk image with Busybox"
echo "  4) Compile pthreads workloads"
echo "  5) Full setup (all of the above)"
echo "  6) Print usage instructions only"
echo ""

if [ $# -eq 0 ]; then
    read -p "Enter choice [1-6]: " choice
else
    choice=$1
fi

case $choice in
    1)
        download_prebuilt
        print_usage
        ;;
    2)
        build_kernel
        print_usage
        ;;
    3)
        create_disk_image
        print_usage
        ;;
    4)
        compile_fs_workloads
        print_usage
        ;;
    5)
        download_prebuilt
        compile_fs_workloads
        print_usage
        ;;
    6)
        print_usage
        ;;
    *)
        echo "Invalid choice"
        exit 1
        ;;
esac






