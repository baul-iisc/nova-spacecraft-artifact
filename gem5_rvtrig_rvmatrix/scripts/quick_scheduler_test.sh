#!/bin/bash
# Quick test for NOVA Scheduler Integration
# Runs a single simulation to verify scheduler is working

set -e

GEM5_ROOT="/data/home/chandraboul/development/gem5-matrix/gem5-nova"
GEM5_BIN="${GEM5_ROOT}/build/RISCV/gem5.opt"
WORKLOAD_DIR="${GEM5_ROOT}/new_spacecraft_workloads"
CONFIG_SCRIPT="${GEM5_ROOT}/configs/deprecated/example/se.py"
OUTPUT_DIR="${GEM5_ROOT}/scheduler_quick_test"

echo "=================================================="
echo "NOVA Scheduler Quick Test"
echo "=================================================="

# Find a workload
WORKLOAD=$(ls ${WORKLOAD_DIR}/*.riscv 2>/dev/null | head -1)

if [ -z "$WORKLOAD" ]; then
    echo "ERROR: No workloads found in ${WORKLOAD_DIR}"
    exit 1
fi

echo "Using workload: $(basename $WORKLOAD)"

# Create output directory
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

# Set environment variables
export NOVA_NUM_TRIG_ACCELS=2
export NOVA_NUM_MAT_ACCELS=2
export NOVA_NUM_VPU_ACCELS=1
export NOVA_NUM_NPU_ACCELS=1
export NOVA_ACCEL_MODE="shared"
export NOVA_SCHEDULER_ENABLED="1"
export NOVA_MISSION_PHASE="LAUNCH"

echo ""
echo "Configuration:"
echo "  Cores: 4"
echo "  TrigAccels: $NOVA_NUM_TRIG_ACCELS"
echo "  MatAccels: $NOVA_NUM_MAT_ACCELS"
echo "  VPU: $NOVA_NUM_VPU_ACCELS"
echo "  NPU: $NOVA_NUM_NPU_ACCELS"
echo "  Phase: $NOVA_MISSION_PHASE"
echo ""

# Build CPU list for 4 cores
CPU_LIST="${WORKLOAD};${WORKLOAD};${WORKLOAD};${WORKLOAD}"

echo "Running simulation..."
echo ""

# Run with timeout of 3 minutes
timeout 180 "$GEM5_BIN" \
    --outdir="$OUTPUT_DIR" \
    "$CONFIG_SCRIPT" \
    --cmd="$CPU_LIST" \
    --cpu-type=TimingSimpleCPU \
    --num-cpus=4 \
    --mem-size=512MB \
    --caches \
    2>&1 | tee "${OUTPUT_DIR}/output.log"

exit_code=$?

echo ""
echo "=================================================="

if [ $exit_code -eq 0 ]; then
    echo "SIMULATION COMPLETED SUCCESSFULLY"
    echo ""
    
    # Check for scheduler output
    if grep -q "NOVA Scheduler" "${OUTPUT_DIR}/output.log" 2>/dev/null; then
        echo "Scheduler Statistics:"
        grep -A 20 "NOVA Scheduler Statistics" "${OUTPUT_DIR}/output.log" 2>/dev/null || echo "  (Scheduler was used)"
    else
        echo "Note: Scheduler statistics not found in output"
    fi
    
    # Check contention stats
    echo ""
    echo "Contention Statistics:"
    grep -A 10 "Shared Accelerator Statistics" "${OUTPUT_DIR}/output.log" 2>/dev/null | head -15 || echo "  (Check output.log for details)"
    
    # Show stats file size
    if [ -f "${OUTPUT_DIR}/stats.txt" ]; then
        echo ""
        echo "Stats file: $(ls -lh ${OUTPUT_DIR}/stats.txt | awk '{print $5}')"
    fi
else
    echo "SIMULATION FAILED (exit code: $exit_code)"
    
    if [ $exit_code -eq 124 ]; then
        echo "Reason: Timeout after 3 minutes"
    fi
fi

echo "=================================================="
echo "Output directory: $OUTPUT_DIR"
echo "=================================================="
