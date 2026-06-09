#!/usr/bin/env python3
"""
NOVA Scheduler Test Script
==========================
Tests the Mission-Aware Scheduling Framework with various configurations.

PhD Research: Spacecraft Multicore Accelerator Study

Features:
- No timeout (simulations run to completion)
- Up to 8 parallel simulations
- Skips problematic workloads
"""

import os
import sys
import subprocess
import time
import json
import csv
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading

# Configuration
GEM5_ROOT = "/data/home/chandraboul/development/gem5-matrix/gem5-nova"
GEM5_BIN = f"{GEM5_ROOT}/build/RISCV/gem5.opt"
CONFIG_SCRIPT = f"{GEM5_ROOT}/configs/deprecated/example/se.py"
WORKLOAD_DIR = f"{GEM5_ROOT}/new_spacecraft_workloads"
OUTPUT_BASE = f"{GEM5_ROOT}/scheduler_test_results"

# Maximum parallel simulations
MAX_PARALLEL = 8

# Workloads known to have issues (skip these)
SKIP_WORKLOADS = [
    "advanced-spacecraft-anomaly-detection",
    "mars-helicopter-control",
    "optimized-spacecraft-formation-control"
]

# Mission phases (matching NOVAMissionPhase enum)
PHASES = {
    0: "LAUNCH",
    1: "CRUISE",
    2: "APPROACH",
    3: "LANDING",
    4: "SURFACE_OPS",
    5: "ECLIPSE",
    6: "SAFE_MODE",
    7: "IDLE"
}

# Scheduling policies
POLICIES = {
    0: "FIFO",
    1: "PRIORITY",
    2: "EDF",
    3: "RATE_MONOTONIC",
    4: "PHASE_AWARE",
    5: "POWER_AWARE",
    6: "HYBRID"
}

# Phase-specific accelerator configurations
PHASE_CONFIGS = {
    "LAUNCH": (4, 4, 2, 1),    # High ADCS
    "CRUISE": (1, 1, 0, 0),    # Minimal
    "APPROACH": (3, 3, 2, 2),  # Navigation
    "LANDING": (4, 4, 1, 1),   # Control
    "SURFACE_OPS": (2, 3, 2, 2), # Science
    "ECLIPSE": (2, 2, 1, 0),   # Limited power
    "SAFE_MODE": (1, 1, 0, 0)  # Critical only
}

# Lock for thread-safe printing
print_lock = threading.Lock()

def safe_print(msg):
    """Thread-safe print."""
    with print_lock:
        print(msg, flush=True)


def find_workloads():
    """Find all RISC-V workloads in the workload directory, excluding problematic ones."""
    workloads = []
    if os.path.exists(WORKLOAD_DIR):
        for f in sorted(os.listdir(WORKLOAD_DIR)):
            if f.endswith('.riscv'):
                # Check if this workload should be skipped
                skip = False
                for pattern in SKIP_WORKLOADS:
                    if pattern in f:
                        skip = True
                        break
                if not skip:
                    workloads.append(os.path.join(WORKLOAD_DIR, f))
    return workloads


def run_simulation(workload, cores, phase, trig, mat, vpu, npu, output_dir):
    """Run a gem5 simulation with specified configuration. No timeout."""
    
    os.makedirs(output_dir, exist_ok=True)
    
    # Set environment variables
    env = os.environ.copy()
    env["NOVA_NUM_TRIG_ACCELS"] = str(trig)
    env["NOVA_NUM_MAT_ACCELS"] = str(mat)
    env["NOVA_NUM_VPU_ACCELS"] = str(vpu)
    env["NOVA_NUM_NPU_ACCELS"] = str(npu)
    env["NOVA_ACCEL_MODE"] = "shared"
    env["NOVA_MISSION_PHASE"] = phase
    env["NOVA_SCHEDULER_ENABLED"] = "1"
    
    # Build command
    cpu_list = ";".join([workload] * cores)
    
    cmd = [
        GEM5_BIN,
        f"--outdir={output_dir}",
        CONFIG_SCRIPT,
        f"--cmd={cpu_list}",
        "--cpu-type=TimingSimpleCPU",
        f"--num-cpus={cores}",
        "--mem-size=512MB",
        "--caches"
    ]
    
    # Run simulation (no timeout)
    stdout_file = os.path.join(output_dir, "stdout.log")
    stderr_file = os.path.join(output_dir, "stderr.log")
    
    try:
        with open(stdout_file, 'w') as out, open(stderr_file, 'w') as err:
            result = subprocess.run(
                cmd,
                env=env,
                stdout=out,
                stderr=err
                # No timeout - runs to completion
            )
            return result.returncode == 0
    except Exception as e:
        safe_print(f"  ERROR: {e}")
        return False


def parse_stats(stats_file):
    """Parse gem5 stats.txt file for scheduler-related metrics."""
    
    metrics = {
        "sim_ticks": 0,
        "sim_insts": 0,
        "trig_requests": 0,
        "mat_requests": 0,
        "trig_queued": 0,
        "mat_queued": 0,
        "trig_avg_depth": 0.0,
        "mat_avg_depth": 0.0,
        "trig_accels_needed": 0,
        "mat_accels_needed": 0,
        "total_stall_ticks": 0
    }
    
    if not os.path.exists(stats_file) or os.path.getsize(stats_file) == 0:
        return metrics
    
    try:
        with open(stats_file, 'r') as f:
            for line in f:
                if "simTicks" in line:
                    parts = line.split()
                    if len(parts) >= 2:
                        metrics["sim_ticks"] = int(parts[1])
                elif "simInsts" in line:
                    parts = line.split()
                    if len(parts) >= 2:
                        metrics["sim_insts"] = int(parts[1])
    except Exception:
        pass
    
    return metrics


def parse_contention_output(log_file):
    """Parse contention statistics from stdout log."""
    
    metrics = {
        "trig_requests": 0,
        "trig_queued": 0,
        "trig_avg_depth": 0.0,
        "trig_accels_needed": 0,
        "mat_requests": 0,
        "mat_queued": 0,
        "mat_avg_depth": 0.0,
        "mat_accels_needed": 0,
        "total_stall_ticks": 0
    }
    
    if not os.path.exists(log_file):
        return metrics
    
    try:
        with open(log_file, 'r') as f:
            content = f.read()
            
            import re
            
            # Parse TrigAccel stats
            match = re.search(r"TrigAccel Statistics:.*?Requests: total=(\d+), queued=(\d+)", 
                            content, re.DOTALL)
            if match:
                metrics["trig_requests"] = int(match.group(1))
                metrics["trig_queued"] = int(match.group(2))
            
            match = re.search(r"TrigAccel.*?Average Queue Depth: ([\d.]+)", content, re.DOTALL)
            if match:
                metrics["trig_avg_depth"] = float(match.group(1))
            
            match = re.search(r"TrigAccel.*?ACCELERATORS NEEDED: (\d+)", content, re.DOTALL)
            if match:
                metrics["trig_accels_needed"] = int(match.group(1))
            
            # Parse MatAccel stats
            match = re.search(r"MatAccel Statistics:.*?Requests: total=(\d+), queued=(\d+)", 
                            content, re.DOTALL)
            if match:
                metrics["mat_requests"] = int(match.group(1))
                metrics["mat_queued"] = int(match.group(2))
            
            match = re.search(r"MatAccel.*?Average Queue Depth: ([\d.]+)", content, re.DOTALL)
            if match:
                metrics["mat_avg_depth"] = float(match.group(1))
            
            match = re.search(r"MatAccel.*?ACCELERATORS NEEDED: (\d+)", content, re.DOTALL)
            if match:
                metrics["mat_accels_needed"] = int(match.group(1))
            
            # Parse total stall
            match = re.search(r"Total Stall Ticks from Contention: (\d+)", content)
            if match:
                metrics["total_stall_ticks"] = int(match.group(1))
    except Exception:
        pass
    
    return metrics


def run_single_test(args):
    """Run a single test - used for parallel execution."""
    test_type, workload, cores, phase, trig, mat, vpu, npu, output_dir = args
    
    wl_name = os.path.basename(workload).replace('.riscv', '')[:35]
    
    start_time = time.time()
    success = run_simulation(workload, cores, phase, trig, mat, vpu, npu, output_dir)
    elapsed = time.time() - start_time
    
    result = {
        "test_type": test_type,
        "phase": phase,
        "workload": wl_name,
        "cores": cores,
        "trig": trig,
        "mat": mat,
        "vpu": vpu,
        "npu": npu,
        "elapsed_sec": elapsed,
        "success": success
    }
    
    if success:
        stats = parse_stats(os.path.join(output_dir, "stats.txt"))
        contention = parse_contention_output(os.path.join(output_dir, "stdout.log"))
        result.update(stats)
        result.update(contention)
        safe_print(f"  [DONE] {test_type}/{wl_name}/{phase}/{cores}cores: {elapsed:.1f}s, {stats['sim_insts']} insts")
    else:
        safe_print(f"  [FAIL] {test_type}/{wl_name}/{phase}/{cores}cores")
    
    return result


def run_all_tests_parallel(workloads):
    """Run all tests in parallel with up to MAX_PARALLEL concurrent simulations."""
    
    # Build list of all tests to run
    all_tests = []
    
    # Test 1: Phase comparison (all phases, first 5 workloads, 4 cores)
    test_workloads = workloads[:5] if len(workloads) >= 5 else workloads
    for phase_name, (trig, mat, vpu, npu) in PHASE_CONFIGS.items():
        for workload in test_workloads:
            wl_name = os.path.basename(workload).replace('.riscv', '')[:35]
            output_dir = os.path.join(OUTPUT_BASE, "phase_test", 
                                     f"{phase_name}_{wl_name}_4cores")
            all_tests.append(("phase", workload, 4, phase_name, trig, mat, vpu, npu, output_dir))
    
    # Test 2: Core scaling (first workload, LAUNCH phase, 1/2/4/8 cores)
    if workloads:
        workload = workloads[0]
        wl_name = os.path.basename(workload).replace('.riscv', '')[:35]
        for cores in [1, 2, 4, 8]:
            trig = min(cores, 4)
            mat = min(cores, 4)
            vpu = min(cores // 2, 2)
            npu = min(cores // 4, 1)
            output_dir = os.path.join(OUTPUT_BASE, "scaling_test", 
                                     f"{wl_name}_{cores}cores")
            all_tests.append(("scaling", workload, cores, "LAUNCH", trig, mat, vpu, npu, output_dir))
    
    # Test 3: Accelerator optimization (first workload, 4 cores, varying configs)
    if workloads:
        workload = workloads[0]
        wl_name = os.path.basename(workload).replace('.riscv', '')[:35]
        configs = [(1, 1, 1, 1), (2, 2, 1, 1), (4, 4, 2, 2)]
        for trig, mat, vpu, npu in configs:
            output_dir = os.path.join(OUTPUT_BASE, "optimization_test", 
                                     f"t{trig}m{mat}v{vpu}n{npu}_4cores")
            all_tests.append(("optimization", workload, 4, "LAUNCH", trig, mat, vpu, npu, output_dir))
    
    print(f"\nTotal tests to run: {len(all_tests)}")
    print(f"Running up to {MAX_PARALLEL} simulations in parallel...")
    print(f"Note: No timeout - simulations run to completion\n")
    
    # Run tests in parallel
    results = []
    with ThreadPoolExecutor(max_workers=MAX_PARALLEL) as executor:
        futures = {executor.submit(run_single_test, test): test for test in all_tests}
        
        completed = 0
        for future in as_completed(futures):
            completed += 1
            result = future.result()
            results.append(result)
            
            # Progress update every 10 completions
            if completed % 10 == 0:
                safe_print(f"  Progress: {completed}/{len(all_tests)} completed")
    
    return results


def generate_report(results):
    """Generate analysis report."""
    
    report_file = os.path.join(OUTPUT_BASE, "scheduler_test_report.txt")
    
    # Separate results by test type
    phase_results = [r for r in results if r.get("test_type") == "phase"]
    scaling_results = [r for r in results if r.get("test_type") == "scaling"]
    optimization_results = [r for r in results if r.get("test_type") == "optimization"]
    
    with open(report_file, 'w') as f:
        f.write("=" * 70 + "\n")
        f.write("NOVA SCHEDULER TEST REPORT\n")
        f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write("=" * 70 + "\n\n")
        
        # Summary
        total = len(results)
        success = sum(1 for r in results if r.get("success"))
        f.write(f"SUMMARY: {success}/{total} tests passed\n\n")
        
        # Phase comparison results
        f.write("1. MISSION PHASE COMPARISON\n")
        f.write("-" * 40 + "\n")
        
        phase_summary = {}
        for r in phase_results:
            if r.get("success"):
                phase = r["phase"]
                if phase not in phase_summary:
                    phase_summary[phase] = {
                        "count": 0, 
                        "total_ticks": 0,
                        "total_stall": 0,
                        "avg_trig_depth": 0,
                        "avg_mat_depth": 0
                    }
                phase_summary[phase]["count"] += 1
                phase_summary[phase]["total_ticks"] += r.get("sim_ticks", 0)
                phase_summary[phase]["total_stall"] += r.get("total_stall_ticks", 0)
                phase_summary[phase]["avg_trig_depth"] += r.get("trig_avg_depth", 0)
                phase_summary[phase]["avg_mat_depth"] += r.get("mat_avg_depth", 0)
        
        for phase, data in phase_summary.items():
            if data["count"] > 0:
                avg_ticks = data["total_ticks"] / data["count"]
                avg_stall = data["total_stall"] / data["count"]
                avg_trig = data["avg_trig_depth"] / data["count"]
                avg_mat = data["avg_mat_depth"] / data["count"]
                f.write(f"  {phase}:\n")
                f.write(f"    Simulations: {data['count']}\n")
                f.write(f"    Avg Sim Ticks: {avg_ticks:.0f}\n")
                f.write(f"    Avg Stall Ticks: {avg_stall:.0f}\n")
                f.write(f"    Avg Trig Queue Depth: {avg_trig:.4f}\n")
                f.write(f"    Avg Mat Queue Depth: {avg_mat:.4f}\n")
        
        f.write("\n")
        
        # Core scaling results
        f.write("2. CORE SCALING\n")
        f.write("-" * 40 + "\n")
        
        for r in sorted(scaling_results, key=lambda x: x.get("cores", 0)):
            if r.get("success"):
                f.write(f"  {r['cores']} cores:\n")
                f.write(f"    Sim Ticks: {r.get('sim_ticks', 0)}\n")
                f.write(f"    Sim Insts: {r.get('sim_insts', 0)}\n")
                f.write(f"    Trig Accels Needed: {r.get('trig_accels_needed', 0)}\n")
                f.write(f"    Mat Accels Needed: {r.get('mat_accels_needed', 0)}\n")
        
        f.write("\n")
        
        # Optimization results
        f.write("3. ACCELERATOR OPTIMIZATION\n")
        f.write("-" * 40 + "\n")
        
        for r in optimization_results:
            if r.get("success"):
                config = f"T{r['trig']}M{r['mat']}V{r['vpu']}N{r['npu']}"
                f.write(f"  {config}:\n")
                f.write(f"    Sim Ticks: {r.get('sim_ticks', 0)}\n")
                f.write(f"    Total Stall: {r.get('total_stall_ticks', 0)}\n")
                f.write(f"    Trig Queue Depth: {r.get('trig_avg_depth', 0):.4f}\n")
                f.write(f"    Mat Queue Depth: {r.get('mat_avg_depth', 0):.4f}\n")
        
        f.write("\n")
        f.write("=" * 70 + "\n")
    
    print(f"\nReport saved to: {report_file}")
    
    # Also save as CSV
    csv_file = os.path.join(OUTPUT_BASE, "scheduler_test_results.csv")
    with open(csv_file, 'w', newline='') as f:
        fieldnames = [
            "test_type", "phase", "workload", "cores", "trig", "mat", "vpu", "npu",
            "sim_ticks", "sim_insts", "trig_requests", "trig_queued", "trig_avg_depth",
            "trig_accels_needed", "mat_requests", "mat_queued", "mat_avg_depth",
            "mat_accels_needed", "total_stall_ticks", "elapsed_sec", "success"
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
        writer.writeheader()
        
        for r in results:
            writer.writerow(r)
    
    print(f"CSV saved to: {csv_file}")


def main():
    """Main test runner."""
    
    print("=" * 70)
    print("NOVA SCHEDULER TEST SUITE (Parallel Edition)")
    print("=" * 70)
    print(f"gem5 Binary: {GEM5_BIN}")
    print(f"Output Directory: {OUTPUT_BASE}")
    print(f"Max Parallel Jobs: {MAX_PARALLEL}")
    print(f"Timeout: NONE (runs to completion)")
    print(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # Check prerequisites
    if not os.path.exists(GEM5_BIN):
        print(f"ERROR: gem5 binary not found: {GEM5_BIN}")
        sys.exit(1)
    
    # Create output directory
    os.makedirs(OUTPUT_BASE, exist_ok=True)
    
    # Find workloads
    workloads = find_workloads()
    print(f"\nFound {len(workloads)} workloads (excluding problematic ones)")
    
    if not workloads:
        print("ERROR: No workloads found!")
        sys.exit(1)
    
    # Run all tests in parallel
    results = run_all_tests_parallel(workloads)
    
    # Generate report
    generate_report(results)
    
    # Print summary
    success = sum(1 for r in results if r.get("success"))
    print("\n" + "=" * 70)
    print(f"ALL TESTS COMPLETED: {success}/{len(results)} passed")
    print("=" * 70)


if __name__ == "__main__":
    main()
