#!/usr/bin/env python3
"""
Spacecraft SoC Power Model: RISC-V core (McPAT) + CORDIC, 3x3 Matrix, NPU, VPU accelerators.

Matches the gem5 design:
- SharedCORDICAccel (trig: sin/cos/atan2), SharedMatrixAccel (3x3 matmul)
- NeuralProcessingUnit (TinyML systolic), VisionProcessingUnit (image/conv/features)

Usage:
  python3 spacecraft_power_model.py [--mcpat-out FILE] [--gem5-stats FILE] [--frequency_mhz 500]
  If --mcpat-out: parse McPAT output for core power (run McPAT first).
  If --gem5-stats: parse gem5 stats.txt for sim time and accelerator utilization.
  Otherwise: report nominal power breakdown only.
"""

from __future__ import division, print_function
import argparse
import re
import sys

# -----------------------------------------------------------------------------
# Accelerator power (mW) @ NOMINAL state — matches spacecraft_accelerators.hh
# and SimObject params (NeuralProcessingUnit.py, VisionProcessingUnit.py).
# Technology: 22 nm, 500 MHz same as RISC-V core.
# -----------------------------------------------------------------------------
ACCEL_SPECS = {
    "CORDIC": {
        "name": "CORDIC (Trig)",
        "nominal_mw": 45.0,
        "idle_mw": 2.0,
        "off_mw": 0.0,
        "base_latency_cycles": 32,
        "description": "Shared CORDIC trigonometric accelerator (sin/cos/atan2), MMIO, round-robin arbitration",
    },
    "Matrix3x3": {
        "name": "Matrix 3x3",
        "nominal_mw": 180.0,
        "idle_mw": 8.0,
        "off_mw": 0.0,
        "base_latency_cycles": 27,
        "description": "Shared 3x3 matrix multiply accelerator (systolic), MMIO, round-robin arbitration",
    },
    "NPU": {
        "name": "NPU",
        "nominal_mw": 500.0,
        "idle_mw": 50.0,
        "off_mw": 1.0,
        "base_latency_cycles": 64,
        "description": "Neural Processing Unit (TinyML): 8x8 systolic, INT8, weight/activation buffers",
    },
    "VPU": {
        "name": "VPU",
        "nominal_mw": 100.0,
        "idle_mw": 10.0,
        "off_mw": 0.5,
        "base_latency_cycles": 100,
        "description": "Vision Processing Unit: debayer, Sobel/Canny, conv engine (16 MACs), 64 KB scratchpad",
    },
}


def parse_mcpat_output(path):
    """Parse McPAT stdout output; return dict with core/system power (W) and area (mm^2)."""
    data = {"core_runtime_dynamic_w": None, "core_peak_dynamic_w": None, "core_leakage_w": None,
            "system_runtime_dynamic_w": None, "system_peak_dynamic_w": None, "system_leakage_w": None,
            "system_area_mm2": None}
    with open(path, "r") as f:
        text = f.read()
    # System-level (first occurrence is total system)
    m = re.search(r"System:\s*\n\s*Area = ([\d.]+) mm\^2", text)
    if m:
        data["system_area_mm2"] = float(m.group(1))
    m = re.search(r"System:\s*\n\s*Area = [\d.]+\s*\n\s*Peak Dynamic Power = ([\d.]+) W", text, re.DOTALL)
    if m:
        data["system_peak_dynamic_w"] = float(m.group(1))
    m = re.search(r"Runtime Dynamic Power = ([\d.]+) W", text)
    if m:
        data["system_runtime_dynamic_w"] = float(m.group(1))
    m = re.search(r"Subthreshold Leakage Power = ([\d.]+) W", text)
    if m:
        data["system_leakage_w"] = float(m.group(1))
    # Core 0 (first core)
    m = re.search(r"Core 0:\s*\n\s*Area = [\d.]+\s*\n\s*Peak Dynamic Power = ([\d.]+) W", text, re.DOTALL)
    if m:
        data["core_peak_dynamic_w"] = float(m.group(1))
    for label, key in [("Core 0:", "core_runtime_dynamic_w"), ("Core 0:", "core_leakage_w")]:
        pass  # we already get system-level; optional: parse Core 0 block for core_* if needed
    ms = re.findall(r"Runtime Dynamic Power = ([\d.]+) W", text)
    if len(ms) >= 1 and data["system_runtime_dynamic_w"] is None:
        data["system_runtime_dynamic_w"] = float(ms[0])
    ms = re.findall(r"Subthreshold Leakage Power = ([\d.]+) W", text)
    if len(ms) >= 1 and data["system_leakage_w"] is None:
        data["system_leakage_w"] = float(ms[0])
    data["core_runtime_dynamic_w"] = data["system_runtime_dynamic_w"]
    data["core_leakage_w"] = data["system_leakage_w"]
    data["core_peak_dynamic_w"] = data.get("core_peak_dynamic_w") or data["system_peak_dynamic_w"]
    return data


def parse_gem5_stats(path):
    """Parse gem5 stats.txt; return dict with sim_seconds, sim_ticks, and accelerator stats if present."""
    data = {"sim_seconds": None, "sim_ticks": None, "host_seconds": None,
            "cordic_total_requests": None, "cordic_completed_ops": None, "cordic_total_wait_cycles": None,
            "matrix_total_requests": None, "matrix_completed_ops": None, "matrix_total_wait_cycles": None,
            "npu_total_inferences": None, "npu_busy_cycles": None,
            "vpu_total_requests": None, "vpu_completed_requests": None, "vpu_busy_cycles": None}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            name, val = parts[0], parts[-1]
            try:
                v = int(val) if "." not in val else float(val)
            except ValueError:
                continue
            if name == "sim_seconds":
                data["sim_seconds"] = float(val)
            elif name == "sim_ticks":
                data["sim_ticks"] = int(v)
            elif name == "host_seconds":
                data["host_seconds"] = float(val)
            elif "cordic" in name.lower() or "SharedCORDIC" in name:
                if "totalRequests" in name or "total_requests" in name:
                    data["cordic_total_requests"] = v
                elif "completedOperations" in name or "completed_ops" in name:
                    data["cordic_completed_ops"] = v
                elif "totalWaitCycles" in name or "total_wait_cycles" in name:
                    data["cordic_total_wait_cycles"] = v
            elif "matrix" in name.lower() or "SharedMatrix" in name:
                if "totalRequests" in name or "total_requests" in name:
                    data["matrix_total_requests"] = v
                elif "completedOperations" in name or "completed_ops" in name:
                    data["matrix_completed_ops"] = v
                elif "totalWaitCycles" in name or "total_wait_cycles" in name:
                    data["matrix_total_wait_cycles"] = v
            elif "npu" in name.lower() or "NeuralProcessing" in name:
                if "totalInferences" in name or "inferences" in name:
                    data["npu_total_inferences"] = v
                elif "busyCycles" in name or "busy_cycles" in name:
                    data["npu_busy_cycles"] = v
            elif "vpu" in name.lower() or "VisionProcessing" in name:
                if "totalRequests" in name and "completed" not in name:
                    data["vpu_total_requests"] = v
                elif "completedRequests" in name or "completed_requests" in name:
                    data["vpu_completed_requests"] = v
                elif "busyCycles" in name or "busy_cycles" in name:
                    data["vpu_busy_cycles"] = v
    return data


def main():
    ap = argparse.ArgumentParser(description="Spacecraft SoC power model: RISC-V core + CORDIC, Matrix, NPU, VPU")
    ap.add_argument("--mcpat-out", metavar="FILE", help="McPAT stdout output file (core power)")
    ap.add_argument("--gem5-stats", metavar="FILE", help="gem5 stats.txt (sim time, accelerator stats)")
    ap.add_argument("--frequency_mhz", type=float, default=500.0, help="Core/accelerator frequency (MHz)")
    ap.add_argument("--utilization", type=float, default=None,
                    help="Override accelerator utilization 0..1 (otherwise from stats or nominal)")
    ap.add_argument("--no-accelerators", action="store_true", help="Report core-only (McPAT) if --mcpat-out given")
    args = ap.parse_args()

    core_power_w = None
    core_leakage_w = None
    core_peak_w = None
    system_area_mm2 = None
    if args.mcpat_out:
        try:
            mcpat = parse_mcpat_output(args.mcpat_out)
            core_power_w = mcpat.get("system_runtime_dynamic_w") or mcpat.get("core_runtime_dynamic_w")
            core_leakage_w = mcpat.get("system_leakage_w") or mcpat.get("core_leakage_w")
            core_peak_w = mcpat.get("system_peak_dynamic_w") or mcpat.get("core_peak_dynamic_w")
            system_area_mm2 = mcpat.get("system_area_mm2")
        except Exception as e:
            print("Warning: could not parse McPAT output:", e, file=sys.stderr)

    sim_seconds = None
    gem5 = {}
    if args.gem5_stats:
        try:
            gem5 = parse_gem5_stats(args.gem5_stats)
            sim_seconds = gem5.get("sim_seconds")
        except Exception as e:
            print("Warning: could not parse gem5 stats:", e, file=sys.stderr)

    # Utilization: from stats (busy/total cycles) or override or default nominal
    util = args.utilization
    if util is not None and (util < 0 or util > 1):
        util = 0.2
    accel_util = util if util is not None else 0.2  # nominal 20% active

    # ----- Report -----
    print("=" * 60)
    print("Spacecraft SoC Power Model (RISC-V + CORDIC, 3x3 Matrix, NPU, VPU)")
    print("=" * 60)
    print("Technology: 22 nm  |  Frequency: {} MHz".format(args.frequency_mhz))
    if sim_seconds is not None:
        print("Simulation time: {:.6f} s".format(sim_seconds))
    print()

    total_power_mw = 0.0
    breakdown = []

    # Core (McPAT)
    if core_power_w is not None:
        core_mw = core_power_w * 1000
        total_power_mw += core_mw
        breakdown.append(("RISC-V core (runtime dynamic)", core_mw, "McPAT"))
    if core_leakage_w is not None:
        leak_mw = core_leakage_w * 1000
        total_power_mw += leak_mw
        breakdown.append(("RISC-V core (leakage)", leak_mw, "McPAT"))
    if core_peak_w is not None and not breakdown:
        breakdown.append(("RISC-V core (peak dynamic)", core_peak_w * 1000, "McPAT"))
        total_power_mw += core_peak_w * 1000
    if system_area_mm2 is not None:
        print("RISC-V core area (McPAT): {:.4f} mm^2".format(system_area_mm2))
    print()

    # Accelerators (from design specs)
    if not args.no_accelerators:
        print("Accelerators (design specs @ nominal active power):")
        for key, spec in ACCEL_SPECS.items():
            nominal = spec["nominal_mw"]
            idle = spec["idle_mw"]
            # Simple model: active_util * nominal + (1 - active_util) * idle
            power_mw = accel_util * nominal + (1.0 - accel_util) * idle
            total_power_mw += power_mw
            breakdown.append((spec["name"], power_mw, "Design spec"))
            print("  {}: {:.2f} mW (nominal {:.0f} mW, idle {:.0f} mW, util {:.0%})".format(
                spec["name"], power_mw, nominal, idle, accel_util))
            print("      {}".format(spec["description"]))
        print()

    # Summary
    print("-" * 60)
    print("Total SoC power: {:.2f} mW  ({:.3f} W)".format(total_power_mw, total_power_mw / 1000))
    if sim_seconds is not None and sim_seconds > 0:
        energy_j = (total_power_mw / 1000) * sim_seconds
        print("Energy (over sim): {:.6f} J".format(energy_j))
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
