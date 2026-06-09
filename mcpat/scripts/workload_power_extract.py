#!/usr/bin/env python3
"""
Workload Power Extraction Script
Extracts gem5 stats and computes per-accelerator power for a workload.

Usage:
  python3 scripts/workload_power_extract.py m5out/stats.txt
  python3 scripts/workload_power_extract.py m5out/stats.txt --freq 500
"""
from __future__ import division, print_function
import argparse, os, sys

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

ACCEL_SPECS = {
    "CORDIC": {"name": "CORDIC Trig Accelerator", "nominal_mw": 45.0, "idle_mw": 2.0, "off_mw": 0.0, "latency_cycles": 32, "prefixes": ["cordic", "SharedCORDIC"]},
    "Matrix3x3": {"name": "3x3 Matrix Accelerator", "nominal_mw": 180.0, "idle_mw": 8.0, "off_mw": 0.0, "latency_cycles": 27, "prefixes": ["matrix", "SharedMatrix"]},
    "NPU": {"name": "NPU (Neural Processing)", "nominal_mw": 500.0, "idle_mw": 50.0, "off_mw": 1.0, "latency_cycles": 64, "prefixes": ["npu", "NeuralProcessing"]},
    "VPU": {"name": "VPU (Vision Processing)", "nominal_mw": 100.0, "idle_mw": 10.0, "off_mw": 0.5, "latency_cycles": 100, "prefixes": ["vpu", "VisionProcessing"]},
}

def parse_stats(path):
    stats = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("---"):
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                stats[parts[0]] = float(parts[1]) if "." in parts[1] else int(parts[1])
            except ValueError:
                stats[parts[0]] = parts[1]
    return stats

def find_stat(stats, keywords):
    for key, val in stats.items():
        if all(kw.lower() in key.lower() for kw in keywords):
            return val
    return None

def get_accel_util(stats, spec, freq_mhz, sim_s):
    total_cyc = sim_s * freq_mhz * 1e6 if sim_s else None
    r = {"requests": 0, "ops": 0, "busy_cyc": 0, "wait_cyc": 0, "util": 0.0}
    for px in spec["prefixes"]:
        for kw in ["totalRequests", "total_requests"]:
            v = find_stat(stats, [px, kw])
            if v is not None: r["requests"] = int(v); break
        for kw in ["completedOp", "completed_ops", "totalInferences", "completedRequests"]:
            v = find_stat(stats, [px, kw])
            if v is not None: r["ops"] = int(v); break
        for kw in ["busyCycles", "busy_cycles"]:
            v = find_stat(stats, [px, kw])
            if v is not None: r["busy_cyc"] = int(v); break
        for kw in ["totalWaitCycles", "total_wait_cycles"]:
            v = find_stat(stats, [px, kw])
            if v is not None: r["wait_cyc"] = int(v); break
    if r["busy_cyc"] > 0 and total_cyc:
        r["util"] = r["busy_cyc"] / total_cyc
    elif r["ops"] > 0 and total_cyc:
        r["util"] = min(r["ops"] * spec["latency_cycles"] / total_cyc, 1.0)
        r["busy_cyc"] = int(r["ops"] * spec["latency_cycles"])
    return r

def fmt_p(mw):
    return "{:.4f} W".format(mw/1000) if mw >= 1000 else "{:.4f} mW".format(mw)

def main():
    ap = argparse.ArgumentParser(description="Per-accelerator workload power")
    ap.add_argument("stats_file", help="gem5 stats.txt")
    ap.add_argument("--freq", type=float, default=500.0, help="MHz")
    ap.add_argument("--core-power-w", type=float, default=1.52, help="Core power from McPAT (W)")
    args = ap.parse_args()

    if not os.path.isfile(args.stats_file):
        print("ERROR: not found: " + args.stats_file, file=sys.stderr)
        return 1

    stats = parse_stats(args.stats_file)
    sim_s = stats.get("sim_seconds")
    if sim_s is None and stats.get("sim_ticks") is not None:
        sim_s = stats["sim_ticks"] / 1e12

    print()
    print("=" * 80)
    print("  WORKLOAD POWER ANALYSIS -- Per-Accelerator Breakdown")
    print("=" * 80)
    print("  Stats: {}".format(args.stats_file))
    print("  Freq:  {} MHz".format(args.freq))
    if sim_s is not None:
        print("  Time:  {:.6f} s  ({:.0f} cycles)".format(sim_s, sim_s * args.freq * 1e6))
    print()

    total_mw = args.core_power_w * 1000
    results = []
    hdr = "  {:<35s} {:>7s} {:>8s} {:>10s} {:>10s}"
    print(hdr.format("Accelerator", "Ops", "Util%", "Active", "Power"))
    print("  " + "-" * 75)

    for key, spec in ACCEL_SPECS.items():
        u = get_accel_util(stats, spec, args.freq, sim_s)
        util = min(max(u["util"], 0.0), 1.0)
        pwr_mw = util * spec["nominal_mw"] + (1.0 - util) * spec["idle_mw"]
        total_mw += pwr_mw
        results.append({"key": key, "spec": spec, "u": u, "util": util, "pwr_mw": pwr_mw})
        at = "{:.6f}s".format(util * sim_s) if sim_s else "N/A"
        print(hdr.format(spec["name"], str(u["ops"]), "{:.2f}%".format(util*100), at, fmt_p(pwr_mw)))

    print("  " + "-" * 75)
    print()

    # Detailed per-accelerator
    for r in results:
        sp, u, ut, pw = r["spec"], r["u"], r["util"], r["pwr_mw"]
        print("  --- {} ---".format(sp["name"]))
        print("    Requests:     {}".format(u["requests"]))
        print("    Completed:    {}".format(u["ops"]))
        print("    Busy cycles:  {}".format(u["busy_cyc"]))
        print("    Wait cycles:  {}".format(u["wait_cyc"]))
        print("    Utilization:  {:.4f} ({:.2f}%)".format(ut, ut*100))
        print("    Active power: {} (nom: {} mW)".format(fmt_p(ut * sp["nominal_mw"]), sp["nominal_mw"]))
        print("    Idle power:   {} (idle: {} mW)".format(fmt_p((1-ut) * sp["idle_mw"]), sp["idle_mw"]))
        print("    Total power:  {}".format(fmt_p(pw)))
        if sim_s and sim_s > 0:
            print("    Energy:       {:.9f} J".format(pw / 1000.0 * sim_s))
        print()

    # SoC summary
    print("  " + "=" * 60)
    print("  SoC POWER SUMMARY (workload-weighted)")
    print("  " + "=" * 60)
    print("    RISC-V Core (McPAT):    {}".format(fmt_p(args.core_power_w * 1000)))
    for r in results:
        print("    {:<26s} {}".format(r["spec"]["name"][:26] + ":", fmt_p(r["pwr_mw"])))
    print("    " + "-" * 40)
    print("    Total SoC:              {}".format(fmt_p(total_mw)))
    if sim_s and sim_s > 0:
        print("    Total energy:           {:.9f} J".format(total_mw / 1000.0 * sim_s))
    print()
    print("  TIP: For detailed McPAT internal breakdown, run:")
    print("    python3 scripts/spacecraft_soc_power_mcpat.py")
    print("=" * 80)
    return 0

if __name__ == "__main__":
    sys.exit(main())
