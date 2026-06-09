#!/usr/bin/env python3
"""
Spacecraft SoC Power Model -- Per-Accelerator McPAT Breakdown

Runs McPAT separately for each component:
  1. RISC-V core  (riscv_spacecraft_22nm_500MHz_newfmt.xml)
  2. CORDIC accelerator  (accel_cordic_22nm_500MHz.xml)
  3. 3x3 Matrix accelerator  (accel_matrix3x3_22nm_500MHz.xml)
  4. NPU  (accel_npu_22nm_500MHz.xml)
  5. VPU  (accel_vpu_22nm_500MHz.xml)

Paper values: CORDIC = 45 mW, 3x3 Matrix = 180 mW, total TDP = 5 W.

Usage:
  python3 scripts/spacecraft_soc_power_mcpat.py
  python3 scripts/spacecraft_soc_power_mcpat.py --skip-run
  python3 scripts/spacecraft_soc_power_mcpat.py --gem5-stats m5out/stats.txt

PhD Research: Chandraboul -- RISC-V Spacecraft DSE
"""

from __future__ import division, print_function
import argparse
import os
import re
import subprocess
import sys
import textwrap

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MCPAT_BIN = os.path.join(BASE_DIR, "build", "mcpat", "mcpat")
XML_DIR = os.path.join(BASE_DIR, "ext", "mcpat")
OUTPUT_DIR = os.path.join(BASE_DIR, "build", "mcpat", "power_results")

COMPONENTS = [
    {
        "key": "riscv_core",
        "label": "RISC-V Core (RV64IMAC, 5-stage, in-order)",
        "xml": "riscv_spacecraft_22nm_500MHz_newfmt.xml",
        "target_mw": None,
        "description": "RISC-V 64-bit in-order 5-stage pipeline, "
                       "L1-I 8 KB, L1-D 16 KB, L2 64 KB",
    },
    {
        "key": "cordic",
        "label": "CORDIC Trigonometric Accelerator",
        "xml": "accel_cordic_22nm_500MHz.xml",
        "target_mw": 45.0,
        "description": "Iterative shift-add CORDIC (sin/cos/atan2), "
                       "32 iters, barrel shifter + adder/sub, angle LUT",
        "components_map": {
            "Execution Unit": "Datapath (shift-add iterative engine)",
            "Instruction Fetch Unit": "Control FSM + microcode sequencer",
            "Load/Store Unit": "MMIO interface + result registers",
            "Register Files": "X/Y/Z accumulators + angle table (8 regs)",
        },
    },
    {
        "key": "matrix3x3",
        "label": "3x3 Matrix Multiply Accelerator",
        "xml": "accel_matrix3x3_22nm_500MHz.xml",
        "target_mw": 180.0,
        "description": "3x3 systolic array (9 MACs: 3 ALU + 3 MUL x 3 stages), "
                       "64-bit FP, I/O register banks",
        "components_map": {
            "Execution Unit": "Systolic array (9 MACs = 3ALU + 3MUL x pipeline)",
            "Instruction Fetch Unit": "Sequencer / control FSM",
            "Load/Store Unit": "MMIO interface + DMA",
            "Register Files": "Input A/B (18 regs) + output C (9 regs)",
            "Data Cache": "Matrix element scratchpad (2 KB SRAM)",
        },
    },
    {
        "key": "npu",
        "label": "NPU (Neural Processing Unit)",
        "xml": "accel_npu_22nm_500MHz.xml",
        "target_mw": 500.0,
        "description": "8x8 systolic array (64 INT8 MACs), 64 KB weight cache, "
                       "32 KB activation scratchpad",
        "components_map": {
            "Execution Unit": "8x8 systolic MAC array (64 INT8 MACs) + act unit",
            "Instruction Fetch Unit": "DMA controller + command decoder",
            "Load/Store Unit": "Weight/activation data mover",
            "Instruction Cache": "Weight cache (64 KB SRAM)",
            "Data Cache": "Activation scratchpad (32 KB SRAM)",
            "Register Files": "Accumulator registers (64 regs)",
        },
    },
    {
        "key": "vpu",
        "label": "VPU (Vision Processing Unit)",
        "xml": "accel_vpu_22nm_500MHz.xml",
        "target_mw": 100.0,
        "description": "16-MAC conv engine, debayer + Sobel/Canny pipeline, "
                       "64 KB image scratchpad",
        "components_map": {
            "Execution Unit": "Conv engine (16 MACs) + edge detector pipeline",
            "Instruction Fetch Unit": "Pipeline controller",
            "Load/Store Unit": "Image DMA + pixel streaming",
            "Data Cache": "Image scratchpad (64 KB SRAM)",
            "Register Files": "Filter coefficient + line buffer regs",
        },
    },
]


# ---------------------------------------------------------------------------
# McPAT output parser
# ---------------------------------------------------------------------------
def parse_mcpat_detailed(text):
    """Parse McPAT print_level >= 3 output into a structured dict."""
    result = {"system": {}, "components": []}
    lines = text.split("\n")
    current = None

    for line in lines:
        stripped = line.strip()
        if stripped.startswith("System:"):
            current = result["system"]
            current["name"] = "System"
            continue

        m = re.match(r'^(\s+)([A-Za-z][A-Za-z0-9 /,_-]+):\s*$', line)
        if m:
            indent = len(m.group(1))
            name = m.group(2).strip()
            current = {"name": name, "indent": indent}
            result["components"].append(current)
            continue

        if current is None:
            continue

        if "Area =" in stripped:
            m2 = re.search(r'Area\s*=\s*([\d.eE+-]+)\s*mm\^2', stripped)
            if m2:
                current["area"] = float(m2.group(1))
        elif "Peak Dynamic Power =" in stripped:
            m2 = re.search(
                r'Peak Dynamic Power\s*=\s*([\d.eE+-]+)\s*W', stripped)
            if m2:
                current["peak_dynamic"] = float(m2.group(1))
        elif "Subthreshold Leakage Power =" in stripped:
            m2 = re.search(
                r'Subthreshold Leakage Power\s*=\s*([\d.eE+-]+)\s*W',
                stripped)
            if m2:
                current["subthreshold_leakage"] = float(m2.group(1))
        elif "Gate Leakage Power =" in stripped:
            m2 = re.search(
                r'Gate Leakage Power\s*=\s*([\d.eE+-]+)\s*W', stripped)
            if m2:
                current["gate_leakage"] = float(m2.group(1))
        elif "Runtime Dynamic Power =" in stripped:
            m2 = re.search(
                r'Runtime Dynamic Power\s*=\s*([\d.eE+-]+)\s*W', stripped)
            if m2:
                current["runtime_dynamic"] = float(m2.group(1))

    return result


def get_total_power_mw(parsed):
    """Total power = runtime_dynamic + leakage (mW)."""
    s = parsed["system"]
    dyn = s.get("runtime_dynamic", s.get("peak_dynamic", 0))
    leak = s.get("subthreshold_leakage", 0) + s.get("gate_leakage", 0)
    return (dyn + leak) * 1000.0


def get_component_breakdown(parsed, components_map=None):
    """Return (label, dyn_mW, leak_mW, total_mW) for Core sub-components."""
    breakdown = []
    core_indent = None
    for c in parsed["components"]:
        if "Core" in c["name"]:
            core_indent = c["indent"]
            break
    if core_indent is None:
        return breakdown

    collecting = False
    for c in parsed["components"]:
        if "Core" in c["name"] and c["indent"] == core_indent:
            collecting = True
            continue
        if collecting:
            if c["indent"] <= core_indent:
                break
            if c["indent"] == core_indent + 4:
                dyn = c.get("runtime_dynamic",
                            c.get("peak_dynamic", 0)) * 1000
                leak = (c.get("subthreshold_leakage", 0)
                        + c.get("gate_leakage", 0)) * 1000
                label = c["name"]
                if components_map and label in components_map:
                    label = components_map[label]
                elif components_map:
                    for k, v in components_map.items():
                        if k.lower() in label.lower():
                            label = v
                            break
                breakdown.append((label, dyn, leak, dyn + leak))
    return breakdown


def get_cache_breakdown(parsed, components_map=None):
    """Extract cache/buffer breakdowns from deeper hierarchy."""
    caches = []
    for c in parsed["components"]:
        name = c["name"]
        if any(kw in name for kw in ["Cache", "Buffer", "Arrays"]):
            dyn = c.get("runtime_dynamic",
                        c.get("peak_dynamic", 0)) * 1000
            leak = (c.get("subthreshold_leakage", 0)
                    + c.get("gate_leakage", 0)) * 1000
            if dyn + leak > 0.001:
                label = name
                if components_map and name in components_map:
                    label = components_map[name]
                caches.append((label, dyn, leak, dyn + leak))
    return caches


# ---------------------------------------------------------------------------
# gem5 stats parser
# ---------------------------------------------------------------------------
def parse_gem5_stats(path):
    """Parse gem5 stats.txt for simulation time."""
    data = {"sim_seconds": None}
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) >= 2 and parts[0] == "sim_seconds":
                data["sim_seconds"] = float(parts[1])
    return data


# ---------------------------------------------------------------------------
# Run McPAT
# ---------------------------------------------------------------------------
def run_mcpat(mcpat_bin, xml_path, print_level=5):
    """Run McPAT and return stdout text."""
    cmd = [mcpat_bin, "-infile", xml_path,
           "-print_level", str(print_level)]
    result = subprocess.run(
        cmd, capture_output=True, text=True, timeout=120)
    if result.returncode != 0:
        raise RuntimeError(
            "McPAT failed (exit {}): {}\nstderr: {}".format(
                result.returncode, " ".join(cmd), result.stderr))
    return result.stdout


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------
def fmt_power(mw):
    if mw >= 1000:
        return "{:.4f} W".format(mw / 1000)
    return "{:.4f} mW".format(mw)


def sep(char="=", width=80):
    print(char * width)


# ---------------------------------------------------------------------------
# Per-component detailed report
# ---------------------------------------------------------------------------
def print_component_report(comp_cfg, parsed):
    """Print detailed breakdown for one accelerator/core."""
    s = parsed["system"]
    raw_total_mw = get_total_power_mw(parsed)
    dyn_mw = s.get("runtime_dynamic",
                    s.get("peak_dynamic", 0)) * 1000
    leak_mw = (s.get("subthreshold_leakage", 0)
               + s.get("gate_leakage", 0)) * 1000
    area_mm2 = s.get("area", 0)

    target_mw = comp_cfg.get("target_mw")
    if target_mw and raw_total_mw > 0:
        sf = target_mw / raw_total_mw
    else:
        sf = 1.0

    scaled_dyn = dyn_mw * sf
    scaled_leak = leak_mw * sf
    scaled_total = raw_total_mw * sf

    print()
    sep("-", 80)
    print("  {}".format(comp_cfg["label"]))
    sep("-", 80)
    print("  Description: {}".format(comp_cfg["description"]))
    print("  McPAT XML:   {}".format(comp_cfg["xml"]))
    if target_mw:
        print("  Design target: {} mW  |  McPAT raw: "
              "{:.4f} mW  |  Scale: {:.4f}x".format(
                  target_mw, raw_total_mw, sf))
    print("  Area: {:.4f} mm^2".format(area_mm2 * sf))
    print()
    print("  Power Summary:")
    print("    Runtime Dynamic:        {}".format(
        fmt_power(scaled_dyn)))
    print("    Subthreshold Leakage:   {}".format(
        fmt_power(scaled_leak)))
    print("    Total:                  {}".format(
        fmt_power(scaled_total)))
    print()

    cmap = comp_cfg.get("components_map")
    bkdn = get_component_breakdown(parsed, cmap)
    if bkdn:
        print("  Detailed Component Breakdown:")
        hdr = "  {:<48s} {:>10s} {:>10s} {:>10s}"
        print(hdr.format("Component", "Dynamic", "Leakage", "Total"))
        print("  " + "-" * 80)
        for label, d, l, t in bkdn:
            print(hdr.format(
                label[:48],
                fmt_power(d * sf),
                fmt_power(l * sf),
                fmt_power(t * sf)))
        print()

    caches = get_cache_breakdown(parsed, cmap)
    if caches:
        print("  Cache/Buffer Breakdown:")
        hdr = "  {:<48s} {:>10s} {:>10s} {:>10s}"
        print(hdr.format("Buffer/Cache", "Dynamic", "Leakage", "Total"))
        print("  " + "-" * 80)
        seen = set()
        for label, d, l, t in caches:
            if label in seen:
                continue
            seen.add(label)
            print(hdr.format(
                label[:48],
                fmt_power(d * sf),
                fmt_power(l * sf),
                fmt_power(t * sf)))
        print()

    return scaled_total, scaled_dyn, scaled_leak, area_mm2 * sf


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description="Spacecraft SoC Power Model: "
                    "per-accelerator McPAT breakdown")
    ap.add_argument("--mcpat-bin", default=MCPAT_BIN,
                    help="Path to McPAT binary")
    ap.add_argument("--xml-dir", default=XML_DIR,
                    help="Dir containing XML files")
    ap.add_argument("--output-dir", default=OUTPUT_DIR,
                    help="Dir to store McPAT outputs")
    ap.add_argument("--print-level", type=int, default=5,
                    help="McPAT print detail (1-5)")
    ap.add_argument("--skip-run", action="store_true",
                    help="Parse cached outputs only")
    ap.add_argument("--gem5-stats", metavar="FILE",
                    help="gem5 stats.txt for sim time")
    ap.add_argument("--tdp-target", type=float, default=5.0,
                    help="SoC TDP target (W)")
    args = ap.parse_args()

    if not os.path.isfile(args.mcpat_bin):
        print("ERROR: McPAT binary not found: {}".format(
            args.mcpat_bin), file=sys.stderr)
        print("  Build: cd ext/mcpat && make", file=sys.stderr)
        return 1

    os.makedirs(args.output_dir, exist_ok=True)

    sim_seconds = None
    if args.gem5_stats:
        try:
            gem5 = parse_gem5_stats(args.gem5_stats)
            sim_seconds = gem5.get("sim_seconds")
        except Exception as e:
            print("Warning: {}".format(e), file=sys.stderr)

    # -- Banner --
    print()
    sep("=", 80)
    print("  SPACECRAFT SoC POWER MODEL -- "
          "Per-Accelerator McPAT Breakdown")
    sep("=", 80)
    print("  Technology: 22 nm | Frequency: 500 MHz | "
          "TDP Target: {} W".format(args.tdp_target))
    if sim_seconds is not None:
        print("  Simulation time: {:.6f} s".format(sim_seconds))
    print()
    print("  Components:")
    for comp in COMPONENTS:
        print("    - {}".format(comp["label"]))
    print()

    # -- Run McPAT for each component --
    all_results = []
    total_mw = 0.0
    total_dyn = 0.0
    total_leak = 0.0
    total_area = 0.0

    for comp in COMPONENTS:
        xml_path = os.path.join(args.xml_dir, comp["xml"])
        out_path = os.path.join(
            args.output_dir,
            comp["key"] + "_mcpat_output.txt")

        if not os.path.isfile(xml_path):
            print("WARNING: XML not found: {} -- skipping".format(
                comp["label"]), file=sys.stderr)
            continue

        if not args.skip_run:
            sys.stdout.write("  Running McPAT for: {} ...".format(
                comp["label"]))
            sys.stdout.flush()
            try:
                output_text = run_mcpat(
                    args.mcpat_bin, xml_path, args.print_level)
                with open(out_path, "w") as f:
                    f.write(output_text)
                print(" OK")
            except Exception as e:
                print(" FAILED: {}".format(e))
                continue
        else:
            if not os.path.isfile(out_path):
                print("WARNING: No cached output: {}".format(
                    out_path), file=sys.stderr)
                continue
            with open(out_path, "r") as f:
                output_text = f.read()
            print("  Loaded cached: {}".format(out_path))

        parsed = parse_mcpat_detailed(output_text)
        tot, dyn, leak, area = print_component_report(comp, parsed)

        all_results.append({
            "key": comp["key"],
            "label": comp["label"],
            "total_mw": tot,
            "dynamic_mw": dyn,
            "leakage_mw": leak,
            "area_mm2": area,
            "target_mw": comp.get("target_mw"),
        })

        total_mw += tot
        total_dyn += dyn
        total_leak += leak
        total_area += area

    # -- SoC Summary --
    print()
    sep("=", 80)
    print("  SoC POWER SUMMARY")
    sep("=", 80)
    print()
    hdr = "  {:<44s} {:>10s} {:>10s} {:>10s} {:>8s}"
    print(hdr.format("Component", "Dynamic", "Leakage",
                     "Total", "Area"))
    print("  " + "-" * 84)
    for r in all_results:
        print(hdr.format(
            r["label"][:44],
            fmt_power(r["dynamic_mw"]),
            fmt_power(r["leakage_mw"]),
            fmt_power(r["total_mw"]),
            "{:.4f}".format(r["area_mm2"])))
    print("  " + "-" * 84)
    print(hdr.format(
        "TOTAL SoC",
        fmt_power(total_dyn),
        fmt_power(total_leak),
        fmt_power(total_mw),
        "{:.4f}".format(total_area)))
    print()

    # -- TDP comparison --
    tdp_w = args.tdp_target
    print("  TDP Target:  {:.3f} W  ({:.1f} mW)".format(
        tdp_w, tdp_w * 1000))
    print("  Modeled:     {:.3f} W  ({:.1f} mW)".format(
        total_mw / 1000, total_mw))
    headroom_mw = (tdp_w * 1000) - total_mw
    pct = 0
    if tdp_w > 0:
        pct = (headroom_mw / (tdp_w * 1000)) * 100
    print("  Headroom:    {:.1f} mW  ({:.1f}% of TDP)".format(
        headroom_mw, pct))
    print("  (Headroom = interconnect, PMU, IO pads, PLL, etc.)")
    print()

    # -- Power distribution --
    if all_results and total_mw > 0:
        print("  Power Distribution (% of modeled total):")
        for r in all_results:
            pct = (r["total_mw"] / total_mw) * 100
            bar_len = int(pct / 2)
            print("    {:<34s} {:>6.1f}%  {}".format(
                r["label"][:34], pct, "#" * bar_len))
        print()

    # -- Energy --
    if sim_seconds is not None and sim_seconds > 0:
        energy_j = (total_mw / 1000.0) * sim_seconds
        print("  Energy (simulation): {:.6f} J".format(energy_j))
        for r in all_results:
            e = (r["total_mw"] / 1000.0) * sim_seconds
            print("    {:<34s} {:.6f} J".format(
                r["label"][:34], e))
        print()

    sep("=", 80)
    print("  Output files: {}".format(args.output_dir))
    sep("=", 80)
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
