#!/usr/bin/env python3
"""
Spacecraft Contention Analysis - Results Analyzer
PhD Research: Chandraboul

This script analyzes the results from the contention study and generates:
  - Summary statistics
  - Comparison charts
  - Contention impact analysis

Usage:
    python3 analyze_contention_results.py <results_directory>
    python3 analyze_contention_results.py results/contention_study_20260113_120000
"""

import os
import sys
import csv
import argparse
from pathlib import Path
from collections import defaultdict

# Try to import optional visualization libraries
try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend for server
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("Warning: matplotlib not available. Charts will not be generated.")

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False


def load_summary_csv(csv_path):
    """Load the summary CSV file."""
    results = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            results.append(row)
    return results


def parse_numeric(value, default=0):
    """Parse a numeric value, returning default if invalid."""
    if value in ['N/A', '', None]:
        return default
    try:
        return float(value)
    except (ValueError, TypeError):
        return default


def analyze_contention_impact(results):
    """Analyze the impact of accelerator contention on performance."""
    
    # Group results by workload
    workloads = defaultdict(dict)
    for row in results:
        workload = row['workload']
        mode = row['mode']
        workloads[workload][mode] = row
    
    analysis = {
        'workloads': {},
        'summary': {
            'avg_slowdown_hybrid': 0,
            'avg_slowdown_shared': 0,
            'max_slowdown_hybrid': 0,
            'max_slowdown_shared': 0,
            'workload_count': 0
        }
    }
    
    slowdowns_hybrid = []
    slowdowns_shared = []
    
    for workload, modes in workloads.items():
        if 'dedicated' not in modes:
            continue
            
        dedicated_time = parse_numeric(modes['dedicated'].get('sim_seconds'))
        if dedicated_time <= 0:
            continue
            
        workload_analysis = {
            'dedicated_time': dedicated_time,
            'hybrid_time': None,
            'shared_time': None,
            'hybrid_slowdown': None,
            'shared_slowdown': None,
            'status': {}
        }
        
        for mode in ['dedicated', 'hybrid', 'shared']:
            if mode in modes:
                workload_analysis['status'][mode] = modes[mode].get('status', 'UNKNOWN')
        
        if 'hybrid' in modes:
            hybrid_time = parse_numeric(modes['hybrid'].get('sim_seconds'))
            if hybrid_time > 0:
                workload_analysis['hybrid_time'] = hybrid_time
                workload_analysis['hybrid_slowdown'] = hybrid_time / dedicated_time
                slowdowns_hybrid.append(workload_analysis['hybrid_slowdown'])
        
        if 'shared' in modes:
            shared_time = parse_numeric(modes['shared'].get('sim_seconds'))
            if shared_time > 0:
                workload_analysis['shared_time'] = shared_time
                workload_analysis['shared_slowdown'] = shared_time / dedicated_time
                slowdowns_shared.append(workload_analysis['shared_slowdown'])
        
        analysis['workloads'][workload] = workload_analysis
    
    # Calculate summary statistics
    if slowdowns_hybrid:
        analysis['summary']['avg_slowdown_hybrid'] = sum(slowdowns_hybrid) / len(slowdowns_hybrid)
        analysis['summary']['max_slowdown_hybrid'] = max(slowdowns_hybrid)
    
    if slowdowns_shared:
        analysis['summary']['avg_slowdown_shared'] = sum(slowdowns_shared) / len(slowdowns_shared)
        analysis['summary']['max_slowdown_shared'] = max(slowdowns_shared)
    
    analysis['summary']['workload_count'] = len(analysis['workloads'])
    
    return analysis


def generate_text_report(analysis, output_path):
    """Generate a text-based analysis report."""
    
    with open(output_path, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("SPACECRAFT WORKLOAD CONTENTION ANALYSIS\n")
        f.write("=" * 80 + "\n\n")
        
        # Summary
        f.write("EXECUTIVE SUMMARY\n")
        f.write("-" * 40 + "\n")
        f.write(f"Total workloads analyzed: {analysis['summary']['workload_count']}\n")
        f.write(f"\n")
        f.write("Contention Impact (slowdown vs dedicated baseline):\n")
        f.write(f"  Hybrid mode (2:1 sharing):\n")
        f.write(f"    - Average slowdown: {analysis['summary']['avg_slowdown_hybrid']:.2f}x\n")
        f.write(f"    - Maximum slowdown: {analysis['summary']['max_slowdown_hybrid']:.2f}x\n")
        f.write(f"  Shared mode (N:1 sharing):\n")
        f.write(f"    - Average slowdown: {analysis['summary']['avg_slowdown_shared']:.2f}x\n")
        f.write(f"    - Maximum slowdown: {analysis['summary']['max_slowdown_shared']:.2f}x\n")
        f.write("\n")
        
        # Per-workload details
        f.write("=" * 80 + "\n")
        f.write("PER-WORKLOAD ANALYSIS\n")
        f.write("=" * 80 + "\n\n")
        
        # Sort by shared slowdown (highest impact first)
        sorted_workloads = sorted(
            analysis['workloads'].items(),
            key=lambda x: x[1].get('shared_slowdown') or 0,
            reverse=True
        )
        
        for workload, data in sorted_workloads:
            f.write(f"\n{workload}\n")
            f.write("-" * len(workload) + "\n")
            
            f.write(f"  Dedicated (baseline): {data['dedicated_time']:.6f}s\n")
            
            if data['hybrid_time']:
                f.write(f"  Hybrid:               {data['hybrid_time']:.6f}s ")
                f.write(f"({data['hybrid_slowdown']:.2f}x slowdown)\n")
            else:
                f.write(f"  Hybrid:               N/A\n")
            
            if data['shared_time']:
                f.write(f"  Shared:               {data['shared_time']:.6f}s ")
                f.write(f"({data['shared_slowdown']:.2f}x slowdown)\n")
            else:
                f.write(f"  Shared:               N/A\n")
        
        # Recommendations
        f.write("\n" + "=" * 80 + "\n")
        f.write("RECOMMENDATIONS\n")
        f.write("=" * 80 + "\n\n")
        
        avg_shared = analysis['summary']['avg_slowdown_shared']
        if avg_shared > 2.0:
            f.write("HIGH CONTENTION IMPACT DETECTED\n")
            f.write(f"  Average {avg_shared:.1f}x slowdown in shared mode indicates significant\n")
            f.write("  accelerator contention. Consider:\n")
            f.write("    1. Adding more accelerator instances\n")
            f.write("    2. Using hybrid allocation for critical workloads\n")
            f.write("    3. Implementing priority-based scheduling\n")
        elif avg_shared > 1.2:
            f.write("MODERATE CONTENTION IMPACT\n")
            f.write(f"  Average {avg_shared:.1f}x slowdown is acceptable for shared resources.\n")
            f.write("  The hybrid mode provides a good balance.\n")
        else:
            f.write("LOW CONTENTION IMPACT\n")
            f.write("  Workloads show minimal contention impact.\n")
            f.write("  Shared accelerator configuration is efficient.\n")
        
        f.write("\n")
    
    print(f"Text report saved to: {output_path}")


def generate_charts(analysis, results_dir):
    """Generate visualization charts."""
    
    if not HAS_MATPLOTLIB:
        print("Skipping chart generation (matplotlib not available)")
        return
    
    # Prepare data
    workloads = []
    dedicated_times = []
    hybrid_times = []
    shared_times = []
    
    for workload, data in sorted(analysis['workloads'].items()):
        workloads.append(workload[:25])  # Truncate long names
        dedicated_times.append(data['dedicated_time'] or 0)
        hybrid_times.append(data['hybrid_time'] or 0)
        shared_times.append(data['shared_time'] or 0)
    
    if not workloads:
        print("No data available for charts")
        return
    
    # Chart 1: Execution Time Comparison (Bar Chart)
    fig, ax = plt.subplots(figsize=(14, 8))
    x = range(len(workloads))
    width = 0.25
    
    bars1 = ax.bar([i - width for i in x], dedicated_times, width, label='Dedicated', color='#2ecc71')
    bars2 = ax.bar(x, hybrid_times, width, label='Hybrid', color='#f39c12')
    bars3 = ax.bar([i + width for i in x], shared_times, width, label='Shared', color='#e74c3c')
    
    ax.set_xlabel('Workload')
    ax.set_ylabel('Simulation Time (seconds)')
    ax.set_title('Execution Time by Accelerator Sharing Mode')
    ax.set_xticks(x)
    ax.set_xticklabels(workloads, rotation=45, ha='right', fontsize=8)
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'execution_time_comparison.png'), dpi=150)
    plt.close()
    print(f"Chart saved: execution_time_comparison.png")
    
    # Chart 2: Slowdown Factor (relative to dedicated)
    fig, ax = plt.subplots(figsize=(14, 6))
    
    hybrid_slowdowns = [data.get('hybrid_slowdown') or 1.0 
                        for _, data in sorted(analysis['workloads'].items())]
    shared_slowdowns = [data.get('shared_slowdown') or 1.0 
                        for _, data in sorted(analysis['workloads'].items())]
    
    x = range(len(workloads))
    width = 0.35
    
    ax.bar([i - width/2 for i in x], hybrid_slowdowns, width, label='Hybrid (2:1)', color='#f39c12')
    ax.bar([i + width/2 for i in x], shared_slowdowns, width, label='Shared (N:1)', color='#e74c3c')
    
    ax.axhline(y=1.0, color='#2ecc71', linestyle='--', linewidth=2, label='Baseline (dedicated)')
    
    ax.set_xlabel('Workload')
    ax.set_ylabel('Slowdown Factor (vs Dedicated)')
    ax.set_title('Contention Impact: Slowdown by Sharing Mode')
    ax.set_xticks(x)
    ax.set_xticklabels(workloads, rotation=45, ha='right', fontsize=8)
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(results_dir, 'slowdown_comparison.png'), dpi=150)
    plt.close()
    print(f"Chart saved: slowdown_comparison.png")
    
    # Chart 3: Summary Box Plot
    fig, ax = plt.subplots(figsize=(8, 6))
    
    valid_hybrid = [s for s in hybrid_slowdowns if s > 0]
    valid_shared = [s for s in shared_slowdowns if s > 0]
    
    if valid_hybrid and valid_shared:
        bp = ax.boxplot([valid_hybrid, valid_shared], labels=['Hybrid (2:1)', 'Shared (N:1)'])
        ax.set_ylabel('Slowdown Factor')
        ax.set_title('Distribution of Contention Slowdown')
        ax.axhline(y=1.0, color='#2ecc71', linestyle='--', alpha=0.7, label='No contention')
        ax.grid(axis='y', alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(results_dir, 'slowdown_distribution.png'), dpi=150)
        plt.close()
        print(f"Chart saved: slowdown_distribution.png")


def main():
    parser = argparse.ArgumentParser(description='Analyze contention study results')
    parser.add_argument('results_dir', help='Path to results directory')
    args = parser.parse_args()
    
    results_dir = Path(args.results_dir)
    csv_path = results_dir / 'contention_summary.csv'
    
    if not csv_path.exists():
        print(f"Error: Summary CSV not found at {csv_path}")
        sys.exit(1)
    
    print(f"Loading results from: {csv_path}")
    results = load_summary_csv(csv_path)
    print(f"Loaded {len(results)} result entries")
    
    # Analyze contention impact
    analysis = analyze_contention_impact(results)
    
    # Generate reports
    generate_text_report(analysis, results_dir / 'contention_detailed_analysis.txt')
    generate_charts(analysis, results_dir)
    
    # Print summary to console
    print("\n" + "=" * 60)
    print("CONTENTION ANALYSIS SUMMARY")
    print("=" * 60)
    print(f"Workloads analyzed: {analysis['summary']['workload_count']}")
    print(f"\nHybrid mode (2:1 sharing):")
    print(f"  Average slowdown: {analysis['summary']['avg_slowdown_hybrid']:.2f}x")
    print(f"  Maximum slowdown: {analysis['summary']['max_slowdown_hybrid']:.2f}x")
    print(f"\nShared mode (N:1 sharing):")
    print(f"  Average slowdown: {analysis['summary']['avg_slowdown_shared']:.2f}x")
    print(f"  Maximum slowdown: {analysis['summary']['max_slowdown_shared']:.2f}x")
    print("=" * 60)


if __name__ == '__main__':
    main()
