#!/usr/bin/env python3
"""
PhD Research: Shared vs Dedicated Accelerators - Analysis Script
Generates plots and statistical analysis for the experiment results.

Usage:
    python3 analyze_phd_study.py <results_directory>
"""

import os
import sys
import csv
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

def load_results(results_dir):
    """Load results from CSV file."""
    csv_file = os.path.join(results_dir, 'all_results.csv')
    if not os.path.exists(csv_file):
        print(f"ERROR: CSV file not found: {csv_file}")
        sys.exit(1)
    
    results = []
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Convert numeric fields
            for key in ['cores', 'workload', 'wall_cycles', 'total_cycles', 
                       'compute_cycles', 'wait_cycles', 'matrix_ops', 
                       'cordic_ops', 'contention_events']:
                if key in row and row[key]:
                    try:
                        row[key] = int(row[key])
                    except ValueError:
                        row[key] = 0
            
            for key in ['throughput', 'checksum', 'sim_seconds']:
                if key in row and row[key]:
                    try:
                        row[key] = float(row[key])
                    except ValueError:
                        row[key] = 0.0
            
            results.append(row)
    
    return results

def plot_execution_time_comparison(results, output_dir):
    """Plot execution time: shared vs dedicated for each workload and core count."""
    workloads = ['combined', 'fdir', 'orbit', 'mars', 'star', 'matrix', 'cordic']
    core_counts = [1, 2, 4, 8]
    
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    axes = axes.flatten()
    
    for idx, workload in enumerate(workloads):
        ax = axes[idx]
        
        shared_times = []
        dedicated_times = []
        
        for cores in core_counts:
            shared = [r for r in results if r['workload_name'] == workload 
                     and r['mode'] == 'shared' and r['cores'] == cores]
            dedicated = [r for r in results if r['workload_name'] == workload 
                        and r['mode'] == 'dedicated' and r['cores'] == cores]
            
            shared_time = shared[0]['wall_cycles'] if shared else 0
            dedicated_time = dedicated[0]['wall_cycles'] if dedicated else 0
            
            shared_times.append(shared_time / 1e6)  # Convert to millions
            dedicated_times.append(dedicated_time / 1e6)
        
        x = np.arange(len(core_counts))
        width = 0.35
        
        bars1 = ax.bar(x - width/2, shared_times, width, label='Shared', color='#e74c3c')
        bars2 = ax.bar(x + width/2, dedicated_times, width, label='Dedicated', color='#2ecc71')
        
        ax.set_xlabel('Cores')
        ax.set_ylabel('Cycles (Millions)')
        ax.set_title(f'{workload.upper()}')
        ax.set_xticks(x)
        ax.set_xticklabels(core_counts)
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # Remove empty subplot
    axes[-1].axis('off')
    
    plt.suptitle('Execution Time: Shared vs Dedicated Accelerators', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'execution_time_comparison.png'), dpi=150)
    plt.close()
    print("  - execution_time_comparison.png")

def plot_contention_analysis(results, output_dir):
    """Plot contention overhead analysis."""
    workloads = ['combined', 'fdir', 'orbit', 'mars', 'star', 'matrix', 'cordic']
    core_counts = [2, 4, 8]  # No contention for 1 core
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    for idx, cores in enumerate(core_counts):
        ax = axes[idx]
        
        overheads = []
        colors = []
        
        for workload in workloads:
            shared = [r for r in results if r['workload_name'] == workload 
                     and r['mode'] == 'shared' and r['cores'] == cores]
            dedicated = [r for r in results if r['workload_name'] == workload 
                        and r['mode'] == 'dedicated' and r['cores'] == cores]
            
            if shared and dedicated and dedicated[0]['wall_cycles'] > 0:
                overhead = ((shared[0]['wall_cycles'] - dedicated[0]['wall_cycles']) 
                           / dedicated[0]['wall_cycles'] * 100)
            else:
                overhead = 0
            
            overheads.append(overhead)
            colors.append('#e74c3c' if overhead > 0 else '#2ecc71')
        
        bars = ax.bar(workloads, overheads, color=colors)
        ax.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
        ax.set_xlabel('Workload')
        ax.set_ylabel('Overhead (%)')
        ax.set_title(f'{cores} Cores')
        ax.tick_params(axis='x', rotation=45)
        ax.grid(True, alpha=0.3, axis='y')
        
        # Add value labels
        for bar, val in zip(bars, overheads):
            height = bar.get_height()
            ax.annotate(f'{val:.1f}%',
                       xy=(bar.get_x() + bar.get_width() / 2, height),
                       xytext=(0, 3 if height >= 0 else -15),
                       textcoords="offset points",
                       ha='center', va='bottom' if height >= 0 else 'top',
                       fontsize=8)
    
    plt.suptitle('Contention Overhead: Shared vs Dedicated (% increase in execution time)', 
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'contention_overhead.png'), dpi=150)
    plt.close()
    print("  - contention_overhead.png")

def plot_scalability(results, output_dir):
    """Plot scalability analysis."""
    workloads = ['combined', 'fdir', 'orbit', 'mars', 'star', 'matrix', 'cordic']
    core_counts = [1, 2, 4, 8]
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Shared mode scalability
    ax = axes[0]
    for workload in workloads:
        times = []
        for cores in core_counts:
            data = [r for r in results if r['workload_name'] == workload 
                   and r['mode'] == 'shared' and r['cores'] == cores]
            times.append(data[0]['wall_cycles'] / 1e6 if data else 0)
        
        if times[0] > 0:
            speedup = [times[0] / t if t > 0 else 0 for t in times]
            ax.plot(core_counts, speedup, 'o-', label=workload, linewidth=2, markersize=8)
    
    ax.plot(core_counts, core_counts, 'k--', label='Ideal', linewidth=1, alpha=0.5)
    ax.set_xlabel('Number of Cores')
    ax.set_ylabel('Speedup')
    ax.set_title('Shared Accelerator Scalability')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xticks(core_counts)
    
    # Dedicated mode scalability
    ax = axes[1]
    for workload in workloads:
        times = []
        for cores in core_counts:
            data = [r for r in results if r['workload_name'] == workload 
                   and r['mode'] == 'dedicated' and r['cores'] == cores]
            times.append(data[0]['wall_cycles'] / 1e6 if data else 0)
        
        if times[0] > 0:
            speedup = [times[0] / t if t > 0 else 0 for t in times]
            ax.plot(core_counts, speedup, 'o-', label=workload, linewidth=2, markersize=8)
    
    ax.plot(core_counts, core_counts, 'k--', label='Ideal', linewidth=1, alpha=0.5)
    ax.set_xlabel('Number of Cores')
    ax.set_ylabel('Speedup')
    ax.set_title('Dedicated Accelerator Scalability')
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xticks(core_counts)
    
    plt.suptitle('Scalability Analysis: Speedup vs Number of Cores', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'scalability_analysis.png'), dpi=150)
    plt.close()
    print("  - scalability_analysis.png")

def plot_compute_vs_wait(results, output_dir):
    """Plot compute vs wait time breakdown."""
    workloads = ['fdir', 'orbit', 'mars', 'star', 'matrix', 'cordic']
    core_counts = [2, 4, 8]
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()
    
    for idx, workload in enumerate(workloads):
        ax = axes[idx]
        
        compute_pcts = []
        wait_pcts = []
        labels = []
        
        for cores in core_counts:
            data = [r for r in results if r['workload_name'] == workload 
                   and r['mode'] == 'shared' and r['cores'] == cores]
            
            if data and (data[0]['compute_cycles'] + data[0]['wait_cycles']) > 0:
                total = data[0]['compute_cycles'] + data[0]['wait_cycles']
                compute_pct = data[0]['compute_cycles'] / total * 100
                wait_pct = data[0]['wait_cycles'] / total * 100
            else:
                compute_pct = 100
                wait_pct = 0
            
            compute_pcts.append(compute_pct)
            wait_pcts.append(wait_pct)
            labels.append(f'{cores}')
        
        x = np.arange(len(core_counts))
        
        ax.bar(x, compute_pcts, label='Compute', color='#2ecc71')
        ax.bar(x, wait_pcts, bottom=compute_pcts, label='Wait (Contention)', color='#e74c3c')
        
        ax.set_xlabel('Cores')
        ax.set_ylabel('Time (%)')
        ax.set_title(f'{workload.upper()} - Shared Mode')
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.legend(loc='upper right')
        ax.set_ylim(0, 105)
        ax.grid(True, alpha=0.3, axis='y')
    
    plt.suptitle('Compute vs Wait Time Breakdown (Shared Accelerator Mode)', 
                 fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'compute_vs_wait.png'), dpi=150)
    plt.close()
    print("  - compute_vs_wait.png")

def plot_throughput_comparison(results, output_dir):
    """Plot throughput comparison."""
    workloads = ['matrix', 'cordic', 'fdir', 'orbit', 'mars', 'star']
    core_counts = [1, 2, 4, 8]
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    
    # Shared mode
    ax = axes[0]
    for workload in workloads:
        throughputs = []
        for cores in core_counts:
            data = [r for r in results if r['workload_name'] == workload 
                   and r['mode'] == 'shared' and r['cores'] == cores]
            throughputs.append(data[0]['throughput'] * 1000 if data else 0)  # ops per 1000 cycles
        ax.plot(core_counts, throughputs, 'o-', label=workload, linewidth=2, markersize=8)
    
    ax.set_xlabel('Number of Cores')
    ax.set_ylabel('Throughput (ops per 1000 cycles)')
    ax.set_title('Shared Accelerator')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xticks(core_counts)
    
    # Dedicated mode
    ax = axes[1]
    for workload in workloads:
        throughputs = []
        for cores in core_counts:
            data = [r for r in results if r['workload_name'] == workload 
                   and r['mode'] == 'dedicated' and r['cores'] == cores]
            throughputs.append(data[0]['throughput'] * 1000 if data else 0)
        ax.plot(core_counts, throughputs, 'o-', label=workload, linewidth=2, markersize=8)
    
    ax.set_xlabel('Number of Cores')
    ax.set_ylabel('Throughput (ops per 1000 cycles)')
    ax.set_title('Dedicated Accelerator')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xticks(core_counts)
    
    plt.suptitle('Throughput Comparison: Operations per 1000 Cycles', fontsize=14, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'throughput_comparison.png'), dpi=150)
    plt.close()
    print("  - throughput_comparison.png")

def generate_report(results, output_dir):
    """Generate detailed text report."""
    report_file = os.path.join(output_dir, 'ANALYSIS_REPORT.txt')
    
    with open(report_file, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("PhD RESEARCH: SHARED VS DEDICATED ACCELERATORS - ANALYSIS REPORT\n")
        f.write("=" * 80 + "\n\n")
        
        # Summary statistics
        f.write("EXECUTIVE SUMMARY\n")
        f.write("-" * 40 + "\n\n")
        
        # Calculate average contention overhead
        overheads = []
        for workload in ['fdir', 'orbit', 'mars', 'star', 'matrix', 'cordic']:
            for cores in [2, 4, 8]:
                shared = [r for r in results if r['workload_name'] == workload 
                         and r['mode'] == 'shared' and r['cores'] == cores]
                dedicated = [r for r in results if r['workload_name'] == workload 
                            and r['mode'] == 'dedicated' and r['cores'] == cores]
                
                if shared and dedicated and dedicated[0]['wall_cycles'] > 0:
                    overhead = ((shared[0]['wall_cycles'] - dedicated[0]['wall_cycles']) 
                               / dedicated[0]['wall_cycles'] * 100)
                    overheads.append(overhead)
        
        if overheads:
            avg_overhead = sum(overheads) / len(overheads)
            max_overhead = max(overheads)
            min_overhead = min(overheads)
            
            f.write(f"Average Contention Overhead (Shared vs Dedicated): {avg_overhead:.1f}%\n")
            f.write(f"Maximum Contention Overhead: {max_overhead:.1f}%\n")
            f.write(f"Minimum Contention Overhead: {min_overhead:.1f}%\n\n")
        
        # Detailed per-workload analysis
        f.write("\nDETAILED WORKLOAD ANALYSIS\n")
        f.write("-" * 40 + "\n\n")
        
        workloads = ['combined', 'fdir', 'orbit', 'mars', 'star', 'matrix', 'cordic']
        core_counts = [1, 2, 4, 8]
        
        for workload in workloads:
            f.write(f"\n{workload.upper()}\n")
            f.write("=" * 40 + "\n")
            
            f.write(f"{'Cores':<6} {'Mode':<10} {'Wall Cycles':<15} {'Compute':<12} {'Wait':<12} {'Overhead':<10}\n")
            f.write("-" * 65 + "\n")
            
            for cores in core_counts:
                for mode in ['dedicated', 'shared']:
                    data = [r for r in results if r['workload_name'] == workload 
                           and r['mode'] == mode and r['cores'] == cores]
                    
                    if data:
                        d = data[0]
                        
                        # Calculate overhead vs dedicated
                        if mode == 'shared':
                            ded = [r for r in results if r['workload_name'] == workload 
                                  and r['mode'] == 'dedicated' and r['cores'] == cores]
                            if ded and ded[0]['wall_cycles'] > 0:
                                overhead = ((d['wall_cycles'] - ded[0]['wall_cycles']) 
                                           / ded[0]['wall_cycles'] * 100)
                                overhead_str = f"{overhead:+.1f}%"
                            else:
                                overhead_str = "N/A"
                        else:
                            overhead_str = "baseline"
                        
                        f.write(f"{cores:<6} {mode:<10} {d['wall_cycles']:<15} "
                               f"{d['compute_cycles']:<12} {d['wait_cycles']:<12} {overhead_str:<10}\n")
        
        # Recommendations
        f.write("\n\nRECOMMENDATIONS FOR SPACECRAFT SYSTEMS\n")
        f.write("-" * 40 + "\n\n")
        
        f.write("Based on the analysis:\n\n")
        f.write("1. SHARED ACCELERATORS are suitable when:\n")
        f.write("   - Core count is low (2-4 cores)\n")
        f.write("   - Area/power constraints are critical\n")
        f.write("   - Workloads have temporal separation\n\n")
        
        f.write("2. DEDICATED ACCELERATORS are recommended when:\n")
        f.write("   - High core counts (8+ cores)\n")
        f.write("   - Real-time performance is critical\n")
        f.write("   - Workloads run simultaneously\n\n")
        
        f.write("3. HYBRID APPROACH:\n")
        f.write("   - Use shared Matrix accelerator (less contention-sensitive)\n")
        f.write("   - Use dedicated CORDIC accelerators (more frequent access)\n\n")
        
        f.write("=" * 80 + "\n")
        f.write("END OF REPORT\n")
        f.write("=" * 80 + "\n")
    
    print(f"  - ANALYSIS_REPORT.txt")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_phd_study.py <results_directory>")
        sys.exit(1)
    
    results_dir = sys.argv[1]
    
    if not os.path.exists(results_dir):
        print(f"ERROR: Directory not found: {results_dir}")
        sys.exit(1)
    
    print(f"\nAnalyzing results from: {results_dir}")
    print("-" * 50)
    
    # Load results
    print("Loading results...")
    results = load_results(results_dir)
    print(f"  Loaded {len(results)} experiments")
    
    # Create plots directory
    plots_dir = os.path.join(results_dir, 'plots')
    os.makedirs(plots_dir, exist_ok=True)
    
    # Generate plots
    print("\nGenerating plots...")
    plot_execution_time_comparison(results, plots_dir)
    plot_contention_analysis(results, plots_dir)
    plot_scalability(results, plots_dir)
    plot_compute_vs_wait(results, plots_dir)
    plot_throughput_comparison(results, plots_dir)
    
    # Generate report
    print("\nGenerating report...")
    generate_report(results, results_dir)
    
    print("\n" + "=" * 50)
    print("ANALYSIS COMPLETE")
    print("=" * 50)
    print(f"Plots saved to: {plots_dir}")
    print(f"Report saved to: {os.path.join(results_dir, 'ANALYSIS_REPORT.txt')}")

if __name__ == "__main__":
    main()

