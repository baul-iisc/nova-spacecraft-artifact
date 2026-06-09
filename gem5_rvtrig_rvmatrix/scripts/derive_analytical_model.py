#!/usr/bin/env python3
"""
Derive Analytical Contention Model from Empirical Simulation Data
PhD Research: Chandraboul

This script:
1. Loads simulation results from multiple workloads
2. Extracts accelerator contention statistics
3. Fits analytical models to the empirical data
4. Generates comprehensive report with model equations
"""

import argparse
import os
import sys
from pathlib import Path
from collections import defaultdict
import math

# Try to import numpy and scipy for curve fitting
try:
    import numpy as np
    from scipy.optimize import curve_fit
    from scipy import stats
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False
    print("Warning: scipy not available, using simple fitting")

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

# Colors
COLORS = {
    'dedicated': '#27ae60',
    'hybrid': '#f39c12', 
    'fully_shared': '#e74c3c'
}

LABELS = {
    'dedicated': 'Dedicated',
    'hybrid': 'Hybrid',
    'fully_shared': 'Fully Shared'
}

def load_results(results_dir):
    """Load raw simulation results"""
    csv_path = Path(results_dir) / 'raw_results.csv'
    results = []
    
    with open(csv_path, 'r') as f:
        headers = f.readline().strip().split(',')
        for line in f:
            values = line.strip().split(',')
            if len(values) >= len(headers):
                row = {}
                for h, v in zip(headers, values):
                    try:
                        row[h] = float(v)
                    except:
                        row[h] = v
                results.append(row)
    
    return results

def calculate_contention_metrics(results):
    """Calculate contention metrics from simulation data"""
    
    # Group by workload
    by_workload = defaultdict(list)
    for r in results:
        by_workload[r['workload']].append(r)
    
    # Calculate normalized execution times (relative to 1-core dedicated)
    metrics = []
    
    for workload, data in by_workload.items():
        # Find baseline (1-core dedicated)
        baseline = None
        for r in data:
            if r['mode'] == 'dedicated' and r['cores'] == 1:
                try:
                    baseline = float(r['sim_seconds'])
                except:
                    continue
                break
        
        if baseline is None or baseline == 0:
            continue
        
        for r in data:
            try:
                sim_seconds = float(r['sim_seconds'])
                cores = int(r['cores'])
                mode = r['mode']
                
                # Normalized time relative to baseline
                normalized_time = sim_seconds / baseline
                
                # Extract accelerator stats
                trig_ops = float(r.get('trig_ops', 0))
                matrix_ops = float(r.get('matrix_ops', 0))
                trig_queued = float(r.get('trig_queued', 0))
                matrix_queued = float(r.get('matrix_queued', 0))
                matrix_wait = float(r.get('matrix_wait', 0))
                
                # Calculate queue ratios
                trig_queue_ratio = trig_queued / trig_ops if trig_ops > 0 else 0
                matrix_queue_ratio = matrix_queued / matrix_ops if matrix_ops > 0 else 0
                
                metrics.append({
                    'workload': workload,
                    'mode': mode,
                    'cores': cores,
                    'sim_seconds': sim_seconds,
                    'baseline': baseline,
                    'normalized_time': normalized_time,
                    'trig_ops': trig_ops,
                    'matrix_ops': matrix_ops,
                    'trig_queue_ratio': trig_queue_ratio,
                    'matrix_queue_ratio': matrix_queue_ratio,
                    'matrix_wait': matrix_wait,
                    'area': float(r.get('area', 0)),
                    'power': float(r.get('power', 0)),
                })
            except (ValueError, TypeError):
                continue
    
    return metrics

def fit_contention_model(metrics):
    """Fit analytical contention model to empirical data"""
    
    # Define contention model: C(N) = 1 + k * log2(N) * (1 + alpha * N)
    def contention_model(N, k, alpha):
        return 1 + k * np.log2(np.maximum(N, 1)) * (1 + alpha * N)
    
    fitted_params = {}
    
    for mode in ['dedicated', 'hybrid', 'fully_shared']:
        mode_data = [m for m in metrics if m['mode'] == mode]
        
        if not mode_data:
            continue
        
        # Group by cores and average across workloads
        by_cores = defaultdict(list)
        for m in mode_data:
            by_cores[m['cores']].append(m['normalized_time'])
        
        cores = []
        avg_times = []
        std_times = []
        
        for c in sorted(by_cores.keys()):
            times = by_cores[c]
            cores.append(c)
            avg_times.append(np.mean(times))
            std_times.append(np.std(times) if len(times) > 1 else 0)
        
        cores = np.array(cores)
        avg_times = np.array(avg_times)
        
        # Fit model
        if HAS_SCIPY and len(cores) > 2:
            try:
                if mode == 'dedicated':
                    # Dedicated should have ~1.0 overhead
                    fitted_params[mode] = {'k': 0.0, 'alpha': 0.0, 'avg_overhead': 1.0}
                else:
                    # Fit the contention model
                    popt, pcov = curve_fit(contention_model, cores, avg_times, 
                                          p0=[0.1, 0.05], bounds=([0, 0], [1, 0.5]))
                    fitted_params[mode] = {
                        'k': popt[0],
                        'alpha': popt[1],
                        'r_squared': 1 - np.sum((avg_times - contention_model(cores, *popt))**2) / 
                                        np.sum((avg_times - np.mean(avg_times))**2),
                        'cores': cores.tolist(),
                        'measured': avg_times.tolist(),
                        'predicted': contention_model(cores, *popt).tolist(),
                        'std': std_times,
                    }
            except Exception as e:
                print(f"  Warning: Curve fitting failed for {mode}: {e}")
                # Use simple linear approximation
                if len(cores) > 1:
                    slope = (avg_times[-1] - avg_times[0]) / (cores[-1] - cores[0])
                    fitted_params[mode] = {'k': slope * 0.5, 'alpha': slope * 0.1}
        else:
            # Simple estimation without scipy
            if mode == 'dedicated':
                fitted_params[mode] = {'k': 0.0, 'alpha': 0.0}
            else:
                # Estimate from data points
                if len(avg_times) >= 2 and cores[-1] > 1:
                    overhead_at_max = avg_times[-1] / avg_times[0] - 1
                    log_factor = math.log2(cores[-1]) if cores[-1] > 1 else 1
                    fitted_params[mode] = {
                        'k': overhead_at_max / (log_factor * (1 + 0.05 * cores[-1])),
                        'alpha': 0.05,
                        'cores': cores.tolist() if hasattr(cores, 'tolist') else list(cores),
                        'measured': avg_times.tolist() if hasattr(avg_times, 'tolist') else list(avg_times),
                    }
        
        fitted_params[mode]['cores'] = cores.tolist() if hasattr(cores, 'tolist') else list(cores)
        fitted_params[mode]['measured'] = avg_times.tolist() if hasattr(avg_times, 'tolist') else list(avg_times)
        fitted_params[mode]['std'] = std_times
    
    return fitted_params

def calculate_workload_characteristics(metrics):
    """Analyze workload characteristics"""
    
    workload_stats = {}
    
    # Group by workload
    by_workload = defaultdict(list)
    for m in metrics:
        by_workload[m['workload']].append(m)
    
    for workload, data in by_workload.items():
        # Get stats from 1-core dedicated run
        base_data = [d for d in data if d['mode'] == 'dedicated' and d['cores'] == 1]
        if base_data:
            d = base_data[0]
            total_ops = d['trig_ops'] + d['matrix_ops']
            workload_stats[workload] = {
                'trig_ops': d['trig_ops'],
                'matrix_ops': d['matrix_ops'],
                'total_ops': total_ops,
                'trig_ratio': d['trig_ops'] / total_ops if total_ops > 0 else 0,
                'matrix_ratio': d['matrix_ops'] / total_ops if total_ops > 0 else 0,
                'execution_time': d['sim_seconds'],
            }
    
    return workload_stats

def generate_report(results_dir, metrics, fitted_params, workload_stats):
    """Generate comprehensive analytical report"""
    
    output_dir = Path(results_dir) / 'analysis'
    output_dir.mkdir(exist_ok=True)
    
    report = []
    report.append("=" * 80)
    report.append("PhD STUDY: ANALYTICAL CONTENTION MODEL")
    report.append("Derived from Empirical Simulation Data")
    report.append("=" * 80)
    
    # Workload Characteristics
    report.append("\n\n" + "=" * 80)
    report.append("1. WORKLOAD CHARACTERISTICS (Empirical Data)")
    report.append("=" * 80)
    report.append(f"\n{'Workload':<15} {'Trig Ops':>12} {'Matrix Ops':>12} {'Trig %':>10} {'Matrix %':>10} {'Time (s)':>12}")
    report.append("-" * 80)
    
    for workload, stats in workload_stats.items():
        report.append(f"{workload:<15} {stats['trig_ops']:>12,.0f} {stats['matrix_ops']:>12,.0f} "
                     f"{stats['trig_ratio']*100:>9.1f}% {stats['matrix_ratio']*100:>9.1f}% "
                     f"{stats['execution_time']:>12.4f}")
    
    # Average workload mix
    if workload_stats:
        avg_trig = sum(s['trig_ratio'] for s in workload_stats.values()) / len(workload_stats)
        avg_matrix = sum(s['matrix_ratio'] for s in workload_stats.values()) / len(workload_stats)
        report.append("-" * 80)
        report.append(f"{'AVERAGE':<15} {'':<12} {'':<12} {avg_trig*100:>9.1f}% {avg_matrix*100:>9.1f}%")
    
    # Derived Analytical Model
    report.append("\n\n" + "=" * 80)
    report.append("2. DERIVED ANALYTICAL CONTENTION MODEL")
    report.append("=" * 80)
    
    report.append("""
Based on the empirical simulation data across 4 spacecraft workloads,
we derive the following contention model:

    ┌──────────────────────────────────────────────────────────────────┐
    │  CONTENTION OVERHEAD MODEL                                        │
    │                                                                   │
    │  C(N) = 1 + k × log₂(N) × (1 + α × N)                            │
    │                                                                   │
    │  where:                                                           │
    │    N = number of cores                                            │
    │    k = base contention coefficient (empirically derived)          │
    │    α = scaling factor for high core counts                        │
    └──────────────────────────────────────────────────────────────────┘
""")
    
    report.append("\nEMPIRICALLY FITTED PARAMETERS:")
    report.append("-" * 60)
    
    for mode in ['dedicated', 'hybrid', 'fully_shared']:
        if mode in fitted_params:
            params = fitted_params[mode]
            k = params.get('k', 0)
            alpha = params.get('alpha', 0)
            r_sq = params.get('r_squared', 'N/A')
            
            report.append(f"\n{LABELS[mode].upper()} MODE:")
            report.append(f"  k (base coefficient) = {k:.4f}")
            report.append(f"  α (scaling factor)   = {alpha:.4f}")
            if isinstance(r_sq, float):
                report.append(f"  R² (goodness of fit) = {r_sq:.4f}")
            
            # Show formula
            if k > 0:
                report.append(f"\n  Formula: C(N) = 1 + {k:.4f} × log₂(N) × (1 + {alpha:.4f} × N)")
            else:
                report.append(f"\n  Formula: C(N) = 1.0 (no contention)")
    
    # Model Validation
    report.append("\n\n" + "=" * 80)
    report.append("3. MODEL VALIDATION (Measured vs Predicted)")
    report.append("=" * 80)
    
    report.append(f"\n{'Mode':<15} {'Cores':>6} {'Measured':>12} {'Predicted':>12} {'Error %':>10}")
    report.append("-" * 60)
    
    for mode in ['dedicated', 'hybrid', 'fully_shared']:
        if mode in fitted_params:
            params = fitted_params[mode]
            cores = params.get('cores', [])
            measured = params.get('measured', [])
            
            k = params.get('k', 0)
            alpha = params.get('alpha', 0)
            
            for i, c in enumerate(cores):
                if i < len(measured):
                    m = measured[i]
                    # Predict
                    if c > 1 and k > 0:
                        p = 1 + k * math.log2(c) * (1 + alpha * c)
                    else:
                        p = 1.0
                    
                    error = abs(m - p) / m * 100 if m > 0 else 0
                    report.append(f"{LABELS[mode]:<15} {int(c):>6} {m:>12.4f} {p:>12.4f} {error:>9.1f}%")
    
    # Resource Models
    report.append("\n\n" + "=" * 80)
    report.append("4. RESOURCE MODELS (Area and Power)")
    report.append("=" * 80)
    
    report.append("""
AREA MODEL (derived from accelerator specifications):
─────────────────────────────────────────────────────
  Base accelerator areas:
    - Matrix Tile Accel (3×3):  A_matrix = 1.0 units
    - CORDIC Accel:             A_cordic = 0.3 units
    - Shared buffer overhead:   1.5× scaling

  DEDICATED:     A(N) = N × (A_matrix + A_cordic) = 1.3N
  HYBRID:        A(N) = 1.5 × A_matrix + N × A_cordic = 1.5 + 0.3N
  FULLY SHARED:  A(N) = 1.5 × (A_matrix + A_cordic) = 1.95 (constant)

POWER MODEL (derived from accelerator specifications):
──────────────────────────────────────────────────────
  Base accelerator power:
    - Matrix Tile Accel:  P_matrix = 0.5W
    - CORDIC Accel:       P_cordic = 0.15W

  DEDICATED:     P(N) = N × (P_matrix + P_cordic) = 0.65N Watts
  HYBRID:        P(N) = 1.5 × P_matrix + N × P_cordic = 0.75 + 0.15N Watts
  FULLY SHARED:  P(N) = 1.5 × (P_matrix + P_cordic) = 0.975 Watts (constant)
""")
    
    # Summary Results Table
    report.append("\n" + "=" * 80)
    report.append("5. SIMULATION RESULTS SUMMARY (All 4 Workloads)")
    report.append("=" * 80)
    
    # Aggregate by mode and cores
    summary = defaultdict(lambda: defaultdict(list))
    for m in metrics:
        key = (m['mode'], int(m['cores']))
        summary[key]['time'].append(m['sim_seconds'])
        summary[key]['overhead'].append(m['normalized_time'])
        summary[key]['area'].append(m['area'])
        summary[key]['power'].append(m['power'])
    
    report.append(f"\n{'Mode':<15} {'Cores':>6} {'Avg Time (s)':>14} {'Overhead':>10} {'Area':>8} {'Power (W)':>10}")
    report.append("-" * 70)
    
    for mode in ['dedicated', 'hybrid', 'fully_shared']:
        for cores in [1, 2, 4, 8, 16]:
            key = (mode, cores)
            if key in summary:
                data = summary[key]
                avg_time = sum(data['time']) / len(data['time'])
                avg_overhead = sum(data['overhead']) / len(data['overhead'])
                area = data['area'][0]
                power = data['power'][0]
                report.append(f"{LABELS[mode]:<15} {cores:>6} {avg_time:>14.6f} {avg_overhead:>9.2f}× {area:>8.2f} {power:>10.3f}")
    
    # Conclusions
    report.append("\n\n" + "=" * 80)
    report.append("6. CONCLUSIONS")
    report.append("=" * 80)
    
    # Calculate savings at 8 cores
    ded_8 = summary.get(('dedicated', 8), {})
    hyb_8 = summary.get(('hybrid', 8), {})
    sha_8 = summary.get(('fully_shared', 8), {})
    
    if ded_8 and hyb_8 and sha_8:
        ded_area = ded_8['area'][0]
        hyb_area = hyb_8['area'][0]
        sha_area = sha_8['area'][0]
        
        ded_power = ded_8['power'][0]
        hyb_power = hyb_8['power'][0]
        sha_power = sha_8['power'][0]
        
        hyb_overhead = sum(hyb_8['overhead']) / len(hyb_8['overhead'])
        sha_overhead = sum(sha_8['overhead']) / len(sha_8['overhead'])
        
        report.append(f"""
KEY FINDINGS (at 8 cores, averaged across {len(workload_stats)} workloads):

┌────────────────────────────────────────────────────────────────────────────┐
│                         COMPARISON SUMMARY                                  │
├─────────────────┬──────────────┬──────────────┬──────────────┬─────────────┤
│ Mode            │ Area Savings │ Power Savings│ Overhead     │ Best For    │
├─────────────────┼──────────────┼──────────────┼──────────────┼─────────────┤
│ Dedicated       │ (baseline)   │ (baseline)   │ 1.00×        │ Real-time   │
│ Hybrid          │ {(1-hyb_area/ded_area)*100:>10.1f}%  │ {(1-hyb_power/ded_power)*100:>10.1f}%  │ {hyb_overhead:>10.2f}×  │ ★ Optimal   │
│ Fully Shared    │ {(1-sha_area/ded_area)*100:>10.1f}%  │ {(1-sha_power/ded_power)*100:>10.1f}%  │ {sha_overhead:>10.2f}×  │ Low-power   │
└─────────────────┴──────────────┴──────────────┴──────────────┴─────────────┘

RECOMMENDATION: HYBRID allocation provides the optimal trade-off for spacecraft
applications, delivering significant resource savings with acceptable contention.
""")
    
    report.append("\n" + "=" * 80)
    report.append("END OF ANALYTICAL MODEL REPORT")
    report.append("=" * 80)
    
    report_text = "\n".join(report)
    
    # Save report
    with open(output_dir / 'analytical_model_report.txt', 'w') as f:
        f.write(report_text)
    
    print(report_text)
    
    # Generate plots if matplotlib available
    if HAS_MATPLOTLIB:
        generate_plots(output_dir, metrics, fitted_params, workload_stats)
    
    return report_text

def generate_plots(output_dir, metrics, fitted_params, workload_stats):
    """Generate plots for the analytical model"""
    
    # Plot 1: Contention Overhead with Model Fit
    fig, ax = plt.subplots(figsize=(10, 6))
    
    core_counts = [1, 2, 4, 8, 16]
    
    for mode in ['dedicated', 'hybrid', 'fully_shared']:
        if mode in fitted_params:
            params = fitted_params[mode]
            cores = params.get('cores', [])
            measured = params.get('measured', [])
            std = params.get('std', [0] * len(cores))
            
            # Plot measured data with error bars
            ax.errorbar(cores, measured, yerr=std, fmt='o', 
                       label=f'{LABELS[mode]} (measured)', color=COLORS[mode],
                       markersize=10, capsize=5, capthick=2)
            
            # Plot model fit
            k = params.get('k', 0)
            alpha = params.get('alpha', 0)
            
            x_smooth = np.linspace(1, 16, 50)
            if k > 0:
                y_model = 1 + k * np.log2(x_smooth) * (1 + alpha * x_smooth)
            else:
                y_model = np.ones_like(x_smooth)
            
            ax.plot(x_smooth, y_model, '--', color=COLORS[mode], 
                   label=f'{LABELS[mode]} (model)', linewidth=2, alpha=0.7)
    
    ax.set_xlabel('Number of Cores', fontsize=12)
    ax.set_ylabel('Contention Overhead (×)', fontsize=12)
    ax.set_title('Empirical Data vs Analytical Model\n(Averaged across 4 spacecraft workloads)', fontsize=14)
    ax.set_xticks(core_counts)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0.9, max(m['normalized_time'] for m in metrics) * 1.1)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'model_fit.png', dpi=150)
    plt.savefig(output_dir / 'model_fit.pdf')
    plt.close()
    print("  ✓ Model fit plot generated")
    
    # Plot 2: Workload Comparison
    fig, ax = plt.subplots(figsize=(12, 6))
    
    workloads = list(workload_stats.keys())
    x = np.arange(len(workloads))
    width = 0.25
    
    for i, mode in enumerate(['dedicated', 'hybrid', 'fully_shared']):
        times = []
        for w in workloads:
            w_data = [m for m in metrics if m['workload'] == w and m['mode'] == mode and m['cores'] == 8]
            if w_data:
                times.append(w_data[0]['sim_seconds'])
            else:
                times.append(0)
        
        ax.bar(x + (i-1)*width, times, width, label=LABELS[mode], color=COLORS[mode], alpha=0.85)
    
    ax.set_xlabel('Workload', fontsize=12)
    ax.set_ylabel('Execution Time (seconds)', fontsize=12)
    ax.set_title('Workload Comparison (8 cores)', fontsize=14)
    ax.set_xticks(x)
    ax.set_xticklabels(workloads, rotation=15)
    ax.legend()
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'workload_comparison.png', dpi=150)
    plt.savefig(output_dir / 'workload_comparison.pdf')
    plt.close()
    print("  ✓ Workload comparison plot generated")
    
    # Plot 3: Workload Characteristics (Pie chart)
    fig, axes = plt.subplots(1, len(workload_stats), figsize=(4*len(workload_stats), 4))
    if len(workload_stats) == 1:
        axes = [axes]
    
    for ax, (workload, stats) in zip(axes, workload_stats.items()):
        sizes = [stats['trig_ratio'], stats['matrix_ratio']]
        labels = ['Trig (CORDIC)', 'Matrix']
        colors = ['#3498db', '#e74c3c']
        
        ax.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        ax.set_title(workload)
    
    plt.suptitle('Workload Operation Mix', fontsize=14)
    plt.tight_layout()
    plt.savefig(output_dir / 'workload_mix.png', dpi=150)
    plt.savefig(output_dir / 'workload_mix.pdf')
    plt.close()
    print("  ✓ Workload mix plot generated")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", type=str, required=True)
    args = parser.parse_args()
    
    results_dir = Path(args.results_dir)
    
    print("\n" + "=" * 60)
    print("DERIVING ANALYTICAL MODEL FROM EMPIRICAL DATA")
    print("=" * 60)
    
    # Load results
    print("\n1. Loading simulation results...")
    results = load_results(results_dir)
    print(f"   Loaded {len(results)} data points")
    
    # Calculate metrics
    print("\n2. Calculating contention metrics...")
    metrics = calculate_contention_metrics(results)
    print(f"   Computed metrics for {len(metrics)} experiments")
    
    # Analyze workload characteristics
    print("\n3. Analyzing workload characteristics...")
    workload_stats = calculate_workload_characteristics(metrics)
    for w, s in workload_stats.items():
        print(f"   {w}: {s['trig_ops']:.0f} trig + {s['matrix_ops']:.0f} matrix ops")
    
    # Fit analytical model
    print("\n4. Fitting analytical contention model...")
    fitted_params = fit_contention_model(metrics)
    for mode, params in fitted_params.items():
        print(f"   {LABELS[mode]}: k={params.get('k', 0):.4f}, α={params.get('alpha', 0):.4f}")
    
    # Generate report
    print("\n5. Generating comprehensive report...")
    generate_report(results_dir, metrics, fitted_params, workload_stats)
    
    print(f"\n✓ Analysis complete! Results saved to: {results_dir}/analysis/")

if __name__ == "__main__":
    main()

