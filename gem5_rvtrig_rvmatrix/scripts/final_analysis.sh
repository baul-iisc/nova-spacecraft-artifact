#!/bin/bash
#
# Final Analysis Script for RT Scheduling Research
# Run after all experiments complete
#

BASE_DIR="/data/home/chandraboul/development/gem5-matrix/gem5-nova"

echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║                 FINAL ANALYSIS - RT Scheduling Research               ║"
echo "╚══════════════════════════════════════════════════════════════════════╝"
echo ""

# Check if experiments are still running
gem5_procs=$(ps aux | grep gem5 | grep -v grep | wc -l)
if [ $gem5_procs -gt 0 ]; then
    echo "WARNING: $gem5_procs gem5 processes still running!"
    echo "Wait for all experiments to complete before running this script."
    echo ""
fi

echo "1. Converting intensity experiment outputs to log format..."
cd "$BASE_DIR/results/intensity_experiments"

for f in *.log; do
    # Check if file is empty (only has start time)
    lines=$(wc -l < "$f")
    if [ $lines -lt 3 ]; then
        base=$(basename "$f" .log)
        outfile="${base}.out"
        if [ -f "$outfile" ]; then
            echo "=== Updating $f ===" > "$f.tmp"
            grep -E "Simulated ticks|queued|Total stall" "$outfile" 2>/dev/null >> "$f.tmp"
            if [ -s "$f.tmp" ]; then
                mv "$f.tmp" "$f"
            else
                rm -f "$f.tmp"
            fi
        fi
    fi
done

echo "2. Running comprehensive analysis..."
cd "$BASE_DIR"
python3 analysis/comprehensive_thesis_analysis.py

echo ""
echo "3. Running analytical model comparison..."
python3 analysis/analytical_model_comparison.py

echo ""
echo "4. Generating summary..."
echo ""

echo "╔══════════════════════════════════════════════════════════════════════╗"
echo "║                      GENERATED FILES                                  ║"
echo "╠══════════════════════════════════════════════════════════════════════╣"

echo "║ Documentation:"
ls -la docs/COMPREHENSIVE_RT_SCHEDULING_DOCUMENTATION.md 2>/dev/null | awk '{print "║   "$9}'
ls -la docs/EXPERIMENTAL_RESULTS_SUMMARY.md 2>/dev/null | awk '{print "║   "$9}'

echo "║"
echo "║ Thesis Figures:"
ls analysis/thesis_figures/*.png 2>/dev/null | awk '{print "║   "$1}'

echo "║"
echo "║ Model Comparison Figures:"
ls analysis/rt_model_comparison/*.png 2>/dev/null | awk '{print "║   "$1}'

echo "║"
echo "║ Tables:"
ls analysis/thesis_figures/*.csv analysis/thesis_figures/*.tex 2>/dev/null | awk '{print "║   "$1}'
ls analysis/rt_model_comparison/*.tex 2>/dev/null | awk '{print "║   "$1}'

echo "╚══════════════════════════════════════════════════════════════════════╝"
echo ""
echo "Analysis complete!"


