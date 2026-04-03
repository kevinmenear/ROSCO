#!/bin/bash
# Compare debug output between baseline (pure Fortran) and integrated (C++) builds.
# Verifies that the C++ Debug translation produces identical data rows.
#
# Header differences are expected (Fortran CHARACTER(15) truncates column names,
# C++ does not; minor spacing differences). Only data rows are compared.
#
# Usage (from ROSCO repo root, inside Docker container):
#
#   # 1. Capture baseline debug output (clean Fortran build)
#   bash scripts/debug_test.sh baseline [--scenario N]
#
#   # 2. Capture integrated debug output (after integrate_all.sh + rebuild)
#   bash scripts/debug_test.sh integrated [--scenario N]
#
#   # 3. Compare
#   bash scripts/debug_test.sh compare
#
# Default scenario is 3 (LoggingLevel=1, produces .RO.dbg).
# All three steps can be run sequentially:
#   bash scripts/debug_test.sh all

set -e

BASELINE_DIR="debug_test/baseline"
INTEGRATED_DIR="debug_test/integrated"
SCENARIO="${2:-3}"

capture() {
    local dest=$1
    mkdir -p "$dest"

    # Remove old debug files from Examples/
    rm -f Examples/*.RO.dbg Examples/*.RO.dbg2 Examples/*.RO.dbg3

    # Run the specified scenario
    cd Examples
    LD_PRELOAD=${LD_PRELOAD:-} python3 vit_sim.py --scenario "$SCENARIO" > /dev/null 2>&1
    cd ..

    # Copy debug files
    local count=0
    for f in Examples/*.RO.dbg; do
        if [ -f "$f" ]; then
            cp "$f" "$dest/"
            count=$((count + 1))
        fi
    done
    for f in Examples/*.RO.dbg2; do
        [ -f "$f" ] && cp "$f" "$dest/"
    done
    for f in Examples/*.RO.dbg3; do
        [ -f "$f" ] && cp "$f" "$dest/"
    done

    if [ "$count" -eq 0 ]; then
        echo "ERROR: No .RO.dbg files produced. Is LoggingLevel > 0?"
        exit 1
    fi
    echo "Captured $count debug file(s) to $dest/"
}

compare() {
    if [ ! -d "$BASELINE_DIR" ]; then
        echo "ERROR: No baseline files. Run: bash scripts/debug_test.sh baseline"
        exit 1
    fi
    if [ ! -d "$INTEGRATED_DIR" ]; then
        echo "ERROR: No integrated files. Run: bash scripts/debug_test.sh integrated"
        exit 1
    fi

    local pass=0
    local fail=0

    for baseline_file in "$BASELINE_DIR"/*.RO.dbg*; do
        local fname=$(basename "$baseline_file")
        local integrated_file="$INTEGRATED_DIR/$fname"

        if [ ! -f "$integrated_file" ]; then
            echo "FAIL $fname: missing in integrated output"
            fail=$((fail + 1))
            continue
        fi

        # Compare row counts
        local bl_rows=$(wc -l < "$baseline_file")
        local il_rows=$(wc -l < "$integrated_file")
        if [ "$bl_rows" -ne "$il_rows" ]; then
            echo "FAIL $fname: row count mismatch (baseline=$bl_rows, integrated=$il_rows)"
            fail=$((fail + 1))
            continue
        fi

        # Compare data rows (skip first 3 header lines)
        local data_diff=$(diff <(tail -n +4 "$baseline_file") <(tail -n +4 "$integrated_file") | head -5)
        if [ -z "$data_diff" ]; then
            local data_rows=$((bl_rows - 3))
            echo "  OK $fname: $data_rows data rows IDENTICAL"
            pass=$((pass + 1))
        else
            echo "FAIL $fname: data rows differ"
            echo "$data_diff"
            fail=$((fail + 1))
        fi
    done

    echo ""
    local total=$((pass + fail))
    echo "=== Debug test: $pass/$total passed ==="
    if [ "$fail" -gt 0 ]; then
        exit 1
    fi
}

case "${1:-}" in
    baseline)
        echo "Capturing baseline debug output (scenario $SCENARIO)..."
        capture "$BASELINE_DIR"
        ;;
    integrated)
        echo "Capturing integrated debug output (scenario $SCENARIO)..."
        capture "$INTEGRATED_DIR"
        ;;
    compare)
        compare
        ;;
    all)
        echo "=== Full debug test (scenario $SCENARIO) ==="
        echo ""
        echo "Step 1: Capturing baseline..."
        capture "$BASELINE_DIR"
        echo ""
        echo "Step 2: Capturing integrated..."
        capture "$INTEGRATED_DIR"
        echo ""
        echo "Step 3: Comparing..."
        compare
        ;;
    clean)
        rm -rf debug_test/
        echo "Cleaned debug test artifacts."
        ;;
    *)
        echo "Usage: bash scripts/debug_test.sh {baseline|integrated|compare|all|clean} [--scenario N]"
        echo ""
        echo "  baseline    - Run sim and capture baseline .RO.dbg files"
        echo "  integrated  - Run sim and capture integrated .RO.dbg files"
        echo "  compare     - Diff data rows between baseline and integrated"
        echo "  all         - Run all three steps sequentially"
        echo "  clean       - Remove debug test artifacts"
        exit 1
        ;;
esac
