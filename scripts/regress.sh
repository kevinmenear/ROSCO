#!/bin/bash
# Full 27-scenario regression for the VIT replication.
#
#   bash scripts/regress.sh <FunctionName>     # gate after integrating a function
#   bash scripts/regress.sh --baseline         # (re)generate the baseline set
#
# Each scenario runs in a separate process. This is not optional: ROSCO's
# Fortran SAVE variables live in DLL memory that dlclose() does not reliably
# release, so running scenarios in one process contaminates later ones with
# earlier state. Measured cost of the isolated run: ~51 s for all 27.
#
# Baselines live in baseline_arrays/ and are generated on THIS fork, not
# copied from the original campaign. The campaign's committed arrays carry a
# known scenario-16 bld_pitch residual; gating against them would fail every
# function for a reason unrelated to that function.
#
# Emits telemetry so regress time appears in per-function cost. Without the
# bracketing, this step is invisible to the joiner and every total is low.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

SCENARIOS="3 4 5 1 2 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27"
BASELINE_DIR="$ROOT/baseline_arrays"
CONTAINER="${VIT_CONTAINER:-vit-dev}"
WORKDIR="/workspace/$(basename "$ROOT")"

MODE="gate"
FUNCTION="${1:-}"
if [ "${1:-}" = "--baseline" ]; then
    MODE="baseline"
    FUNCTION=""
fi

if [ "$MODE" = "baseline" ]; then
    OUT_DIR="$BASELINE_DIR"
    OUT_CONTAINER="$WORKDIR/baseline_arrays"
else
    OUT_DIR="$ROOT/.regress_current"
    OUT_CONTAINER="$WORKDIR/.regress_current"
    if [ -z "$FUNCTION" ]; then
        echo "usage: regress.sh <FunctionName> | --baseline" >&2
        exit 2
    fi
fi

telemetry() {
    docker exec "$CONTAINER" bash -c \
        "cd $WORKDIR && VIT_TELEMETRY_LOG=$WORKDIR/.vit/cycle_log.jsonl \
         PYTHONPATH=/workspace/vit python3 -m vit.telemetry $*" 2>/dev/null
}

START_EPOCH=$(date +%s)
telemetry start --function "${FUNCTION:-__baseline__}" --command regress

rm -rf "$OUT_DIR"
mkdir -p "$OUT_DIR"

FAILED_SCENARIOS=""
for s in $SCENARIOS; do
    if ! docker exec "$CONTAINER" bash -c \
        "cd $WORKDIR/Examples && python3 vit_sim.py --scenario $s --output-dir $OUT_CONTAINER" \
        > /dev/null 2>&1; then
        FAILED_SCENARIOS="$FAILED_SCENARIOS $s"
    fi
done

RC=0
if [ -n "$FAILED_SCENARIOS" ]; then
    echo "SCENARIOS FAILED TO RUN:$FAILED_SCENARIOS"
    RC=1
elif [ "$MODE" = "baseline" ]; then
    echo "Baseline written: $(ls "$BASELINE_DIR" | wc -l | tr -d ' ') scenario files"
else
    docker exec "$CONTAINER" python3 - "$WORKDIR" <<'PY'
import sys, numpy as np, os
root = sys.argv[1]
bad = total = 0
missing = []
for s in range(1, 28):
    b = os.path.join(root, 'baseline_arrays', 'scenario_%d.npz' % s)
    c = os.path.join(root, '.regress_current', 'scenario_%d.npz' % s)
    if not (os.path.exists(b) and os.path.exists(c)):
        missing.append(s)
        continue
    B, C = np.load(b), np.load(c)
    for k in sorted(B.files):
        total += 1
        if k not in C.files or not np.array_equal(B[k], C[k]):
            bad += 1
            print('  MISMATCH scenario_%d %s' % (s, k))
if missing:
    print('  MISSING scenarios: %s' % missing)
print('channels compared: %d  mismatched: %d' % (total, bad))
sys.exit(1 if (bad or missing) else 0)
PY
    RC=$?
    if [ $RC -eq 0 ]; then
        echo "REGRESS PASS: 27/27 scenarios bit-identical"
    else
        echo "REGRESS FAIL"
    fi
fi

telemetry end --function "${FUNCTION:-__baseline__}" --command regress \
    --exit-code "$RC" --duration-s "$(( $(date +%s) - START_EPOCH ))"

exit $RC
