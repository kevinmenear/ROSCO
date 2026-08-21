#!/bin/bash
# The POST-INTEGRATION harness's red test for unit #65, perturbed IN THE WRAPPER.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/WindSpeedEstimator/run_postintegration_redtest.sh
#
# Post mode measures the MARSHALLING, not the arithmetic: after integration the
# Fortran body IS the translation, so both sides run the same code by
# construction and a failure means an argument was corrupted crossing the view.
# The perturbation therefore has to be in the WRAPPER (unit #56's rule), and the
# one that matters here is the copy-back: this unit writes 7 LocalVar scalars on
# every path plus the whole nested `WE` state, and without
# `vit_copy_scalars_to_localvariables` NONE of it reaches the caller's type.
#
# THE PREDICTION, WRITTEN BEFORE THE RUN: near 63,020 of 63,020. Unit #58's rule
# is that a copy-back red test's PASS set is "the unit changed NOTHING in the
# type", not "the unit wrote nothing" -- so a case passes only if every value
# the unit writes already equalled what was there. `LocalVar%WE_Op_Last` is
# assigned `LocalVar%WE_Op` and both are flags over {0,1}, so about half the
# corpus has them already equal; the other six writes are computed reals drawn
# independently. A shortfall of more than a handful means one of those seven is
# not moving.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
F=rosco/controller/src/ControllerBlocks.f90
KEEP=$(mktemp)
cp "$F" "$KEEP"
trap 'cp "$KEEP" "$F"; rm -f "$KEEP"; echo "run_postintegration_redtest: wrapper restored"' EXIT

python3 - "$F" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
start = s.index("    SUBROUTINE WindSpeedEstimator(")
end   = s.index("    END SUBROUTINE WindSpeedEstimator", start)
body  = s[start:end]
line  = "        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)\n"
assert line in body, "the copy-back line is not in this unit's wrapper"
p.write_text(s[:start] + body.replace(line, "", 1) + s[end:])
print("red test: vit_copy_scalars_to_localvariables deleted from this unit's wrapper")
PY

docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j8 > /tmp/rt_build.log 2>&1; rc=\$?; test \$rc -eq 0 || { tail -5 /tmp/rt_build.log; exit 1; }; cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so" || exit 1

bash scripts/harness.sh WindSpeedEstimator ControllerBlocks windspeedestimator \
    rosco/controller/src/ControllerBlocks.f90 \
    --post-integration \
    --out harness/WindSpeedEstimator.postintegration.redtest.json \
    --red-test "the wrapper's own vit_copy_scalars_to_localvariables deleted" \
    2>&1 | grep -E "POST-INTEGRATION|no JSON" || true
