#!/bin/bash
# The differential harness's red test for unit #65: the unit as a NO-OP.
#
#   bash evidence/WindSpeedEstimator/run_harness_redtest.sh
#
# Swaps the stub in, runs the SAME corpus through `harness.sh`, and puts the
# translation back whatever happens -- the `trap` is the point, not decoration:
# `vit_mutate.py`'s in-place edit has left a mutant behind three times in this
# campaign when it was killed, and a red test is the same shape.
#
# `--against translation`, both `--transitive-read-set` sources and both
# `--persist-nested` members are repeated verbatim from the green run. They have to be: the flags decide the corpus, and
# unit #26's finding is that a red result and the green it certifies must name
# the same case count.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

CPP=translations/ControllerBlocks/windspeedestimator.cpp
STUB=evidence/WindSpeedEstimator/windspeedestimator.noop-harness-stub.cpp
KEEP=$(mktemp)

cp "$CPP" "$KEEP"
trap 'cp "$KEEP" "$CPP"; rm -f "$KEEP"; echo "run_harness_redtest: translation restored"' EXIT

cp "$STUB" "$CPP"
bash scripts/harness.sh WindSpeedEstimator ControllerBlocks windspeedestimator \
    rosco/controller/src/ControllerBlocks.f90 \
    --against translation \
    --out harness/WindSpeedEstimator.redtest.json \
    --red-test "the unit as a NO-OP: every argument taken, nothing written" \
    --transitive-read-set rosco/controller/src/Functions.f90 \
    --transitive-read-set rosco/controller/src/Filters.f90 \
    --persist-nested LocalVar.WE \
    --persist-nested LocalVar.FP \
    2>&1 | grep -E "^HARNESS|no JSON|case\(s\)$|red_test" || true
