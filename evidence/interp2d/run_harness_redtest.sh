#!/bin/bash
# ONE differential-harness red test per invocation, for unit #45 `interp2d`.
#
#   bash scripts/run_if_time_remains.sh 300 \
#       bash evidence/interp2d/run_harness_redtest.sh <stub.cpp> <out.json> "<what was perturbed>"
#
# THE CORPUS IS REGENERATED, AND THE CASE COUNT IS THE CHECK. Unit #26's rule is
# that a red result and the green it certifies must name the same count.
# `vit_harness.py` derives the corpus from the FORTRAN reference alone, so a
# red-test stub containing no predicate cannot collapse the corpus that proves
# the harness can fail. If a run below reports a different case count from the
# committed `harness/interp2d.json`, the two results are not about one corpus
# and the difference is the finding.
#
# RUN IT ON THE CLEAN TREE. `interp2d` calls `interp1d`, which is integrated: on
# an integrated tree the Fortran reference reaches the C++ interp1d through its
# wrapper and the C++ side reaches the same object, so the callee is not a
# control at all. `scripts/harness.sh` drops the generated callee bridge exactly
# when the tree is integrated AND the object is linked, which is the same
# question read from the same place.
#
# The EXIT trap restores the translation from git. COMMIT THE TRANSLATION FIRST:
# unit #43 measured that an uncommitted edit to it is destroyed silently here.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/Functions/interp2d.cpp
E=evidence/interp2d
trap 'git checkout -- "$CPP"; echo "restored $CPP"' EXIT
cp "$E/$1" "$CPP"
bash scripts/harness.sh interp2d Functions interp2d \
    rosco/controller/src/Functions.f90 \
    --against translation --out "$2" --red-test "$3" 2>&1 | tail -12
