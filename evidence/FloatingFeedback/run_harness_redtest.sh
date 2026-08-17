#!/bin/bash
# ONE differential-harness red test per invocation, for unit #51 `FloatingFeedback`.
#
#   bash scripts/run_if_time_remains.sh 600 \
#       bash evidence/FloatingFeedback/run_harness_redtest.sh <stub.cpp> <out.json> "<what was perturbed>"
#
# Copied from evidence/FlapControl/run_harness_redtest.sh (unit #50) with the
# unit, the module and the path changed. Its rules hold unchanged:
#
# THE CORPUS IS REGENERATED, AND THE CASE COUNT IS THE CHECK. Unit #26's rule is
# that a red result and the green it certifies must name the same count.
# `vit_harness.py` derives the corpus from the FORTRAN reference alone, so a
# red-test stub containing no predicate cannot collapse the corpus that proves
# the harness can fail. If a run below reports anything other than the committed
# `harness/FloatingFeedback.json`'s count, the two results are not about one corpus
# and the difference is the finding.
#
# RUN IT ON THE CLEAN TREE. BOTH of this unit's callees -- interp1d and
# PIController -- ARE
# integrated: on an integrated tree the Fortran reference reaches the C++ callee
# through its wrapper and the C++ side reaches the same object, so none of them
# is a control at all. `scripts/harness.sh` decides that per tree and prints
# which way it went ("keeping the <callee>_c bridge" on a clean tree).
#
# The EXIT trap restores the translation from git. COMMIT THE TRANSLATION FIRST:
# unit #43 measured that an uncommitted edit to it is destroyed silently here.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/Controllers/floatingfeedback.cpp
E=evidence/FloatingFeedback
git show HEAD:"$CPP" > /dev/null 2>&1 || {
    echo "run_harness_redtest: $CPP is not in HEAD. The trap below would DELETE it."; exit 1; }
trap 'git checkout -- "$CPP"; echo "restored $CPP"' EXIT
cp "$E/$1" "$CPP"
bash scripts/harness.sh FloatingFeedback Controllers floatingfeedback \
    rosco/controller/src/Controllers.f90 \
    --against translation \
    --out "$2" --red-test "$3" 2>&1 | tail -12
