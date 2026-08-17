#!/bin/bash
# ONE differential-harness red test per invocation, for unit #48 `AeroDynTorque`.
#
#   bash scripts/run_if_time_remains.sh 300 \
#       bash evidence/AeroDynTorque/run_harness_redtest.sh <stub.cpp> <out.json> "<what was perturbed>"
#
# Copied from evidence/ActiveWakeControl/run_harness_redtest.sh (unit #47) with
# the unit, the module and the path changed, plus ONE addition stated below.
# Its three rules hold here unchanged:
#
# THE CORPUS IS REGENERATED, AND THE CASE COUNT IS THE CHECK. Unit #26's rule is
# that a red result and the green it certifies must name the same count.
# `vit_harness.py` derives the corpus from the FORTRAN reference alone, so a
# red-test stub containing no predicate cannot collapse the corpus that proves
# the harness can fail. If a run below reports anything other than the committed
# `harness/AeroDynTorque.json`'s 1131, the two results are not about one corpus
# and the difference is the finding.
#
# THE ADDITION: `--disable R13_staging_capacity`, WHICH THE GREEN ALSO CARRIES.
# That is the whole reason it is here -- the flag changes the corpus (1387 ->
# 1131), so a red test taken without it would be a red test over a corpus the
# green was never taken on, which is exactly the count check above. The reason
# the green carries it is in evidence/AeroDynTorque/harness.staging_composition.txt.
#
# RUN IT ON THE CLEAN TREE. This unit calls `interp2d`, which IS integrated: on
# an integrated tree the Fortran reference reaches the C++ interp2d through its
# wrapper and the C++ side reaches the same object, so the callee is not a
# control at all. `scripts/harness.sh` decides that per tree and prints which
# way it went ("keeping the interp2d_c bridge" on a clean tree).
#
# The EXIT trap restores the translation from git. COMMIT THE TRANSLATION FIRST:
# unit #43 measured that an uncommitted edit to it is destroyed silently here.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/Functions/aerodyntorque.cpp
E=evidence/AeroDynTorque
trap 'git checkout -- "$CPP"; echo "restored $CPP"' EXIT
cp "$E/$1" "$CPP"
bash scripts/harness.sh AeroDynTorque Functions aerodyntorque \
    rosco/controller/src/Functions.f90 \
    --against translation --disable R13_staging_capacity \
    --out "$2" --red-test "$3" 2>&1 | tail -12
