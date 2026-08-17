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
# `harness/AeroDynTorque.json`'s 1373, the two results are not about one corpus
# and the difference is the finding.
#
# THE ADDITION IS GONE, AND SO IS THE FLAG (second dispatch). It used to read
# `--disable R13_staging_capacity`, WHICH THE GREEN ALSO CARRIED -- the flag
# changed the corpus (1387 -> 1131) and a red test taken without it would have
# been taken over a corpus the green was never taken on. The ablation is now a
# STATED HOLE of fourteen capacities in `harness/ranges.toml`
# (`ErrVar_ErrMsg = { staging_capacity_excludes = [16, 29] }`), which is read
# from the file by both runs, so neither needs a flag and the count check above
# still binds. The corpus is 1373, and the reason is in
# evidence/AeroDynTorque/harness.staging_composition.txt.
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
    --against translation \
    --out "$2" --red-test "$3" 2>&1 | tail -12
