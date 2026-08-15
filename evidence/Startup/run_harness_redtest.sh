#!/bin/bash
# ONE differential-harness red test per invocation, for unit #42 `Startup`.
#
#   bash scripts/run_if_time_remains.sh 300 \
#       bash evidence/Startup/run_harness_redtest.sh <stub.cpp> <out.json> "<what was perturbed>"
#
# THE CORPUS IS REGENERATED, AND THE CASE COUNT IS THE CHECK. Unit #26's rule is
# that a red result and the green it certifies must name the same count.
# `vit_harness.py` derives the corpus from the FORTRAN reference alone -- the
# literals, the predicate knobs and the state plan are all read out of the .f90,
# deliberately, so that a red-test stub containing no predicate cannot collapse
# the corpus that proves the harness can fail. The generating run wrote 7103
# cases; if a run below reports any other number, the two results are not about
# one corpus and the difference is the finding.
#
# The EXIT trap restores the translation from git. `vit_mutate.py` and every stub
# runner in this campaign edit the shipped .cpp IN PLACE, and three of three hard
# kills in this campaign have left the edit behind.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/ControllerBlocks/startup.cpp
E=evidence/Startup
trap 'git checkout -- "$CPP"; echo "restored $CPP"' EXIT
cp "$E/$1" "$CPP"
bash scripts/harness.sh Startup ControllerBlocks startup \
    rosco/controller/src/ControllerBlocks.f90 \
    --against translation --out "$2" --red-test "$3" 2>&1 | tail -12
