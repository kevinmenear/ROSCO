#!/bin/bash
# ONE kernel hand red test per invocation, for unit #43 `StructuralControl`.
#
# `vit verify` DECLINED to construct its own red test here and printed
# NON_DISCRIMINATING -- the same refusal it gave units #40, #41 and #42, and for
# the same stated reason: every argument arrives behind a pointer or inside a
# derived type, so there is no by-value floating-point input it can perturb
# without changing what BOTH implementations compute. So the kernel's
# discriminating power has to be measured by hand or not at all.
#
# ONE STUB PER COMMAND, deliberately. Unit #40 tried four in one foreground
# command and was killed at the 600-second tool ceiling inside its third,
# orphaning a `vit verify` in the container. Each `vit verify` on this kernel
# takes ~230s at 62 cases.
#
#   bash scripts/run_if_time_remains.sh 400 \
#       bash evidence/StructuralControl/run_one_kernel_stub.sh <stub.cpp> "<name>" "<asks>" "<expect>"
#
# `-f` IS THE PRE-INTEGRATION FILE, NOT THE LIVE ONE. `vit integrate` has since
# replaced this unit's Fortran body with a wrapper that calls
# `structuralcontrol_c`, and the green these stubs are red tests FOR was taken
# against the body. The kernel directory carries its own generated Fortran and
# `-f` supplies the signature, so a gitignored copy taken from the commit before
# the integration is both sufficient and exactly what the green used (RUNBOOK,
# unit #11):
#
#   git show 57308e02^:rosco/controller/src/Controllers.f90 \
#       > .vit/Controllers.preint.f90
#
# The EXIT trap restores the translation AND vit.yaml from git: `vit verify`
# edits the .cpp path it is handed and rewrites vit.yaml (stripping every
# comment) on every run. A killed dispatch that does not restore leaves a MUTANT
# live in the shipped translation -- unit #40's third stub did exactly that.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/Controllers/structuralcontrol.cpp
E=evidence/StructuralControl; OUT=$E/kernel.stubs.txt
REF=.vit/Controllers.preint.f90
[ -s "$REF" ] || { echo "run_one_kernel_stub: $REF is missing; see this script's header" >&2; exit 2; }
trap 'git checkout -- "$CPP" vit.yaml; docker exec vit-dev bash -lc "pkill -9 -f \"vit verify StructuralControl\"" >/dev/null 2>&1; echo "restored $CPP and vit.yaml"' EXIT
{ echo "=================================================================="
  echo "STUB      $2"; echo "ASKS      $3"; echo "EXPECT    $4"; echo "RESULT:"; } >> "$OUT"
cp "$E/$1" "$CPP"
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && vit verify StructuralControl $CPP \
    -f $REF --kernel-dir kernel/StructuralControl" 2>&1 \
  | grep -E "VERIFICATION|NON_DISCRIMINATING|passed|FAILED|mismatch" | head -6 | sed 's/^/    /' >> "$OUT"
echo >> "$OUT"; tail -8 "$OUT"
