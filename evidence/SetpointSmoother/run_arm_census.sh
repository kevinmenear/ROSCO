#!/bin/bash
# ARM CENSUS for unit #40. Two probes, each deleting exactly one arm, run over
# the same 5599-case corpus as the green harness and the no-op red test.
#
# WHY. The no-op red test fails 5355 of 5599 and its mismatch list -- truncated
# at eight -- names only ELSE-arm cases (`ref` is 0000000000000000, which is the
# ELSE arm's constant). The kernel's 62 cases are the mirror image: stub 2 of
# `run_kernel_stubs.sh` changed the ELSE arm's 0 to 12345.0 and still passed
# 62 of 62, so NO captured case takes that arm. Neither instrument can say on
# its own that both arms are covered. Two counts that sum to 5355 can.
#
# Restores from `git checkout` on an EXIT trap.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
CPP=translations/ControllerBlocks/setpointsmoother.cpp
E=evidence/SetpointSmoother
trap 'git checkout -- "$CPP"; echo "restored $CPP"' EXIT

run () {
    local stub="$1" out="$2" why="$3"
    cp "$E/$stub" "$CPP"
    bash scripts/harness.sh SetpointSmoother ControllerBlocks setpointsmoother \
         rosco/controller/src/ControllerBlocks.f90 --against translation \
         --out "$out" --red-test "$why" 2>&1 | grep -E "^HARNESS|^  case|RED TEST"
    git checkout -- "$CPP"
}

run setpointsmoother.if-arm-deleted-stub.cpp   "$E/harness.if-arm-deleted.json" \
    "arm census: the SS_Mode == 1 arm deleted, the ELSE arm intact"
run setpointsmoother.else-arm-deleted-stub.cpp "$E/harness.else-arm-deleted.json" \
    "arm census: the ELSE arm's SS_DelOmegaF = 0 deleted, the SS_Mode == 1 arm intact"
