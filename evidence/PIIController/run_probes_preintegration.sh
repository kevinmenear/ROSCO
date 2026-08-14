#!/bin/bash
# Run this unit's counting probes on the tree the committed green was taken on:
# every OTHER unit integrated, this unit's Fortran body still the real body.
#
#   bash evidence/PIIController/run_probes_preintegration.sh
#
# WHY A DEDICATED SCRIPT AND NOT reset_to_clean.sh. Two tree states produce two
# different corpora and only one of them is the one `harness/PIIController.json`
# and `mutation/PIIController.json` were taken over:
#
#   fully integrated      the reference is a marshalling wrapper: 4607 cases,
#                         and the no-op scores 0 -- both sides run the probe
#   this unit reverted    the reference is the 45-line Fortran body: 4528 cases
#   fully clean           every OTHER unit's body is also restored, so the
#                         C++ side's callees are different objects again
#
# `git checkout <pre-integration commit> --` on the three integration-carrying
# paths lands on the middle one exactly. It is NOT the reset/restore pair and
# does not trip its marker, so the window is three files wide instead of the
# whole tree -- and it is put back from HEAD unconditionally at the end.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
PRE=6b38144        # the commit before `vit integrate --apply`
FILES="rosco/controller/src/Controllers.f90 rosco/controller/CMakeLists.txt rosco/controller/src/vit_translated.h"

build() {
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/rosco/controller/build && cmake . >/dev/null && cmake --build . -j4 >/dev/null 2>&1 && \
         cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
}

restore() {
    git checkout HEAD -- $FILES
    build
    echo "run_probes: the three integration paths restored from HEAD and rebuilt"
}
trap restore EXIT

git checkout "$PRE" -- $FILES
build

# P10 FIRST, ALWAYS. A probe number from a comparison with one side is not a
# number. This is the same file the committed red test used.
bash evidence/PIIController/run_probe.sh \
     evidence/PIIController/piicontroller.noop-stub.cpp \
     evidence/PIIController/probes/noop-control-preintegration.json \
     "P10 CONTROL: the unit as a no-op, on the tree the probes run on"

for p in control-else-arm control-reset-arm itermlast2-repaired; do
    bash evidence/PIIController/run_probe.sh \
         "evidence/PIIController/probes/$p.cpp" \
         "evidence/PIIController/probes/$p.json" "$p"
done
