#!/bin/bash
# DEMONSTRATION of the instrument the sanitiser amendment names, on the one
# mutant this unit could not kill. It changes NO default, writes NO score, and
# `mutation/FindLine.json` does not know it happened.
#
#   bash scripts/reset_to_clean.sh
#   bash evidence/FindLine/run_asan_demo.sh
#   bash scripts/restore_integrated.sh
#
# WHAT IT IS FOR. `ca75abea` (`const_tweak '2048' -> '2049'`) makes
# `char_assign` write one byte past the caller's 2048-byte buffer. It reports 0
# against the differential harness (2514 cases) and 0 against the gate
# (5,252,000 values), and both zeros have a positive control at the same site,
# so the mutant is genuinely invisible to a value comparison rather than sitting
# behind a corpus that is too narrow. `DECISIONS.md` proposes a sanitiser build
# as the instrument that would kill it and every mutant of its class.
#
# That proposal has been carried across three units on an ARGUMENT. Two
# questions it does not answer, and this script answers both:
#
#   1. does the instrument fire on the mutant at all, on this toolchain?
#   2. does it stay silent on the CORRECT program -- i.e. is a kill under it a
#      statement about the mutant rather than about the Fortran runtime?
#
# The second is the one that decides whether the amendment is one CMake option
# or a project. A sanitiser that reports on the unmutated translation would make
# every mutant "killed" and the score meaningless -- the exact shape of a green
# that established nothing, one sign flipped.
#
# WHAT IT IS NOT FOR. It does not answer the campaign-wide question the
# amendment actually turns on: whether the other 31 translations are clean under
# it. That is a sweep and it is a dispatch of its own.
#
# THE TREE MUST BE CLEAN. The pre-integration harness links this campaign's
# Fortran objects; after integration `ROSCO_Helpers.f90.o` IS the wrapper and
# there is no independent reference left. Same requirement as
# `run_harness_stub.sh`, and the hash-verify below is that script's (unit #23).
#
# LEAK DETECTION IS OFF. libgfortran's I/O buffers are reachable at exit and
# LeakSanitizer reports them; they are not what this measures, and leaving them
# on would put a report on the correct program's run and answer question 2 with
# a false yes.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

LIVE=translations/ROSCO_Helpers/findline.cpp
DIR=translations/ROSCO_Helpers/findline_test
OUT=evidence/FindLine/asan_demo
KEEP=$(mktemp); cp "$LIVE" "$KEEP"
restore() { cp "$KEEP" "$LIVE"; rm -f "$KEEP"; }
trap restore EXIT
mkdir -p "$OUT"

build_and_run() {                 # $1 = the .cpp to put in place, $2 = label
    local want got
    cp "$1" "$LIVE"
    want=$(md5 -q "$1")
    for _ in 1 2 3; do
        got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$LIVE | cut -d' ' -f1")
        [ "$want" = "$got" ] && break
        cp "$1" "$LIVE"
    done
    [ "$want" = "$got" ] || { echo "HASH MISMATCH on $LIVE"; return 1; }
    echo "=== $2   (md5 verified in-container: $got)"
    # The report goes to an absolute path INSIDE the container. Written
    # relative, it lands in the harness directory `cd` put us in -- which is
    # where the first run of this script wrote nothing at all and reported
    # `exit 1` for both the mutant and the correct program, i.e. a result that
    # looked like the sanitiser firing on both.
    docker exec vit-dev bash -lc "
        cd /workspace/ROSCO-r2/$DIR &&
        rm -f findline_test.o findline.hpp test &&
        make CXX='g++ -fsanitize=address -g -fno-omit-frame-pointer' test > /tmp/asan_build.log 2>&1 || {
            echo 'BUILD FAILED -- the instrument does not link on this toolchain'
            tail -5 /tmp/asan_build.log; exit 9; }
        ASAN_OPTIONS=detect_leaks=0 ./test findline_cases.bin > /dev/null 2>/tmp/asan_run.txt
        rc=\$?
        cp /tmp/asan_run.txt /workspace/ROSCO-r2/$OUT/$2.asan.txt
        echo \"./test exit \$rc, \$(wc -c < /tmp/asan_run.txt) byte(s) on stderr\"
        head -3 /tmp/asan_run.txt
    " 2>&1
}

# The correct program FIRST. If this one reports, nothing below it means
# anything, and reading the mutant's result first would let that pass unnoticed.
build_and_run "$KEEP" original
build_and_run evidence/FindLine/findline.linewidth-mutant.cpp mutant-ca75abea

restore; trap - EXIT
echo "--- shipped translation restored to $(git hash-object $LIVE)"
