#!/bin/bash
# WHAT `--disable R13_staging_capacity` COST, MEASURED RATHER THAN ASSERTED.
#
# RETIRED BY THE SECOND DISPATCH -- DO NOT RE-RUN AS IT STANDS, for the same
# reason as run_survivors_on_full_corpus.sh beside it: it regenerates the corpus
# with `--disable R13_staging_capacity`, which is no longer what the scored
# corpus carries. Its measurement is what the replacement rests on.
#
#   bash scripts/run_if_time_remains.sh 400 bash evidence/AeroDynTorque/run_r13_price.sh
#
# The scored corpus is the ABLATED one (1131 cases) and mutant `88466711` --
# `s.size() > n_ErrMsg_cap` -> `>=` in `assign_errmsg` -- survives it. The
# question this script answers with a number is whether R13's 256 cap-swept
# cases WOULD have killed it. Two runs on the FULL 1387-case corpus:
#
#   the unmutated translation   the committed first take, 14 failed
#                               (harness.first_take.staging_composition.json)
#   the mutant                  this run
#
# Both runs are RED -- the full corpus is red for this unit, which is the whole
# reason it was ablated -- so the comparison is between two red numbers and the
# DIFFERENCE is the measurement. The mutant is killed by R13 if and only if it
# fails cases the unmutated translation passes.
#
# Four of the five other units at this site (ReadAvrSWAP, ExtController,
# UpdateZeroMQ, interp1d, interp2d) declared this mutant UNREACHABLE with a
# measurement -- and interp2d's corpus HAD R13 applied, with `s.size() == cap`
# reached 0 times, because interp2d prefixes only on the bilinear path and the
# R13 base case does not take it. This unit prefixes on EVERY case with
# aviFAIL < 0, so the same declaration would be false here and is not made.
#
# The .cpp is restored from git by the EXIT trap; it is committed at 4a368e5a.
# The ablated corpus is REGENERATED at the end and its sha256 asserted against
# the one the green and the mutation score were taken on.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/Functions/aerodyntorque.cpp
E=evidence/AeroDynTorque
CORPUS=translations/Functions/aerodyntorque_test/aerodyntorque_cases.bin
WANT=0893b9ebd947a0e14decb7ed027a4c32786f733b1f43f25d69bcc57308a98972

restore() {
    git checkout -- "$CPP"
    echo "restored $CPP"
    # PUT THE SCORED CORPUS BACK. This script generated the FULL one over it.
    bash scripts/harness.sh AeroDynTorque Functions aerodyntorque \
        rosco/controller/src/Functions.f90 --against translation \
        --disable R13_staging_capacity 2>&1 | tail -2
    got=$(shasum -a 256 "$CORPUS" | cut -d' ' -f1)
    if [ "$got" = "$WANT" ]; then
        echo "corpus restored, sha256 $got -- the one the score was taken on"
    else
        echo "CORPUS NOT RESTORED: $got != $WANT" >&2
        exit 4
    fi
}
trap restore EXIT

cp "$E/aerodyntorque.r13-boundary-mutant.cpp" "$CPP"
bash scripts/harness.sh AeroDynTorque Functions aerodyntorque \
    rosco/controller/src/Functions.f90 --against translation \
    --out evidence/AeroDynTorque/harness.r13-boundary-mutant.FULL-corpus.json \
    --red-test "mutant 88466711: assign_errmsg's capacity guard s.size() > cap -> >=, run over the FULL 1387-case corpus that --disable R13_staging_capacity removes" \
    2>&1 | tail -8
