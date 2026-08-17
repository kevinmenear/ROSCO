#!/bin/bash
# WHICH OF THE FOUR SURVIVORS R13 WOULD HAVE KILLED, MEASURED FOR ALL FOUR.
#
# RETIRED BY THE SECOND DISPATCH -- DO NOT RE-RUN AS IT STANDS. Its result stands
# (mutation.survivors_on_full_corpus.txt) and it is what justified replacing the
# whole-rule ablation with a fourteen-capacity hole. But its EXIT trap
# regenerates the corpus with `--disable R13_staging_capacity` and asserts a
# sha256 that no longer describes the scored corpus, so running it now would
# leave the wrong `aerodyntorque_cases.bin` on disk and report a MISMATCH it
# caused itself. Kept because the measurement it took is cited, not because it
# is still a tool.
#
#   bash scripts/run_if_time_remains.sh 700 \
#       bash evidence/AeroDynTorque/run_survivors_on_full_corpus.sh
#
# `run_r13_price.sh` measured this for ONE survivor (88466711: 15 failed against
# the unmutated 14). Stating "R13's kill set here is exactly one mutant" on that
# basis would be a claim about four things measured on one -- the shape this
# campaign's P10 exists to refuse. So all four are run, and the comparison is
# between FAILING CASE SETS rather than between counts: a mutant that turned one
# of the baseline's own 14 failures into a pass while adding one elsewhere would
# show the same count and a different set.
#
# THE WHOLE THING RUNS INSIDE ONE RESET WINDOW THAT IT OPENS AND CLOSES ITSELF.
# The mutation comparison needs the CLEAN tree (on an integrated one both sides
# of the harness end in the same C++), and the window is where commits are
# refused -- so everything here must be committed before it is run and nothing
# here writes an artifact that is not restored afterwards. The EXIT trap runs
# `restore_integrated.sh` unconditionally.
#
# It also regenerates the ABLATED corpus at the end and asserts its sha256
# against the one `harness/AeroDynTorque.json` and `mutation/AeroDynTorque.json`
# were taken on, because this script overwrites it with the full one.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"; cd "$ROOT"
CPP=translations/Functions/aerodyntorque.cpp
D=translations/Functions/aerodyntorque_test
E=evidence/AeroDynTorque
WANT_ABLATED=0893b9ebd947a0e14decb7ed027a4c32786f733b1f43f25d69bcc57308a98972

finish() {
    git checkout -- "$CPP"
    cp /tmp/adt_full_test_pristine.cpp "$D/aerodyntorque_test.cpp" 2>/dev/null || true
    echo "--- regenerating the ABLATED corpus (the one the score was taken on) ---"
    bash scripts/harness.sh AeroDynTorque Functions aerodyntorque \
        rosco/controller/src/Functions.f90 --against translation \
        --disable R13_staging_capacity 2>&1 | tail -2
    got=$(shasum -a 256 "$D/aerodyntorque_cases.bin" | cut -d' ' -f1)
    [ "$got" = "$WANT_ABLATED" ] \
        && echo "corpus sha256 $got -- matches the scored corpus" \
        || echo "CORPUS MISMATCH: $got != $WANT_ABLATED" >&2
    echo "--- closing the reset window ---"
    bash scripts/restore_integrated.sh 2>&1 | tail -3
}
trap finish EXIT

bash scripts/reset_to_clean.sh 2>&1 | tail -3

echo "=== generating the FULL 1387-case corpus (R13 applied) ==="
bash scripts/harness.sh AeroDynTorque Functions aerodyntorque \
    rosco/controller/src/Functions.f90 --against translation 2>&1 | tail -2

# Raise the emitted test's own diff cap so every failing case index is recorded,
# not the first eight. `--no-generate` keeps the test source, so this patch
# persists across the five runs below and is undone by the trap.
cp "$D/aerodyntorque_test.cpp" /tmp/adt_full_test_pristine.cpp
python3 - "$D/aerodyntorque_test.cpp" <<'PY'
import sys
p = sys.argv[1]
s = open(p).read()
assert s.count("diffs.size() < 16") == 1
open(p, "w").write(s.replace("diffs.size() < 16", "diffs.size() < 4000"))
print("diff cap raised to 4000")
PY

run_one() {   # $1 = label, $2 = .cpp to install
    cp "$2" "$CPP"
    bash scripts/harness.sh AeroDynTorque Functions aerodyntorque \
        rosco/controller/src/Functions.f90 --no-generate > /dev/null 2>&1
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$D && make test >/dev/null 2>&1 && ./test aerodyntorque_cases.bin 2>/dev/null" \
      | python3 -c "
import json,sys
d=json.load(sys.stdin)
cases=sorted({m['case'] for m in d['mismatches']})
print('$1  checked %d  failed %d  cases %s' % (d['checked'], d['failed'], cases))
"
}

echo "=== the unmutated translation, FULL corpus (the baseline for the sets below) ==="
git checkout -- "$CPP"
run_one "BASELINE      " "$CPP"

for m in 88466711 11c1e326 06f7d2c8 26021804; do
    echo "=== mutant $m ==="
    run_one "MUTANT $m" "$E/mutant.$m.cpp"
done
