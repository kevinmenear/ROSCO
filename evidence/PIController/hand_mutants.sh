#!/bin/bash
# The mutation measurement `cppmutate` cannot make for THIS unit, made by hand.
#
# WHY THIS EXISTS, and it is a different gap from unit #24's. `cppmutate` DOES
# have three call operators -- `drop_call`, `swap_call_args`, `swap_callee` --
# and unit #24 is the unit that added them. All three are gated on a TABLE of
# callee names (`_VALUE_PRESERVING`, `_SIBLINGS`), and every entry in both
# tables is a C standard-library name. `saturate_c` is not one, so the two calls
# this unit makes are the two sites its 22 generated mutants do not touch:
#
#     piP->ITerm[i]       = saturate_c(piP->ITerm[i], minValue, maxValue);
#     PIController_result = saturate_c(PTerm + piP->ITerm[i], minValue, maxValue);
#
# The table is a MEASURED restriction, not conservatism -- `cppmutate`'s own
# comment records that letting `drop_call` fire on every call made four of this
# campaign's units 38-73% unbuildable -- and `saturate_c` would satisfy the
# property the table exists to guarantee (result type == first argument's type,
# all three parameters `double`). Adding it is therefore a plausible amendment
# AND a campaign-wide re-take of every scored unit, which X3 forbids mid-run.
# So the measurement is made here instead, exactly as unit #24 made it before
# the operator existed, and the amendment is proposed in DECISIONS.md.
#
# Two RUNBOOK rules are load-bearing in the recipe and both were paid for
# elsewhere in this campaign:
#   * `make -s test`, NOT a bare `make` -- the generated Makefile's FIRST target
#     is `picontroller.hpp: <the translation>`, so a bare `make` copies the
#     header, relinks nothing, and ./test reports the PREVIOUS mutant's verdict.
#   * `|| true` on the run -- ./test exits non-zero when cases fail, which under
#     `set -e` would kill the script and leave an empty artifact.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
CPP=translations/Controllers/picontroller.cpp
TESTDIR=translations/Controllers/picontroller_test

A='piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);'
B='PIController_result = saturate_c(PTerm + piP->ITerm[i], minValue, maxValue);'

cp "$CPP" /tmp/picontroller.shipped.cpp

run_one() {   # $1 = label, $2 = replacement for A, $3 = replacement for B
    python3 - "$2" "$3" <<'PY'
import sys, pathlib
a_new, b_new = sys.argv[1], sys.argv[2]
s = pathlib.Path('/tmp/picontroller.shipped.cpp').read_text()
A = 'piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);'
B = 'PIController_result = saturate_c(PTerm + piP->ITerm[i], minValue, maxValue);'
assert s.count(A) == 1, 'call site A not found exactly once -- refusing to write a mutant'
assert s.count(B) == 1, 'call site B not found exactly once -- refusing to write a mutant'
pathlib.Path('translations/Controllers/picontroller.cpp').write_text(
    s.replace(A, a_new).replace(B, b_new))
PY
    # The bind-mount hazard unit #23 and unit #30 both paid for: a file written
    # on the Mac and stat()ed from inside the container does not reliably carry
    # a newer mtime than the artifact built from it a moment earlier. Delete the
    # derived files rather than trusting a timestamp, and hash the input from
    # INSIDE the container before building anything.
    want=$(md5 -q "$CPP")
    got=""
    for i in 1 2 3; do
        got=$(docker exec vit-dev bash -lc "md5sum /workspace/ROSCO-r2/$CPP | cut -d' ' -f1")
        [ "$want" = "$got" ] && break
        sleep 1
    done
    if [ "$want" != "$got" ]; then
        printf '%-58s HASH MISMATCH -- not measured\n' "$1"; return
    fi
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$TESTDIR && rm -f picontroller.hpp picontroller_test.o test && \
         make -s test >/dev/null 2>&1 && ./test 2>/dev/null" \
        > /tmp/hand_mutant.out 2>/dev/null || true
    read -r failed checked <<<"$(python3 - <<'PY'
import json
t = open('/tmp/hand_mutant.out').read()
d, dec = None, json.JSONDecoder()
for i, ch in enumerate(t):
    if ch == '{':
        try:
            obj, _ = dec.raw_decode(t[i:])
        except ValueError:
            continue
        if isinstance(obj, dict) and 'checked' in obj:
            d = obj
print(f"{d['failed']} {d['checked']}" if d else "? ?")
PY
)"
    printf '%-58s killed on %5s of %5s cases\n' "$1" "$failed" "$checked"
}

echo "# hand-run mutants of translations/Controllers/picontroller.cpp"
echo "# the two saturate_c call sites, which cppmutate's callee TABLE excludes"
echo "# instrument: this unit's own differential corpus"
echo "#"
run_one "BASELINE (the shipped translation, unmutated)" "$A" "$B"
echo "#"
echo "# drop_call -- the call replaced by its first argument"
run_one "ITerm clamp dropped"        'piP->ITerm[i] = piP->ITerm[i];' "$B"
run_one "output clamp dropped"       "$A" 'PIController_result = PTerm + piP->ITerm[i];'
run_one "BOTH clamps dropped"        'piP->ITerm[i] = piP->ITerm[i];' 'PIController_result = PTerm + piP->ITerm[i];'
echo "#"
echo "# swap_call_args -- the FIRST TWO arguments exchanged, which is the shape"
echo "# cppmutate's operator produces"
run_one "ITerm clamp: value <-> minValue"  'piP->ITerm[i] = saturate_c(minValue, piP->ITerm[i], maxValue);' "$B"
run_one "output clamp: value <-> minValue" "$A" 'PIController_result = saturate_c(minValue, PTerm + piP->ITerm[i], maxValue);'
echo "#"
echo "# the two BOUNDS transposed -- not a shape cppmutate offers at all, and the"
echo "# defect unit #24's own post-integration red test was built from"
run_one "ITerm clamp: bounds transposed"   'piP->ITerm[i] = saturate_c(piP->ITerm[i], maxValue, minValue);' "$B"
run_one "output clamp: bounds transposed"  "$A" 'PIController_result = saturate_c(PTerm + piP->ITerm[i], maxValue, minValue);'

cp /tmp/picontroller.shipped.cpp "$CPP"
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/$TESTDIR && make -s test >/dev/null 2>&1" || true
echo "#"
if diff -q /tmp/picontroller.shipped.cpp "$CPP" >/dev/null; then
    echo "# translation restored, byte-identical to the shipped file"
else
    echo "# WARNING: translation NOT restored"
fi
