#!/bin/bash
# One unit's worth of the mutation measurement `cppmutate` cannot make.
#
# WHY THIS EXISTS. This translation's body is a CALL and nothing else:
#
#     return std::fmin(std::fmax(inputValue, minValue), maxValue);
#
# Every one of cppmutate's nine operators needs an arithmetic operator, a
# comparison, a subscript or a numeric literal. A call has none, and neither
# its ARGUMENT ORDER nor its CALLEE NAME is a site -- so `mutants()` returns
# ZERO and the score is not low, it is absent. Adding an operator is a
# campaign-wide re-take (X3); unit #22 established the meanwhile, which is to
# make the missing measurement by hand and commit it.
#
# Two RUNBOOK rules are load-bearing in the recipe and both were paid for:
#   * `make -s test`, NOT a bare `make` -- the generated Makefile's FIRST target
#     is `saturate.hpp: <the translation>`, so a bare `make` copies the header,
#     relinks nothing, and ./test reports the PREVIOUS mutant's verdict.
#   * `|| true` on the run -- ./test exits non-zero when cases fail, which under
#     `set -e` would kill the script and leave an empty artifact.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
CPP=translations/Functions/saturate.cpp
TESTDIR=translations/Functions/saturate_test
SHIPPED='return std::fmin(std::fmax(inputValue, minValue), maxValue);'

cp "$CPP" /tmp/saturate.shipped.cpp

run_one() {   # $1 = label, $2 = replacement body
    python3 - "$1" "$2" <<'PY'
import sys, pathlib
label, body = sys.argv[1], sys.argv[2]
p = pathlib.Path('translations/Functions/saturate.cpp')
s = pathlib.Path('/tmp/saturate.shipped.cpp').read_text()
shipped = 'return std::fmin(std::fmax(inputValue, minValue), maxValue);'
assert shipped in s, 'shipped body not found -- refusing to write a mutant'
p.write_text(s.replace(shipped, body))
PY
    # The Makefile's prerequisites are correct (`saturate_test.o: ... saturate.hpp`
    # and `saturate.hpp: <the translation>`), so unit #18's missing-prerequisite
    # defect is NOT the hazard here. The hazard is the BIND MOUNT: a file written
    # on the Mac and stat()ed from inside the container does not reliably carry a
    # newer mtime than the artifact built from it a moment earlier, so `make`
    # decides the header is up to date and ./test reports the PREVIOUS mutant's
    # verdict. A first run of this script did exactly that -- passthrough read
    # 100 where scripts/harness.sh had measured 202. So delete the derived files
    # rather than trusting a timestamp, and hash the input from INSIDE the
    # container before building anything.
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
        "cd /workspace/ROSCO-r2/$TESTDIR && rm -f saturate.hpp saturate_test.o test && \
         make -s test >/dev/null 2>&1 && ./test 2>/dev/null" \
        > /tmp/hand_mutant.out 2>/dev/null || true
    # The harness prints ONE pretty-printed JSON object, not JSON lines, and it
    # prints prose before it -- so take the LAST parseable object in the stream,
    # which is the rule vit_harness.py and vit_mutate.py already follow.
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

echo "# hand-run mutants of translations/Functions/saturate.cpp"
echo "# instrument: the unit's own differential corpus, 451 cases"
echo "#"
run_one "BASELINE (the shipped translation, unmutated)" "$SHIPPED"
run_one "fmin/fmax transposed"                  'return std::fmax(std::fmin(inputValue, minValue), maxValue);'
run_one "the two BOUNDS transposed"             'return std::fmin(std::fmax(inputValue, maxValue), minValue);'
run_one "upper clamp dropped (the MIN deleted)" 'return std::fmax(inputValue, minValue);'
run_one "lower clamp dropped (the MAX deleted)" 'return std::fmin(inputValue, maxValue);'
run_one "both clamps dropped (passthrough)"     'return inputValue;'
run_one "clamped against minValue twice"        'return std::fmin(std::fmax(inputValue, minValue), minValue);'
run_one "clamped against maxValue twice"        'return std::fmin(std::fmax(inputValue, maxValue), maxValue);'
run_one "std::min/std::max instead of fmin/fmax" 'return std::min(std::max(inputValue, minValue), maxValue);'
run_one "branch spelling A (first arg wins ties)" 'double t = (inputValue > minValue) ? inputValue : minValue; return (t < maxValue) ? t : maxValue;'
run_one "branch spelling B (second arg wins ties)" 'double t = (minValue > inputValue) ? minValue : inputValue; return (maxValue < t) ? maxValue : t;'
echo "#"
echo "# COMMUTATIVITY -- fmax(a,b) and fmax(b,a) agree on every input INCLUDING"
echo "# a signed zero and a NaN (evidence/saturate/minmax_probe.txt), so these two"
echo "# are EQUIVALENT mutants rather than uncaught ones. Listed to be counted."
run_one "fmax arguments swapped (equivalent)"   'return std::fmin(std::fmax(minValue, inputValue), maxValue);'
run_one "fmin arguments swapped (equivalent)"   'return std::fmin(maxValue, std::fmax(inputValue, minValue));'

cp /tmp/saturate.shipped.cpp "$CPP"
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/$TESTDIR && make -s test >/dev/null 2>&1" || true
echo "#"
if diff -q /tmp/saturate.shipped.cpp "$CPP" >/dev/null; then
    echo "# translation restored, byte-identical to the shipped file"
else
    echo "# WARNING: translation NOT restored"
fi
