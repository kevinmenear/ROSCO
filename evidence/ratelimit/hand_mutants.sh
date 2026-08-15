#!/bin/bash
# The mutation measurements `cppmutate` cannot make for THIS unit, made by hand.
#
#   RUN IT ON THE CLEAN TREE, after
#     bash scripts/harness.sh ratelimit Functions ratelimit \
#          rosco/controller/src/Functions.f90 --against translation --no-generate
#   so the link keeps the saturate_c BRIDGE and drops the stale saturate.cpp.o.
#   On an integrated tree both sides reach the same C++ saturate and the callee
#   is not a control at all.
#
# TWO GAPS, DIFFERENT IN KIND, and neither is visible in `score 1.000`.
#
# GAP 1 -- THE TWO `swap_operands` MUTANTS DID NOT COMPILE, so they are excluded
# from numerator AND denominator. "Excluded" is a statement about the mutant's
# text, not about the site, and both sites are real:
#
#     const int i = *inst - 1;                    ->  *1 - inst      no compile
#     (inputSignal - rlP->LastSignal[i]) / DT     ->  rlP->LastSignal
#                                                     - inputSignal[i]  no compile
#
# cppmutate swaps the two operands of the TEXT it matched, and in both cases the
# text stops one token short of the expression -- `*inst` and `rlP->LastSignal[i]`.
# The shapes it was reaching for are `1 - *inst` and
# `(rlP->LastSignal[i] - inputSignal) / DT`, and those DO compile. They are
# measured below.
#
# GAP 2 -- THE `saturate_c` CALL HAS NO GENERATED MUTANT AT ALL. All three of
# cppmutate's call operators are gated on tables of C standard-library callee
# names (`_VALUE_PRESERVING`, `_SIBLINGS`); `saturate_c` is in neither, so the
# one call this unit makes is a site its 19 generated mutants do not touch. That
# is unit #33's finding restated one function over, and the amendment it
# proposed -- adding `saturate_c` to the table -- is still a campaign-wide
# re-take that X3 forbids mid-run. So the measurement is made here instead.
#
# Two RUNBOOK rules are load-bearing in the recipe and both were paid for
# elsewhere in this campaign:
#   * `make -s test`, NOT a bare `make` -- the generated Makefile's FIRST target
#     is `ratelimit.hpp: <the translation>`, so a bare `make` copies the header,
#     relinks nothing, and ./test reports the PREVIOUS mutant's verdict.
#   * `|| true` on the run -- ./test exits non-zero when cases fail, which under
#     `set -e` would kill the script and leave an empty artifact.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
CPP=translations/Functions/ratelimit.cpp
TESTDIR=translations/Functions/ratelimit_test

cp "$CPP" /tmp/ratelimit.shipped.cpp

run_one() {   # $1 = label, $2 = literal to find, $3 = replacement
    python3 - "$2" "$3" "$CPP" <<'PY'
import sys, pathlib
old, new, dest = sys.argv[1], sys.argv[2], sys.argv[3]
s = pathlib.Path('/tmp/ratelimit.shipped.cpp').read_text()
assert s.count(old) == 1, f'{old!r} not found exactly once -- refusing to write a mutant'
pathlib.Path(dest).write_text(s.replace(old, new))
PY
    # The bind-mount hazard units #23 and #30 both paid for: a file written on
    # the Mac and stat()ed from inside the container does not reliably carry a
    # newer mtime than the artifact built from it a moment earlier. Delete the
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
        printf '%-56s HASH MISMATCH -- not measured\n' "$1"; return
    fi
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$TESTDIR && rm -f ratelimit.hpp ratelimit_test.o test && \
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
print(f"{d['failed']} {d['checked']}" if d else "BUILD-OR-RUN-FAILED ?")
PY
)"
    printf '%-56s killed on %5s of %5s cases\n' "$1" "$failed" "$checked"
}

echo "# hand-run mutants of translations/Functions/ratelimit.cpp"
echo "# instrument: this unit's own 2904-case differential corpus, CLEAN tree"
echo "#"
run_one "BASELINE (the shipped translation, unmutated)" \
        'const int i = *inst - 1;' 'const int i = *inst - 1;'
echo "#"
echo "# GAP 1 -- the two swap_operands sites, in the shape that COMPILES"
run_one "index expression swapped: 1 - *inst   (UB: reads [-5..])" \
        'const int i = *inst - 1;' 'const int i = 1 - *inst;'
run_one "rate numerator swapped: last - in" \
        'double rate = (inputSignal - rlP->LastSignal[i]) / DT;' \
        'double rate = (rlP->LastSignal[i] - inputSignal) / DT;'
echo "#"
echo "# GAP 2 -- the saturate_c call, which cppmutate's callee TABLE excludes"
echo "# drop_call -- the call replaced by its first argument"
run_one "the rate clamp dropped" \
        'rate = saturate_c(rate, minRate, maxRate);' 'rate = rate;'
echo "# swap_call_args -- the FIRST TWO arguments exchanged"
run_one "clamp: value <-> minRate" \
        'rate = saturate_c(rate, minRate, maxRate);' \
        'rate = saturate_c(minRate, rate, maxRate);'
echo "# the two BOUNDS transposed -- a shape cppmutate offers at no unit"
run_one "clamp: bounds transposed" \
        'rate = saturate_c(rate, minRate, maxRate);' \
        'rate = saturate_c(rate, maxRate, minRate);'

cp /tmp/ratelimit.shipped.cpp "$CPP"
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/$TESTDIR && make -s test >/dev/null 2>&1" || true
echo "#"
if diff -q /tmp/ratelimit.shipped.cpp "$CPP" >/dev/null; then
    echo "# translation restored, byte-identical to the shipped file"
else
    echo "# WARNING: translation NOT restored"
fi
