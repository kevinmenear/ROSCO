#!/bin/bash
# The mutation measurement `cppmutate` cannot make for THIS unit, made by hand.
#
# WHY THIS EXISTS. All three of `cppmutate`'s call operators -- `drop_call`,
# `swap_call_args`, `swap_callee` -- are gated on TABLES of callee names
# (`_VALUE_PRESERVING`, `_SIBLINGS`), and every entry in both tables is a C
# standard-library name. `saturate_c` is not one, so the THREE calls this unit
# makes are three sites its generated mutants do not touch:
#
#     piP->ITerm[i]        = saturate_c(piP->ITerm[i],  minValue, maxValue);
#     piP->ITerm2[i]       = saturate_c(piP->ITerm2[i], minValue, maxValue);
#     PIIController_result = saturate_c(PIIController_result, minValue, maxValue);
#
# Unit #33 measured the same gap at two sites and unit #24 measured it before
# the operators existed at all. The restriction is MEASURED and right --
# `cppmutate`'s own comment records that letting `drop_call` fire on every call
# made four of this campaign's units 38-73% unbuildable -- and amending it is a
# campaign-wide re-take of every scored unit, which X3 forbids mid-run. So the
# measurement is made here and the amendment stays a proposal in DECISIONS.md.
#
# WHAT IS NEW HERE relative to unit #33: a THIRD site, and the first pair of
# saturate_c calls in this campaign that are SIBLINGS of one another. Dropping
# both integrator clamps at once is a shape no single-site operator produces,
# and it is the one that says whether the corpus separates the two channels.
#
# Two RUNBOOK rules are load-bearing in the recipe and both were paid for
# elsewhere in this campaign:
#   * `make -s test`, NOT a bare `make` -- the generated Makefile's FIRST target
#     is `piicontroller.hpp: <the translation>`, so a bare `make` copies the
#     header, relinks nothing, and ./test reports the PREVIOUS mutant's verdict.
#   * `|| true` on the run -- ./test exits non-zero when cases fail, which under
#     `set -e` would kill the script and leave an empty artifact.
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
CPP=translations/Controllers/piicontroller.cpp
TESTDIR=translations/Controllers/piicontroller_test

A='piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);'
B='piP->ITerm2[i] = saturate_c(piP->ITerm2[i], minValue, maxValue);'
C='PIIController_result = saturate_c(PIIController_result, minValue, maxValue);'

cp "$CPP" /tmp/piicontroller.shipped.cpp

run_one() {   # $1 = label, $2/$3/$4 = replacements for A/B/C
    python3 - "$2" "$3" "$4" <<'PY'
import sys, pathlib
a_new, b_new, c_new = sys.argv[1], sys.argv[2], sys.argv[3]
s = pathlib.Path('/tmp/piicontroller.shipped.cpp').read_text()
A = 'piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);'
B = 'piP->ITerm2[i] = saturate_c(piP->ITerm2[i], minValue, maxValue);'
C = 'PIIController_result = saturate_c(PIIController_result, minValue, maxValue);'
for name, site in (('A', A), ('B', B), ('C', C)):
    assert s.count(site) == 1, f'call site {name} not found exactly once -- refusing to write a mutant'
pathlib.Path('translations/Controllers/piicontroller.cpp').write_text(
    s.replace(A, a_new).replace(B, b_new).replace(C, c_new))
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
        printf '%-58s HASH MISMATCH -- not measured\n' "$1"; return
    fi
    docker exec vit-dev bash -lc \
        "cd /workspace/ROSCO-r2/$TESTDIR && rm -f piicontroller.hpp piicontroller_test.o test && \
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

DA='piP->ITerm[i] = piP->ITerm[i];'
DB='piP->ITerm2[i] = piP->ITerm2[i];'
DC='PIIController_result = PIIController_result;'

echo "# hand-run mutants of translations/Controllers/piicontroller.cpp"
echo "# the three saturate_c call sites, which cppmutate's callee TABLE excludes"
echo "# instrument: this unit's own differential corpus"
echo "#"
run_one "BASELINE (the shipped translation, unmutated)" "$A" "$B" "$C"
echo "#"
echo "# drop_call -- the call replaced by its first argument"
run_one "ITerm clamp dropped"        "$DA" "$B"  "$C"
run_one "ITerm2 clamp dropped"       "$A"  "$DB" "$C"
run_one "output clamp dropped"       "$A"  "$B"  "$DC"
run_one "BOTH integrator clamps dropped" "$DA" "$DB" "$C"
run_one "ALL THREE clamps dropped"   "$DA" "$DB" "$DC"
echo "#"
echo "# swap_call_args -- the FIRST TWO arguments exchanged, which is the shape"
echo "# cppmutate's operator produces. Proved EQUIVALENT on this toolchain at"
echo "# evidence/saturate/minmax_probe.txt: fmax(a,b) and fmax(b,a) agree on"
echo "# every input including a signed zero and a NaN."
run_one "ITerm clamp:  value <-> minValue"  'piP->ITerm[i] = saturate_c(minValue, piP->ITerm[i], maxValue);' "$B" "$C"
run_one "ITerm2 clamp: value <-> minValue"  "$A" 'piP->ITerm2[i] = saturate_c(minValue, piP->ITerm2[i], maxValue);' "$C"
run_one "output clamp: value <-> minValue"  "$A" "$B" 'PIIController_result = saturate_c(minValue, PIIController_result, maxValue);'
echo "#"
echo "# the two BOUNDS transposed -- not a shape cppmutate offers at all, and the"
echo "# defect unit #24's own post-integration red test was built from"
run_one "ITerm clamp:  bounds transposed"   'piP->ITerm[i] = saturate_c(piP->ITerm[i], maxValue, minValue);' "$B" "$C"
run_one "ITerm2 clamp: bounds transposed"   "$A" 'piP->ITerm2[i] = saturate_c(piP->ITerm2[i], maxValue, minValue);' "$C"
run_one "output clamp: bounds transposed"   "$A" "$B" 'PIIController_result = saturate_c(PIIController_result, maxValue, minValue);'
echo "#"
echo "# swap_callee -- the two integrator channels' clamps exchanged, so ITerm is"
echo "# clamped into ITerm2's slot and back. A shape that exists only because this"
echo "# unit has a SIBLING PAIR of clamps; no operator produces it at any unit."
run_one "the two integrator clamps exchanged" \
        'piP->ITerm[i] = saturate_c(piP->ITerm2[i], minValue, maxValue);' \
        'piP->ITerm2[i] = saturate_c(piP->ITerm[i], minValue, maxValue);' "$C"

cp /tmp/piicontroller.shipped.cpp "$CPP"
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/$TESTDIR && make -s test >/dev/null 2>&1" || true
echo "#"
if diff -q /tmp/piicontroller.shipped.cpp "$CPP" >/dev/null; then
    echo "# translation restored, byte-identical to the shipped file"
else
    echo "# WARNING: translation NOT restored"
fi
