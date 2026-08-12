#!/bin/bash
# THE PRICE OF THE OPERATOR, PAID RATHER THAN PROMISED.
#
# Unit #24 added three call-expression operators to `harness/cppmutate.py`
# (`drop_call`, `swap_call_args`, `swap_callee`) because `saturate`'s body is a
# call and the nine existing operators reached NOTHING in it -- P12 failed by
# name as `mutation_no_mutants`, and that is a different failure from a low
# score. Unit #22 recorded why that addition had been refused up to now:
#
#     a new operator changes the mutant set of every unit already scored, and
#     unlike a corpus addition -- which can only kill more -- it can produce a
#     SURVIVOR in a unit that closed at 1.000.
#
# That is a claim about 23 other units, and it is CHEAP TO SETTLE. Two facts,
# both measured before anything was run:
#
#   1. The addition is PURELY ADDITIVE. `_mid` is content-derived
#      (unit|operator|before|after|nth), so no existing mutant's id moves.
#      Across all 24 already-scored units: 0 existing ids lost, 35 new mutants
#      gained. Every committed artifact remains a true statement about exactly
#      the mutants it scored, and every declared equivalence still resolves.
#   2. 7 of the 24 gain mutants. 6 of those are units other than `saturate`.
#      It was 13 units and 231 mutants before `drop_call` and `swap_call_args`
#      were restricted to the value-preserving callee table -- see
#      `_VALUE_PRESERVING` in cppmutate.py, a restriction this sweep FORCED.
#
# So this script scores the 29 new mutants belonging to those 6 units, and
# writes each unit's result to its OWN file -- `mutation/<U>.call_operators.json`
# -- rather than over `mutation/<U>.json`. The committed artifact is not
# rewritten, because it is not wrong: it says what the nine operators found, and
# it still does. `vit_mutate.py --operator` stamps `operators_filter` into the
# output so a filtered run cannot be read as a full one.
#
# WHAT A SURVIVOR HERE MEANS, stated before the numbers arrive so the reading is
# not chosen afterwards: it is a defect class the unit's harness does not catch,
# newly visible rather than newly created. It does not retroactively falsify
# that unit's committed score and it does not by itself reopen the unit -- that
# is the Driver's call, and DECISIONS.md carries it. What it does do is put the
# class on the record with a name and an id instead of leaving it in the gap
# between "unmodelled" and "measured".
#
# PRECONDITION -- THE BUILD TREE MUST BE CLEAN (pre-integration).
# The differential harness compares the translation against the FORTRAN
# reference. With the integrated build in place, `Functions.f90.o` holds a
# wrapper calling `saturate_c` and the harness link leaves it undefined, so the
# baseline will not build and `vit_mutate.py` refuses to score -- loudly, which
# is the right direction, but this script would then measure nothing 6 times.
#
#     bash scripts/reset_to_clean.sh          # before
#     bash evidence/saturate/call_operator_retake.sh
#     bash scripts/restore_integrated.sh      # after
set -u
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
LOOP=/workspace/translation-loop
WORK=/workspace/ROSCO-r2
OPS="--operator drop_call --operator swap_call_args --operator swap_callee"

# unit:module:cpp-stem -- the 6 already-scored units the new operators reach.
# The other 17 gain ZERO new mutants and are deliberately absent: running them
# would produce seventeen artifacts asserting nothing. The membership is not a
# judgement, it is `mutants()` run over each translation and diffed against the
# pre-change tool -- the same script printed in DECISIONS.md under unit #24.
UNITS="
ColemanTransform:Functions:colemantransform
ColemanTransformInverse:Functions:colemantransforminverse
GetPath:ROSCO_Helpers:getpath
GetRoot:ROSCO_Helpers:getroot
GetWords:ROSCO_Helpers:getwords
ReadAvrSWAP:ReadSetParameters:readavrswap
"

echo "# re-take of the 6 already-scored units the THREE NEW call operators reach"
echo "# each writes mutation/<U>.call_operators.json; mutation/<U>.json is untouched"
echo "#"
for row in $UNITS; do
    U="${row%%:*}"; rest="${row#*:}"; M="${rest%%:*}"; S="${rest#*:}"
    DIR="translations/$M/${S}_test"

    # A LIBS ENTRY THAT IS NOT ON DISK. `scripts/harness.sh` records this as a
    # KNOWN FRAGILITY and it is what actually stopped six of these twelve units
    # on the first run: `vit test-validate` derived LIBS from the CMake target
    # at a moment when an extraction had left `kgen_utils.f90.o` in the build
    # tree, so the generated Makefile links an object that
    # `scripts/reset_to_clean.sh` correctly deletes. The baseline then fails to
    # LINK -- `cannot find .../kgen_utils.f90.o` -- and vit_mutate.py refuses to
    # score, which is right and which measures nothing.
    #
    # The rule is the object's existence, not a name: an object that is not on
    # disk cannot be part of the reference build, so it goes. That maintains
    # itself for whatever a future extraction leaves behind, and it cannot
    # remove a real dependency, because a real one is present. The unit's OWN
    # `<stem>.cpp.o` goes too, for harness.sh's reason -- the harness compiles
    # its own copy and two definitions do not link.
    #
    # These directories are UNTRACKED (git ls-files translations | grep _test/
    # is empty), so this repairs a generated file and changes no committed
    # artifact.
    # THE EXISTENCE TEST RUNS ON THE PATH THE MAKEFILE HOLDS, WHICH IS A
    # CONTAINER PATH. The first version of this block tested
    # `/workspace/ROSCO-r2/...` with `os.path.isfile` on the MAC, where nothing
    # under /workspace exists, and dropped EVERY object from all twelve
    # Makefiles -- twelve baselines that then failed for a reason this script
    # had introduced. The bind mount is the whole of the translation and it is
    # written down here rather than assumed.
    python3 - "$DIR/Makefile" "$S" "$ROOT" <<'PY'
import os, re, sys
mk, stem, root = sys.argv[1], sys.argv[2], sys.argv[3]
if not os.path.isfile(mk):
    raise SystemExit(0)
MOUNT = os.path.dirname(root)          # ~/Artifacts/vit_translation  <->  /workspace


def host(p):
    return MOUNT + p[len("/workspace"):] if p.startswith("/workspace/") else p


lines = open(mk).read().splitlines()
# A LIBS ASSIGNMENT THAT IS NOT THERE AT ALL. The first version of this block
# emptied `keep`, and an empty `keep` emitted no lines -- so the `LIBS =`
# assignment DISAPPEARED and the repair below then had nothing to match, which
# is why eleven units stayed red through a run that fixed the twelfth. If the
# assignment is gone, put an empty one back before `LIBS +=` and let the
# repair-by-contents fill it.
if not any(re.match(r"^LIBS\s*=[^+=]", ln) or re.match(r"^LIBS\s*=\s*$", ln)
           for ln in lines):
    at = next((n for n, ln in enumerate(lines) if re.match(r"^LIBS\s*\+=", ln)),
              len(lines))
    lines.insert(at, "LIBS =")
out, i, dropped, restored = [], 0, [], []
while i < len(lines):
    if re.match(r"^LIBS\s*=[^+=]", lines[i]) or re.match(r"^LIBS\s*=\s*$", lines[i]):
        block = []
        while True:
            block.append(lines[i].rstrip("\\").rstrip())
            cont = lines[i].rstrip().endswith("\\")
            i += 1
            if not cont or i >= len(lines):
                break
        toks = " ".join(block).split()
        toks = toks[2:] if toks[:2] == ["LIBS", "="] else toks[1:]
        keep, seen = [], set()
        for t in toks:
            if t.endswith(f"/{stem}.cpp.o"):
                dropped.append(os.path.basename(t)); continue
            if t.endswith(".o") and not os.path.isfile(host(t)):
                dropped.append(os.path.basename(t)); continue
            if t not in seen:
                seen.add(t); keep.append(t)
        # REPAIRED BY CONTENTS, NOT BY A LIST -- harness.sh's rule, and here it
        # also undoes the damage the first version did: every object CMake has
        # built for this target is added if it is not already present.
        objdir = "/workspace/ROSCO-r2/rosco/controller/build/CMakeFiles/discon.dir/src"
        for d in (objdir, objdir + "/SysFiles"):
            hd = host(d)
            if not os.path.isdir(hd):
                continue
            for f in sorted(os.listdir(hd)):
                if not f.endswith(".o") or f == f"{stem}.cpp.o":
                    continue
                p = f"{d}/{f}"
                if p not in seen:
                    seen.add(p); keep.append(p); restored.append(f)
        for n, t in enumerate(keep):
            pre = "LIBS = " if n == 0 else "       "
            out.append(pre + t + (" \\" if n < len(keep) - 1 else ""))
        continue
    out.append(lines[i]); i += 1
if dropped or restored:
    open(mk, "w").write("\n".join(out) + "\n")
    note = []
    if dropped:
        note.append("dropped " + ", ".join(sorted(set(dropped))))
    if restored:
        note.append(f"re-added {len(set(restored))} object(s) present in the build tree")
    print(f"#   {stem}: " + "; ".join(note))
PY

    EQ=""
    # An existing equivalences file is PASSED THROUGH. Its ids are for the nine
    # operators and cannot match a call mutant, so it changes no verdict here --
    # but omitting it would mean the two runs were given different information,
    # and a difference between runs that nobody intended is the kind that gets
    # read as a result.
    [ -f "mutation/$U.equivalences.json" ] && EQ="--equivalences mutation/$U.equivalences.json"
    out="mutation/$U.call_operators.json"
    log="/tmp/retake.$U.log"
    docker exec vit-dev bash -lc \
        "cd $WORK && python3 $LOOP/scripts/vit_mutate.py $U --root $WORK \
             --cpp translations/$M/$S.cpp --module $M $OPS $EQ --out $out" \
        > "$log" 2>&1
    rc=$?
    if [ $rc -ne 0 ] || [ ! -s "$out" ]; then
        # NOT SILENT. A unit whose baseline will not build has not been measured,
        # and reporting it as "no survivors" is the exact shape this campaign
        # keeps finding: a green that established nothing.
        printf '%-24s NOT MEASURED (rc=%s) -- %s\n' "$U" "$rc" \
               "$(grep -iE 'error|refus|baseline is not' "$log" | head -1)"
        continue
    fi
    python3 - "$U" "$out" <<'PY'
import json, sys
u, p = sys.argv[1], sys.argv[2]
d = json.load(open(p))
surv = d.get("survivors", [])
print(f"{u:24s} {d['killed']:3d} of {d['mutants']:3d} new mutant(s) killed"
      f"   score {d['score']:.3f}"
      + (f"   nocompile {d['nocompile']}" if d.get("nocompile") else "")
      + ("" if not surv else
         "\n" + "\n".join(f"{'':24s}   SURVIVOR {s['id']} {s['operator']}: "
                          f"{s['before']!r} -> {s['after']!r}" for s in surv)))
PY
done
echo "#"
echo "# done. mutation/<U>.json for these units is unchanged by this script."
