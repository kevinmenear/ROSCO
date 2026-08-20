#!/bin/bash
# Build and run the standalone differential probe for ReadControlParameterFileSub.
#
#   bash evidence/ReadControlParameterFileSub/run_probe.sh
#   bash evidence/ReadControlParameterFileSub/run_probe.sh --red-test <sed-expr> <label>
#
# WHY THIS IS A SCRIPT. The probe is the unit's ONLY instrument -- the
# differential harness refused and integration has no copy-back for the ~50
# ALLOCATABLE outputs, so the gate cannot run either. An instrument that lives
# in somebody's shell history is not an instrument.
#
# --red-test applies one `sed -E` expression to a COPY of the translation,
# rebuilds from that copy, runs, and puts the original back. The copy is what
# the build reads, so there is no window in which a perturbed file could be
# committed -- the campaign's own lesson that a runner must restore the file the
# BUILD read, not only the file it perturbed, is answered by never perturbing
# the tracked file at all.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CONTAINER="${VIT_CONTAINER:-vit-dev}"
WORK="/workspace/$(basename "$ROOT")"
CPP="translations/ReadSetParameters/readcontrolparameterfilesub.cpp"

SEDX=""
LABEL="baseline"
OUT=""
POST=""
while [ $# -gt 0 ]; do
    case "$1" in
        --red-test) SEDX="${2:?--red-test needs a sed -E expression}"
                    LABEL="${3:?--red-test needs a label}"; shift 2 ;;
        # --out writes a RESULT ARTIFACT, not just a log. `revcheck` and the
        # done-condition read JSON under harness/, mutation/ and gate/ and ask
        # which instrument produced it; a probe whose only output is text is a
        # measurement nothing can check the provenance of. Stamped with
        # loop_rev and vit_rev by the campaign's own _harness_stamp.py rather
        # than by a second reading of the same two repositories.
        --out)      OUT="${2:?--out needs a path}"; shift ;;
        # --post marks the artifact as a POST-INTEGRATION run. It changes no
        # part of the run: after `vit integrate --apply` the Fortran entry
        # point IS the wrapper, so the same command compares the wrapper's
        # marshalling instead of the reference's arithmetic. It changes what
        # the artifact SAYS it measured, which is the whole of E4.5.
        --post)     POST="post" ;;
        *) echo "run_probe: unknown argument $1" >&2; exit 2 ;;
    esac
    shift
done

# The driver is GENERATED from ROSCO_Types.f90's own TYPE definition, every run,
# so a component added there is compared without an edit here.
python3 "$ROOT/evidence/ReadControlParameterFileSub/gen_probe.py" \
    > "$ROOT/evidence/ReadControlParameterFileSub/vit_rcpfs_probe.f90"

# THE EXPRESSION NEVER GOES THROUGH THE SHELL STRING. Every useful red test on
# this unit matches a C++ character literal and therefore contains a SINGLE
# QUOTE; interpolated into the double-quoted `docker exec bash -lc "..."` below
# it closes the quoting early, the perturbation block is skipped, and the run
# reports the BASELINE under the red test's name. Measured -- see the C12 block
# in probe.redtests.txt. It goes in over stdin, to a file, and `sed -E -f`
# reads it.
docker exec "$CONTAINER" bash -lc 'mkdir -p /tmp/probe && rm -f /tmp/probe/red.sed'
if [ -n "$SEDX" ]; then
    printf '%s\n' "$SEDX" | docker exec -i "$CONTAINER" \
        bash -lc 'cat > /tmp/probe/red.sed'
fi

RUNLOG="$(mktemp)"
docker exec "$CONTAINER" bash -lc "
set -e
cd $WORK
mkdir -p /tmp/probe
cp $CPP /tmp/probe/unit.cpp
if [ -s /tmp/probe/red.sed ]; then
    sed -E -i -f /tmp/probe/red.sed /tmp/probe/unit.cpp
    if cmp -s $CPP /tmp/probe/unit.cpp; then
        echo 'run_probe: the --red-test expression changed NOTHING; refusing' >&2
        exit 3
    fi
    diff $CPP /tmp/probe/unit.cpp | head -8 || true
fi
g++ -std=c++17 -O2 -fPIC -ffp-contract=off -I rosco/controller/src \\
    -include rosco/controller/src/vit_translated.h \\
    -c /tmp/probe/unit.cpp -o /tmp/probe/rcpfs.o
python3 scripts/_integration_shim.py ReadControlParameterFileSub \\
    -f rosco/controller/src/ReadSetParameters.f90 > /tmp/probe/shim.cpp
g++ -std=c++17 -O2 -fPIC -ffp-contract=off -I rosco/controller/src \\
    -c /tmp/probe/shim.cpp -o /tmp/probe/shim.o
gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off \\
    -I rosco/controller/build/ftnmods \\
    -o /tmp/probe/vit_rcpfs_probe evidence/ReadControlParameterFileSub/vit_rcpfs_probe.f90 \\
    /tmp/probe/rcpfs.o /tmp/probe/shim.o -L rosco/controller/build -ldiscon -lstdc++
cd Examples
LD_LIBRARY_PATH=$WORK/rosco/controller/build /tmp/probe/vit_rcpfs_probe \\
    DISCON*.IN ../evidence/ReadControlParameterFileSub/corpus/*.IN \\
    2>&1 | grep -v 'ROSCO Warning' | grep -v '^        COPYBK' || true
# echo1.IN sets Echo=1, so BOTH sides OPEN <RootName>.RO.echo -- and its
# EXISTENCE is the only evidence that arm ran at all, so report it before
# removing it. The file is a by-product and not an artifact: left behind it
# dirties the tree, and capture_done_check.sh refuses on a dirty tree.
#
# NO BACKTICKS IN THIS BLOCK. It is inside a double-quoted string the HOST
# shell expands before docker sees it, so a backtick here is a host command
# substitution -- which is what the first version of this comment did, running
# 'echo1.IN' and 'capture_done_check.sh' as host commands. The campaign already
# has this lesson under --note; it reaches any double-quoted docker exec.
C=../evidence/ReadControlParameterFileSub/corpus
for e in \$C/*_REF.RO.echo; do
    [ -f \"\$e\" ] || continue
    p=\$(echo \$e | sed s/_REF/_CPP/)
    echo \"  echo   ref \$(wc -c < \$e) bytes   cpp \$(wc -c < \$p) bytes\"
    echo \"  echo   first difference: \$(cmp \$e \$p 2>&1 | head -1)\"
    echo \"  echo   the CPP file, in full, against the reference prefix:\"
    head -c \$(wc -c < \$p) \$e | cmp - \$p > /dev/null 2>&1 \\
        && echo \"  echo   IDENTICAL over all \$(wc -c < \$p) bytes the translation wrote\" \\
        || echo \"  echo   DIFFERS inside the translation-written prefix\"
done
rm -f \$C/*.RO.echo
" | tee "$RUNLOG"
echo "run_probe: $LABEL"

if [ -n "$OUT" ]; then
    python3 "$ROOT/evidence/ReadControlParameterFileSub/probe_artifact.py" \
        "$RUNLOG" "$OUT" "$LABEL" "$SEDX" $POST
    if [ -n "$SEDX" ]; then
        python3 "$ROOT/scripts/_harness_stamp.py" "$OUT" --pre \
            --red-test "$LABEL: $SEDX"
    else
        python3 "$ROOT/scripts/_harness_stamp.py" "$OUT" --pre
    fi
fi
rm -f "$RUNLOG"
