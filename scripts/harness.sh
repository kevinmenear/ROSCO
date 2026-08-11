#!/bin/bash
# Build and run the differential harness for one unit. P11.
#
#   bash scripts/harness.sh ColemanTransform Functions colemantransform \
#        rosco/controller/src/Functions.f90 [--against integrated]
#
# WHY THIS WRAPPER EXISTS -- instrument skew, not a workaround.
#
# `vit_harness.py` (pinned loop repo d58a418) writes its generated case file
# and test source to a PER-UNIT directory:
#
#     translations/<Module>/<unit>_test/
#
# citing VIT dev note 202608092223. The VIT installed in `vit-dev` predates that
# note -- its newest dev note is 202608071650 -- and `vit test-validate` still
# writes the Makefile and the Fortran bridge to the PER-MODULE directory:
#
#     translations/<Module>/
#
# So vit_harness.py generated 257 cases and then died on `make: *** No rule to
# make target 'test'`: the Makefile it needs was one directory up. The two
# halves of this campaign's instrument set disagree about a path.
#
# Upgrading VIT mid-campaign is the change NOT made here. VIT is what produced
# this unit's extract/verify/integrate evidence; swapping it out now would mean
# the unit's evidence came from two different tools. So this script reconciles
# the layouts instead, and the reconciliation is a committed script rather than
# steps somebody remembers to repeat.
#
# The relocation is safe to do mechanically: the generated Makefile's own paths
# are all relative to its directory (the LIBS list and the .cpp source are
# absolute), so it builds identically from either location.
#
# KNOWN FRAGILITY, stated rather than left to be found: the LIBS list ends with
# `kgen_utils.f90.o`, which exists only because an extraction left it in the
# build tree. A from-scratch build directory would not have it and the link
# would fail. That object is not part of the translation under test.

set -euo pipefail

UNIT="${1:?usage: harness.sh <Unit> <Module> <stem> <fortran-file> [--post-integration] [--out P] [args...]}"
MODULE="${2:?}"
STEM="${3:?}"
FFILE="${4:?}"
shift 4

MODE=pre
OUT=""
REDTEST=""
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --post-integration) MODE=post ;;
        --out) OUT="$2"; ARGS+=("$1" "$2"); shift ;;
        # --red-test "<what was perturbed>": this run is EXPECTED to fail, and
        # the perturbation is recorded into the artifact by the tool instead of
        # being typed in afterwards. A red artifact that does not say what was
        # perturbed is not evidence about the instrument.
        --red-test) REDTEST="$2"; shift ;;
        *) ARGS+=("$1") ;;
    esac
    shift
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="${VIT_CONTAINER:-vit-dev}"
WORKDIR="/workspace/$(basename "$ROOT")"
LOOP="${VIT_LOOP_REPO:-/workspace/translation-loop}"
MOD_DIR="translations/$MODULE"
UNIT_DIR="$MOD_DIR/${STEM}_test"

# 1. VIT writes the Makefile, the Fortran bridge and a skeleton.
#
# ONLY IN PRE MODE, and the restriction is new. Under the old per-module layout
# the skeleton landed in `translations/<Module>/` and step 2 deleted it. Under
# VIT d07a716's per-unit layout it lands in `<stem>_test/` -- ON TOP of the test
# .cpp vit_harness.py generated. The post-integration run recompiles
# `<stem>_test.o` from that file, so running `vit test-validate` again before it
# would build the SKELETON against the integrated library and report on a file
# nobody wrote. Post mode has no need of it anyway: the Makefile, the bridge and
# the case file all persist from the generating run, which is exactly what the
# note further down says post mode reuses.
if [ "$MODE" = "pre" ]; then
    docker exec "$CONTAINER" bash -lc \
        "cd $WORKDIR && vit test-validate $UNIT $MOD_DIR/$STEM.cpp -f $FFILE -m $MODULE --force" \
        > /dev/null
fi

# 2. Move the build files down into the directory vit_harness.py uses.
#
# THE SKEW THIS STEP RECONCILES IS GONE AS OF VIT d07a716, and the step now
# detects that rather than failing on it. Reconciling VIT with canonical (unit
# #2, second pass) brought in the per-unit directory of dev note 202608092223,
# so `vit test-validate` writes Makefile, bridge and skeleton straight into
# `translations/<Module>/<stem>_test/` -- the directory vit_harness.py already
# used. The two halves of the instrument set now agree about the path.
#
# Measured, not assumed: on 2026-08-10 at 20:00 this script exited 1 with
# "vit test-validate did not write Makefile" while the Makefile sat one level
# down, timestamped four seconds earlier by that same command.
#
# The branch is kept rather than deleted. Deleting it would silently make this
# script require the new VIT, and the artifacts already committed for this unit
# came from the old layout; a script that cannot reproduce them stops being able
# to say where they came from. Which layout was used is now PRINTED, so a run's
# log says which instrument produced it instead of leaving it to be inferred.
#
# The skeleton test .cpp is deliberately discarded in both layouts:
# vit_harness.py writes its own, and the skeleton is a prompt for a human.
mkdir -p "$ROOT/$UNIT_DIR"
if [ "$MODE" = "pre" ]; then
    if [ -f "$ROOT/$MOD_DIR/Makefile" ]; then
        echo "harness.sh: VIT wrote the per-module layout ($MOD_DIR); relocating"
        for f in Makefile "${STEM}_bridge.f90"; do
            [ -f "$ROOT/$MOD_DIR/$f" ] || { echo "harness.sh: vit test-validate did not write $f" >&2; exit 1; }
            mv "$ROOT/$MOD_DIR/$f" "$ROOT/$UNIT_DIR/$f"
        done
    elif [ -f "$ROOT/$UNIT_DIR/Makefile" ] && [ -f "$ROOT/$UNIT_DIR/${STEM}_bridge.f90" ]; then
        echo "harness.sh: VIT wrote the per-unit layout directly ($UNIT_DIR); no move needed"
    else
        echo "harness.sh: vit test-validate wrote a Makefile to NEITHER $MOD_DIR nor $UNIT_DIR" >&2
        exit 1
    fi
    # 2b. Drop THIS UNIT'S OWN object from the generated LIBS list.
    #
    # `vit test-validate` derives LIBS from the CMake target, and after this
    # unit was integrated once, that target compiles
    # `rosco/controller/src/<stem>.cpp`. `reset_to_clean.sh` restores the
    # Fortran body but deliberately leaves CMakeLists alone, so the build tree
    # holds `<stem>.cpp.o` -- a second definition of the very function the
    # harness compiles its own copy of. The link dies:
    #
    #   multiple definition of `ColemanTransform(double*, double, int, double*, double*)'
    #
    # It did not happen on this unit's first pass because nothing had been
    # integrated yet, so the object did not exist. Every later unit, and every
    # re-run of an earlier one, hits it.
    #
    # ONLY this unit's object is removed. Other units' `.cpp.o` files stay:
    # integrated callees reached through their `_c` bridges are part of the
    # reference build and dropping them would break the link for real.
    #
    # It is edited into the MAKEFILE, not passed on one `make` command line,
    # because `vit_mutate.py` runs `make -C <dir> test` itself. A LIBS fix that
    # lived only in this script would leave every mutant failing to link --
    # scored as `killed (no compile)`, which is a 1.000 that measured nothing.
    # `vit_mutate.py` refuses to score when the baseline will not build, so it
    # would have failed loudly rather than lied; the point is that it must not
    # need to.
    #
    # This is the same removal `--post-integration` already performs on its own
    # command line, for the same reason, and the two now agree.
    mk="$ROOT/$UNIT_DIR/Makefile"
    if grep -q "/${STEM}\.cpp\.o" "$mk"; then
        echo "harness.sh: dropping ${STEM}.cpp.o from LIBS -- the harness compiles its own copy"
        sed -i.bak "\#/${STEM}\.cpp\.o#d" "$mk"
        rm -f "$mk.bak"
        # Asserted, not assumed: a sed that silently matched nothing would let
        # the same link failure through with a reassuring message above it.
        if grep -q "/${STEM}\.cpp\.o" "$mk"; then
            echo "harness.sh: LIBS edit did not take -- ${STEM}.cpp.o is still in $mk" >&2
            exit 1
        fi
    fi

    # The skeleton in MOD_DIR is discarded: nothing reads it there and leaving
    # it invites a human to edit the file the harness does not use.
    #
    # The one in UNIT_DIR is NOT deleted, and that is deliberate. vit_harness.py
    # writes its own <stem>_test.cpp to that exact path in step 3, so deleting
    # it first buys nothing -- and it cost a run: removing the file from the
    # host and then writing the same path from the container raised
    # `FileNotFoundError` on a directory both sides could list, 2026-08-10
    # 20:01. The bind mount does not make a host unlink and a container create
    # of one path atomic with respect to each other. Overwrite, do not delete.
    rm -f "$ROOT/$MOD_DIR/${STEM}_test.cpp"
else
    for f in Makefile "${STEM}_bridge.f90" "${STEM}_cases.bin" "${STEM}_test.cpp"; do
        [ -f "$ROOT/$UNIT_DIR/$f" ] || {
            echo "harness.sh --post-integration: no $UNIT_DIR/$f. Post mode reuses the" >&2
            echo "  generating run's files and does not regenerate them; run pre mode first." >&2
            exit 1; }
    done
fi

# 3. Generate the cases, build, run, and write the artifact.
if [ "$MODE" = "pre" ]; then
    docker exec "$CONTAINER" bash -lc \
        "cd $WORKDIR && python3 $LOOP/scripts/vit_harness.py $UNIT --root $WORKDIR \
            --file $FFILE --cpp $MOD_DIR/$STEM.cpp --module $MODULE ${ARGS[*]}"
    exit $?
fi

# --- post-integration mode (E4.5) --------------------------------------------
#
# `bash scripts/harness.sh <...> --post-integration` re-links the SAME case file
# against the integrated build and re-runs it.
#
# WHAT THIS CAN AND CANNOT MEASURE, stated because the distinction is the whole
# point. After integration the Fortran ColemanTransform IS the C++ -- the body
# is a wrapper calling `colemantransform_c`. There is no independent Fortran
# reference left in the build, by construction, so no harness against the
# integrated build can compare the ARITHMETIC. Anything claiming to is comparing
# the translation with itself.
#
# What is left to compare, and is not covered anywhere else, is the INTEGRATION
# WRAPPER: the marshalling between the Fortran caller and the C++ callee. That
# is a generated bridge, and two other generated bridges in this same campaign
# were found dropping the rank of `rootMOOP(3)` -- one declaring it a by-value
# scalar, one a one-element buffer read three deep. So:
#
#     ref  = Fortran bridge -> integrated Functions.f90 wrapper
#              -> colemantransform_c -> the translation
#     got  = the translation, called directly
#
# Identical arithmetic on both sides on purpose. A mismatch means the wrapper
# corrupted an argument or an output on the way through, which is the only thing
# this run is asking about. The arithmetic is covered by the pre-integration
# harness (against real Fortran), the kernel, and the gate.
#
# The shim exists because the integrated `colemantransform.cpp.o` defines
# `ColemanTransform` too, and linking it beside the harness's own copy is a
# duplicate definition. Dropping that object and forwarding `colemantransform_c`
# to the harness's copy keeps exactly one definition and routes the ref side
# through the real wrapper.

# The post-integration run reuses whatever case file the last generating run
# left in place; it does not regenerate. That is deliberate -- after integration
# `literals_from(Functions.f90)` reads a source whose arithmetic has been
# replaced by a wrapper, so a regenerated set would be drawn from different
# literals. The count therefore need not match the pre-integration run's, and
# the artifact records what it actually checked.

[ -n "$OUT" ] || { echo "harness.sh --post-integration needs --out" >&2; exit 2; }

cat > "$ROOT/$UNIT_DIR/vit_integration_shim.cpp" <<SHIM
// Generated by scripts/harness.sh --post-integration. See the note above.
void ${UNIT}(double*, double, int, double*, double*);
extern "C" void ${STEM}_c(double* a, double b, int c, double* d, double* e) {
    ${UNIT}(a, b, c, d, e);
}
SHIM

# ASK MAKE WHAT LIBS IS. Do not re-parse makefile syntax with sed.
#
# The previous version did: grep from `^LIBS =` to the first blank line, strip
# the `LIBS =` prefix, join. It worked on the Makefile the pre-merge VIT wrote
# and broke silently-then-loudly on the one VIT d07a716 writes, which adds a
# second assignment:
#
#     LIBS = <objects...>          <- continued over 16 lines
#     LIBS += -lgfortran -lm       <- NEW; `^LIBS =` does not match `LIBS +=`
#
# so the words `LIBS` and `+=` were passed to the linker as filenames:
# `/usr/bin/ld: cannot find LIBS`. A hand-rolled parser that understands one
# assignment operator and not the other is a parser that will break again on the
# next generator change. `make` already knows how to evaluate its own variable,
# including `+=` and line continuations, so it is asked.
#
# `include Makefile` and an explicit target: naming the target means the
# generated Makefile's own default rule is not run, so this only reads.
docker exec "$CONTAINER" bash -lc "cd $WORKDIR/$UNIT_DIR && \
  g++ -O2 -fPIC -ffp-contract=off -c vit_integration_shim.cpp -o vit_integration_shim.o && \
  printf 'include Makefile\nvit-print-libs:\n\t@echo \$(LIBS)\n' > .vit_libs.mk && \
  LIBS=\$(make -s -f .vit_libs.mk vit-print-libs | sed 's#[^ ]*${STEM}\.cpp\.o##') && \
  rm -f .vit_libs.mk && \
  case \"\$LIBS\" in *LIBS*|*'+='*) echo \"harness.sh: LIBS did not evaluate: \$LIBS\" >&2; exit 3;; esac && \
  rm -f test ${STEM}_test.o && \
  make test LIBS=\"\$LIBS $WORKDIR/$UNIT_DIR/vit_integration_shim.o\" >/dev/null && \
  ./test ${STEM}_cases.bin > $WORKDIR/$OUT" && rc=0 || rc=$?

# STAMP EVEN WHEN THE RUN WENT RED, and the distinction matters more than it
# looks. `./test` exits non-zero when cases mismatch, so under `set -e` this
# script used to abort one line short of the stamp -- meaning a RED
# post-integration artifact was written with `against: "translation"`, no
# `measures`, and no instrument revisions. The red test is the one run whose
# artifact most needs to say what it measured and what produced it, and it was
# the only one that could not. Observed 2026-08-10 on this unit's wrapper-swap
# red test: 199/199 failed, artifact unstamped.
#
# A link failure is still a hard stop: no artifact is written at all, so there
# is nothing to stamp and nothing was measured.
if [ ! -s "$ROOT/$OUT" ] || ! python3 -c "import json,sys; json.load(open(sys.argv[1]))" "$ROOT/$OUT" 2>/dev/null; then
    echo "harness.sh: no parseable artifact at $OUT -- the build or the run died (rc=$rc)" >&2
    exit "${rc:-1}"
fi

if [ -n "$REDTEST" ]; then
    python3 "$ROOT/scripts/_harness_stamp.py" "$ROOT/$OUT" --red-test "$REDTEST"
else
    python3 "$ROOT/scripts/_harness_stamp.py" "$ROOT/$OUT"
fi
