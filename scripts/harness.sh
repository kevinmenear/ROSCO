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
ARGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --post-integration) MODE=post ;;
        --out) OUT="$2"; ARGS+=("$1" "$2"); shift ;;
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

# 1. VIT writes the Makefile, the Fortran bridge and a skeleton -- per module.
docker exec "$CONTAINER" bash -lc \
    "cd $WORKDIR && vit test-validate $UNIT $MOD_DIR/$STEM.cpp -f $FFILE -m $MODULE --force" \
    > /dev/null

# 2. Move the build files down into the directory vit_harness.py uses. The
#    skeleton test .cpp is deliberately NOT moved: vit_harness.py writes its own
#    there, and the skeleton is a prompt for a human, not a harness.
mkdir -p "$ROOT/$UNIT_DIR"
for f in Makefile "${STEM}_bridge.f90"; do
    [ -f "$ROOT/$MOD_DIR/$f" ] || { echo "harness.sh: vit test-validate did not write $f" >&2; exit 1; }
    mv "$ROOT/$MOD_DIR/$f" "$ROOT/$UNIT_DIR/$f"
done
rm -f "$ROOT/$MOD_DIR/${STEM}_test.cpp"

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

docker exec "$CONTAINER" bash -lc "cd $WORKDIR/$UNIT_DIR && \
  g++ -O2 -fPIC -ffp-contract=off -c vit_integration_shim.cpp -o vit_integration_shim.o && \
  LIBS=\$(grep -A200 '^LIBS =' Makefile | sed -n '1,/^\$/p' | tr -d '\\\\' \
        | sed 's#^LIBS =##' | tr '\n' ' ' | sed 's#[^ ]*${STEM}\.cpp\.o##') && \
  rm -f test ${STEM}_test.o && \
  make test LIBS=\"\$LIBS $WORKDIR/$UNIT_DIR/vit_integration_shim.o\" >/dev/null && \
  ./test ${STEM}_cases.bin > $WORKDIR/$OUT"

python3 "$ROOT/scripts/_harness_stamp.py" "$ROOT/$OUT"
