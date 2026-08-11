#!/bin/bash
# Restore clean pre-integration Fortran, and undo what extraction leaves behind.
#
#   bash scripts/reset_to_clean.sh            # clean source, rebuild, verify
#   bash scripts/reset_to_clean.sh --no-build # sources only
#   bash scripts/restore_integrated.sh        # put the wrappers back
#
# THE PAIR MUST BE USED TOGETHER. Leaving the tree clean and then gating builds
# a library with no C++ in it at all -- which passes 27/27 and means nothing.
#
# WHY THIS EXISTS, measured on this tree rather than inherited:
#
#   `vit extract` is not read-only. Every run strips CRLF from DISCON.F90 (162
#   lines, content identical) -- including under --dry-run -- patches
#   CMakeLists.txt to add src/kgen_utils.f90, and leaves include.ini,
#   strace.log, kgen.log, model/, elapsedtime/, state/, .vit_build_wrapper.sh
#   and .kgen_org backups behind. `make recover` in state/ does NOT restore
#   clean Fortran: it restores the PRAGMA-CARRYING state and leaves the rest.
#
#   Worse, extraction installs an INSTRUMENTED libdiscon.so into rosco/lib/ (27
#   defined kgen symbols). Any gate run afterwards measures the instrumented
#   build. That is silent: the sim runs fine and the gate still prints a verdict.
#
# So this script both reverts and REBUILDS, then asserts the installed library
# carries no kgen symbols. A reset that half-works is a contaminated gate, and
# the failure mode announces nothing.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Last commit known to contain no integration wrappers. Pinned, not `HEAD`:
# once units are integrated, HEAD carries wrappers and reverting to it is a
# no-op that reports success.
BASELINE="${VIT_CLEAN_BASELINE:-54dd134}"
CONTAINER="${VIT_CONTAINER:-vit-dev}"
WORKDIR="/workspace/$(basename "$ROOT")"
SRC="rosco/controller/src"
BUILD=1
[ "${1:-}" = "--no-build" ] && BUILD=0

# --- 1. wrapper-carrying sources back to the clean baseline -------------------
#
# The test is whether HEAD's version calls a `<name>_c(` bridge. That is what an
# integration wrapper looks like and nothing else in this tree has one, so the
# rule maintains itself as units are added rather than needing a list.
n=0; skipped=""
while read -r f; do
    [ -z "$f" ] && continue
    if ! grep -qE '[a-z0-9_]+_c\(' "$f" 2>/dev/null; then
        skipped="$skipped $(basename "$f")"
        continue
    fi
    git show "$BASELINE:$f" > "$f"
    n=$((n + 1))
done < <(git ls-tree -r --name-only "$BASELINE" -- "$SRC" | grep -iE '\.f90$|\.f$' || true)

# --- 2. files whose ONLY difference is line endings ---------------------------
#
# Narrow on purpose. DISCON.F90 comes back CRLF-stripped from every extraction,
# content byte-identical. Reverting any file that merely DIFFERS would destroy
# in-flight work; reverting one whose sole change is CR-at-EOL cannot, because
# there is no content in that change to lose.
crlf=0
while read -r f; do
    [ -z "$f" ] && continue
    if ! git diff --quiet -- "$f" 2>/dev/null && \
       git diff --quiet --ignore-cr-at-eol -- "$f" 2>/dev/null; then
        git checkout -- "$f"
        crlf=$((crlf + 1))
    fi
done < <(git ls-files "$SRC" rosco/controller/CMakeLists.txt || true)

# --- 3. extraction leftovers --------------------------------------------------
rm -rf "$ROOT/kernel" "$ROOT/state" "$ROOT/model" "$ROOT/elapsedtime"
rm -f  "$ROOT/kgen.log" "$ROOT/include.ini" "$ROOT/strace.log" \
       "$ROOT/.vit_build_wrapper.sh" "$ROOT/.vit_wrapper.log"
rm -f  "$SRC"/*.kgen_org "$SRC"/kgen_utils.f90
left=0
if [ -f rosco/controller/CMakeLists.txt.vit_backup ]; then
    rm -f rosco/controller/CMakeLists.txt.vit_backup; left=1
fi
# The wrapper's sed adds src/kgen_utils.f90 to the source list. Revert only if
# that is the sole change -- same reasoning as the CRLF rule above.
if ! git diff --quiet -- rosco/controller/CMakeLists.txt 2>/dev/null; then
    if [ "$(git diff -U0 -- rosco/controller/CMakeLists.txt \
            | grep -cE '^\+[^+]' || true)" = "1" ] && \
       git diff -- rosco/controller/CMakeLists.txt | grep -q 'kgen_utils\.f90'; then
        git checkout -- rosco/controller/CMakeLists.txt; left=1
    fi
fi

# --- 3b. translated .cpp sources out of the build ----------------------------
#
# WITHOUT THIS THE SCRIPT DOES NOT DO WHAT IT IS NAMED. Once a unit is
# integrated, CMakeLists lists its translation, so the build tree holds a SECOND
# definition of the function -- and the pre-integration harness, which compiles
# that same translation itself, cannot link. That hits every unit after the
# first.
#
# Leaving harness.sh to compensate was the alternative and is worse: every
# future consumer would have to know to compensate, and `vit_mutate.py` drives
# make on its own, which is a second path to the same wall. It also matters more
# than tidiness -- a link that fails for EVERY mutant scores 1.000 while
# measuring nothing, so each route removed is a route to a fabricated perfect
# score removed.
#
# ONLY the `src/*.cpp` source lines go. Upstream ROSCO has no C++ at all, so any
# such line is a translation and the rule maintains itself. What must SURVIVE is
# everything else integration added -- `enable_language(CXX)`, the stdc++ link,
# and above all E1.2's -ffp-contract=off, which was measured on this tree taking
# 105 of 200 random inputs from ~1 ULP apart to 0. A blanket `git checkout` of
# this file would silently revert that flag and invalidate the baselines that
# were regenerated for it.
cpp=0
if [ -f rosco/controller/CMakeLists.txt ]; then
    cpp=$(grep -cE '^[[:space:]]*src/.*\.cpp[[:space:]]*$' rosco/controller/CMakeLists.txt || true)
    if [ "$cpp" != "0" ]; then
        grep -vE '^[[:space:]]*src/.*\.cpp[[:space:]]*$' rosco/controller/CMakeLists.txt \
            > rosco/controller/CMakeLists.txt.reset && \
            mv rosco/controller/CMakeLists.txt.reset rosco/controller/CMakeLists.txt
    fi
fi

echo "reset_to_clean: $n wrapper-carrying source(s) restored to $BASELINE"
[ -n "$skipped" ] && echo "  left at HEAD (no wrappers):$skipped"
echo "  $crlf file(s) reverted whose only change was line endings"
echo "  extraction leftovers removed (kernel/ state/ model/ elapsedtime/ logs, .kgen_org)"
echo "  $cpp translated .cpp source(s) removed from CMakeLists (E1.2 flags kept)"
[ "$left" = "1" ] && echo "  CMakeLists.txt un-patched"

[ "$BUILD" = "0" ] && { echo "  --no-build: NOT rebuilt. The installed library may still be instrumented."; exit 0; }

# --- 4. rebuild, install, and PROVE the library is clean ----------------------
docker exec "$CONTAINER" bash -lc \
    "cd $WORKDIR/rosco/controller/build && cmake --build . -j4 >/dev/null 2>&1 && \
     cp libdiscon.so $WORKDIR/rosco/lib/libdiscon.so"

# `|| true` goes INSIDE the container command: `grep -c` exits 1 on a count of
# zero, and an outer `|| echo "?"` appended a second line to a correct "0" --
# the check then cried wolf on a clean library. Wrong in the safe direction, but
# a verification that misreports is not a verification.
sym=$(docker exec "$CONTAINER" bash -lc \
      "nm -D $WORKDIR/rosco/lib/libdiscon.so 2>/dev/null | grep -ci kgen || true")
echo "  rebuilt and installed; kgen symbols in rosco/lib/libdiscon.so: $sym"
if [ "$sym" != "0" ]; then
    echo "reset_to_clean: FAILED -- the installed library is still instrumented." >&2
    echo "  A gate run now would measure the instrumented build and say nothing about it." >&2
    exit 1
fi
