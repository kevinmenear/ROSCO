#!/bin/bash
# Undo reset_to_clean.sh: put the committed, integrated sources back.
#
#   bash scripts/restore_integrated.sh
#
# The integrated state IS the committed state in this tree -- a unit is
# committed only after its own gate passes -- so restoring is a checkout, not a
# replay of `vit integrate`.
#
# COMMIT BEFORE RESETTING. This restores from HEAD; anything uncommitted under
# rosco/controller/src is lost. That is precisely the in-flight case: a unit is
# integrated but not yet committed until its gate passes, so a checkout here
# silently deletes the wrapper of the unit being worked on. Nothing else reports
# it -- the .cpp survives untracked, CMakeLists.txt is outside src/ so it still
# compiles, and the gate passes because clean and integrated are bit-identical
# by design.
#
# vit_types.h is EXCLUDED, and the exclusion is load-bearing: it is GENERATED,
# and `vit translate` appends a view struct to it whenever it meets a derived
# type with ALLOCATABLE fields that has no view yet. A blanket checkout reverts
# it and silently deletes the in-flight unit's type, so the next build fails on
# an unknown `*_view_t` -- or worse, succeeds against a stale struct.
# reset_to_clean.sh already leaves it alone; this keeps the pair symmetric.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

CONTAINER="${VIT_CONTAINER:-vit-dev}"
WORKDIR="/workspace/$(basename "$ROOT")"

# `|| true` INSIDE the pipeline, for `reset_to_clean.sh`'s reason one script
# over: `grep -v` exits 1 when it emits NO lines, which on a CLEAN tree is the
# normal case here. Under `set -euo pipefail` that failed the assignment and
# ended the script at this line -- exit 1, no message, and the
# rebuild-and-install at the bottom never reached. Captured before the fix in
# `evidence/_scripts/restore_integrated.clean-tree-refusal.txt` (C12), found at
# the close of rosco-r2 unit #56 by running this script on a tree that was
# already integrated and fully committed, which is exactly the state a dispatch
# ENDS in. Harmless to every run that follows a `reset_to_clean.sh`, because
# that leaves nine tracked sources modified; the hazard is a caller that
# discards the exit code and believes the library was rebuilt from HEAD.
dirty=$(git status --porcelain rosco/controller/src \
        | { grep -v '^?? ' || true; } | wc -l | tr -d ' ')
if [ "$dirty" != "0" ]; then
    echo "restore_integrated: WARNING -- $dirty tracked file(s) under src/ are modified" >&2
    echo "  and will be overwritten from HEAD. If a unit is integrated but not yet" >&2
    echo "  committed, its wrapper is about to be lost. Commit first, or re-run" >&2
    echo "  \`vit integrate ... --apply\` afterwards and discard any gate result" >&2
    echo "  taken since." >&2
fi

n=0
while read -r f; do
    [ -z "$f" ] && continue
    case "$f" in */vit_types.h) continue;; esac
    git checkout HEAD -- "$f"
    n=$((n + 1))
done < <(git ls-files rosco/controller/src)

# CMakeLists.txt lives OUTSIDE src/, so the loop above never saw it -- and
# reset_to_clean.sh now strips the translated .cpp sources from it. Without this
# the pair is asymmetric: reset removes them, restore does not put them back,
# and the rebuilt library quietly contains no C++ at all while gating 27/27.
git checkout HEAD -- rosco/controller/CMakeLists.txt

# The window is closed: clear the sentinel the reset raised, which re-enables
# commits. Cleared HERE, after the checkout, so a failure part-way through
# leaves it raised rather than clearing it on a tree still half-reverted.
rm -f "$ROOT/.loop-run/TREE_IS_DE_INTEGRATED"
n=$((n + 1))

echo "restore_integrated: $n file(s) restored from HEAD (vit_types.h untouched)"
echo "  CMakeLists.txt restored: the translated .cpp sources are back in the build"

docker exec "$CONTAINER" bash -lc \
    "cd $WORKDIR/rosco/controller/build && cmake --build . -j4 >/dev/null 2>&1 && \
     cp libdiscon.so $WORKDIR/rosco/lib/libdiscon.so"
echo "  rebuilt and installed. Gate now measures the INTEGRATED build."
