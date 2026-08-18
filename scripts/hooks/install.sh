#!/bin/bash
# Install this repository's git hooks, or check the installed copies for drift.
#
#   bash scripts/hooks/install.sh           # install / re-install
#   bash scripts/hooks/install.sh --check   # exit 1 if installed != tracked
#
# WHY A COPY AND NOT A SYMLINK. A symlink into the worktree would break during
# `reset_to_clean.sh`, which is exactly the window the pre-commit guard exists
# to cover -- a guard that disappears when the tree is reverted protects nothing
# at the only moment it is needed. So the hook is copied, and `--check` is how
# the copy is kept honest.
#
# WHY --check EXISTS AT ALL. Two files that are supposed to be identical, with
# nothing comparing them, drift and say nothing about it. Run --check from CI or
# from preflight; a hook that has silently diverged from its tracked source is
# indistinguishable from one that has not.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SRC="$ROOT/scripts/hooks"
DST="$ROOT/.git/hooks"

[ -d "$DST" ] || { echo "no .git/hooks at $DST -- not a git worktree?" >&2; exit 2; }

mode="${1:-install}"
rc=0
found=0

for src in "$SRC"/*; do
    name="$(basename "$src")"
    [ "$name" = "install.sh" ] && continue
    [ -f "$src" ] || continue
    found=$((found + 1))
    dst="$DST/$name"

    if [ "$mode" = "--check" ]; then
        if [ ! -f "$dst" ]; then
            echo "MISSING  $name -- tracked but not installed; run: bash scripts/hooks/install.sh" >&2
            rc=1
        elif ! cmp -s "$src" "$dst"; then
            echo "DRIFTED  $name -- installed copy differs from scripts/hooks/$name" >&2
            diff -u "$src" "$dst" | head -20 >&2
            rc=1
        else
            echo "ok       $name"
        fi
    else
        cp -p "$src" "$dst"
        chmod +x "$dst"
        echo "installed $name -> .git/hooks/$name"
    fi
done

if [ "$found" -eq 0 ]; then
    echo "no hooks found in $SRC -- nothing to do, which is not the same as success" >&2
    exit 2
fi

exit "$rc"
