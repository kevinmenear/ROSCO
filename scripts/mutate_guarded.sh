#!/bin/bash
# Raise a marker for the duration of a mutation sweep; clear it only once the
# translation is PROVED unchanged.
#
# WHY THIS EXISTS. `vit_mutate.py` mutates the translation IN PLACE and restores
# it when it finishes. Killed, it does not. That has now happened on three of
# three hard kills. The most recent left `VS_ControlMode < 0` reading `< 1`, and
# `restore_integrated.sh`'s last act is a rebuild-and-install -- so the mutant
# was one command away from being compiled into libdiscon.so and gated as the
# translation.
#
# The existing rule (RUNBOOK.md, unit #11) is a `cp` before and a `diff` after,
# written in prose and executed by whoever remembers. A hard kill is exactly the
# case where nobody remembers, because the session that would have remembered is
# the thing that died. This records the intended content hash BEFORE the sweep,
# so the check outlives the session that started it.
#
# CLEARING IS CONDITIONAL, which is the whole point. The marker comes down only
# when the .cpp hashes back to what it was. A kill leaves it up because nothing
# ran to take it down; a sweep whose own internal restore failed leaves it up
# because the hash disagrees. Both are the case it exists for, and a marker that
# cleared unconditionally would report the second as a clean run.
#
# The hash is recorded, not compared against HEAD, so a sweep taken on an
# in-flight uncommitted translation is still guarded.
#
#   bash scripts/mutate_guarded.sh <the .cpp> <command...>
#
# Exit: the command's own status on a proved restore, 3 on a live mutant.
set -uo pipefail

# A GUARD THAT CANNOT MEASURE MUST NOT REPORT A CLEAN RESTORE.
#
# `git` is on neither vit-dev nor vit-harness. Invoked INSIDE a container every
# `git` call below failed, BEFORE and AFTER were both the empty string, they
# compared EQUAL, and the marker came down -- announcing a proved restore having
# proved nothing, and printing `restored to ` with the hash missing. That is the
# exact disarm this script exists to prevent, arriving through the script itself.
# Measured 2026-08-18 on the first `--sanitize` sweep.
#
# So it refuses rather than degrades. It wraps `docker exec` FROM THE HOST; it
# does not run inside the container.
command -v git >/dev/null 2>&1 || {
    cat >&2 <<'NOGIT'
mutate_guarded: REFUSING TO RUN -- no `git` on PATH.

This script proves the restore by hashing the .cpp through git, so without git
it cannot measure anything and would clear its marker unconditionally. Neither
container carries git: run this ON THE HOST, wrapping the container command.

  bash scripts/mutate_guarded.sh <the .cpp> docker exec vit-dev python3 ...
NOGIT
    exit 4
}

ROOT="$(git rev-parse --show-toplevel)"
CPP="$1"; shift
M="$ROOT/.loop-run/MUTATE_IN_PROGRESS"
BEFORE="$(git -C "$ROOT" hash-object "$CPP")"

# An unhashable file is the same absence of a measurement as a missing git --
# a mistyped path, or the .cpp argument omitted so the command lands in $1.
[ -n "$BEFORE" ] || {
    echo "mutate_guarded: REFUSING TO RUN -- cannot hash '$CPP'." >&2
    echo "  Argument 1 must be the .cpp to guard, then the command." >&2
    exit 4
}

mkdir -p "$ROOT/.loop-run"
cat > "$M" <<SENTINEL
$(date -u +%Y-%m-%dT%H:%M:%SZ) a mutation sweep is live (pid $$).
  file      $CPP
  intended  $BEFORE
  cmd       $*
vit_mutate.py mutates that file IN PLACE. If this marker is still here, either
the sweep is still running or it was killed with a MUTANT left in the tree.
  CHECK:  git hash-object $CPP        # must equal the intended hash above
  FIX:    git checkout HEAD -- $CPP   # if it does not
Commits are refused while this file exists.
SENTINEL

"$@"; rc=$?

AFTER="$(git -C "$ROOT" hash-object "$CPP")"
# `-n "$AFTER"` carries its weight: an empty AFTER must never satisfy this, or a
# hash that failed at the END would clear the marker the same way it did above.
if [ -n "$AFTER" ] && [ "$AFTER" = "$BEFORE" ]; then
    rm -f "$M"
    echo "mutate_guarded: $CPP restored to $BEFORE; marker cleared; sweep exit $rc"
    exit $rc
fi

cat >&2 <<MSG
mutate_guarded: REFUSING TO CLEAR THE MARKER.

  $CPP
    is       $AFTER
    intended $BEFORE

A MUTANT IS LIVE IN THE TRANSLATION. The sweep exited $rc, but its in-place
restore did not put the file back. Do NOT build, integrate or gate until:

  git checkout HEAD -- $CPP

The marker stays raised, so commits stay refused, until that hash agrees.
MSG
exit 3
