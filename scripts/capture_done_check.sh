#!/bin/bash
# Capture done_check.py for one unit into evidence/<unit>/done_check.txt, in the
# only way that records a true reading -- and carry the reason in the file.
#
# WHY THIS EXISTS, and it is not a convenience wrapper.
#
# `python3 scripts/done_check.py <U> > evidence/<U>/done_check.txt` CANNOT record
# a true reading of P2. The shell truncates the redirect target BEFORE the
# subprocess runs, so done_check sees a modified tracked file and reports
# `P2 clean tree: dirty_tree`. Measured on a clean tree, same command, same
# instant -- the only difference being where stdout went:
#
#     stdout to a file OUTSIDE the tree   INCOMPLETE (12/13)   P2  ok
#     stdout redirected into the tree     INCOMPLETE (11/13)   P2  FAIL dirty_tree
#
# A reader of the second has no way to tell that failure from a real one.
#
# THIS HAS ALREADY GONE WRONG TWICE, THE SECOND TIME AS A REGRESSION. 48b9ae3
# carried a header naming the self-reference and annotating the P2 line. 73d3521
# re-captured by plain redirection and the annotation went with it, leaving HEAD
# with an evidence file reading 11/13 and an unexplained dirty-tree failure while
# its own commit message said 12 of 13. The explanation that reconciled them was
# deleted by a routine re-run. So the fix cannot be "re-capture and remember to
# re-add the header" -- forgetting is what happened. The capture writes it.
#
# IT REFUSES ON A DIRTY TREE. The 09:54 capture was taken mid-dispatch with the
# tree dirty for unrelated reasons and recorded a P2 failure that was nothing to
# do with the unit. A capture taken then is not a reading of the committed state,
# so it is refused rather than annotated.
#
#   bash scripts/capture_done_check.sh CheckInputs
#
# Exit 0 on a capture written, 2 on a refusal. The tree is left with exactly one
# modified file: the evidence it just wrote.
set -uo pipefail
ROOT="$(git rev-parse --show-toplevel)"
U="${1:?usage: capture_done_check.sh <unit>}"
OUT="$ROOT/evidence/$U/done_check.txt"

dirty="$(git -C "$ROOT" status --porcelain)"
if [ -n "$dirty" ]; then
    echo "capture_done_check: REFUSING -- the tree is dirty, so a capture would" >&2
    echo "record a P2 failure that is not about $U. Commit or restore first:" >&2
    echo "$dirty" | sed 's/^/    /' >&2
    exit 2
fi

TMP="$(mktemp)"; trap 'rm -f "$TMP"' EXIT
python3 "$ROOT/scripts/done_check.py" "$U" > "$TMP" 2>&1; rc=$?
head="$(git -C "$ROOT" log -1 --format='%h ("%s")')"

mkdir -p "$(dirname "$OUT")"
{
  echo "# done_check.py $U -- captured $(date -u +%Y-%m-%dT%H:%M:%SZ) by scripts/capture_done_check.sh"
  echo
  echo "Run against the committed state at $head, with a CLEAN tree, with stdout"
  echo "going to a file outside the repository. The run is reproduced below verbatim."
  echo
  echo "ON P2, WHICH IS SELF-REFERENTIAL IN ANY REDIRECTED CAPTURE. \`>\` truncates its"
  echo "target before the subprocess runs, so"
  echo
  echo "    python3 scripts/done_check.py $U > evidence/$U/done_check.txt"
  echo
  echo "always sees a modified tracked file and reports \`P2 clean tree: dirty_tree\`,"
  echo "one predicate worse than the truth. That is why this file is captured out of"
  echo "tree and moved in, and why it is written by a script rather than by hand: the"
  echo "header was added once at 48b9ae3 and a plain re-capture at 73d3521 removed it."
  echo "Reproduce the reading below with no redirection:"
  echo
  echo "    python3 scripts/done_check.py $U"
  echo
  echo "done_check exited $rc (0 only when COMPLETE)."
  echo
  cat "$TMP"
} > "$OUT"

echo "capture_done_check: wrote $OUT (done_check exit $rc)"
grep -E "Verdict|FAIL" "$OUT" | sed 's/^/  /'
exit 0
