#!/bin/bash
# Red test for scripts/mutate_guarded.sh and the MUTATE_IN_PROGRESS hook clause.
#
# Constructs the defect the guard exists to find -- a mutation sweep that leaves
# a mutant in the translation -- and asserts the guard finds it, AND ONLY it.
# The mutant used is the real one: `VS_ControlMode < 0` read `< 1` after the
# 2026-08-13 timeout kill, four characters, one rebuild away from libdiscon.so.
#
# Runs entirely in a scratch git repo under `mktemp -d`, OUTSIDE both workspaces,
# so it never mutates the campaign's own translation to test the check that
# guards it. Cases 4 and 5 exist because this hook was restructured to hold two
# sentinels: a guard that fires on everything is not a guard.
#
#   bash scripts/mutate_guarded.redtest.sh     # exit 0 = all five as expected
set -uo pipefail
SRC="$(cd "$(dirname "$0")/.." && pwd)"
T="$(mktemp -d)"; trap 'rm -rf "$T"' EXIT
cd "$T" && git init -q . && git config user.email t@t && git config user.name t
mkdir -p scripts translations/M .loop-run
cp "$SRC/scripts/mutate_guarded.sh" scripts/
cp "$SRC/.git/hooks/pre-commit" .git/hooks/pre-commit
chmod +x scripts/mutate_guarded.sh .git/hooks/pre-commit
C=translations/M/u.cpp
printf 'if (VS_ControlMode < 0) { fail(); }\n' > $C
git add -A && git commit -qm base
MUT="sed -i.b 's/< 0/< 1/' $C; rm -f $C.b"
UNMUT="sed -i.b 's/< 1/< 0/' $C; rm -f $C.b"
fail=0
ck() { # <name> <got> <want>
    if [ "$2" = "$3" ]; then echo "  ok   $1 ($2)"; else echo "  FAIL $1: got $2 want $3"; fail=1; fi
}
reset() { rm -f .loop-run/MUTATE_IN_PROGRESS .loop-run/TREE_IS_DE_INTEGRATED; git checkout -q HEAD -- $C; }
marker() { [ -f .loop-run/MUTATE_IN_PROGRESS ] && echo raised || echo clear; }
commits() { git commit -q --allow-empty -m probe >/dev/null 2>&1 && echo allowed || echo refused; }

echo "1 GREEN  sweep mutates and restores"
reset; bash scripts/mutate_guarded.sh $C bash -c "$MUT; $UNMUT; exit 0" >/dev/null 2>&1
ck "exit"   "$?"          "0";       ck "marker" "$(marker)"  "clear"; ck "commit" "$(commits)" "allowed"

echo "2 RED    sweep EXITS 0 with a mutant left behind"
reset; bash scripts/mutate_guarded.sh $C bash -c "$MUT; exit 0" >/dev/null 2>&1
ck "exit"   "$?"          "3";       ck "marker" "$(marker)"  "raised"; ck "commit" "$(commits)" "refused"

echo "3 RED    the guard itself SIGKILLed mid-sweep"
reset; bash scripts/mutate_guarded.sh $C bash -c "$MUT; kill -9 \$PPID" >/dev/null 2>&1
ck "exit"   "$?"          "137";     ck "marker" "$(marker)"  "raised"; ck "commit" "$(commits)" "refused"

echo "4 RED    the pre-existing de-integration sentinel still fires"
reset; echo probe > .loop-run/TREE_IS_DE_INTEGRATED
ck "commit" "$(commits)"  "refused"

echo "5 GREEN  neither sentinel -- the guard must not fire on everything"
reset
ck "commit" "$(commits)"  "allowed"

[ $fail -eq 0 ] && echo "mutate_guarded.redtest: all five as expected" || echo "mutate_guarded.redtest: FAILURES ABOVE"
exit $fail
