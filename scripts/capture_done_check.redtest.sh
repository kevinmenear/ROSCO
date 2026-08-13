#!/bin/bash
# Red test for scripts/capture_done_check.sh.
#
# Four cases in a scratch repo under `mktemp -d`, with a STUB done_check.py that
# reports whether it saw a dirty tree -- which is the whole mechanism. The real
# done_check is not needed and would drag in plan.json, git history and the loop.
#
#   bash scripts/capture_done_check.redtest.sh    # exit 0 = all four as expected
set -uo pipefail
SRC="$(cd "$(dirname "$0")" && pwd)/capture_done_check.sh"
T="$(mktemp -d)"; trap 'rm -rf "$T"' EXIT
cd "$T" && git init -q . && git config user.email t@t && git config user.name t
mkdir -p scripts evidence/U
cp "$SRC" scripts/
cat > scripts/done_check.py <<'STUB'
import subprocess, sys
d = subprocess.run(["git","status","--porcelain"],capture_output=True,text=True).stdout.strip()
print(f"U: Verdict.INCOMPLETE  ({'11' if d else '12'}/13 PASS)")
print(f"[ {'FAIL' if d else ' ok '} ] P2   clean tree   {'dirty_tree' if d else ''}")
STUB
echo seed > seed.txt; git add -A; git commit -qm base
fail=0
ck(){ if [ "$2" = "$3" ]; then echo "  ok   $1"; else echo "  FAIL $1: got '$2' want '$3'"; fail=1; fi; }

echo "1 GREEN  clean tree -> capture written, and it reads 12/13"
bash scripts/capture_done_check.sh U >/dev/null 2>&1
ck "exit"            "$?"                                                    "0"
ck "file exists"     "$([ -f evidence/U/done_check.txt ] && echo y || echo n)" "y"
ck "records 12/13"   "$(grep -c '12/13' evidence/U/done_check.txt)"          "1"
ck "P2 not failing"  "$(grep -c 'FAIL.*P2' evidence/U/done_check.txt)"       "0"

echo "2 GREEN  the header the regression deleted is present"
ck "names self-reference" "$(grep -c 'SELF-REFERENTIAL' evidence/U/done_check.txt)" "1"
ck "names 73d3521"        "$(grep -c '73d3521' evidence/U/done_check.txt)"          "1"
ck "gives reproduce cmd"  "$(grep -c 'done_check.py U$' evidence/U/done_check.txt)" "1"

echo "3 RED    a dirty tree is REFUSED, not annotated"
git add -A; git commit -qm captured; echo dirt >> seed.txt
bash scripts/capture_done_check.sh U >/dev/null 2>&1
ck "exit"          "$?"                                              "2"
ck "wrote nothing" "$(git status --porcelain evidence/U | wc -l | tr -d ' ')" "0"

echo "4 RED    plain redirection is the defect -- it must read 11/13"
git checkout -q HEAD -- seed.txt
python3 scripts/done_check.py U > evidence/U/done_check.txt 2>&1
ck "redirect self-references" "$(grep -c '11/13' evidence/U/done_check.txt)" "1"

[ $fail -eq 0 ] && echo "capture_done_check.redtest: all four as expected" \
                || echo "capture_done_check.redtest: FAILURES ABOVE"
exit $fail
