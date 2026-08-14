#!/bin/bash
# Red test for scripts/run_if_time_remains.sh.
#
# Five cases in a scratch repo. The command under guard writes a marker file, so
# "was it refused" is answered by whether the work HAPPENED, not by the exit code
# alone -- a guard that returns 3 while still running the command would pass an
# exit-code-only test.
#
#   bash scripts/run_if_time_remains.redtest.sh    # exit 0 = all five as expected
set -uo pipefail
SRC="$(cd "$(dirname "$0")" && pwd)/run_if_time_remains.sh"
T="$(mktemp -d)"; trap 'rm -rf "$T"' EXIT
cd "$T" && git init -q . && git config user.email t@t && git config user.name t
mkdir -p scripts .loop-run && cp "$SRC" scripts/ && chmod +x scripts/run_if_time_remains.sh
echo seed > seed.txt && git add -A && git commit -qm base
G=scripts/run_if_time_remains.sh
fail=0
ck(){ if [ "$2" = "$3" ]; then echo "  ok   $1"; else echo "  FAIL $1: got '$2' want '$3'"; fail=1; fi; }
did(){ [ -f ran.marker ] && echo yes || echo no; }
work(){ echo x > ran.marker; return "${1:-0}"; }
export -f work

echo "1 RED    no deadline recorded is a REFUSAL, not a pass"
rm -f ran.marker
bash $G 10 bash -c 'work' >/dev/null 2>&1
ck "exit" "$?" "4"; ck "work did NOT run" "$(did)" "no"

echo "2 GREEN  --start records a deadline"
bash $G --start 3600 >/dev/null 2>&1
ck "exit" "$?" "0"; ck "deadline file" "$([ -f .loop-run/DISPATCH_DEADLINE ] && echo y || echo n)" "y"

echo "3 GREEN  ample time -- the work RUNS and its exit propagates"
rm -f ran.marker
bash $G 10 bash -c 'work' >/dev/null 2>&1
ck "exit" "$?" "0"; ck "work ran" "$(did)" "yes"
rm -f ran.marker
bash $G 10 bash -c 'work 7' >/dev/null 2>&1
ck "propagates failure" "$?" "7"; ck "work ran" "$(did)" "yes"

echo "4 RED    work that cannot finish is REFUSED and NOT started"
rm -f ran.marker
bash $G 999999 bash -c 'work' >/dev/null 2>&1
ck "exit" "$?" "3"; ck "work did NOT run" "$(did)" "no"

echo "5 RED    an EXPIRED deadline refuses even a tiny job"
echo $(( $(date +%s) - 60 )) > .loop-run/DISPATCH_DEADLINE
rm -f ran.marker
bash $G 1 bash -c 'work' >/dev/null 2>&1
ck "exit" "$?" "3"; ck "work did NOT run" "$(did)" "no"

[ $fail -eq 0 ] && echo "run_if_time_remains.redtest: all five as expected" \
                || echo "run_if_time_remains.redtest: FAILURES ABOVE"
exit $fail
