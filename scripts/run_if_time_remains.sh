#!/bin/bash
# Refuse to START work that cannot finish before the dispatch deadline.
#
# WHY THIS EXISTS. Unit #29's third dispatch was killed at 06:59:40 by the 7200s
# timeout while STARTING a seventh per-operator mutation sweep with 13 minutes
# left. It had already done the right thing -- split the sweep by operator and
# run each in the foreground, exactly as the method requires -- and it still lost
# the work, because nothing told it how much clock remained. Three units of this
# campaign have lost work to that. Finding 3 states it in general: a verification
# step longer than one foreground command needs a defined way to be split, or it
# gets backgrounded; the corollary is that a split is useless if the last piece
# is started too late to finish.
#
# The refusal is the point. A piece of work that cannot finish is not merely
# wasted -- a mutation sweep killed mid-flight leaves a MUTANT in the
# translation (see scripts/mutate_guarded.sh), so starting one you cannot finish
# is worse than not starting it.
#
#   bash scripts/run_if_time_remains.sh --start 7200        # at dispatch open
#   bash scripts/run_if_time_remains.sh 400 <command...>    # needs 400s to run
#
# NO DEADLINE RECORDED IS A REFUSAL, not a pass. "I do not know how much time is
# left" is precisely the state that killed unit #29, and a tool that shrugged at
# it would reproduce the failure while appearing to guard against it.
#
# Exit: the command's own status when it ran, 3 when refused for want of time,
# 4 when no deadline is recorded. Reports elapsed against the estimate on every
# run, so the next estimate is informed by a measurement rather than a guess.
set -uo pipefail
ROOT="$(git rev-parse --show-toplevel)"
D="$ROOT/.loop-run/DISPATCH_DEADLINE"

if [ "${1:-}" = "--start" ]; then
    secs="${2:?usage: --start <seconds>}"
    mkdir -p "$(dirname "$D")"
    echo $(( $(date +%s) + secs )) > "$D"
    echo "run_if_time_remains: deadline set $(date -r "$(cat "$D")" +%H:%M:%S) (${secs}s from now)"
    exit 0
fi

need="${1:?usage: run_if_time_remains.sh <seconds-needed> <command...>}"; shift
[ $# -gt 0 ] || { echo "run_if_time_remains: no command given" >&2; exit 2; }

if [ ! -f "$D" ]; then
    cat >&2 <<MSG
run_if_time_remains: REFUSING -- no dispatch deadline is recorded.

Not knowing how much clock remains is the state that killed unit #29 mid-sweep.
Record it at the top of the dispatch:

    bash scripts/run_if_time_remains.sh --start <the --timeout-s value>
MSG
    exit 4
fi

left=$(( $(cat "$D") - $(date +%s) ))
if [ "$left" -lt "$need" ]; then
    cat >&2 <<MSG
run_if_time_remains: REFUSING -- ${left}s remain and this needs ${need}s.

  not started:  $*

Work that cannot finish is worse than work not started: a mutation sweep killed
mid-flight leaves a MUTANT in the translation. Commit what is already measured,
report which pieces were not taken, and let the next dispatch take them.
MSG
    exit 3
fi

echo "run_if_time_remains: ${left}s remain, ${need}s needed -- starting"
t0=$(date +%s)
"$@"; rc=$?
el=$(( $(date +%s) - t0 ))
echo "run_if_time_remains: exit $rc after ${el}s (estimated ${need}s; $(( $(cat "$D") - $(date +%s) ))s now remain)"
exit $rc
