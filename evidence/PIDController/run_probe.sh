#!/bin/bash
# Run one counting probe through the committed differential harness and keep the
# artifact. On the CLEAN tree, from the repository root.
#
#   bash evidence/PIDController/run_probe.sh <probe.cpp> <out.json> "<what it counts>"
#
# THE COUNTING CHANNEL IS `piP%ITerm2`, and the choice is the whole reason these
# numbers can be read. This unit NEVER touches ITerm2 -- it belongs to
# PIIController, which shares the type -- and R4 compares it as one of the 174
# out-parameters. So the reference's value for it is the input's, unconditionally,
# and a probe that writes a sentinel there fails EXACTLY the cases that reached
# the arm it was written in. No real answer can collide with it, which is the
# thing unit #32's `LineNum = -7` had to argue for and this does not.
#
# The translation is restored from HEAD afterwards, always -- these probes are
# edits to the file `vit_mutate.py` mutates in place, and this campaign has lost
# three sessions to a mutant left in the tree.
set -eu
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$ROOT"
PROBE="$1"; OUT="$2"; WHAT="$3"
CPP=translations/Controllers/pidcontroller.cpp

cp "$PROBE" "$CPP"
bash scripts/harness.sh PIDController Controllers pidcontroller \
     rosco/controller/src/Controllers.f90 --against translation \
     --out "$OUT" --red-test "$WHAT" > /tmp/probe.log 2>&1 || true
git checkout HEAD -- "$CPP"
python3 - "$OUT" "$PROBE" <<'PYEOF'
import json, sys
d = json.load(open(sys.argv[1]))
print(f"{sys.argv[2]:<62} {d['failed']:>6} of {d['checked']}")
PYEOF
