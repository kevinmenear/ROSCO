#!/bin/bash
# Which LINES of the shipped translation does the 16,512-case corpus execute?
#
#   bash evidence/ParseDbAry_Opt/run_line_coverage_probe.sh      # tree must be CLEAN
#
# WHY. 79 mutants survive the sweep. "Survived" has three causes and the artifact
# cannot tell them apart: the line never ran; it ran and the mutant is
# behaviour-preserving; it ran, behaved differently, and no compared output
# carries the difference. The first is a claim about the CORPUS and is the one
# `vit_mutate.py --unreachable` wants -- and it is decidable by measurement
# rather than by argument.
#
# HOW. `vit test-validate` writes the translation to `parsedbary_opt.hpp` and the
# generated test `#include`s it, so one gcov run over the test's own object
# gives per-line counts for the translation itself. Nothing is copied and
# nothing is re-implemented: the file measured IS the file the sweep mutates and
# the file `vit integrate` ships.
#
# -O0 rather than the harness's -O2: line attribution under -O2 folds and
# reorders, and a line reported 0 because the optimiser merged it into its
# neighbour is exactly the false `unreachable` this probe exists to avoid.
# Reachability of a SOURCE LINE does not depend on the optimisation level; the
# ACCURACY of gcov's attribution does.
#
# THE CONTROL IS IN THE OUTPUT: the count for the unit's own entry line must
# equal the number of cases, or the probe measured a different program.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
D=translations/ROSCO_Helpers/parsedbary_opt_test
OUT=evidence/ParseDbAry_Opt/line_coverage.txt

docker exec vit-dev bash -lc "
set -e
cd /workspace/ROSCO-r2/$D
# ASK MAKE WHAT LIBS IS -- harness.sh's own rule, and for its own reason: the
# generated Makefile uses both \`=\` and \`+=\` over continued lines, and a
# hand-rolled parser that understands one and not the other breaks on the next
# generator change. \`include Makefile\` with an explicit target only reads.
printf 'include Makefile\nvitprintlibs:\n\t@echo \$(LIBS)\n' > .covmk
LIBS=\$(make -f .covmk vitprintlibs)
rm -f .covmk
[ -n \"\$LIBS\" ] || { echo 'LIBS came back EMPTY -- refusing to link a probe that would measure nothing' >&2; exit 2; }
rm -f *.gcda *.gcno *.gcov cov_test cov_test.o
# The two Fortran objects are make's, not this script's: \`harness.sh\` rewrites
# \`parsedbary_opt_callees.f90\` per tree (bridges kept on a clean tree, dropped on
# an integrated one) and the .o on disk is whichever tree ran last. Linking the
# stale one gives \`undefined reference to findline_c\`, measured 2026-08-18.
make parsedbary_opt_callees.o parsedbary_opt_bridge.o
g++ -O0 -g -fPIC -ffp-contract=off -I. --coverage -c parsedbary_opt_test.cpp -o cov_test.o
g++ --coverage -o cov_test cov_test.o parsedbary_opt_bridge.o parsedbary_opt_callees.o \$LIBS
./cov_test parsedbary_opt_cases.bin > /dev/null 2>&1 || true
gcov -o . cov_test.o parsedbary_opt_test.cpp > /dev/null 2>&1 || true
ls *.gcov
"

cp "$ROOT/$D/parsedbary_opt.hpp.gcov" /tmp/parsedbary_opt.hpp.gcov

python3 - /tmp/parsedbary_opt.hpp.gcov "$OUT" <<'PY'
import sys, pathlib, re
src, out = sys.argv[1], sys.argv[2]
zero, run, nocode = [], 0, 0
lines = []
for raw in pathlib.Path(src).read_text(errors="replace").splitlines():
    m = re.match(r"\s*([^:]+):\s*(\d+):(.*)", raw)
    if not m:
        continue
    cnt, ln, text = m.group(1).strip(), int(m.group(2)), m.group(3)
    if ln == 0:
        continue
    if cnt == "-":
        nocode += 1
    elif cnt == "#####":
        zero.append((ln, text.strip()))
    else:
        run += 1
    lines.append((ln, cnt, text))
body = [f"ParseDbAry_Opt -- line coverage of the SHIPPED translation under the",
        f"16,512-case differential harness corpus, measured by gcov over the",
        f"generated test's own object (the test #includes parsedbary_opt.hpp).",
        f"",
        f"  executable lines RUN      {run}",
        f"  executable lines NOT RUN  {len(zero)}",
        f"  non-code lines            {nocode}",
        f"",
        f"CONTROL: the entry line's count must equal the case count.",
        ]
for ln, cnt, text in lines:
    if "void ParseDbAry_Opt(" in text or "ParseDbAry_Opt(char*" in text:
        body.append(f"  L{ln}  count={cnt}  {text.strip()[:70]}")
body += ["", "LINES NEVER EXECUTED:", ""]
for ln, text in zero:
    body.append(f"  L{ln:4d}  {text}")
pathlib.Path(out).write_text("\n".join(body) + "\n")
print("\n".join(body[:12]))
print(f"... {len(zero)} never-executed line(s); full list in {out}")
PY
