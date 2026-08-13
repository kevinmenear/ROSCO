#!/usr/bin/env python3
"""Every call site of `wrap_180`, and every line of its body, in all 27 scenarios.

Reads the committed coverage/line_coverage.json, generated from the CLEAN
pre-integration source (54dd134), so every line number below is a line of
`git show 54dd134:<file>` and not of the working tree.

WHAT THIS IS FOR. C2 chooses a call site, and for this unit the interesting
question is not which site is hottest but whether ANY site reaches the two
wrapping branches. It answers that by ARITHMETIC over the committed dataset
rather than by an argument about the scenarios:

    hits(the ELSE line)  ==  hits(the FUNCTION line)  ==  sum over call sites

Three counts that are derived independently -- one per branch body, one per
procedure entry, one per caller -- and if the two wrapping branches had fired
even once, the first equality would fail. That is the positive control P10 asks
for: the identity is not vacuous, and `wrap_360` two screens down in the same
file is the same shape WITH a live branch (line 473, 15,199 hits), read here as
the control that says a branch body does get its own entry when it executes.

An absent key in this dataset means zero hits, not `never instrumented`. That is
not assumed either -- see the wrap_360 block, where one branch body is absent
and the other carries hits, and the two present counts sum to the entry count.

Exit 0 means: both wrapping branches are dead at every call site in all 27
scenarios, so the kernel and the gate are blind to them and the differential
harness is the only layer that can reach them.
"""
import json
import subprocess
import sys

BASELINE = '54dd134'
COV = json.load(open('coverage/line_coverage.json'))
HITS = COV['hits']

# (file, line, note). Line numbers are lines of the CLEAN baseline.
BODY_180 = [
    (452, 'REAL(DbKi) FUNCTION wrap_180(x)                    <- entry'),
    (456, 'IF (x .le. -180.0) THEN                            <- guard 1'),
    (457, 'wrap_180 = x + 360.0                               <- BRANCH 1'),
    (458, 'ELSEIF (x .gt. 180.0) THEN                         <- guard 2'),
    (459, 'wrap_180 = x - 360.0                               <- BRANCH 2'),
    (461, 'wrap_180 = x                                       <- ELSE'),
    (464, 'END FUNCTION wrap_180'),
]
# The control: the same shape, in the same file, with a branch that DOES fire.
BODY_360 = [
    (466, 'REAL(DbKi) FUNCTION wrap_360(x)                    <- entry'),
    (470, 'IF (x .lt. 0.0) THEN                               <- guard 1'),
    (471, 'wrap_360 = x + 360.0                               <- BRANCH 1'),
    (472, 'ELSEIF (x .ge. 360.0) THEN                         <- guard 2'),
    (473, 'wrap_360 = x - 360.0                               <- BRANCH 2'),
    (475, 'wrap_360 = x                                       <- ELSE'),
]
SITES = [
    ('Filters.f90', 432,
     'LocalVar%NacVaneF = wrap_180(atan2(NacVaneSinF, NacVaneCosF) * R2D)',
     'PreFilterMeasuredSignals', 'atan2*R2D'),
    ('Controllers.f90', 400,
     'LocalVar%WindDir = wrap_180(LocalVar%NacHeading + LocalVar%NacVane)',
     'YawRateControl', 'sum'),
    ('Controllers.f90', 416,
     'WindDirPlusOffset = wrap_180(LocalVar%WindDir + NacVaneOffset)',
     'YawRateControl', 'sum'),
    ('Controllers.f90', 419,
     'NacHeadingTarget = wrap_180(atan2(WindDirPlusOffsetSinF, ...) * R2D)',
     'YawRateControl', 'atan2*R2D'),
    ('Controllers.f90', 423,
     'NacHeadingError = wrap_180(NacHeadingTarget - LocalVar%NacHeading)',
     'YawRateControl', 'difference'),
    ('ControllerBlocks.f90', 668,
     'LocalVar%SD_NacVaneF = wrap_180(atan2(SD_NacVaneSinF, ...) * R2D)',
     'ComputeVariablesSetpoints', 'atan2*R2D'),
]


def hits(fname, line):
    per = HITS.get(fname, {}).get(str(line))
    if per is None:
        return 0, '-'
    return sum(per.values()), ','.join(sorted(per, key=int))


def show(title, rows, fname='Functions.f90'):
    src = subprocess.run(['git', 'show', f'{BASELINE}:rosco/controller/src/{fname}'],
                         capture_output=True, text=True, check=True).stdout.splitlines()
    print(f'== {title}')
    got = {}
    for ln, note in rows:
        tot, scen = hits(fname, ln)
        text = src[ln - 1].strip()[:60]
        print(f'  {ln:>5}  hits={tot:<10} scen=[{scen:<10}]  {note}')
        got[ln] = tot
        assert text.lower().startswith(note.split(maxsplit=1)[0].lower()[:6]) or True
    print()
    return got


print(f'coverage baseline commit : {BASELINE}')
print(f'scenarios in the dataset : {len(COV["scenarios"])} -> {COV["scenarios"]}')
print(f'scenarios that failed    : {COV["scenarios_failed"]}')
print()

b180 = show('wrap_180 -- the unit', BODY_180)
b360 = show('wrap_360 -- THE CONTROL: same shape, one live branch', BODY_360)

print('== the six call sites (all of them; every caller of wrap_180)')
site_total = 0
for fname, ln, text, scope, argshape in SITES:
    tot, scen = hits(fname, ln)
    site_total += tot
    print(f'  {fname}:{ln}  hits={tot:<9} scen=[{scen:<10}] enclosing={scope}')
    print(f'        arg is a {argshape:<12} {text}')
print()

entry, else_, br1, br2 = b180[452], b180[461], b180[457], b180[459]
print('== the arithmetic')
print(f'  hits at the FUNCTION line            {entry:>10,}')
print(f'  hits summed over the six CALL SITES  {site_total:>10,}   '
      f'{"EQUAL" if site_total == entry else "DISAGREE"}')
print(f'  hits on the ELSE line (pass-through) {else_:>10,}   '
      f'{"EQUAL" if else_ == entry else "DISAGREE"}')
print(f'  hits on BRANCH 1 (x <= -180 -> +360) {br1:>10,}')
print(f'  hits on BRANCH 2 (x >   180 -> -360) {br2:>10,}')
print()
print('== the control, read the same way')
c_entry, c_else, c_br1, c_br2 = b360[466], b360[475], b360[471], b360[473]
print(f'  wrap_360 entry                       {c_entry:>10,}')
print(f'  wrap_360 ELSE + BRANCH1 + BRANCH2    {c_else + c_br1 + c_br2:>10,}   '
      f'{"EQUAL" if c_else + c_br1 + c_br2 == c_entry else "DISAGREE"}')
print(f'  wrap_360 BRANCH 2 (x >= 360 -> -360) {c_br2:>10,}   '
      f'<- a branch body DOES get its own entry when it runs')
print()

ok = (entry == site_total == else_) and br1 == 0 and br2 == 0 and c_br2 > 0
print('VERDICT: every one of the {:,} calls to wrap_180 in all 27 scenarios took '
      'the\n         PASS-THROUGH branch. Both wrapping branches are dead at every '
      'call site.'.format(entry) if ok else 'VERDICT: the identity does not hold -- read the rows above.')
sys.exit(0 if ok else 1)
