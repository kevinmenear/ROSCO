#!/usr/bin/env python3
"""`PIDController`'s ONE call site, and the whole of its body, in all 27 scenarios.

Adapted BY ADDITION from evidence/unwrap/coverage_deadness.py (unit #26, P5):
same reading, different lines. Reads the committed coverage/line_coverage.json,
generated from the CLEAN pre-integration source (54dd134) -- so every line
number below is a line of `git show 54dd134:<file>`, not of the working tree.

WHY THE GUARD IS PRINTED BESIDE THE CALL (unit #21): an empty coverage entry
cannot tell `never ran` from `never instrumented`, but a guard with 407,976 hits
one line above a CALL with zero can.

AND WHY THE PATH THROUGH ReadSetParameters IS PRINTED TOO (unit #26): reading
the guard alone says "no scenario configures it", which is FALSE here.
`vit_sim.py`'s scenario 10 exists for exactly this unit -- its docstring is
"OL_Mode=2 azimuth tracking to exercise PIDController" -- and it sets
OL_Mode = 2, Ind_GenTq = 5, Ind_Azimuth = 6 and RP_Gains = 1000 100 500 0.1.
It reaches ReadSetParameters.f90:778, `Read_OL_Input` fails because
Examples/example_inputs/OL_Mode2_Input.dat is absent from this tree (unit #17),
and the RETURN two statements later takes the scenario out before any
controller code runs at all.

Exit 0 means the body is dead.
"""
import json, subprocess, sys

COV = json.load(open('coverage/line_coverage.json'))
HITS = COV['hits']
BASELINE = '54dd134'

BODY_LO, BODY_HI = 1072, 1128          # FUNCTION PIDController .. END FUNCTION

SITES = [
    ('Controllers.f90', [
        (322, 'IF ((CntrPar%OL_Mode > 0) .AND. (CntrPar%Ind_GenTq > 0)) THEN   <- guard'),
        (329, 'IF (CntrPar%OL_Mode == 2) THEN                                  <- guard'),
        (343, 'LocalVar%OL_Azimuth = interp1d(...)'),
        (344, 'LocalVar%AzError = LocalVar%OL_Azimuth - LocalVar%AzUnwrapped'),
        (346, 'LocalVar%GenTqAz = PIDController(...)                <- THE ONLY CALL SITE'),
        (347, 'LocalVar%GenTq = LocalVar%GenTq + LocalVar%GenTqAz'),
    ]),
    ('ReadSetParameters.f90', [
        (773, 'IF (CntrPar%OL_Mode == 2) THEN                                  <- guard'),
        (778, 'CALL Read_OL_Input(...)'),
        (780, 'RETURN            <- the reference stops here in 10, 14, 24'),
        (806, 'IF (CntrPar%Ind_Azimuth > 0) THEN                               <- guard'),
        (807, 'CntrPar%OL_Azimuth = Unwrap(...)'),
    ]),
    ('Controllers.f90', [
        (1072, 'REAL(DbKi) FUNCTION PIDController(...)                     <- the body'),
        (1100, 'EFilt = LPFilter(...)                                <- the CALLEE'),
        (1103, 'IF (reset) THEN'),
        (1107, 'PIDController = I0'),
        (1110, 'PTerm = kp*error'),
        (1113, 'piP%ITerm(...) = piP%ITerm(...) + DT*ki*error'),
        (1114, 'piP%ITerm(...) = saturate(...)                       <- the CALLEE'),
        (1117, 'DTerm = kd * (EFilt - piP%ELast(...)) / DT'),
        (1120, 'PIDController = saturate(PTerm + ITerm + DTerm, ...) <- the CALLEE'),
        (1123, 'piP%ITermLast(...) = piP%ITerm(...)'),
        (1124, 'piP%ELast(...) = EFilt'),
        (1126, 'objInst%instPI = objInst%instPI + 1'),
    ]),
]

print(f'coverage baseline commit : {BASELINE}')
print(f'scenarios in the dataset : {COV["scenarios"]}')
print(f'scenarios that failed    : {COV["scenarios_failed"]}')
print()
for fname, lines in SITES:
    src = subprocess.run(['git', 'show', f'{BASELINE}:rosco/controller/src/{fname}'],
                         capture_output=True, text=True, check=True).stdout.splitlines()
    print(f'== {fname}')
    for ln, note in lines:
        per = HITS.get(fname, {}).get(str(ln))
        tot = sum(per.values()) if per else 0
        scen = ','.join(sorted(per, key=int)) if per else '-'
        text = src[ln - 1].strip()[:72] if ln <= len(src) else '??'
        print(f'  {ln:>5}  hits={tot:<10} scenarios=[{scen:<12}]  {text}')
    print()

# THE CONTINUATION RULE (unit #33): a coverage zero at a call site's own line is
# not by itself a dead call site, because gcov attributes a continued
# statement's hits to its LAST continuation line. Line 346 is a single physical
# statement with no `&`, and the whole enclosing block is checked as a RANGE
# below, so neither reading depends on one line.
blk = {int(k): sum(v.values()) for k, v in HITS.get('Controllers.f90', {}).items()
       if 323 <= int(k) <= 349}
print(f'Controllers.f90 323..349 (the whole OL_Mode>0 block) with ANY hit: '
      f'{sorted(blk) or "NONE"}')

body = {int(k): sum(v.values()) for k, v in HITS.get('Controllers.f90', {}).items()
        if BODY_LO <= int(k) <= BODY_HI}
print(f'Controllers.f90 {BODY_LO}..{BODY_HI} (the whole of PIDController) with ANY hit: '
      f'{sorted(body) or "NONE"}')
print(f'total hits inside PIDController across all 27 scenarios: {sum(body.values())}')

# POSITIVE CONTROL (P10). A reader of the two "NONE"s above is owed proof that
# this dict lookup CAN be non-empty -- an empty answer from a mis-keyed file
# name or a wrong line base reads exactly the same. PIController's body sits in
# the same file, read out of the same dict, by the same expression.
PIC_LO, PIC_HI = 1031, 1068
ctl = {int(k): sum(v.values()) for k, v in HITS.get('Controllers.f90', {}).items()
       if PIC_LO <= int(k) <= PIC_HI}
print(f'positive control -- PIController body {PIC_LO}..{PIC_HI}: '
      f'{len(ctl)} line(s) with hits, {sum(ctl.values())} total')

ok = not body and not blk and bool(ctl)
print()
print('VERDICT:', 'DEAD (and the control is alive)' if ok else 'NOT DEAD or CONTROL FAILED')
sys.exit(0 if ok else 1)
