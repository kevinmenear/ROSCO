#!/usr/bin/env python3
"""Both `unwrap` call sites, and the whole of its body, in all 27 scenarios.

Reads the committed coverage/line_coverage.json, which was generated from the
CLEAN pre-integration source (54dd134) -- so every line number below is a line
of `git show 54dd134:<file>`, not of the working tree.

The point of printing the GUARD beside the CALL is unit #21's: an empty entry
cannot tell `never ran` from `never instrumented`, but a guard with hundreds of
thousands of hits one line above a CALL with zero can.
"""
import json, subprocess, sys

COV = json.load(open('coverage/line_coverage.json'))
HITS = COV['hits']
BASELINE = '54dd134'

SITES = [
    ('Controllers.f90', [
        (322, 'IF ((CntrPar%OL_Mode > 0) .AND. (CntrPar%Ind_GenTq > 0)) THEN   <- guard'),
        (329, 'IF (CntrPar%OL_Mode == 2) THEN                                  <- guard'),
        (337, 'LocalVar%AzBuffer(1) = LocalVar%AzBuffer(2)'),
        (338, 'LocalVar%AzBuffer(2) = LocalVar%Azimuth'),
        (339, 'LocalVar%AzBuffer = UNWRAP(LocalVar%AzBuffer, ErrVar)           <- CALL SITE 1'),
        (340, 'LocalVar%AzUnwrapped = LocalVar%AzBuffer(2)'),
    ]),
    ('ReadSetParameters.f90', [
        (773, 'IF (CntrPar%OL_Mode == 2) THEN                                  <- guard'),
        (778, 'CALL Read_OL_Input(...)'),
        (780, 'RETURN            <- the reference stops here in 10, 14, 24'),
        (783, 'CntrPar%OL_Breakpoints = CntrPar%OL_Channels(:,Ind_Breakpoint)'),
        (806, 'IF (CntrPar%Ind_Azimuth > 0) THEN                               <- guard'),
        (807, 'CntrPar%OL_Azimuth = Unwrap(...)                                <- CALL SITE 2'),
    ]),
    ('Functions.f90', [
        (518, 'FUNCTION unwrap(x, ErrVar) result(y)'),
        (537, 'y = x'),
        (538, 'DO i = 2, SIZE(x)'),
        (539, 'DO while (y(i) - y(i-1) .LE. -PI)'),
        (540, 'y(i:SIZE(x)) = y(i:SIZE(x)) + 2 * PI'),
        (543, 'DO while (y(i) - y(i-1) .GE. PI)'),
        (544, 'y(i:SIZE(x)) = y(i:SIZE(x)) - 2 * PI'),
        (549, 'IF (ErrVar%aviFAIL < 0) THEN'),
        (550, "ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)"),
    ]),
]

print(f'coverage baseline commit : {BASELINE}')
print(f'scenarios in the dataset : {COV["scenarios"]}')
print(f'scenarios that failed    : {COV["scenarios_failed"]}')
print()
total_body = 0
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
        if fname == 'Functions.f90':
            total_body += tot
    print()

# every executable line of the body, not just the ones listed above
body = {int(k): sum(v.values()) for k, v in HITS.get('Functions.f90', {}).items()
        if 518 <= int(k) <= 557}
print(f'Functions.f90 lines 518..557 (the whole of unwrap) with ANY hit: {sorted(body) or "NONE"}')
print(f'total hits inside unwrap across all 27 scenarios: {sum(body.values())}')
sys.exit(0 if not body else 1)
