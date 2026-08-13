#!/usr/bin/env python3
"""What the 41 captured kernel cases hand `wrap_360`, and which of its three arms they take.

WHY THIS EXISTS. `41 of 41 IDENTICAL` says the kernel agreed; it does not say
what it agreed about. For a three-armed branch the question C6 cannot answer on
its own is which arms the capture reaches -- unit #27's kernel ran 62 cases and
touched neither of its unit's branches.

The kernel's only compared field is `strazimuth`, the caller's local at
Controllers.f90:845:

    StrAzimuth = wrap_360(360*LocalVar%Time*CntrPar%AWC_freq(1))*D2R

so the captured reference is `wrap_360(x) * D2R`, and `x` itself is not in the
field log. A FIRST version of this script recovered x from a linear model of the
call site (`x = 0.45*(n-1)` degrees, read off cases 1 and 2). The model puts
every case in the right arm and it does NOT reproduce the captured values bit
for bit -- 6 of 41 -- because `LocalVar%Time` accumulates by `DT` rather than
being multiplied out. It is kept in the git history as what it was: an argument,
where a measurement was available for the price of a build.

WHAT IS USED INSTEAD is three stub runs that already had to happen as red tests,
read for a second thing they can also say. No model, no arithmetic on the
scenario's configuration:

  * PASSTHROUGH (`return x`, both wrapping arms deleted). A case where this is
    IDENTICAL to the reference is a case the reference did not wrap -- the ELSE
    arm, by definition and not by inference. 20 of 41.
  * NO-LOW-BRANCH (the `x < 0` arm deleted, the high arm kept). 41 of 41
    IDENTICAL, so no captured case takes the low arm. This is what makes the
    remaining 21 unambiguous: a wrapped case is either `x<0 -> x+360` or
    `x>=360 -> x-360`, and one of the two is now excluded by measurement.
  * NO-HIGH-BRANCH (the `x >= 360` arm deleted, the low arm kept). 21 of 41
    NOT IDENTICAL -- the same 21 -- which is the same fact from the other side
    and makes the partition explicit: 0 + 21 = 21, and the arms are disjoint so
    deleting them singly must partition what deleting both moves.

Every one of the 21 failing cases differs from the reference by exactly
`360 * D2R` -- 360 degrees in the caller's post-multiplied units -- so the
deleted statement is measured as the value it subtracts, not merely as a
mismatch.

AND `D2R` HERE IS ROSCO'S OWN CONSTANT, NOT `pi/180`. `Constants.f90:23` gives
`D2R = 0.01745329251`, eleven digits, so `360*D2R` is 6.2831853036 and `2*PI`
is 6.2831853072 -- they differ at the tenth digit, which is above the printed
precision of the kernel's own difference line. A first version of this check
compared against `2*math.pi` and reported DOES NOT MATCH on a correct run; the
constant is parsed out of the source now. P7 reaches the probe as well as the
translation.

Exit 0 means: the capture reaches the pass-through arm 20 times and the
`x >= 360` arm 21 times, the `x < 0` arm 0 times, and the coverage dataset
agrees about the last one over all 27 scenarios.
"""
import csv
import json
import math
import re
import sys

E = 'evidence/wrap_360/'

# ROSCO's own D2R, parsed rather than assumed -- see the docstring.
_c = re.search(r'::\s*D2R\s*=\s*([\d.]+)',
               open('rosco/controller/src/Constants.f90').read())
D2R = float(_c.group(1))
WRAP_IN_RADIANS = 360.0 * D2R

ROWS = sorted(
    csv.DictReader(open(E + 'kernel.Controllers845.translation.verify_fields.csv')),
    key=lambda r: int(r['case'].split('.')[-1]),
)


def stub_verdicts(path):
    """{case index: (is_identical, difference or None)} from a kernel.exe log."""
    text = open(path).read()
    out = {}
    for blk in re.split(r"\*+ Verification against '", text)[1:]:
        case = int(blk.split("'", 1)[0].split('.')[-1])
        ident = 'is IDENTICAL' in blk
        m = re.search(r'Difference is\s+([-\dEe.+]+)', blk)
        out[case] = (ident, float(m.group(1)) if m else None)
    return out


passthru = stub_verdicts(E + 'kernel.Controllers845.passthrough-stub.run.txt')
no_low = stub_verdicts(E + 'kernel.Controllers845.no-low-branch-stub.run.txt')
no_high = stub_verdicts(E + 'kernel.Controllers845.no-high-branch-stub.run.txt')

assert len(passthru) == len(no_low) == len(no_high) == len(ROWS) == 41

print(f'compared field : {ROWS[0]["field"]}  ({len(set(r["field"] for r in ROWS))} distinct)')
print(f'cases          : {len(ROWS)}   indices {min(passthru)}..20, 12000..{max(passthru)}')
print()
print('== the three stub runs, counted')
for name, d in (('passthrough  (both arms deleted)', passthru),
                ('no-low-branch (x<0 arm deleted) ', no_low),
                ('no-high-branch(x>=360 deleted)  ', no_high)):
    ident = sum(1 for v in d.values() if v[0])
    print(f'  {name}  {ident:>3} IDENTICAL  {len(d) - ident:>3} NOT IDENTICAL')
print()

low_arm_reached = any(not v[0] for v in no_low.values())
else_cases = sorted(c for c, v in passthru.items() if v[0])
wrapped = sorted(c for c, v in passthru.items() if not v[0])
high_cases = [] if low_arm_reached else wrapped

diffs = {round(v[1], 12) for c, v in passthru.items() if not v[0]}
print('== what the deleted statement is worth, per failing case')
print(f'  distinct |difference| over the {len(wrapped)} wrapped cases : {sorted(diffs)}')
print(f'  360 * D2R, D2R = {D2R!r} from Constants.f90:23   : {WRAP_IN_RADIANS!r}')
print(f'  2*PI, for contrast -- NOT what the caller multiplies by  : {2.0 * math.pi!r}')
print(f'  {"MATCHES -- the mismatch IS the subtracted 360" if len(diffs) == 1 and abs(sorted(diffs)[0] - WRAP_IN_RADIANS) < 1e-9 else "DOES NOT MATCH"}')
print()

print('== the recovered input domain (x, degrees), read back through the arm each case took')
xs = []
for r in ROWS:
    n = int(r['case'].split('.')[-1])
    ref_deg = float(r['reference']) / D2R
    x = ref_deg if n in else_cases else ref_deg + 360.0
    xs.append((n, x, 'else (pass-through)' if n in else_cases else 'high (x >= 360)'))
for n, x, arm in xs[:3] + xs[19:22] + xs[-1:]:
    print(f'  case {n:>5}   x = {x:>12.4f} deg   -> {arm}')
print(f'  ...   the ELSE cases span [{min(x for _, x, a in xs if a.startswith("else")):.4f}, '
      f'{max(x for _, x, a in xs if a.startswith("else")):.4f}] '
      f'and the HIGH cases span [{min(x for _, x, a in xs if a.startswith("high")):.4f}, '
      f'{max(x for _, x, a in xs if a.startswith("high")):.4f}]')
print()
print('  NOTE the high-arm inputs are ~5400 degrees and the results are ~5040. The')
print('  reference wraps ONCE and leaves the rest, which is why `std::fmod` is not')
print('  the same function -- it would have returned ~0..360 for every one of them.')
print()

cov = json.load(open('coverage/line_coverage.json'))['hits']
c_else = sum(cov['Functions.f90']['475'].values())
c_hi = sum(cov['Functions.f90']['473'].values())
c_lo = sum(cov['Functions.f90'].get('471', {}).values())
site845 = sum(cov['Controllers.f90']['845'].values())
site515 = sum(cov['Controllers.f90']['515'].values())
print('== the independent check: gcov over all 27 scenarios (clean baseline 54dd134)')
print(f'  Controllers.f90:845 hits (scenario 22)         {site845:>10,}')
print(f'  Controllers.f90:515 hits (scenario 2)          {site515:>10,}')
print(f'  Functions.f90:475 ELSE   (all scenarios)       {c_else:>10,}')
print(f'  Functions.f90:473 x>=360 (all scenarios)       {c_hi:>10,}')
print(f'  Functions.f90:471 x<0    (all scenarios)       {c_lo:>10,}')
print(f'  ELSE + HIGH + LOW = {c_else + c_hi + c_lo:,}   vs the two sites\' '
      f'{site845 + site515:,}   '
      f'{"EQUAL" if c_else + c_hi + c_lo == site845 + site515 else "DISAGREE"}')
print(f'  the LOW arm is reached {c_lo} times in 27 scenarios at BOTH call sites, and')
print(f'  {"0" if not low_arm_reached else "SOME"} times in this kernel capture -- the two agree.')
print()

ok = (len(else_cases) == 20 and len(high_cases) == 21 and not low_arm_reached
      and len(diffs) == 1 and abs(sorted(diffs)[0] - WRAP_IN_RADIANS) < 1e-9
      and c_else + c_hi + c_lo == site845 + site515 and c_lo == 0)
print('VERDICT: the kernel capture reaches the pass-through arm 20 times and the\n'
      '         `x >= 360` arm 21 times, each wrapped case differing from the\n'
      '         unwrapped answer by exactly 360*D2R. The `x < 0` arm is reached 0\n'
      '         times here and 0 times in all 27 scenarios at both call sites.'
      if ok else 'VERDICT: the identity does not hold -- read the rows above.')
sys.exit(0 if ok else 1)
