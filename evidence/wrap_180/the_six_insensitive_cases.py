#!/usr/bin/env python3
"""Two red tests report `130 of 136`. They are blind to DIFFERENT six cases.

WHY THIS EXISTS, and it is a correction rather than a check. The commit message
of `ad9f755` (the post-integration commit) explained the matching counts like
this:

    130 is the same count the pre-integration no-op produced, and for the same
    reason: 6 of the 136 cases hold an x for which the perturbation is not
    observable -- 0.0 and -0.0 map to themselves under negation, and the
    symmetric rungs of the magnitude ladder pair up.

That was written from an argument, not from a measurement, and it is wrong twice.
The sets are not the same six, and neither of the two mechanisms it names is the
one that operates. The wrong claim is left standing in the git log on purpose
(C12) and this is the artifact that corrects it.

WHAT IS ACTUALLY TRUE.

  * The no-op (`return 0.0`) is invisible where `ref(x)` IS 0.0 in bits: the four
    cases at x = 0.0, plus x = +360.0 and x = -360.0, which the reference folds to
    exactly 0.0. Nothing to do with negation.

  * The sign flip (`-x` for `x`) is invisible where `ref(x) == ref(-x)` in bits,
    and that is a statement about THIS unit's asymmetry rather than about the
    corpus. `.le.` on the low guard and `.gt.` on the high one send BOTH endpoints
    to +180: `wrap_180(-180) = -180 + 360 = +180` and `wrap_180(+180) = +180`. So
    all four boundary cases are sign-blind -- the exact property the translation's
    own comment is about is what hides the perturbation there.

  * The two sets overlap in TWO cases (x = +/-360.0), not six.

  * `-0.0` appears in NEITHER set. The corpus does carry a negative zero (R6's
    signed-zero rung, the one unit #14 added), and it is NOT invisible to either
    perturbation: `ref(-0.0) = -0.0`, whose bits differ from `+0.0`, so the no-op
    moves it; and `ref(-(-0.0)) = +0.0`, so the sign flip moves it too. The
    mechanism the wrong claim reached for is the one thing the corpus rules
    already close.

THE GENERAL SHAPE, worth more than this unit: two red tests with the same
FAILURE COUNT on the same corpus are not the same measurement. Unit #26's census
compares red-test counts across corpora and finds six skewed pairs; this is the
same hazard with the corpus held fixed, where nothing at all looks wrong.

Exit 0 means both counts are reproduced from the case file AND the two blind sets
are confirmed distinct.
"""
import struct
import sys

STEM = 'translations/Functions/wrap_180_test/wrap_180'
raw = open(f'{STEM}_cases.bin', 'rb').read()
STRIDE = 12  # 4-byte marker + one double, as <stem>_test.cpp reads it
xs = [struct.unpack_from('<d', raw, k * STRIDE + 4)[0] for k in range(len(raw) // STRIDE)]


def ref(x):
    """The reference, transcribed."""
    return x + 360.0 if x <= -180.0 else (x - 360.0 if x > 180.0 else x)


def bits(v):
    """The harness compares bit patterns, so this must too: +0.0 != -0.0."""
    return struct.pack('<d', v)


noop = {i for i, x in enumerate(xs) if bits(ref(x)) == bits(0.0)}
negate = {i for i, x in enumerate(xs) if bits(ref(x)) == bits(ref(-x))}

print(f'cases in the stream                               {len(xs)}')
print(f'invisible to the NO-OP  (ref(x) == 0.0 in bits)   {len(noop):>3}  -> fails '
      f'{len(xs) - len(noop)}   harness/wrap_180.redtest.json says 130')
print(f'invisible to -x         (ref(x) == ref(-x))       {len(negate):>3}  -> fails '
      f'{len(xs) - len(negate)}   harness/wrap_180.postintegration.redtest.json says 130')
print()
print(f'the same six cases?          {noop == negate}')
print(f'cases in both blind sets     {sorted(noop & negate)}   '
      f'(x = {[xs[i] for i in sorted(noop & negate)]})')
print()

for label, s, why in (
    ('no-op    ', noop, 'ref(x) is 0.0, so returning 0.0 is correct there'),
    ('negation ', negate, 'ref(x) == ref(-x), so the sign cannot be observed'),
):
    print(f'{label} cannot see -- {why}:')
    for i in sorted(s):
        print(f'   case {i:>4}  x = {xs[i]!r:>24}   ref(x) = {ref(xs[i])!r}')
    print()

nz = [i for i, x in enumerate(xs) if bits(x) == bits(-0.0)]
print(f'negative zero in the corpus: {len(nz)} case(s) at {nz}')
for i in nz:
    print(f'   ref(-0.0) = {ref(xs[i])!r}   in noop-blind set: {i in noop}   '
          f'in negation-blind set: {i in negate}')
print()

ok = (len(xs) - len(noop) == 130 and len(xs) - len(negate) == 130
      and noop != negate and len(noop & negate) == 2)
if ok:
    print('VERDICT: both artifacts report 130 of 136 and the two are blind to')
    print('         DIFFERENT six cases, overlapping in two. The equal count is a')
    print('         coincidence of set size; the negation-blind set is exactly the')
    print('         four boundary cases, because this unit sends both endpoints')
    print('         to +180.')
else:
    print('VERDICT: the counts or the sets do not come out as stated -- read above.')
sys.exit(0 if ok else 1)
