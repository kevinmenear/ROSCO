#!/usr/bin/env python3
"""What kills the three thinnest mutants, counted in the case stream itself.

`mutation/wrap_360.json` reports `'<' -> '<='` killed on 5 of 134 cases,
`'>=' -> '>'` on 2, and one of the three `'360.0' -> '361.0'` mutants on 3.
Small margins, and the artifact does not say what is in them, so this reads the
generated case file and says.

Those mutants differ from the reference on a HANDFUL OF INPUT VALUES and agree
everywhere else, because a comparison mutant changes the predicate's answer only
at the values that sit on the boundary it moves. So the kill count is not a
sample statistic: it is the number of cases in the stream that hold those
values, and if the stream held none, the mutant would survive at ANY corpus
size.

Adapted from evidence/wrap_180/boundary_margin.py, and the reason it is not a
copy is the third value. `wrap_180`'s two guards move one input each. Here the
LOW guard is `x < 0.0`, whose boundary is a ZERO -- and there are two of those.
`-0.0 < 0.0` is false and `-0.0 <= 0.0` is true, so the `'<' -> '<='` mutant
differs at BOTH `0.0` and `-0.0`, and the R6 rung that survives the ladders'
`dict.fromkeys` dedup (unit #14, whose value showed up at unit #24) is carrying
part of this unit's margin too.

The case file is `<stem>_cases.bin` as `<stem>_test.cpp` reads it: a 4-byte
marker per case followed by this unit's single `double`.

Exit 0 means every reported kill count is accounted for exactly by the values
present in the stream, and nothing rests on a near-miss.
"""
import json
import struct
import sys
from collections import Counter

STEM = 'translations/Functions/wrap_360_test/wrap_360'
raw = open(f'{STEM}_cases.bin', 'rb').read()

MARKER_SIZE, VALUE_SIZE = 4, 8
stride = MARKER_SIZE + VALUE_SIZE
if len(raw) % stride:
    print(f'case file is {len(raw)} bytes, not a multiple of {stride}')
    sys.exit(1)

n = len(raw) // stride
markers, xs = set(), []
for k in range(n):
    off = k * stride
    markers.add(struct.unpack_from('<i', raw, off)[0])
    xs.append(struct.unpack_from('<d', raw, off + MARKER_SIZE)[0])

print(f'case file        {len(raw):,} bytes -> {n} cases at {stride} bytes each')
print(f'markers          {len(markers)} distinct value(s) {sorted(markers)}   '
      f'<- one marker per case, so the stride is right')
print(f'distinct x       {len(set(xs))} of {n}')
print(f'range of x       [{min(xs):.6g}, {max(xs):.6g}]')
print()


# Reference and mutants, as functions, so each margin is COMPUTED and not asserted.
def ref(x):
    return x + 360.0 if x < 0.0 else (x - 360.0 if x >= 360.0 else x)


def low_mutant(x):    # '<' -> '<='   (the low guard admits the zeros)
    return x + 360.0 if x <= 0.0 else (x - 360.0 if x >= 360.0 else x)


def high_mutant(x):   # '>=' -> '>'   (the high guard drops x == 360)
    return x + 360.0 if x < 0.0 else (x - 360.0 if x > 360.0 else x)


def const_mutant(x):  # '360.0' -> '361.0' in the COMPARISON only
    return x + 360.0 if x < 0.0 else (x - 360.0 if x >= 361.0 else x)


# `signbit`-aware equality: 0.0 and -0.0 compare equal in Python and are
# DIFFERENT ANSWERS here, so the comparison goes through repr as wrap_180's did.
def differs(a, b):
    return repr(a) != repr(b)


CASES = [
    ("'<' -> '<='", low_mutant, [0.0, -0.0], 5),
    ("'>=' -> '>'", high_mutant, [360.0], 2),
    # This one's differing set is an INTERVAL, not a value: the mutant lifts the
    # high threshold, so every case in [360.0, 361.0) stops wrapping. The stream
    # holds 360.0 twice and the next representable double above it once.
    ("'360.0' -> '361.0' (the comparison)", const_mutant,
     [x for x in sorted(set(xs)) if 360.0 <= x < 361.0], 3),
]

ok = True
for name, mut, boundary, reported in CASES:
    differing = [x for x in xs if differs(ref(x), mut(x))]
    at_boundary = [x for x in xs if any(repr(x) == repr(b) for b in boundary)]
    same_set = sorted(map(repr, set(differing))) == sorted(map(repr, set(at_boundary)))
    print(f'{name}')
    print(f'   cases where it differs from the reference   {len(differing)}')
    print(f'   cases holding x in {str([repr(b) for b in boundary]):<18}      {len(at_boundary)}')
    print(f'   the differing cases are exactly those       {same_set}')
    print(f'   mutation/wrap_360.json reports killed on    {reported} of {n}')
    if len(differing) != reported or not at_boundary or not same_set:
        ok = False
    print()

print('where those values come from -- the generator said so on its own stdout:')
print('   PREDICATE KNOB: x at [-1.0, 0.0, 1.0, 2.0, 359.0, 360.0, 361.0]')
print('   plus R6\'s signed-zero rung, "1 scalar real(s) at NEGATIVE ZERO"')
mult = Counter(repr(x) for x in xs if repr(x) in ('0.0', '-0.0', '360.0'))
print(f'   multiplicity in the stream: {dict(mult)}')
print()

score = json.load(open('mutation/wrap_360.json'))
print(f'mutation/wrap_360.json: {score["killed"]} of {score["mutants"]} killed, '
      f'{score["equivalent_declared"]} declared equivalent, {score["nocompile"]} '
      f'did not compile, score {score["score"]}')
print()

if ok:
    print('VERDICT: the three thin mutants are killed by the cases holding the boundary')
    print('         VALUES (and, for the const mutant, the interval it opens) and by')
    print('         nothing else. Remove R6\'s predicate knob and its signed-zero rung')
    print('         and all three survive at any corpus size -- the margin is two')
    print('         corpus rules, not a sample size.')
else:
    print('VERDICT: a margin does not add up -- read the rows above.')
sys.exit(0 if ok else 1)
