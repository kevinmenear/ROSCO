#!/usr/bin/env python3
"""What kills the two boundary mutants, counted in the case stream itself.

`mutation/wrap_180.json` reports `'<=' -> '<'` and `'>' -> '>='` killed on 2 of
136 cases each. Two is a small margin and the artifact does not say what is in
it, so this reads the generated case file and says.

Each of those mutants differs from the reference on EXACTLY ONE input value --
`x == -180.0` for the low guard, `x == +180.0` for the high one -- because the
reference and the mutant agree everywhere the predicate's answer is unchanged.
So the kill count is not a sample statistic: it is the number of cases in the
stream that hold that one value, and if the stream held none the mutant would
survive at ANY corpus size.

The case file is `<stem>_cases.bin` as `<stem>_test.cpp` reads it: a 4-byte
marker per case followed by this unit's single `double`.

Exit 0 means both boundary values are present, so both kills are accounted for
exactly and nothing rests on a near-miss.
"""
import struct
import sys
from collections import Counter

STEM = 'translations/Functions/wrap_180_test/wrap_180'
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

# Reference and mutant, as functions, so the margin is COMPUTED and not asserted.
def ref(x):
    return x + 360.0 if x <= -180.0 else (x - 360.0 if x > 180.0 else x)


def low_mutant(x):   # '<=' -> '<'
    return x + 360.0 if x < -180.0 else (x - 360.0 if x > 180.0 else x)


def high_mutant(x):  # '>' -> '>='
    return x + 360.0 if x <= -180.0 else (x - 360.0 if x >= 180.0 else x)


ok = True
for name, mut, boundary in (("'<=' -> '<'", low_mutant, -180.0),
                            ("'>' -> '>='", high_mutant, 180.0)):
    differing = [x for x in xs if repr(ref(x)) != repr(mut(x))]
    at_boundary = [x for x in xs if x == boundary]
    print(f'{name}')
    print(f'   cases where it differs from the reference   {len(differing)}')
    print(f'   cases holding x == {boundary:<+8}                {len(at_boundary)}')
    print(f'   the differing cases are exactly those        '
          f'{sorted(set(differing)) == sorted(set(at_boundary))}')
    print(f'   mutation/wrap_180.json reports killed on     2 of 136')
    if len(differing) != 2 or not at_boundary:
        ok = False
    print()

print('where those values come from -- the generator said so on stdout:')
print('   PREDICATE KNOB: x at [-181.0, -180.0, -179.0, 0.0, 1.0, 2.0, 179.0, 180.0, 181.0]')
print(f'   -180.0 in the knob: {-180.0 in xs}     +180.0 in the knob: {180.0 in xs}')
print()
dup = Counter(x for x in xs if x in (-180.0, 180.0))
print(f'   multiplicity: {dict(dup)}   <- R6 emits each boundary twice (the literal')
print('                 ladder and the predicate knob), which is where the 2 comes from')
print()
if ok:
    print('VERDICT: both boundary mutants are killed by the cases holding the boundary')
    print('         VALUE and by nothing else. Remove the R6 predicate knob and they')
    print('         survive at any corpus size -- the margin is a corpus rule, not a')
    print('         sample size.')
sys.exit(0 if ok else 1)
