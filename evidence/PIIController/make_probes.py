#!/usr/bin/env python3
"""Generate evidence/PIIController/probes/*.cpp from the SHIPPED translation.

Each probe is the shipped file with ONE marked block inserted. Generating them
rather than hand-copying is unit #24's rule read forward: a probe that has
drifted from the translation counts something else, and the drift is invisible
in the number it prints.

THE COUNTING CHANNEL IS `piP%ELast`. This unit never touches that field -- it
belongs to PIDController, which shares the type -- and R4 compares it as one of
six out-parameters, so the reference's value for it is the input's,
unconditionally. A probe that writes a sentinel there fails EXACTLY the cases
that reach the arm it sits in, and no real answer can collide with it. Unit #34
used `piP%ITerm2` for the same job; ITerm2 is unavailable here because this unit
writes it.

The third file is NOT a sentinel probe. It is the shipped translation with the
upstream ITermLast2 asymmetry REPAIRED, and its failing count is the evidence
that the asymmetry is observable to this harness rather than merely transcribed.
"""
import pathlib

real = pathlib.Path('translations/Controllers/piicontroller.cpp').read_text()
out = pathlib.Path('evidence/PIIController/probes')
out.mkdir(parents=True, exist_ok=True)

HDR = """// COUNTING PROBE, generated from the shipped translation by
// evidence/PIIController/make_probes.py. The only edit is the marked block:
// a sentinel written into `piP%ELast`, a field this unit never touches and R4
// compares, so the failing count IS the number of cases that reach the arm.
"""


def probe(name, anchor, insert):
    assert real.count(anchor) == 1, name
    (out / name).write_text(HDR + real.replace(anchor, insert + anchor, 1))


# 1. THE ELSE ARM (P10's positive control for probe 3, and half of the partition)
probe('control-else-arm.cpp',
      '        // PTerm = kp*error\n',
      '        piP->ELast[i] = 1.0;  // PROBE\n')

# 2. THE RESET ARM -- the other half. 1 + 2 must equal the corpus size, and a
#    partition that closes is what says neither count is measuring the other.
probe('control-reset-arm.cpp',
      '        piP->ITerm[i] = I0;\n',
      '        piP->ELast[i] = 1.0;  // PROBE\n')

# 3. THE ASYMMETRY REPAIRED. The reference writes `piP%ITermLast(inst)` in the
#    ELSE arm and does NOT write `piP%ITermLast2(inst)` there, although the RESET
#    arm initialises both. The translation transcribes that (P7). This probe
#    "fixes" it, and its failing count is what says the harness can see the
#    difference -- the no-op red test does not name ITermLast2, and without this
#    that absence would be indistinguishable from a channel the corpus cannot
#    reach.
anchor = '        piP->ITermLast[i] = piP->ITerm[i];\n'
assert real.count(anchor) == 1
(out / 'itermlast2-repaired.cpp').write_text(
    "// PROBE -- NOT the translation, and NOT a sentinel probe. This is the\n"
    "// shipped file with the upstream asymmetry REPAIRED: `piP%ITermLast2` set\n"
    "// from `piP%ITerm2` in the ELSE arm, mirroring what the reference does for\n"
    "// ITermLast and ITerm. Its failing count is the number of cases on which\n"
    "// the harness would REJECT that repair -- i.e. the evidence that the\n"
    "// asymmetry is observable here rather than merely transcribed.\n"
    + real.replace(anchor, anchor + '        piP->ITermLast2[i] = piP->ITerm2[i];  // PROBE\n', 1))

print('\n'.join(sorted(str(p) for p in out.glob('*.cpp'))))
