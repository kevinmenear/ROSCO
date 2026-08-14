#!/usr/bin/env python3
"""Generate evidence/PIDController/probes/*.cpp from the SHIPPED translation.

Each probe is the shipped file with ONE marked block inserted. Generating them
rather than hand-copying is unit #24's rule read forward: a probe that has
drifted from the translation counts something else, and the drift is invisible
in the number it prints.

THE COUNTING CHANNEL IS `piP%ITerm2`. This unit never touches that field -- it
belongs to PIIController, which shares the type -- and R4 compares it as one of
174 out-parameters, so the reference's value for it is the input's,
unconditionally. A probe that writes a sentinel there fails EXACTLY the cases
that reach the arm it sits in, and no real answer can collide with it.
"""
import pathlib

real = pathlib.Path('translations/Controllers/pidcontroller.cpp').read_text()
out = pathlib.Path('evidence/PIDController/probes')
out.mkdir(parents=True, exist_ok=True)

HDR = """// COUNTING PROBE, generated from the shipped translation by
// evidence/PIDController/make_probes.py. The only edit is the marked block:
// a sentinel written into `piP%ITerm2`, a field this unit never touches and R4
// compares, so the failing count IS the number of cases that reach the arm.
"""


def probe(name, anchor, insert):
    assert anchor in real, name
    (out / name).write_text(HDR + real.replace(anchor, insert + anchor, 1))


# 0. POSITIVE CONTROL (P10): the channel itself, written unconditionally in the
#    ELSE arm. A zero from any probe below is worth nothing until this is 4610.
probe('control-else-arm.cpp',
      '        // PTerm = kp*error',
      '        piP->ITerm2[i] = 1.0;  // PROBE\n')

# 1. THE OUTER CLAMP IS INACTIVE -- the raw sum lies strictly inside the bounds,
#    so PTerm, ITerm and DTerm all reach the return value.
probe('outer-clamp-inactive.cpp',
      '        piP->ITermLast[i] = piP->ITerm[i];',
      '        {   // PROBE\n'
      '            const double raw = PTerm + piP->ITerm[i] + DTerm;\n'
      '            if (raw > minValue && raw < maxValue) piP->ITerm2[i] = 1.0;\n'
      '        }\n')

# 2. THE ITerm CLAMP IS INACTIVE -- the integrator update lies strictly inside
#    the bounds, so `DT*ki*error` survives into the state and the return.
probe('iterm-clamp-inactive.cpp',
      '        piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);',
      '        if (piP->ITerm[i] > minValue && piP->ITerm[i] < maxValue)  // PROBE\n'
      '            piP->ITerm2[i] = 1.0;\n')

# 3. BOTH INACTIVE AT ONCE -- the cases in which the arithmetic this unit does
#    is end-to-end visible in an output.
probe('both-clamps-inactive.cpp',
      '        piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);',
      '        const double vit_probe_unclamped_iterm =\n'
      '            (piP->ITerm[i] > minValue && piP->ITerm[i] < maxValue) ? 1.0 : 0.0;  // PROBE\n')
t = (out / 'both-clamps-inactive.cpp').read_text().replace(
    '        piP->ITermLast[i] = piP->ITerm[i];',
    '        {   // PROBE\n'
    '            const double raw = PTerm + piP->ITerm[i] + DTerm;\n'
    '            if (vit_probe_unclamped_iterm != 0.0 && raw > minValue && raw < maxValue)\n'
    '                piP->ITerm2[i] = 1.0;\n'
    '        }\n'
    '        piP->ITermLast[i] = piP->ITerm[i];', 1)
(out / 'both-clamps-inactive.cpp').write_text(t)

# 4. minValue < maxValue at all -- an interval a value could fall inside of.
probe('bounds-form-an-interval.cpp',
      '        // PTerm = kp*error',
      '        if (minValue < maxValue) piP->ITerm2[i] = 1.0;  // PROBE\n')
