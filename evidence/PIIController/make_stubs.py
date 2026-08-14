#!/usr/bin/env python3
"""Generate evidence/PIIController/piicontroller.*-stub.cpp from the SHIPPED
translation.

Each stub is the shipped file with ONE named edit, applied by exact string
replacement with an assertion that the anchor occurs exactly once. Generating
them rather than hand-copying is unit #24's rule read forward: a stub that has
drifted from the translation measures something else, and the drift is
invisible in the number it prints.

Two stubs are written from scratch rather than derived, because they delete the
whole body: the no-op (P3's red test for the differential harness) and the
wrong-constant (P10's positive control for the kernel).

THE WRONG CONSTANT IS -7.25, CHOSEN AGAINST THE CALL SITE'S ARGUMENTS AND NOT
AGAINST THE TYPE -- unit #25's rule. `Controllers.f90:684` passes `I0` as the
literal `0.0_DbKi`, so a stub returning 0.0 returns the RIGHT answer for every
case that takes the reset arm, and would be measuring this unit's argument list
instead of the kernel. Nothing at either call site can produce -7.25: both
bounds are `±CntrPar%Flp_MaxPit` and the result is clamped between them.
"""
import pathlib

REAL = pathlib.Path('translations/Controllers/piicontroller.cpp')
OUT = pathlib.Path('evidence/PIIController')
real = REAL.read_text()

SIG = ('double PIIController(double error, double error2, double kp, double ki, '
       'double ki2, double minValue, double maxValue, double DT, double I0, '
       'piparams_t* piP, int32_t reset, int* inst)')
assert SIG in real, 'signature drifted -- refusing to generate stubs'

VOIDS = ('    (void)error; (void)error2; (void)kp; (void)ki; (void)ki2;\n'
         '    (void)minValue; (void)maxValue; (void)DT; (void)I0;\n'
         '    (void)piP; (void)reset; (void)inst;\n')

HDR = """// STUB -- NOT the translation. Generated from the shipped file by
// evidence/PIIController/make_stubs.py; the ONE edit is marked below.
"""


def stub(name, anchor, replacement):
    assert real.count(anchor) == 1, f'{name}: anchor occurs {real.count(anchor)} times'
    (OUT / f'piicontroller.{name}.cpp').write_text(HDR + real.replace(anchor, replacement, 1))


def whole(name, why, body):
    (OUT / f'piicontroller.{name}.cpp').write_text(
        f'// STUB -- NOT the translation. {why}\n'
        '#include "vit_types.h"\n'
        f'{SIG} {{\n{VOIDS}{body}}}\n')


# ---------------------------------------------------------------- whole-body
# The unit as a complete no-op. P3's red test for the differential harness: it
# must fail every case AND the mismatch list must NAME every output this unit
# is supposed to write.
#
# THE DEAD `saturate_c` CALL IS LOAD-BEARING, and it is unit #34's rule. VIT
# decides whether to generate the callee bridges by looking for a `_c(` call in
# the .cpp; a bare no-op has none, so `piicontroller_callees.o` is left out of
# the link and a stale sibling object's `saturate_c` becomes an undefined
# reference. A no-op without it does not measure a WEAKER instrument, it
# measures a DIFFERENT one.
whole('noop-stub',
      'The unit as a complete no-op: reads no argument, writes no output,\n'
      '// returns 0.0. P3\'s red test for the differential harness.',
      '    if (false) { (void)saturate_c(0.0, 0.0, 0.0); }  // link parity, unit #34\n'
      '    return 0.0;\n')

# The determinate wrong constant. P10's positive control for the kernel: if
# this PASSES, the kernel is comparing nothing this unit produces.
whole('wrong-constant-stub',
      'Reads no argument, writes no state, returns the determinate\n'
      '// constant -7.25, which no argument at either call site can produce.',
      '    if (false) { (void)saturate_c(0.0, 0.0, 0.0); }  // link parity, unit #34\n'
      '    return -7.25;\n')

# ------------------------------------------------------------ one-edit stubs
# Each of the four writes the ELSE arm makes, deleted one at a time, plus the
# increment and the two error inputs. Together they partition what this unit
# does into parts an instrument can be asked about separately.

stub('no-iterm-clamp-stub',
     '        piP->ITerm[i] = saturate_c(piP->ITerm[i], minValue, maxValue);\n',
     '        /* piP->ITerm[i] = saturate_c(...); DELETED */\n')

stub('no-iterm2-clamp-stub',
     '        piP->ITerm2[i] = saturate_c(piP->ITerm2[i], minValue, maxValue);\n',
     '        /* piP->ITerm2[i] = saturate_c(...); DELETED */\n')

stub('no-output-clamp-stub',
     '        PIIController_result = saturate_c(PIIController_result, minValue, maxValue);\n',
     '        /* PIIController_result = saturate_c(...); DELETED */\n')

stub('no-itermlast-stub',
     '        piP->ITermLast[i] = piP->ITerm[i];\n',
     '        /* piP->ITermLast[i] = piP->ITerm[i]; DELETED */\n')

stub('no-increment-stub',
     '    *inst = *inst + 1;\n',
     '    /* *inst = *inst + 1; DELETED */\n')

# The two integral channels, blinded one at a time. `error2` is integral-only,
# so blinding it removes the whole of what distinguishes this unit from
# PIController; `error` carries both a proportional and an integral term.
stub('ignores-error-stub',
     'double PIIController(double error, double error2,',
     'double PIIController(double vit_unused_error, double error2,')
stub('ignores-error2-stub',
     'double PIIController(double error, double error2,',
     'double PIIController(double error, double vit_unused_error2,')

for extra, name in (('const double error = 0.0;  // STUB: input blinded\n', 'ignores-error-stub'),
                    ('const double error2 = 0.0;  // STUB: input blinded\n', 'ignores-error2-stub')):
    p = OUT / f'piicontroller.{name}.cpp'
    t = p.read_text()
    anchor = '    const int i = *inst - 1;\n'
    assert t.count(anchor) == 1, name
    p.write_text(t.replace(anchor, '    ' + extra + anchor, 1))

print('\n'.join(sorted(str(p) for p in OUT.glob('piicontroller.*-stub.cpp'))))
