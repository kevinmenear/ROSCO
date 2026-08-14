#!/usr/bin/env python3
"""Generate evidence/PitchSaturation/pitchsaturation.*-stub.cpp from the SHIPPED
translation.

Each stub is the shipped file with ONE named edit, applied by exact string
replacement with an assertion that the anchor occurs exactly once. Generating
them rather than hand-copying is unit #24's rule read forward: a stub that has
drifted from the translation measures something else, and the drift is
invisible in the number it prints.

One stub is written from scratch rather than derived, because it deletes the
whole body: the no-op, which is P3's red test for the differential harness and
P10's control for every probe run through it.

THE WRONG CONSTANT IS -7.25, CHOSEN AGAINST THE CALL SITE AND NOT AGAINST THE
TYPE -- unit #25's rule. The reference returns a minimum blade pitch in radians;
`evidence/PitchSaturation/kernel.corpus_variation.txt` records the 62 captured
returns spanning [0, 0.0654], and `Controllers.f90:88` immediately raises the
result to at least `CntrPar%PC_FinePit`. A stub returning 0.0 would return the
RIGHT answer for the 21 cases whose `ps_min_pitch` is exactly 0.0, so 0.0 is not
a control. Nothing at this call site produces a negative pitch of that size.

WHAT EACH STUB ASKS. Three of the seven are expected to fail and four to pass;
the four that pass are the point of the exercise, because each names something
the 62 captured cases cannot see.

  noop                the whole unit deleted                  -> must FAIL
  wrong-constant      the determinate wrong answer -7.25      -> must FAIL
  unfiltered-wind     WE_Vw read where WE_Vw_F is meant       -> must FAIL
  no-max              max() dropped, first operand returned   -> ? PRC_Min_Pitch is 0.0 in all 62
  max-swapped         the two operands exchanged              -> ? commutative, expected EQUIVALENT
  no-errmsg           the aviFAIL tail deleted                -> ? aviFAIL is 0 in all 62
  no-ps-min-pitch     the write to LocalVar%PS_Min_Pitch      -> ? is that field even an output here?
                      dropped, the same value still returned
"""
import pathlib

REAL = pathlib.Path('translations/ControllerBlocks/pitchsaturation.cpp')
OUT = pathlib.Path('evidence/PitchSaturation')
real = REAL.read_text()

SIG = ('double PitchSaturation(localvariables_view_t* LocalVar, controlparameters_view_t* CntrPar,\n'
       '                       objectinstances_t* objInst, debugvariables_t* DebugVar,\n'
       '                       errorvariables_view_t* ErrVar)')
assert SIG in real, 'signature drifted -- refusing to generate stubs'

HDR = """// STUB -- NOT the translation. Generated from the shipped file by
// evidence/PitchSaturation/make_stubs.py; the ONE edit is marked below.
"""


def derive(name: str, before: str, after: str, note: str) -> None:
    assert real.count(before) == 1, f'{name}: anchor occurs {real.count(before)} times'
    text = HDR + f'// EDIT: {note}\n' + real.replace(before, after)
    (OUT / f'pitchsaturation.{name}-stub.cpp').write_text(text)
    print(f'  wrote pitchsaturation.{name}-stub.cpp -- {note}')


# --- the two written from scratch -------------------------------------------
# A no-op needs no include but keeps `vit_types.h`, so that the types in the
# signature resolve exactly as they do in the real file.
SCRATCH = HDR + """// EDIT: %s
#include "vit_types.h"

""" + SIG + """ {
    (void)LocalVar; (void)CntrPar; (void)objInst; (void)DebugVar; (void)ErrVar;
    return %s;
}
"""

(OUT / 'pitchsaturation.noop-stub.cpp').write_text(
    SCRATCH % ('the whole unit deleted; reads nothing, writes nothing, returns 0.0', '0.0'))
print('  wrote pitchsaturation.noop-stub.cpp -- the whole unit as a no-op')

(OUT / 'pitchsaturation.wrong-constant-stub.cpp').write_text(
    SCRATCH % ('the whole unit replaced by a determinate wrong answer', '-7.25'))
print('  wrote pitchsaturation.wrong-constant-stub.cpp -- returns -7.25')

# --- the five derived by one edit each ---------------------------------------
derive('unfiltered-wind',
       'LocalVar->WE_Vw_F, ErrVar);',
       'LocalVar->WE_Vw, ErrVar);',
       'interp1d reads WE_Vw (unfiltered) where the reference reads WE_Vw_F')

derive('no-max',
       '    const double PitchSaturation_result =\n'
       '        std::max(LocalVar->PS_Min_Pitch, LocalVar->PRC_Min_Pitch);',
       '    const double PitchSaturation_result = LocalVar->PS_Min_Pitch;',
       'max() dropped -- the first operand is returned unconditionally')

derive('max-swapped',
       'std::max(LocalVar->PS_Min_Pitch, LocalVar->PRC_Min_Pitch);',
       'std::max(LocalVar->PRC_Min_Pitch, LocalVar->PS_Min_Pitch);',
       "max()'s two operands exchanged")

derive('no-errmsg',
       '    if (ErrVar->aviFAIL < 0) {\n'
       "        assign_errmsg(ErrVar, std::string(RoutineName) + ':' + errmsg_trim(ErrVar));\n"
       '    }',
       '    // the aviFAIL tail deleted',
       'the RoutineName prefix never written, whatever aviFAIL is')

derive('no-ps-min-pitch',
       '    LocalVar->PS_Min_Pitch =\n'
       '        interp1d_c(',
       '    const double ps_min_pitch_local =\n'
       '        interp1d_c(',
       'LocalVar%PS_Min_Pitch never written; the same value still returned')

# The no-ps-min-pitch edit leaves two later reads of the field, which must read
# the local instead -- otherwise the stub is testing an uninitialised read and
# not the missing write. Applied as a second exact replacement on that file
# alone, and asserted.
p = OUT / 'pitchsaturation.no-ps-min-pitch-stub.cpp'
t = p.read_text()
assert t.count('std::max(LocalVar->PS_Min_Pitch, LocalVar->PRC_Min_Pitch)') == 1
t = t.replace('std::max(LocalVar->PS_Min_Pitch, LocalVar->PRC_Min_Pitch)',
              'std::max(ps_min_pitch_local, LocalVar->PRC_Min_Pitch)')
p.write_text(t)
print('  (no-ps-min-pitch: the downstream read redirected to the local)')
