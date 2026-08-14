#!/usr/bin/env python3
"""Generate evidence/PIDController/probes/clamp-census.cpp from the SHIPPED
translation.

WHY A CENSUS AND NOT ANOTHER COUNTING PROBE. `make_probes.py` writes a sentinel
into `piP%ITerm2` and the failing count IS the number of cases reaching one arm
-- one BIT per case, one harness run per question. The question this dispatch
has to answer is not a bit: it is "what does the corpus put in the two bounds
and in the three terms, case by case", because the remedy is a change to the
INPUTS and a remedy chosen without seeing the distribution is a guess.

So this probe changes NO output. It appends one CSV row per call to a file
outside the compared state, and both sides of the comparison still answer
exactly what they answered before -- the run it produces is a GREEN one, and
that is the check that the census did not perturb what it measures.

Generated from the shipped translation rather than hand-copied: unit #24's rule
read forward -- a probe that has drifted from the translation counts something
else, and the drift is invisible in the number it prints.
"""
import pathlib

real = pathlib.Path('translations/Controllers/pidcontroller.cpp').read_text()
out = pathlib.Path('evidence/PIDController/probes')
out.mkdir(parents=True, exist_ok=True)

CSV = "/workspace/ROSCO-r2/evidence/PIDController/clamp_census.csv"

HDR = f"""// CENSUS PROBE, generated from the shipped translation by
// evidence/PIDController/make_census.py. It writes NO output the harness
// compares -- one CSV row per call to {CSV}
// -- so the run it produces must be GREEN, and that is what says the census
// did not perturb what it measures.
#include <cstdio>
#include <cmath>

static FILE *vit_census = nullptr;
static void vit_census_open() {{
    if (!vit_census) {{
        vit_census = fopen("{CSV}", "w");
        if (vit_census)
            fprintf(vit_census, "reset,minValue,maxValue,error,kp,ki,kd,tf,DT,I0,"
                                "EFilt,PTerm,ITerm_in,ITerm_upd,ITerm_sat,DTerm,"
                                "raw,result\\n");
    }}
}}
"""

ROW = ('        vit_census_open();\n'
       '        if (vit_census) {\n'
       '            fprintf(vit_census,\n'
       '                    "%d,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,'
       '%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\\n",\n'
       '                    (int)(reset != 0), minValue, maxValue, error, kp, ki,'
       ' kd, tf, DT, I0,\n'
       '                    EFilt, @TERMS@);\n'
       '            fflush(vit_census);\n'
       '        }\n')

# The RESET arm: the three terms do not exist there, and a row that invented
# zeros for them would be a value where there is an absence (P6). NaN is what
# this file writes for "the arm never computed it".
RESET_ROW = ROW.replace('@TERMS@',
                        'vit_nan, vit_nan, vit_nan, vit_nan, vit_nan, vit_nan, '
                        'PIDController_result')
ELSE_ROW = ROW.replace('@TERMS@',
                       'PTerm, vit_raw_iterm_in, vit_raw_iterm_upd, '
                       'piP->ITerm[i], DTerm, vit_raw_sum, PIDController_result')

text = HDR + real

# `vit_nan` and the two captured intermediates. Declared at the top of the
# function so both arms can name them; nothing else reads them.
anchor_decl = '    double PIDController_result;'
text = text.replace(
    anchor_decl,
    '    const double vit_nan = std::nan("");  // CENSUS\n'
    '    double vit_raw_iterm_in = vit_nan, vit_raw_iterm_upd = vit_nan,'
    ' vit_raw_sum = vit_nan;  // CENSUS\n'
    '    (void)vit_raw_iterm_in; (void)vit_raw_iterm_upd; (void)vit_raw_sum;\n'
    + anchor_decl, 1)

# ITerm as it arrived, and as the update left it BEFORE the clamp.
anchor_upd = '        piP->ITerm[i] = piP->ITerm[i] + DT * ki * error;'
text = text.replace(
    anchor_upd,
    '        vit_raw_iterm_in = piP->ITerm[i];  // CENSUS\n'
    + anchor_upd
    + '\n        vit_raw_iterm_upd = piP->ITerm[i];  // CENSUS', 1)

# The sum the outer clamp is handed, computed from the same three names the
# reference's own line uses and in the same order.
anchor_sum = '        PIDController_result = saturate_c(PTerm + piP->ITerm[i] + DTerm,'
text = text.replace(
    anchor_sum,
    '        vit_raw_sum = PTerm + piP->ITerm[i] + DTerm;  // CENSUS\n'
    + anchor_sum, 1)

# One row per arm, at the END of each arm so every field is final.
text = text.replace('        PIDController_result = I0;',
                    '        PIDController_result = I0;\n' + RESET_ROW, 1)
text = text.replace('        piP->ELast[i] = EFilt;',
                    '        piP->ELast[i] = EFilt;\n' + ELSE_ROW, 1)

for a in (anchor_decl, anchor_upd, anchor_sum):
    assert a in real, a
assert text.count('vit_census_open();') == 2, 'both arms must write a row'

(out / 'clamp-census.cpp').write_text(text)
print('wrote', out / 'clamp-census.cpp')
