#!/usr/bin/env python3
"""Derive corpus round 5: eight files, each aimed at a NAMED survivor.

Round 4 closed the arms the sweep could reach by changing a VALUE. What was
left were sites the corpus reaches but does not DECIDE, and the census
(`site_census.py`) is what separated the two: a record boundary that is
evaluated 13,571 times and strips a CR 247 times still cannot see a mutant that
pops the last character of every record, because every record in every file
ends inside a trailing COMMENT. So these files move the thing the site actually
reads -- a name at the end of a line, a file with no trailing newline whose last
record matters, an index vector with a zero in it, an open-loop channel nothing
has ever set.

Each file names the mutant ids it is for. That is a prediction, and the sweep
either confirms it or does not; `evidence/ReadControlParameterFileSub/
mutation.survivors.md` records which.

ADDITION, NOT MODIFICATION (P5): rounds 2 and 4 are untouched.

    python3 evidence/ReadControlParameterFileSub/derive_corpus_r5.py

Run from the repository root.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
BASE = ROOT / 'Examples' / 'DISCON.IN'
OUT = ROOT / 'evidence' / 'ReadControlParameterFileSub' / 'corpus'

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from derive_corpus import sub, drop            # noqa: E402  (P4: not re-typed)
from derive_corpus_r4 import ol                # noqa: E402

OLF = '/workspace/ROSCO-r2/Examples/example_inputs/Example_OL_Input.dat'


def truncate_after_name(lines, key):
    """Cut a parameter line off immediately after its NAME.

    `1.57   ! PC_MaxPit    - Maximum physical pitch limit` becomes
    `1.57   ! PC_MaxPit`. The name is then the last thing on the record, which
    is the one shape in which dropping a record's last character changes what
    the parser can find."""
    out = list(lines)
    pat = re.compile(r'^(.*?!\s*' + re.escape(key) + r')\b')
    for i, l in enumerate(out):
        m = pat.match(l)
        if m:
            out[i] = m.group(1)
            return out
    raise SystemExit('derive_corpus_r5: no line whose comment names %r' % key)


def main():
    base = BASE.read_text().split('\n')
    OUT.mkdir(parents=True, exist_ok=True)
    files, raw = {}, {}

    # 1. A NAME AT THE END OF A RECORD.  ids 3dc20a1c
    #    `if (!cur.empty() && cur.back() == '\r') cur.pop_back()` inverted pops
    #    the last character of every record that does NOT end in CR. Every
    #    record in every other file ends inside a trailing comment, so the
    #    inverted form is invisible. PC_MaxPit's AllowDefault is
    #    `PC_ControlMode == 0`, which is FALSE in DISCON.IN -- so losing the `t`
    #    from its name is a missing REQUIRED parameter, not a default.
    files['name_at_eol.IN'] = truncate_after_name(base, 'PC_MaxPit')

    # 2. THE SAME THING AT THE UNTERMINATED LAST RECORD.
    #    ids 0e3f9c71, 7f43e1ea, 0fb080e8
    #    The last line of DISCON.IN is StC_GroupIndex, whose AllowDefault is
    #    `StC_Mode == 0` -- true, so round 4's `noeol.IN` could lose the whole
    #    record and nothing moved. StC_Mode = 2 makes it required. Then:
    #      * dropping the final flush (`if (any)`) loses the parameter
    #      * popping the final record's last character loses the name
    #    and both are the reference erroring where the mutant does not.
    n = sub(base, 'StC_Mode', '2')
    n = truncate_after_name(n, 'StC_GroupIndex')
    while n and not n[-1].strip():
        n.pop()
    raw['name_at_eol_noeol.IN'] = '\n'.join(n)          # NO trailing newline

    # 3. A REQUIRED PARAMETER ON THE FIRST RECORD.  ids aac8f93d
    #    `for (int i = 0; i < records.size(); ++i)` starting at 1 skips record
    #    0, which is a banner comment in every file the corpus has.
    f = drop(base, 'PC_MaxPit')
    f = [l for l in base if re.match(r'^.*!\s*PC_MaxPit\b', l)] + f
    files['first_line_param.IN'] = f

    # 4-6. Ind_BldPitch SHAPES. Rounds 1-4 have (2,2,2), (2,3,4) and (0,3,4):
    #      every neighbour of a positive entry is positive, and every
    #      duplicate test compares two equal non-zeros or two distinct ones in
    #      the same direction. These three separate the three index pairs.
    #        (2,0,3)  ids 36908c5e, 4d5b1a2c   -- bp[1] is the ONLY zero
    #        (2,2,3)  id  5080dda5             -- bp[1]==bp[0] but bp[2] does not
    #        (2,3,2)  id  1056fee9             -- bp[2]==bp[0] but not bp[1]
    #      OL_Count is written out for each; Example_OL_Input.dat has 4 columns.
    #        (2,0,3): 1 +1(bp1) +0 +1(bp3 distinct) +1(GenTq) = 4
    #        (2,2,3): 1 +1 +0(dup) +1 +1(GenTq) = 4
    #        (2,3,2): 1 +1 +1(bp2 distinct) +0(bp3 == bp1) +1(GenTq) = 4
    for stem, vec in [('bp_2_0_3', '2 0 3'), ('bp_2_2_3', '2 2 3'),
                      ('bp_2_3_2', '2 3 2')]:
        files[f'{stem}.IN'] = ol(base, OL_Mode=1, Ind_BldPitch=vec,
                                 Ind_GenTq='4', Ind_YawRate='0',
                                 Ind_Azimuth='0', Ind_R_Speed='0',
                                 Ind_R_Torque='0', Ind_R_Pitch='0',
                                 Ind_CableControl='0', Ind_StructControl='0')

    # 7-8. THE THREE POWER-REFERENCE OPEN-LOOP CHANNELS, which no file in the
    #      corpus has ever set. ids 892f54f6, 79924765 (R_Speed, R_Torque) and
    #      1278433e (R_Pitch). Their `OL_Count = OL_Count + 1` is the only
    #      statement in each arm that anything downstream reads.
    #        R_Speed+R_Torque: 1 +1(bp, all duplicates) +1 +1 = 4
    #        R_Pitch:          1 +1 +1(GenTq) +1 = 4
    files['ol_rspeed_rtorque.IN'] = ol(
        base, OL_Mode=1, Ind_BldPitch='2 2 2', Ind_GenTq='0', Ind_YawRate='0',
        Ind_Azimuth='0', Ind_R_Speed='3', Ind_R_Torque='4', Ind_R_Pitch='0',
        Ind_CableControl='0', Ind_StructControl='0')
    files['ol_rpitch.IN'] = ol(
        base, OL_Mode=1, Ind_BldPitch='2 2 2', Ind_GenTq='3', Ind_YawRate='0',
        Ind_Azimuth='0', Ind_R_Speed='0', Ind_R_Torque='0', Ind_R_Pitch='4',
        Ind_CableControl='0', Ind_StructControl='0')

    for name, lines in sorted(files.items()):
        (OUT / name).write_text('\n'.join(lines))
        print('%-24s %4d lines' % (name, len(lines)))
    for name, blob in sorted(raw.items()):
        (OUT / name).write_text(blob, newline='')
        print('%-24s %4d bytes, no trailing newline' % (name, len(blob)))


if __name__ == '__main__':
    sys.exit(main())
