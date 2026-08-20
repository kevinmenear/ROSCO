#!/usr/bin/env python3
"""Derive corpus round 2 for the ReadControlParameterFileSub probe.

Round 1 is `Examples/DISCON*.IN` -- 28 files, admissible by construction, and
`probe.redtests.txt` names four things they cannot reach. These five are
derived from `Examples/DISCON.IN` by editing ONE value column each (two of them
a small block), so every one is still a real DISCON.IN and each carries a
stated purpose. They live under `evidence/` and not in `Examples/`, because
`Examples/` is read by the gate and by the coverage build and this corpus is
read only by this unit's probe.

The value column is replaced in place, so a derived file differs from its base
in exactly the bytes named below -- `diff` against `Examples/DISCON.IN` is the
control that this script did what it says.

    python3 evidence/ReadControlParameterFileSub/derive_corpus.py

Run from the repository root.
"""
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
BASE = ROOT / 'Examples' / 'DISCON.IN'
OUT = ROOT / 'evidence' / 'ReadControlParameterFileSub' / 'corpus'


def sub(lines, key, newvalue):
    """Replace the VALUE column of the line whose comment names `key`."""
    out = list(lines)
    pat = re.compile(r'^(\S+(?:\s+\S+)*?)(\s+!\s*' + re.escape(key) + r'\b)')
    for i, l in enumerate(out):
        m = pat.match(l)
        if m:
            out[i] = newvalue + m.group(2) + l[m.end():]
            return out
    raise SystemExit('derive_corpus: no line whose comment names %r' % key)


def drop(lines, key):
    pat = re.compile(r'^\S+(?:\s+\S+)*?\s+!\s*' + re.escape(key) + r'\b')
    out = [l for l in lines if not pat.match(l)]
    if len(out) == len(lines):
        raise SystemExit('derive_corpus: no line whose comment names %r' % key)
    if len(out) != len(lines) - 1:
        raise SystemExit('derive_corpus: %r matched %d lines, expected 1'
                         % (key, len(lines) - len(out)))
    return out


def main():
    base = BASE.read_text().split('\n')
    OUT.mkdir(parents=True, exist_ok=True)
    files = {}

    # 1. THE ECHO ARM. `CntrPar%Echo` is 0 in all 28 shipped files, so the echo
    #    OPEN, its five header records, its `Cannot open file` arm and `UnEc>0`
    #    at all 192 downstream calls are unexecuted by round 1 entirely.
    files['echo1.IN'] = sub(base, 'Echo', '1')

    # 2. THE ERROR PATH. PC_MaxPit deleted, with PC_ControlMode left at 1 so
    #    its AllowDefault predicate `CntrPar%PC_ControlMode == 0` is FALSE and a
    #    missing line is FATAL. Reaches aviFAIL < 0, the ErrMsg assignment, one
    #    of the early `return`s and the RoutineName prefix at the end -- none of
    #    which any of the 28 shipped files executes, because they all parse
    #    cleanly.
    files['missing_required.IN'] = drop(base, 'PC_MaxPit')

    # 3. THE TWO OPTIONAL FILTER-INDEX BLOCKS. `IF (F_GenSpdNotch_N > 0)` and
    #    `IF (F_TwrTopNotch_N > 0)` are dead in all 28: the probe reports
    #    F_GenSpdNotch_Ind and F_TwrTopNotch_Ind as unallocated on the
    #    reference side in every case. Two notch filters also give the three
    #    F_Notch* arrays a length of 2 instead of the ROSCO-default 1.
    n = sub(base, 'F_NumNotchFilts', '2')
    n = sub(n, 'F_NotchFreqs', '1.5000  2.5000')
    n = sub(n, 'F_NotchBetaNum', '0.0100  0.0200')
    n = sub(n, 'F_NotchBetaDen', '0.2500  0.3500')
    n = sub(n, 'F_GenSpdNotch_N', '1')
    n = sub(n, 'F_GenSpdNotch_Ind', '1')
    n = sub(n, 'F_TwrTopNotch_N', '1')
    n = sub(n, 'F_TwrTopNotch_Ind', '1')
    files['notch2.IN'] = n

    # 4. THE OTHER ARM OF THE DT_Out DEFAULT. Every shipped file has DT_Out 0,
    #    so `IF (CntrPar%DT_Out == 0) CntrPar%DT_Out = LocalVar%DT` always fires
    #    and `n_DT_Out = NINT(DT_Out/DT)` is always NINT(1). 0.025 over the
    #    probe's DT of 0.01 is 2.5 -- which is also the value that separates
    #    NINT (half away from zero: 3) from rint (half to even: 2).
    files['dt_out.IN'] = sub(base, 'DT_Out', '0.025')

    # 6-9. THE OPEN-LOOP CHANNEL ASSEMBLY, which NOTHING reaches today.
    #
    # Examples/ ships three OL files (DISCON_ol_mode1, _ol_mode2, _ol_cc_stc)
    # and all three name an OL_Filename that does not exist in this workspace
    # -- the toolbox writes it at test time. So `Read_OL_Input` fails, the
    # `IF (ErrVar%aviFAIL < 0) RETURN` right after it fires, and the ~80 lines
    # that follow -- the column extraction, the Unwrap, the two strided-row
    # ALLOCATE blocks -- are unexecuted by the entire round-1 corpus. The probe
    # printed the three error messages, which is how this was found:
    #
    #   ERR  DISCON_ol_mode1.IN  aviFAIL -1
    #        Read_OL_Input:.../OL_Mode1_Input.dat does not exist
    #
    # `Example_OL_Input.dat` DOES exist: 4000 rows, 4 columns (Time, BldPitch,
    # GenTq, YawRate). Read_OL_Input reads exactly OL_Count columns, so each
    # case below is built to make OL_Count come out at 4.
    OLF = '/workspace/ROSCO-r2/Examples/example_inputs/Example_OL_Input.dat'

    def ol(lines, **kw):
        out = sub(lines, 'OL_Mode', str(kw.pop('OL_Mode')))
        out = sub(out, 'OL_Filename', OLF)
        out = sub(out, 'Ind_Breakpoint', '1')
        for k, v in kw.items():
            out = sub(out, k, v)
        return out

    # 6. Ind_BldPitch all EQUAL -- the two "if there are duplicate indices,
    #    don't increment OL_Count" branches, which are the only place OL_Count
    #    does not follow the string. OL_Count = 1 +1 +0 +0 +1 +1 = 4.
    files['ol_dup.IN'] = ol(base, OL_Mode=1, Ind_BldPitch='2 2 2',
                            Ind_GenTq='3', Ind_YawRate='4',
                            Ind_Azimuth='0', Ind_R_Speed='0',
                            Ind_R_Torque='0', Ind_R_Pitch='0',
                            Ind_CableControl='0', Ind_StructControl='0')

    # 7. Ind_BldPitch all DISTINCT -- the same two branches on their other arm.
    #    OL_Count = 1 +1 +1 +1 = 4.
    files['ol_distinct.IN'] = ol(base, OL_Mode=1, Ind_BldPitch='2 3 4',
                                 Ind_GenTq='0', Ind_YawRate='0',
                                 Ind_Azimuth='0', Ind_R_Speed='0',
                                 Ind_R_Torque='0', Ind_R_Pitch='0',
                                 Ind_CableControl='0', Ind_StructControl='0')

    # 8. OL_Mode = 2 with Ind_Azimuth set -- the ONLY path that calls Unwrap,
    #    and the second PRINT. OL_Count = 1 +1 +0 +0 +1 +1 = 4.
    files['ol_azimuth.IN'] = ol(base, OL_Mode=2, Ind_BldPitch='2 2 2',
                                Ind_GenTq='3', Ind_YawRate='0',
                                Ind_Azimuth='4', Ind_R_Speed='0',
                                Ind_R_Torque='0', Ind_R_Pitch='0',
                                Ind_CableControl='0', Ind_StructControl='0')

    # 9. Cable and structural control -- the two ALLOCATE blocks whose
    #    assignment is a STRIDED ROW of a column-major array, the one shape in
    #    this unit that cannot be a memcpy. OL_Count = 1 +1 +0 +0 +1 +1 = 4.
    c = ol(base, OL_Mode=1, Ind_BldPitch='2 2 2',
           Ind_GenTq='0', Ind_YawRate='0', Ind_Azimuth='0',
           Ind_R_Speed='0', Ind_R_Torque='0', Ind_R_Pitch='0',
           Ind_CableControl='3', Ind_StructControl='4')
    c = sub(c, 'CC_Mode', '2')
    c = sub(c, 'CC_Group_N', '1')
    c = sub(c, 'CC_GroupIndex', '1')
    c = sub(c, 'StC_Mode', '2')
    c = sub(c, 'StC_Group_N', '1')
    c = sub(c, 'StC_GroupIndex', '1')
    files['ol_cc_stc.IN'] = c

    # 5. THE Fl_n DEFAULT ARM. `IF (CntrPar%Fl_n == 0) CntrPar%Fl_n = 1` and,
    #    with it, the far side of Fl_U's `CntrPar%Fl_n == 1` AllowDefault.
    files['fl_n0.IN'] = sub(base, 'Fl_n', '0')

    for name, lines in sorted(files.items()):
        p = OUT / name
        p.write_text('\n'.join(lines))
        if len(lines) == len(base):
            nd = sum(1 for a, b in zip(base, lines) if a != b)
            how = '%d line(s) edited in place' % nd
        else:
            how = '%d line(s) removed' % (len(base) - len(lines))
        print('%-24s %4d lines, %s' % (name, len(lines), how))


if __name__ == '__main__':
    sys.exit(main())
