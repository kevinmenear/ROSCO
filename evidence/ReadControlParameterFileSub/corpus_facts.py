#!/usr/bin/env python3
"""What the 63-case corpus HOLDS CONSTANT, per surviving mutant that needs it.

The site census (`site_census.py`) answers "does the corpus reach this line".
Four survivors need the other kind of fact: the line is reached, and the corpus
holds the INPUT that would make it decide. That is a property of the .IN files
rather than of the run, so it is measured here, over exactly the file list the
probe globs.

Each block prints the number that makes the `unreachable` declaration checkable
AND names the file that would refute it -- because an `unreachable` set with one
common cause is a corpus gap wearing a declaration's name, and the way to tell
the difference is to say what would close it.

    python3 evidence/ReadControlParameterFileSub/corpus_facts.py

Run from the repository root.
"""
import glob
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def files():
    return (sorted(glob.glob(str(ROOT / 'Examples' / 'DISCON*.IN')))
            + sorted(glob.glob(str(ROOT / 'evidence' / 'ReadControlParameterFileSub'
                                   / 'corpus' / '*.IN'))))


def value(text, key):
    """The value column of the line whose comment names `key`, or None."""
    m = re.search(r'^(\S+(?:[ \t]+\S+)*?)[ \t]+!\s*' + re.escape(key) + r'\b',
                  text, re.M)
    return m.group(1) if m else None


def main():
    fs = files()
    print(f'corpus: {len(fs)} file(s)\n')

    no_echo = [Path(f).name for f in fs if value(Path(f).read_text(), 'Echo') is None]
    print(f'[bf876a89]  files with NO `Echo` line: {len(no_echo)}  {no_echo}')
    print('            The literal is ParseInput\'s has_AllowDefault at the Echo '
          'call. Absent AllowDefault means ALLOWED (parseinput_int_opt.cpp:455), '
          'so passing an explicit .FALSE. makes Echo REQUIRED -- which only '
          'decides for a file that does not have one. `drop(base, "Echo")` '
          'would close it.\n')

    vs2 = [Path(f).name for f in fs
           if (value(Path(f).read_text(), 'VS_ControlMode') or '').strip() == '2']
    vs2_missing = [n for n in vs2
                   if value((ROOT / 'Examples' / n).read_text()
                            if (ROOT / 'Examples' / n).is_file() else
                            (ROOT / 'evidence/ReadControlParameterFileSub/corpus'
                             / n).read_text(), 'F_VSRefSpdCornerFreq') is None]
    print(f'[fdfd4622]  files with VS_ControlMode == 2: {len(vs2)}  {vs2}')
    print(f'            of those, files with NO F_VSRefSpdCornerFreq line: '
          f'{len(vs2_missing)}  {vs2_missing}')
    print('            `(VS_ControlMode < 2)` and `<=` differ only AT 2, and only '
          'when the guarded line is missing. VS_ControlMode = 2 plus '
          '`drop(base, "F_VSRefSpdCornerFreq")` would close it.\n')

    notch = []
    for f in fs:
        t = Path(f).read_text()
        n = (value(t, 'F_GenSpdNotch_N') or '').strip()
        if n not in ('', '0'):
            notch.append((Path(f).name, n, value(t, 'F_GenSpdNotch_Ind') is not None))
    print(f'[8a8c4e4d]  files with F_GenSpdNotch_N > 0: {len(notch)}')
    for n, v, has_ind in notch:
        print(f'              {n:<28} N={v:<4} F_GenSpdNotch_Ind present: {has_ind}')
    print('            The call sits inside `IF (F_GenSpdNotch_N > 0)`, so the '
          'predicate `(N == 0)` is FALSE wherever it is evaluated and the mutant '
          '`(N /= 0)` is TRUE -- allowed instead of required. It decides only for '
          'a file with N > 0 and no F_GenSpdNotch_Ind line, and every file with '
          'N > 0 has one. `notch2.IN` minus F_GenSpdNotch_Ind would close it.\n')

    missing = [f for f in fs if not Path(f).is_file()]
    print(f'[cad91b36]  corpus files that do not exist: {len(missing)}')
    print('            The mutated `std::_Exit(2)` is the arm the translation '
          'takes when fopen fails, matching the Fortran runtime\'s own abort. A '
          'case that reached it would take the PROBE PROCESS down with it '
          '(the reference aborts too, by construction), so no corpus that also '
          'produces a comparison can contain one. This is unreachable by the '
          'shape of the instrument, not by an accident of the file list.\n')

    return 0


if __name__ == '__main__':
    sys.exit(main())
