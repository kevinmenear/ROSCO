#!/usr/bin/env python3
"""Transcribe ReadControlParameterFileSub's 193-call parse block to C++.

WHY A SCRIPT AND NOT A HAND TRANSCRIPTION. The block between the echo-file
set-up and `IF (UnEc > 0) CLOSE(UnEc)` is 193 `CALL ParseInput` / `CALL
ParseAry` statements, each carrying a parameter name, a target field, an
optional array length, an optional `AllowDefault` predicate and an optional
`UnEc`. Typing 193 of those by hand is 193 chances to transpose a predicate,
and a transposed predicate is exactly the defect no instrument on this unit can
see (`evidence/.../harness.plan-dump.*` -- the corpus supplies no file, so the
whole block runs on one DISCON.IN per scenario).

A script is CHECKABLE: re-run it and diff against the committed translation.
That is the property a hand transcription does not have.

    python3 evidence/ReadControlParameterFileSub/transcribe_parse_block.py \
        > /tmp/block.cpp
    diff /tmp/block.cpp <the block inside readcontrolparameterfilesub.cpp>

The C++ spelling of each field is taken from `vit_types.h`, NOT from the
Fortran: the reference writes `CntrPar%FL_Mode` where the view struct declares
`Fl_Mode`, and Fortran is case-insensitive while C++ is not.

Run from the repository root.
"""
import re
import sys

FORTRAN = 'rosco/controller/src/ReadSetParameters.f90'
TYPES = 'rosco/controller/src/vit_types.h'

# The block: from the first parse call AFTER the echo-file set-up (the `Echo`
# parse itself is above it, and is hand-written beside the OPEN it feeds) to
# the CLOSE of the echo unit.
FIRST = "CALL ParseInput(FileLines,'LoggingLevel'"
LAST = 'IF (UnEc > 0) CLOSE(UnEc)'


def load_fields():
    """field-name (lowercased) -> (canonical C spelling, kind, width)."""
    vt = open(TYPES).read()
    start = vt.index('typedef struct {', vt.index('CONTROLPARAMETERS_VIEW_T_H'))
    end = vt.index('} controlparameters_view_t;')
    block = vt[start:end]
    scal, arr, canon = {}, {}, {}
    for m in re.finditer(r'^\s{4}(int|double|char)\s+(\w+)(\[(\d+)\])?;', block, re.M):
        scal[m.group(2).lower()] = (m.group(1), m.group(4))
        canon[m.group(2).lower()] = m.group(2)
    for m in re.finditer(r'^\s{4}(int|double)\*\s+(\w+);', block, re.M):
        arr[m.group(2).lower()] = m.group(1)
        canon[m.group(2).lower()] = m.group(2)
    return scal, arr, canon


SCAL, ARR, CANON = load_fields()


def cfield(name):
    return 'CntrPar->' + CANON.get(name.lower(), name)


def split_args(s):
    """Top-level commas only; a quoted comma is not a separator."""
    out, depth, cur, inq = [], 0, '', False
    for ch in s:
        if ch == "'":
            inq = not inq
        if not inq:
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
            elif ch == ',' and depth == 0:
                out.append(cur.strip())
                cur = ''
                continue
        cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def expr(e):
    """A Fortran scalar/logical expression over CntrPar -> C++."""
    c = re.sub(r'CntrPar%(\w+)', lambda m: cfield(m.group(1)), e.strip())
    c = c.replace('.NE.', ' != ').replace('/=', ' != ')
    c = c.replace('.OR.', ' || ').replace('.AND.', ' && ')
    return re.sub(r'\s+', ' ', c).strip()


def allow_default(e):
    """-> (has_AllowDefault, AllowDefault) as the two C arguments."""
    e = e.strip()
    if e == '.TRUE.':
        return '1', '1'
    if e == '.FALSE.':
        return '1', '0'
    return '1', '(%s) ? 1 : 0' % expr(e)


def parse_call(line):
    m = re.match(r'^CALL\s+(ParseInput|ParseAry)\s*\((.*?)\)\s*(!.*)?$', line, re.I)
    if not m:
        return None
    kind, args = m.group(1).lower(), split_args(m.group(2))
    unec_kw = None
    kept = []
    for a in args:
        km = re.match(r'^UnEc\s*=\s*(\w+)$', a, re.I)
        if km:
            unec_kw = km.group(1)
        else:
            kept.append(a)
    args = kept
    assert args[0] == 'FileLines', line
    name = args[1].strip("'")
    field = re.match(r'^CntrPar%(\w+)$', args[2]).group(1)
    if kind == 'parseinput':
        arylen, rest = None, args[3:]
    else:
        arylen, rest = args[3], args[4:]
    assert rest[0] == 'accINFILE(1)' and rest[1] == 'ErrVar', line
    tail = rest[2:]
    has_ad, ad = ('0', '0')
    unec = unec_kw
    if tail:
        has_ad, ad = allow_default(tail[0])
        if len(tail) > 1:
            unec = tail[1]
    has_unec, unec_v = ('1', 'UnEc') if unec else ('0', '0')
    lf = field.lower()
    if kind == 'parseinput':
        ckind, width = SCAL[lf]
        if ckind == 'char':
            return (f'parse_str("{name}", {cfield(field)}, {width}, '
                    f'{has_ad}, {ad}, {has_unec}, {unec_v});')
        star = '&' + cfield(field)
        fn = 'parse_int' if ckind == 'int' else 'parse_dbl'
        return f'{fn}("{name}", {star}, {has_ad}, {ad}, {has_unec}, {unec_v});'
    alen = expr(arylen) if arylen.startswith('CntrPar%') else arylen
    fn = 'parse_inary' if ARR[lf] == 'int' else 'parse_dbary'
    return (f'{fn}("{name}", &{cfield(field)}, &CntrPar->n_'
            f'{CANON[lf]}, {alen}, {has_ad}, {ad}, {has_unec}, {unec_v});')


def main():
    lines = open(FORTRAN).read().split('\n')
    body = [l.replace('\t', ' ').strip() for l in lines[234:793]]
    i = next(k for k, l in enumerate(body) if l.startswith(FIRST))
    j = next(k for k, l in enumerate(body) if l.startswith(LAST))
    out, indent, n = [], 4, 0
    for raw in body[i:j]:
        line = raw.strip()
        if not line:
            out.append('')
            continue
        if line.startswith('!'):
            out.append(' ' * indent + '//' + line[1:])
            continue
        call = parse_call(line)
        if call:
            out.append(' ' * indent + call)
            n += 1
            continue
        if re.match(r'^IF\s*\(\s*ErrVar%aviFAIL\s*<\s*0\s*\)\s*RETURN$', line, re.I):
            out.append(' ' * indent + 'if (ErrVar->aviFAIL < 0) return;')
            continue
        m = re.match(r'^IF\s*\((.*)\)\s*THEN$', line, re.I)
        if m:
            out.append(' ' * indent + 'if (%s) {' % expr(m.group(1)))
            indent += 4
            continue
        if re.match(r'^END\s*IF$|^ENDIF$', line, re.I):
            indent -= 4
            out.append(' ' * indent + '}')
            continue
        m = re.match(r'^IF\s*\((.*?)\)\s*CntrPar%(\w+)\s*=\s*(.*?)\s*(!.*)?$', line, re.I)
        if m:
            out.append(' ' * indent + 'if (%s) %s = %s;'
                       % (expr(m.group(1)), cfield(m.group(2)), expr(m.group(3))))
            continue
        raise SystemExit('untranscribed statement: %r' % line)
    print('\n'.join(out))
    print('// %d parse call(s) transcribed' % n, file=sys.stderr)


if __name__ == '__main__':
    main()
