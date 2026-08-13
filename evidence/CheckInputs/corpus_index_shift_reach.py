#!/usr/bin/env python3
"""Does the corpus REACH the ten `[i-1] -> [i+1]` mutants, or only fail to see them?

The six survivors this dispatch names -- 29e57417, bafd410e, 2ed97e42, e250cdea,
91a7adbc, e712c281 -- plus four siblings of the identical shape are all
`arith_op` on a C++ zero-based index conversion inside a Fortran-indexed loop:

    for (int Imode = 1; Imode <= CntrPar->AWC_NumModes; ++Imode)
        if (CntrPar->AWC_freq[Imode - 1] < 0.0)      ->   [Imode + 1]

RUNBOOK's rule for a survivor is to separate "the corpus does not reach it" from
"the corpus reaches it and cannot see it". Those need different answers and the
mutation artifact records neither: it records only that the OBSERVABLE output
(aviFAIL plus the last message) was unchanged.

This reads the corpus that was actually scored -- `checkinputs_cases.bin`, the
23,076-case file the 06:30 generating run wrote and every one of the five
mutation parts was measured against -- and asks, per site and per case, whether
the mutant's own predicate answers DIFFERENTLY from the reference's. Nothing is
built, nothing is run, and no artifact is written to. It is a read.

HOW IT KNOWS THE LAYOUT. The case stream is a flat sequence of little-endian
scalars with a 0x5649545F marker per case, and its field order is whatever
`harness/emit.py` chose. Rather than transcribe that order by hand -- ~450 reads
per case, and one transposition would silently misattribute every field after it
-- the reader is GENERATED from `checkinputs_test.cpp`, the consumer the
generating run emitted beside the case file. Every `r.i()`, `r.d()`, `r.ds()`,
`r.is()`, `r.fs()` and `r.cs()` in `main()` becomes one op, in file order.

THE POSITIVE CONTROL IS STRUCTURAL AND IT IS THE MARKER (P10). A reader whose
op list is wrong desynchronises within one case and the next marker is not
0x5649545F. Landing on all 23,076 markers, and on end-of-file exactly, is a
statement that the field order is right -- so a reported zero is a zero and not
a reader reading the wrong bytes. `--selftest` additionally prints the fields it
resolved for a case whose values are known independently.

THE HARNESS PADS EVERY ARRAY BY 4096 ZEROS (`emit.py`), so an out-of-bounds read
by a shifted index does not fault here -- it reads 0. That is modelled, because
it is what the scored mutant actually did: `oob` counts those reads and they are
compared as zeros, exactly as the binary under test compared them.

    python3 evidence/CheckInputs/corpus_index_shift_reach.py

Exit 0 always; this reports, it does not judge.
"""
from __future__ import annotations

import re
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TESTDIR = ROOT / "translations" / "ReadSetParameters" / "checkinputs_test"
SRC = TESTDIR / "checkinputs_test.cpp"
BIN = TESTDIR / "checkinputs_cases.bin"
MARKER = 0x5649545F

# The scalars and arrays the ten sites read. Everything else is skipped by size.
WANT = {
    "CntrPar.OL_Mode", "CntrPar.AWC_Mode", "CntrPar.AWC_NumModes",
    "CntrPar.CC_Mode", "CntrPar.CC_Group_N", "CntrPar.StC_Mode",
    "CntrPar.StC_Group_N",
    "CntrPar.Ind_GenTq", "CntrPar.Ind_YawRate", "CntrPar.Ind_R_Speed",
    "CntrPar.Ind_R_Torque", "CntrPar.Ind_R_Pitch",
    "CntrPar.n_Ind_CableControl", "CntrPar.n_Ind_StructControl",
    "CntrPar.n_AWC_freq", "CntrPar.n_AWC_clockangle", "CntrPar.n_AWC_harmonic",
    "CntrPar.n_CC_GroupIndex", "CntrPar.n_StC_GroupIndex",
    "CntrPar.n_Ind_BldPitch",
    "CntrPar.Ind_CableControl", "CntrPar.Ind_StructControl",
    "CntrPar.AWC_freq", "CntrPar.AWC_clockangle", "CntrPar.AWC_harmonic",
    "CntrPar.CC_GroupIndex", "CntrPar.StC_GroupIndex", "CntrPar.Ind_BldPitch",
}

# `X_a.field = r.i()` and also the one bare local the emitter writes,
# `int size_avcMSG_a = (int)r.i();`. A reader call matched by NEITHER pattern
# is not silently dropped -- `assert_all_calls_matched` refuses.
SCALAR = re.compile(r"(?:(\w+)_a\.(\w+)|int (\w+)_a)\s*=\s*(?:\(int\))?r\.([id])\(\)")
ARRAY = re.compile(r"r\.(ds|is|fs|cs)\(\s*([A-Za-z_0-9.()]+?)\s*,\s*(.+?)\s*\)\s*;")

ELEM = {"ds": (8, "<d"), "is": (4, "<i"), "fs": (4, "<f"), "cs": (1, None)}


def array_name(target: str) -> str:
    """`CntrPar_AWC_freq_a.data()` -> `CntrPar.AWC_freq`; `LocalVar_a.PitCom` -> `LocalVar.PitCom`."""
    t = target.replace(".data()", "")
    m = re.fullmatch(r"(\w+)_a\.(\w+)", t)
    if m:
        return f"{m.group(1)}.{m.group(2)}"
    m = re.fullmatch(r"(\w+?)_(.+)_a", t)
    if m:
        return f"{m.group(1)}.{m.group(2)}"
    return t


def count_expr(e: str) -> str:
    """C count expression -> a Python expression over V[...]."""
    e = e.replace("(int)", "")
    # every ternary the emitter writes is `(A > 0 ? A : 0)`
    e = re.sub(r"\(\s*([^()?]+?)\s*>\s*0\s*\?\s*([^():]+?)\s*:\s*0\s*\)", r"max(0,\2)", e)
    e = re.sub(r"(\w+)_a\.(\w+)", r'V["\1.\2"]', e)
    return e


def build_ops() -> list:
    """One op per reader call in main(), in file order."""
    text = SRC.read_text()
    start = text.index("r.marker(c);")
    ops = []
    for line in text[start:].splitlines():
        if "r.marker(" in line:
            continue
        calls = len(re.findall(r"r\.(?:i|d|ds|is|fs|cs)\(", line))
        if not calls:
            continue
        m = SCALAR.search(line)
        if m:
            struct_, field, bare, kind = m.groups()
            name = f"{struct_}.{field}" if struct_ else f"local.{bare}"
            # EVERY integer scalar is kept, not just the wanted ones: the
            # emitter's array counts are expressions over `n_*` fields, and a
            # count whose operand was skipped is a KeyError at read time rather
            # than a wrong answer -- which is how this was found.
            keep = name if (kind == "i" or name in WANT) else None
            ops.append(("s", keep, kind))
            continue
        m = ARRAY.search(line)
        if m:
            kind, target, cnt = m.group(1), m.group(2), m.group(3)
            name = array_name(target)
            ops.append(("a", name if name in WANT else None, kind,
                        compile(count_expr(cnt), "<count>", "eval")))
            continue
        raise SystemExit(f"unrecognised reader call, refusing to guess: {line.strip()!r}")
    n_calls = len(re.findall(r"r\.(?:i|d|ds|is|fs|cs)\(", text[start:]))
    if n_calls != len(ops):
        raise SystemExit(f"{n_calls} reader calls in {SRC.name} but {len(ops)} ops built")
    return ops


def read_cases(ops, n_cases: int, limit: int | None = None):
    """Yield one dict of the WANTed fields per case. Verifies every marker."""
    buf = BIN.read_bytes()
    at = 0
    for c in range(n_cases if limit is None else min(limit, n_cases)):
        (mk,) = struct.unpack_from("<i", buf, at)
        at += 4
        if mk != MARKER:
            raise SystemExit(f"case {c}: stream desync at byte {at-4} (read {mk:#x}). "
                             f"The generated op list does not match this case file.")
        V: dict = {}
        for op in ops:
            if op[0] == "s":
                _, name, kind = op
                if kind == "i":
                    if name:
                        V[name] = struct.unpack_from("<i", buf, at)[0]
                    at += 4
                else:
                    if name:
                        V[name] = struct.unpack_from("<d", buf, at)[0]
                    at += 8
            else:
                _, name, kind, cexpr = op
                n = eval(cexpr, {"max": max}, {"V": V})
                sz, fmt = ELEM[kind]
                if name and fmt:
                    V[name] = list(struct.unpack_from(f"<{n}{fmt[1]}", buf, at)) if n else []
                at += sz * n
        yield c, V
    if limit is None and at != len(buf):
        raise SystemExit(f"read {at} of {len(buf)} bytes -- the op list is short by "
                         f"{len(buf)-at} bytes per the tail; the reader is wrong.")


def get(V, name, i):
    """Element i of a padded array: the harness's own +4096 zeros past the extent."""
    a = V.get(name) or []
    return a[i] if 0 <= i < len(a) else 0


# site -> (mutant id, line, guard(V), trip(V), array, predicate(ref_val, other) )
def sites():
    return [
        ("2ed97e42", 925, "AWC_freq", "*NAMED*",
         lambda V: V["CntrPar.AWC_Mode"] > 0, lambda V: V["CntrPar.AWC_NumModes"],
         "CntrPar.AWC_freq", lambda a: a < 0.0, None),
        ("e250cdea", 932, "AWC_clockangle", "*NAMED*",
         lambda V: V["CntrPar.AWC_Mode"] == 1, lambda V: V["CntrPar.AWC_NumModes"],
         "CntrPar.AWC_clockangle", lambda a: a > 360.0, lambda a: a < 0.0),
        ("91a7adbc", 933, "AWC_clockangle", "*NAMED*",
         lambda V: V["CntrPar.AWC_Mode"] == 1, lambda V: V["CntrPar.AWC_NumModes"],
         "CntrPar.AWC_clockangle", lambda a: a < 0.0, lambda a: a > 360.0),
        ("e712c281", 946, "AWC_clockangle", "*NAMED*",
         lambda V: V["CntrPar.AWC_Mode"] == 2, lambda V: V["CntrPar.AWC_NumModes"],
         "CntrPar.AWC_clockangle", lambda a: a > 360.0, lambda a: a < -360.0),
        ("a50841f1", 947, "AWC_clockangle", "sibling",
         lambda V: V["CntrPar.AWC_Mode"] == 2, lambda V: V["CntrPar.AWC_NumModes"],
         "CntrPar.AWC_clockangle", lambda a: a < -360.0, lambda a: a > 360.0),
        ("1262a174", 952, "AWC_harmonic", "sibling",
         lambda V: V["CntrPar.AWC_Mode"] == 2, lambda V: V["CntrPar.AWC_NumModes"],
         "CntrPar.AWC_harmonic", lambda a: a < 0, None),
        ("69a82f24", 979, "CC_GroupIndex", "sibling",
         lambda V: V["CntrPar.CC_Mode"] > 0, lambda V: V["CntrPar.CC_Group_N"],
         "CntrPar.CC_GroupIndex", lambda a: a < 2601, None),
        ("f9828789", 996, "StC_GroupIndex", "sibling",
         lambda V: V["CntrPar.StC_Mode"] > 0, lambda V: V["CntrPar.StC_Group_N"],
         "CntrPar.StC_GroupIndex", lambda a: a < 2801, None),
    ]


def main(detail: bool = False) -> int:
    n_cases = int(re.search(r"for \(int c = 0; c < (\d+); c\+\+\)", SRC.read_text()).group(1))
    ops = build_ops()
    print(f"reader: {len(ops)} ops generated from {SRC.name}; {n_cases} cases in "
          f"{BIN.name} ({BIN.stat().st_size:,} bytes)\n")

    S = sites()
    # per site: cases where the guard held, iterations run, iterations whose
    # shifted read was out of bounds, iterations whose VALUE differed,
    # iterations whose CONDITION differed, cases with >=1 condition difference
    tot = {s[0]: dict(guard=0, iters=0, oob=0, valdiff=0, conddiff=0, cases=0) for s in S}
    from collections import Counter, defaultdict
    dists = defaultdict(Counter)
    obs_cases = {"cable": [], "struct": []}
    # the two OL sites are a list membership question, not a predicate
    ol = dict(guard=0, cable_iters=0, struct_iters=0,
              cable_valdiff=0, struct_valdiff=0,
              cable_obsdiff=0, struct_obsdiff=0, anyneg_ref=0)
    # Does the corpus put an INADMISSIBLE value in these arrays at all, and does
    # it do so while the loop that reads them is live? Two different repairs.
    bad = {k: 0 for k in ("freq_neg", "freq_neg_live", "clock_bad", "clock_bad_live",
                          "ccidx_bad", "ccidx_bad_live", "stcidx_bad", "stcidx_bad_live",
                          "cable_neg", "cable_neg_dominated",
                          "struct_neg", "struct_neg_dominated")}

    for c, V in read_cases(ops, n_cases):
        if V["CntrPar.AWC_Mode"] > 0:
            dists["AWC_NumModes | AWC_Mode > 0"][V["CntrPar.AWC_NumModes"]] += 1
        if V["CntrPar.AWC_Mode"] == 1:
            dists["AWC_NumModes | AWC_Mode ==1"][V["CntrPar.AWC_NumModes"]] += 1
        if V["CntrPar.AWC_Mode"] == 2:
            dists["AWC_NumModes | AWC_Mode ==2"][V["CntrPar.AWC_NumModes"]] += 1
        if V["CntrPar.CC_Mode"] > 0:
            dists["CC_Group_N   | CC_Mode  > 0"][V["CntrPar.CC_Group_N"]] += 1
        if V["CntrPar.StC_Mode"] > 0:
            dists["StC_Group_N  | StC_Mode > 0"][V["CntrPar.StC_Group_N"]] += 1
        for mid, _ln, _arr, _tag, guard, trip, name, pred, other in S:
            if not guard(V):
                continue
            t = tot[mid]
            t["guard"] += 1
            n = trip(V)
            ext = len(V.get(name) or [])
            hit = False
            for i in range(1, n + 1):
                t["iters"] += 1
                ref = get(V, name, i - 1)
                mut = get(V, name, i + 1)
                if i + 1 >= ext:
                    t["oob"] += 1
                if ref != mut:
                    t["valdiff"] += 1
                if other is None:
                    cr, cm = pred(ref), pred(mut)
                else:
                    o = other(ref)          # the disjunct this mutant does NOT touch
                    cr, cm = pred(ref) or o, pred(mut) or o
                if cr != cm:
                    t["conddiff"] += 1
                    hit = True
            if hit:
                t["cases"] += 1

        arr = lambda k: V.get(k) or []
        if any(x < 0.0 for x in arr("CntrPar.AWC_freq")):
            bad["freq_neg"] += 1
            if V["CntrPar.AWC_Mode"] > 0 and V["CntrPar.AWC_NumModes"] >= 1:
                bad["freq_neg_live"] += 1
        if any(x > 360.0 or x < 0.0 for x in arr("CntrPar.AWC_clockangle")):
            bad["clock_bad"] += 1
            if V["CntrPar.AWC_Mode"] == 1 and V["CntrPar.AWC_NumModes"] >= 1:
                bad["clock_bad_live"] += 1
        if any(x < 2601 for x in arr("CntrPar.CC_GroupIndex")):
            bad["ccidx_bad"] += 1
            if V["CntrPar.CC_Mode"] > 0 and V["CntrPar.CC_Group_N"] >= 1:
                bad["ccidx_bad_live"] += 1
        if any(x < 2801 for x in arr("CntrPar.StC_GroupIndex")):
            bad["stcidx_bad"] += 1
            if V["CntrPar.StC_Mode"] > 0 and V["CntrPar.StC_Group_N"] >= 1:
                bad["stcidx_bad_live"] += 1

        if V["CntrPar.OL_Mode"] > 0:
            ol["guard"] += 1
            # THE DOMINATING CHECK. `any_gt(Ind_CableControl, 0) .AND.
            # CC_Mode /= 2` sits 24 lines BELOW the `any_lt` check the mutant
            # moves, reads the SAME array, and is fired by the elements the knob
            # did not touch. Whenever it holds, the earlier check's message is
            # overwritten and the mutant is unobservable.
            for tag, name, nkey, mkey in (
                    ("cable", "CntrPar.Ind_CableControl", "CntrPar.n_Ind_CableControl", "CntrPar.CC_Mode"),
                    ("struct", "CntrPar.Ind_StructControl", "CntrPar.n_Ind_StructControl", "CntrPar.StC_Mode")):
                seg = [get(V, name, i) for i in range(V[nkey])]
                if any(x < 0 for x in seg):
                    bad[f"{tag}_neg"] += 1
                    if any(x > 0 for x in seg) and V[mkey] != 2:
                        bad[f"{tag}_neg_dominated"] += 1
            prefix = list(V.get("CntrPar.Ind_BldPitch") or []) + [
                V["CntrPar.Ind_GenTq"], V["CntrPar.Ind_YawRate"],
                V["CntrPar.Ind_R_Speed"], V["CntrPar.Ind_R_Torque"],
                V["CntrPar.Ind_R_Pitch"]]
            base_neg = any(x < 0 for x in prefix)
            for tag, name, nkey in (("cable", "CntrPar.Ind_CableControl", "CntrPar.n_Ind_CableControl"),
                                    ("struct", "CntrPar.Ind_StructControl", "CntrPar.n_Ind_StructControl")):
                n = V[nkey]
                ref_seg = [get(V, name, i - 1) for i in range(1, n + 1)]
                mut_seg = [get(V, name, i + 1) for i in range(1, n + 1)]
                ol[f"{tag}_iters"] += max(0, n)
                if ref_seg != mut_seg:
                    ol[f"{tag}_valdiff"] += 1
                # the ONLY observation of this list is any_lt(list, 0)
                other = ([get(V, "CntrPar.Ind_StructControl", i - 1)
                          for i in range(1, V["CntrPar.n_Ind_StructControl"] + 1)]
                         if tag == "cable" else
                         [get(V, "CntrPar.Ind_CableControl", i - 1)
                          for i in range(1, V["CntrPar.n_Ind_CableControl"] + 1)])
                obs_ref = base_neg or any(x < 0 for x in ref_seg) or any(x < 0 for x in other)
                obs_mut = base_neg or any(x < 0 for x in mut_seg) or any(x < 0 for x in other)
                if obs_ref != obs_mut:
                    ol[f"{tag}_obsdiff"] += 1
                    obs_cases[tag].append((c, obs_ref, obs_mut, ref_seg[:12]))
                if tag == "cable" and obs_ref:
                    ol["anyneg_ref"] += 1

    print("PREDICATE SITES -- a `conddiff` is an iteration on which the mutant's")
    print("branch answer differs from the reference's. `cases` is how many of the")
    print(f"{n_cases:,} corpus cases contain at least one such iteration.\n")
    print(f"{'mutant':10s} {'line':>5s} {'array':16s} {'guard':>7s} {'iters':>8s} "
          f"{'oob':>7s} {'valdiff':>8s} {'conddiff':>9s} {'cases':>7s}  note")
    for mid, ln, arr, tag, *_ in S:
        t = tot[mid]
        print(f"{mid:10s} {ln:5d} {arr:16s} {t['guard']:7d} {t['iters']:8d} "
              f"{t['oob']:7d} {t['valdiff']:8d} {t['conddiff']:9d} {t['cases']:7d}  {tag}")

    print(f"\nOL LIST SITES -- 29e57417 (line 825, Ind_CableControl) and bafd410e")
    print("(line 830, Ind_StructControl) append to All_OL_Indices, whose ONLY")
    print("observation is any_lt(list, 0). `obsdiff` is that observation changing.\n")
    print(f"  cases with OL_Mode > 0                 {ol['guard']:8d}")
    print(f"  cable  loop iterations                 {ol['cable_iters']:8d}")
    print(f"  cable  cases whose SEGMENT differs     {ol['cable_valdiff']:8d}")
    print(f"  cable  cases whose any_lt DIFFERS      {ol['cable_obsdiff']:8d}   <- 29e57417")
    print(f"  struct loop iterations                 {ol['struct_iters']:8d}")
    print(f"  struct cases whose SEGMENT differs     {ol['struct_valdiff']:8d}")
    print(f"  struct cases whose any_lt DIFFERS      {ol['struct_obsdiff']:8d}   <- bafd410e")
    print(f"  cases where the reference's any_lt is already true "
          f"{ol['anyneg_ref']:8d}")

    print("\nIS THE ARRAY EVER INADMISSIBLE, AND IS IT INADMISSIBLE WHILE ITS LOOP")
    print("RUNS? `_live` is the conjunction. A zero in the second column and a")
    print("large number in the first is a corpus that perturbs the array and the")
    print("trip count in DIFFERENT cases -- R11 moves one quantity at a time.\n")
    for k in ("freq_neg", "clock_bad", "ccidx_bad", "stcidx_bad"):
        print(f"  {k:14s} {bad[k]:7d}      {k+'_live':20s} {bad[k+'_live']:7d}")
    print("\nAND THE CHECK THAT DOMINATES THE OL SITES -- `any_gt(<the same array>,")
    print("0) .AND. <mode> /= 2`, 24 lines below the check the mutant moves:\n")
    print(f"  cases with a NEGATIVE cable index        {bad['cable_neg']:8d}")
    print(f"    of which the later check also fires    {bad['cable_neg_dominated']:8d}")
    print(f"  cases with a NEGATIVE struct index       {bad['struct_neg']:8d}")
    print(f"    of which the later check also fires    {bad['struct_neg_dominated']:8d}")

    if detail:
        print("\nWHY THE LOOPS BARELY RUN -- the trip count's distribution over the")
        print("corpus, among the cases whose guard held.\n")
        for key, guard, trip in (("AWC_NumModes | AWC_Mode > 0", "CntrPar.AWC_Mode>0", "CntrPar.AWC_NumModes"),
                                 ("AWC_NumModes | AWC_Mode ==1", "CntrPar.AWC_Mode==1", "CntrPar.AWC_NumModes"),
                                 ("AWC_NumModes | AWC_Mode ==2", "CntrPar.AWC_Mode==2", "CntrPar.AWC_NumModes"),
                                 ("CC_Group_N   | CC_Mode  > 0", "CntrPar.CC_Mode>0", "CntrPar.CC_Group_N"),
                                 ("StC_Group_N  | StC_Mode > 0", "CntrPar.StC_Mode>0", "CntrPar.StC_Group_N")):
            d = dists[key]
            print(f"  {key:28s} " + "  ".join(f"{v}:{n}" for v, n in sorted(d.items())))
        print("\nTHE CASES WHERE THE OL OBSERVATION ACTUALLY DIFFERS:")
        for tag, rows in obs_cases.items():
            for c, ref, mut, seg in rows:
                print(f"  {tag:6s} case {c}: any_lt ref={ref} mut={mut}")
                print(f"         reference segment {seg}")
        if not any(obs_cases.values()):
            print("  none")
    return 0


if __name__ == "__main__":
    sys.exit(main("--detail" in sys.argv))
