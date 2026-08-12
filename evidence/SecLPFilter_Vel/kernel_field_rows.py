#!/usr/bin/env python3
"""Regenerate kernel_field_rows.txt from the four committed kernel run logs.

`kernel/` is untracked and reset_to_clean.sh removes it, so VIT's
verify_fields.csv does not survive the cycle (RUNBOOK, unit #17). The kernel's
own stdout does, and it carries the same per-case per-field verdicts.

Unit #19 adds one column the SecLPFilter version of this script did not need:
KGen writes `NOT IDENTICAL(within tolerance)` for a field whose absolute RMS
difference is under 1e-14, and COUNTS THAT CASE AS PASSED. So a row can move
while the verdict line still reads PASSED, and the two have to be reported
separately or the zero-output stub's log reads as a green.

    python3 kernel_field_rows.py > kernel_field_rows.txt
"""
import re, collections

LOGS = (
    ("real translation",        "kernel.verification-PASSES-62of62.run.txt"),
    ("zero stub",               "kernel.zero-stub-FAILS.run.txt"),
    ("hardcoded-argument stub", "kernel.hardcoded-arguments-stub-PASSES.run.txt"),
    ("zero-output stub",        "kernel.zero-output-stub-PASSES-verdict-63-rows-move.run.txt"),
)


def parse(path):
    txt = open(path, errors="replace").read()
    cases = txt.split("***************** Verification against")
    rows, moved = [], collections.defaultdict(set)
    for c in cases[1:]:
        name = re.match(r"\s*'([^']+)'", c).group(1)
        idx = int(name.rsplit(".", 1)[1])
        for mm in re.finditer(r"^\s*(\S+) is (IDENTICAL|NOT IDENTICAL)(\(within tolerance\))?", c, re.M):
            status = "IDENTICAL" if mm.group(2) == "IDENTICAL" else (
                "NOT IDENTICAL (within tol)" if mm.group(3) else "NOT IDENTICAL (out of tol)")
            rows.append((idx, mm.group(1), status))
            if status != "IDENTICAL":
                moved[mm.group(1)].add(idx)
    verdict = re.search(r"Number of verification-passed cases :\s*(\d+)", txt)
    total = re.search(r"Total number of verification cases\s*:\s*(\d+)", txt)
    return len(cases) - 1, rows, moved, (verdict.group(1) if verdict else "?"), (total.group(1) if total else "?")


for label, path in LOGS:
    n, rows, moved, passed, total = parse(path)
    c = collections.Counter(s for _, _, s in rows)
    print(f"{label}: {n} cases, {len(rows)} compared field rows")
    print(f"    kernel verdict line: {passed} of {total} cases PASSED")
    print(f"    row statuses: {dict(c)}")
    for k in sorted(moved):
        idxs = sorted(moved[k])
        span = f"{idxs[0]}..{idxs[-1]}" if len(idxs) > 1 else str(idxs[0])
        print(f"    moved in {len(idxs):>3} of {n} case(s) [{span}]: {k}")
    print()
