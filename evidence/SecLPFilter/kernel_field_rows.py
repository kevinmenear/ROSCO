#!/usr/bin/env python3
"""Regenerate kernel_field_rows.txt from the three committed kernel run logs.

`kernel/` is untracked and reset_to_clean.sh removes it, so VIT's
verify_fields.csv does not survive the cycle (RUNBOOK, unit #17). The kernel's
own stdout does, and it carries the same per-case per-field verdicts.
"""
import re, collections, sys

def parse(path):
    txt = open(path, errors='replace').read()
    cases = txt.split("***************** Verification against")
    rows = []
    for c in cases[1:]:
        name = re.match(r"\s*'([^']+)'", c).group(1)
        for mm in re.finditer(r'^\s*(\S+) is (IDENTICAL|NOT IDENTICAL)', c, re.M):
            rows.append((name, mm.group(1), mm.group(2)))
    return len(cases) - 1, rows

for label, path in (("real translation", "kernel.verification-PASSES-62of62.run.txt"),
                    ("zero stub", "kernel.zero-stub-FAILS.run.txt"),
                    ("hardcoded-argument stub", "kernel.hardcoded-arguments-stub-PASSES.run.txt")):
    n, rows = parse(path)
    c = collections.Counter(s for _, _, s in rows)
    moved = collections.Counter(f for _, f, s in rows if s != 'IDENTICAL')
    print(f"{label}: {n} cases, {len(rows)} compared field rows, {dict(c)}")
    for k, v in sorted(moved.items()):
        print(f"    moved in {v:>3} of {n} case(s): {k}")
    print()
