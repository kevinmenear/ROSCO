#!/usr/bin/env python3
"""Red test for scripts/revcheck.py -- construct each defect, and ONLY it.

Seven cases. Three build the failures revcheck exists to find; one builds the
FALSE POSITIVE it exists not to raise (`2e2295f-nogit` and `2e2295f` are one
commit, and a hand comparison that compared the suffixed strings reported a
split that was not there); one is NOT_EVALUABLE, which is never a pass; and two
check that a finding on an already-closed unit is advisory by default and
counts under --strict.

Each red case asserts the OTHER two codes are absent, because a check that fires
on everything distinguishes nothing. Runs in a scratch tree under mktemp, never
against this campaign's own artifacts.

    python3 scripts/revcheck.redtest.py     # exit 0 = all seven as expected
"""
from __future__ import annotations

import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SRC = Path(__file__).resolve().parent / "revcheck.py"
OPEN_U, CLOSED_U = "OpenUnit", "ClosedUnit"


def build(tmp: Path, arts: dict[str, str | None], closed: dict[str, str | None] | None = None):
    for d in ("scripts", "mutation", "harness", "gate"):
        (tmp / d).mkdir(parents=True, exist_ok=True)
    shutil.copy(SRC, tmp / "scripts" / "revcheck.py")
    (tmp / "plan.json").write_text(json.dumps({"units": [
        {"name": OPEN_U, "order": 1, "disposition": None},
        {"name": CLOSED_U, "order": 2, "disposition": "integrated"},
    ]}))
    for rel, rev in list(arts.items()) + list((closed or {}).items()):
        doc = {"checked": 10, "failed": 0}
        if rev is not None:
            doc["loop_rev"] = rev
        (tmp / rel).write_text(json.dumps(doc))


def run(tmp: Path, *args) -> tuple[int, str]:
    r = subprocess.run([sys.executable, str(tmp / "scripts" / "revcheck.py"), *args],
                       capture_output=True, text=True)
    return r.returncode, r.stdout + r.stderr


FAIL = 0


def ck(name, got, want):
    global FAIL
    ok = got == want
    print(f"  {'ok  ' if ok else 'FAIL'} {name}: {got!r}" + ("" if ok else f" want {want!r}"))
    if not ok:
        FAIL = 1


def case(title, arts, *, exit_want, codes_want, closed=None, extra_args=()):
    print(f"\n{title}")
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        build(tmp, arts, closed)
        rc, out = run(tmp, "--unit", OPEN_U, *extra_args)
        ck("exit", rc, exit_want)
        for code in ("BASE-SHA SPLIT", "NO loop_rev", "DIRTY TREE"):
            ck(f"reports {code}", code in out, code in codes_want)


GOOD = {"mutation/OpenUnit.json": "aaaaaaa", "harness/OpenUnit.json": "aaaaaaa",
        "gate/OpenUnit.json": "aaaaaaa"}

case("1 GREEN  every artifact one clean rev", GOOD, exit_want=0, codes_want=())

case("2 RED    one artifact at a different base SHA",
     {**GOOD, "harness/OpenUnit.postintegration.json": "bbbbbbb"},
     exit_want=2, codes_want=("BASE-SHA SPLIT",))

case("3 RED    one artifact with no loop_rev",
     {**GOOD, "harness/OpenUnit.postintegration.json": None},
     exit_want=2, codes_want=("NO loop_rev",))

case("4 RED    one artifact taken at a dirty tree",
     {**GOOD, "harness/OpenUnit.postintegration.json": "aaaaaaa-dirty"},
     exit_want=2, codes_want=("DIRTY TREE",))

case("5 GREEN  -nogit and bare are ONE commit, not a split",
     {"mutation/OpenUnit.json": "aaaaaaa-nogit", "harness/OpenUnit.json": "aaaaaaa",
      "gate/OpenUnit.json": "aaaaaaa-nogit"},
     exit_want=0, codes_want=())

case("6 n/e    no result artifact is NOT_EVALUABLE, never a pass",
     {}, exit_want=3, codes_want=())

print("\n7 ADVISORY  a finding on a CLOSED unit")
with tempfile.TemporaryDirectory() as t:
    tmp = Path(t)
    build(tmp, GOOD, {"mutation/ClosedUnit.json": "ccccccc",
                      "harness/ClosedUnit.json": "ddddddd"})
    rc, out = run(tmp)
    ck("audit exit (advisory)", rc, 0)
    ck("names it advisory", "ADVISORY" in out, True)
    rc2, _ = run(tmp, "--strict")
    ck("--strict exit", rc2, 2)

print("\nrevcheck.redtest: all seven as expected" if not FAIL else
      "\nrevcheck.redtest: FAILURES ABOVE")
raise SystemExit(FAIL)
