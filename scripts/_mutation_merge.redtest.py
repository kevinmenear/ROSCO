#!/usr/bin/env python3
"""Can `_mutation_merge.py`'s partition check FAIL? Six cases, five of them red.

WHY THIS EXISTS. Unit #53 relaxed "one operator, one part" to "one operator,
any number of parts whose `scored_ids` partition it". The relaxation is the
thing that lets an operator too large for one foreground call be scored at all,
and it is also the thing that would let a union count a mutant twice, or miss
one, and still write an artifact reading `score 0.9`. A weakened guard that
nobody made go red is indistinguishable from a removed one.

So each case below perturbs ONE property of a union that is otherwise green,
and asserts the specific refusal. The population is stubbed rather than asked
of `harness.cppmutate`, because the question here is the union arithmetic and
not the mutator: a real .cpp would make the test depend on a translation and on
docker, and the failure it is meant to catch lives in neither.

    python3 scripts/_mutation_merge.redtest.py

Exit 0 when every case behaved as stated, 1 otherwise. It writes only to a
temporary directory.
"""
from __future__ import annotations

import contextlib
import io
import json
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _mutation_merge as M  # noqa: E402

# Two operators: `alpha` with four mutants, `beta` with two. `alpha` is the one
# split across two parts, which is the shape the relaxation exists for.
POP = {"alpha": ["a1", "a2", "a3", "a4"], "beta": ["b1", "b2"]}


def part(path: Path, ops: list[str], ids: list[str], *,
         killed: int | None = None, survivors: list[str] | None = None,
         total: int | None = None, **over) -> str:
    survivors = survivors if survivors is not None else []
    n = len(ids) if total is None else total
    killed = (n - len(survivors)) if killed is None else killed
    doc = {
        "unit": "Alpha", "operators_filter": ops, "scored_ids": ids,
        "mutant_slice": {"offset": 0, "limit": 0, "scored": len(ids),
                         "of": len(ids)},
        "operators_offered": ["alpha", "beta"],
        "compared_against": "fortran_reference_on_a_clean_tree",
        "total": n, "mutants": n, "nocompile": 0, "killed": killed,
        "survived": len(survivors), "equivalent_declared": 0, "score": 1.0,
        "loop_rev": "aaaaaaa", "vit_rev": "bbbbbbb",
        "survivors": [{"id": s, "operator": ops[0], "before": "x",
                       "after": "y"} for s in survivors],
    }
    doc.update(over)
    path.write_text(json.dumps(doc))
    return str(path)


def run(parts: list[str], out: Path) -> tuple[int, str]:
    argv = ["_mutation_merge", "--unit", "Alpha", "--cpp", "x.cpp",
            "--out", str(out), "--why", "redtest"]
    for p in parts:
        argv += ["--part", p]
    old, err = sys.argv, io.StringIO()
    sys.argv = argv
    try:
        with contextlib.redirect_stderr(err), contextlib.redirect_stdout(io.StringIO()):
            rc = M.main()
    finally:
        sys.argv = old
    return rc, err.getvalue()


def main() -> int:
    M.population_from_cppmutate = lambda unit, cpp: POP  # the stub, stated
    tmp = Path(tempfile.mkdtemp(prefix="mmredtest-"))
    out = tmp / "merged.json"
    bad = 0

    def case(label: str, parts: list[str], want_rc: int, want: str) -> None:
        nonlocal bad
        rc, err = run(parts, out)
        ok = rc == want_rc and (want in err if want else True)
        print(f"[{'  ok  ' if ok else ' FAIL '}] {label}")
        if not ok:
            print(f"           rc={rc} (wanted {want_rc}); stderr={err.strip()[:200]}")
            bad += 1

    # 1. GREEN. `alpha` split across two parts, `beta` whole. The control: if
    #    this is not accepted, every red below is red for the wrong reason.
    g1 = part(tmp / "g1.json", ["alpha"], ["a1", "a2"])
    g2 = part(tmp / "g2.json", ["alpha"], ["a3", "a4"])
    g3 = part(tmp / "g3.json", ["beta"], ["b1", "b2"])
    case("GREEN  a split operator whose two slices partition it", [g1, g2, g3], 0, "")

    # 2. RED. The two slices overlap on a3 -- the hazard the old "one operator,
    #    one part" rule prevented by construction.
    o2 = part(tmp / "o2.json", ["alpha"], ["a2", "a3"])
    case("RED    slices overlap: a mutant scored by two parts",
         [g1, o2, part(tmp / "o3.json", ["alpha"], ["a4"]), g3], 2,
         "scored by both")

    # 3. RED. Together the slices miss a4. Counts alone cannot see this if the
    #    part totals are honest, which they are here.
    case("RED    slices miss a mutant of an operator they name",
         [g1, part(tmp / "m3.json", ["alpha"], ["a3"]), g3], 2, "in NO part")

    # 4. RED. An id that is not in the population at all. Coverage is COMPLETE
    #    here -- a1..a4 and b1,b2 are all scored -- so the "in NO part" check
    #    passes and this is the only thing left to catch it. A count check
    #    would see 7 scored against a population of 6 and could report a
    #    number; it could not say WHICH, and a part naming a mutant the mutator
    #    does not produce was scored against a different .cpp.
    case("RED    a scored id the mutator does not produce",
         [g1, part(tmp / "x4.json", ["alpha"], ["a3", "a4", "zz"]), g3], 2,
         "not in this unit's population")

    # 5. RED. The artifact disagrees with itself: 2 ids, total 3.
    case("RED    scored_ids and total disagree",
         [g1, part(tmp / "t5.json", ["alpha"], ["a3", "a4"], total=3,
                   killed=3), g3], 2, "does not agree with itself")

    # 6. RED. The old rule still governs parts with no ids: two parts sharing
    #    an operator, one of them pre-`scored_ids`, is refused.
    noid = json.loads(Path(g2).read_text())
    del noid["scored_ids"]
    (tmp / "n6.json").write_text(json.dumps(noid))
    case("RED    a shared operator where one part carries no scored_ids",
         [g1, str(tmp / "n6.json"), g3], 2, "carries no `scored_ids`")

    print(f"\n{6 - bad} of 6 cases behaved as stated")
    return 1 if bad else 0


if __name__ == "__main__":
    raise SystemExit(main())
