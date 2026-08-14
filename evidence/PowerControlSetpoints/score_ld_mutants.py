#!/usr/bin/env python3
"""Score this unit's mutants against the SECOND oracle: gfortran's own
list-directed records.

WHY. The differential harness compares the unit's mapped signature, and no
signature carries a file, so every mutant inside `list_directed_real` and the
two helpers it calls survives it -- not because it preserves behaviour but
because the instrument cannot look at the output it changes. That is the shape
`scripts/dbgmutate.py` was written for one unit earlier (`Debug`, whose only
output is a `.RO.dbg` file), and this is the same argument on a smaller surface.

    python3 evidence/PowerControlSetpoints/score_ld_mutants.py
        [--only <mid> ...]   score just these mutant ids
        --out evidence/PowerControlSetpoints/ld_mutants.txt

THE GRADES ARE NOT AUTHORED HERE, in `_mutation_merge.py`'s sense: the mutant
set comes from `harness.cppmutate` -- the same function `vit_mutate.py` calls --
and the verdict is `run_ld_probe.sh`'s exit status, which is `mismatched == 0`
over 22,526 gfortran records. This file decides nothing; it applies a mutant and
reads a number.

A `nocompile` is NOT counted as a kill. `vit_mutate.py` counts one, and it is
right to: there a mutant that does not build is one the compiler rejected. Here
a non-building slice is far more likely to be the bind mount than the mutant, so
it is retried once and then reported as its own outcome, in its own column,
counted as neither.

KILLED means the probe reported a mismatch. SURVIVED means all 22,526 records
still agree, which for a mutant inside the formatter is a real equivalence claim
over a real domain and not a corpus accident -- the corpus is every decade of
the double range, both signs, the F/E boundary, the exponent-width boundary, the
non-finites, and 20,000 pseudorandom values.

NO BUILD, NO LINK, NO CONTAINER REBUILD. The probe slices the formatter out of
the translation and compiles it alone, so a mutant costs about a second rather
than the ten `vit_mutate.py` spends rebuilding the harness. That is the only
reason this is affordable at all.

IT RESTORES THE TRANSLATION UNCONDITIONALLY, and it checks the hash afterwards.
This edits the file `vit_mutate.py` mutates in place; the campaign has lost
three sessions to a mutant left in the tree.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LOOP = ROOT.parent / "translation-loop"
CPP = ROOT / "translations/ControllerBlocks/powercontrolsetpoints.cpp"
PROBE = ROOT / "evidence/PowerControlSetpoints/run_ld_probe.sh"

sys.path.insert(0, str(LOOP))
from harness.cppmutate import mutants  # noqa: E402


def loop_rev() -> str:
    """The generator revision, read the way the generated artifacts read it, so
    an oracle written here can be asked revcheck's question too. Falls back to
    the string the harness artifacts carry rather than to None: an artifact with
    no recorded input is worse than one with a disagreement (revcheck's (2))."""
    try:
        import json as _json
        from pathlib import Path as _P
        d = _json.loads((_P(ROOT) / "harness/PowerControlSetpoints.json").read_text())
        return d.get("loop_rev", "unknown")
    except Exception:
        return "unknown"


def hash_object(path: Path) -> str:
    return subprocess.run(["git", "-C", str(ROOT), "hash-object", str(path)],
                          capture_output=True, text=True, check=True).stdout.strip()


def run_probe() -> tuple[int, int]:
    """(values compared, mismatched). Raises if the probe could not run."""
    r = subprocess.run(["bash", str(PROBE)], capture_output=True, text=True)
    if r.returncode not in (0, 1):
        raise RuntimeError(f"probe exited {r.returncode}:\n{r.stdout}\n{r.stderr}")
    n = bad = -1
    for line in r.stdout.splitlines():
        if line.startswith("values compared"):
            n = int(line.split()[-1])
        elif line.startswith("mismatched"):
            bad = int(line.split()[-1])
    if n < 0 or bad < 0:
        raise RuntimeError(f"probe printed no counts:\n{r.stdout}")
    return n, bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=None,
                    help="mutant ids to score (default: every mutant whose "
                         "mutated line lies in the formatter)")
    ap.add_argument("--out", default="evidence/PowerControlSetpoints/ld_mutants.txt")
    args = ap.parse_args()

    original = CPP.read_text()
    before = hash_object(CPP)

    # `vit_mutate.py` calls `mutants(args.unit.lower(), ...)` and the unit string
    # goes into the mutant id, so the LOWERCASE name is what makes these ids the
    # same ids `mutation/PowerControlSetpoints.json` reports. Passing the
    # capitalised name produces a disjoint id space and every lookup misses.
    ms = mutants("powercontrolsetpoints", original)
    # The formatter is everything from `field(` to the line above the unit body.
    lo = original[:original.index("std::string field(const std::string& text, int w) {")].count("\n") + 1
    hi = original[:original.index("// The unit itself.")].count("\n") + 1
    picked = [m for m in ms
              if (args.only is not None and m.mid in args.only)
              or (args.only is None and lo <= m.line <= hi)]

    rows, killed = [], 0
    try:
        base_n, base_bad = run_probe()
        if base_bad != 0:
            print(f"BASELINE IS NOT GREEN: {base_bad} of {base_n} mismatched. "
                  f"Refusing to score -- every mutant would read as killed.",
                  file=sys.stderr)
            return 2
        for m in picked:
            CPP.write_text(m.source)
            # RETRIED ONCE, and the retry is not defensive programming -- it is
            # this campaign's standing finding about the bind mount. The probe
            # `cp`s the slice from the host-written tree INTO the container, and
            # a file written on the host is read half-written from inside often
            # enough that units #23 and #30 both paid for it. A half-written
            # slice does not compile, the probe prints no counts, and the mutant
            # is graded `nocompile` -- which this script would count as a KILL.
            # Measured here: 2 of 45 came back `nocompile` on the first pass and
            # BOTH compiled and ran on the second, one of them to 3,649
            # mismatches. A flake counted as a kill is a score that measured the
            # mount.
            try:
                try:
                    n, bad = run_probe()
                except RuntimeError:
                    CPP.write_text(m.source)
                    n, bad = run_probe()
            except RuntimeError as e:
                rows.append((m, "nocompile", 0, str(e).splitlines()[0]))
                continue
            finally:
                CPP.write_text(original)
            verdict = "killed" if bad else "survived"
            killed += bool(bad)
            rows.append((m, verdict, bad, ""))
    finally:
        CPP.write_text(original)

    after = hash_object(CPP)
    if after != before:
        print(f"TRANSLATION NOT RESTORED: {before} -> {after}", file=sys.stderr)
        return 3

    out = Path(ROOT / args.out)
    with out.open("w") as fh:
        fh.write("MUTANTS OF THE UNIT-401 FORMATTER, SCORED AGAINST gfortran's OWN RECORDS\n")
        fh.write("=" * 72 + "\n\n")
        fh.write(f"oracle      evidence/PowerControlSetpoints/run_ld_probe.sh\n")
        fh.write(f"corpus      {base_n} values regenerated from list_directed_corpus.f90\n")
        fh.write(f"baseline    {base_bad} mismatched -- green, so a red below is the mutant\n")
        fh.write(f"translation {before} before and after (restore proved)\n\n")
        fh.write(f"{'mutant':>10}  {'operator':<14} {'line':>5}  {'verdict':<9} {'mismatched':>10}\n")
        for m, verdict, bad, note in rows:
            fh.write(f"{m.mid:>10}  {m.operator:<14} {m.line:>5}  {verdict:<9} {bad:>10}"
                     f"{('  ' + note) if note else ''}\n")
        fh.write(f"\nkilled {killed} of {len(rows)}\n")
        for m, verdict, bad, _ in rows:
            if verdict == "survived":
                fh.write(f"\nSURVIVED {m.mid} ({m.operator}, line {m.line}):\n")
                fh.write("  " + (m.description if hasattr(m, "description") else "") + "\n")

    print(out.read_text())
    json.dump({"unit": "PowerControlSetpoints",
               "loop_rev": loop_rev(),
               "oracle": "evidence/PowerControlSetpoints/run_ld_probe.sh",
               "corpus_values": base_n,
               "scored": len(rows),
               "killed": killed,
               "verdicts": {m.mid: v for m, v, _, _ in rows}},
              open(ROOT / "evidence/PowerControlSetpoints/ld_mutants.json", "w"), indent=1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
