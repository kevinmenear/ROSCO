#!/usr/bin/env python3
"""Stamp a post-integration harness artifact with what it measured. E4.5.

Called by `scripts/harness.sh --post-integration`. Separate file rather than an
inline heredoc so the shell script stays readable and this stays testable.

The `measures` field is the point. A post-integration harness artifact that says
only `checked 217 failed 0` reads exactly like the pre-integration one, which
compared against real Fortran -- and it is not that. After integration the
Fortran body IS the translation, so both sides of this comparison run the same
arithmetic and only the wrapper between them is under test.
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

MEASURES = (
    "the integration wrapper's marshalling ONLY. After integration the Fortran "
    "body IS the translation, so no harness against this build can compare "
    "arithmetic; both sides run the same code, and a failure means the wrapper "
    "corrupted an argument or an output on the way through. The arithmetic is "
    "covered by the pre-integration harness, the kernel and the gate."
)


def main(argv: list[str]) -> int:
    red = None
    if len(argv) == 3 and argv[1] == "--red-test":
        red, argv = argv[2], argv[:1]
    if len(argv) != 1:
        print("usage: _harness_stamp.py <artifact.json> [--red-test <what was perturbed>]",
              file=sys.stderr)
        return 2
    p = Path(argv[0])
    d = json.loads(p.read_text())
    d["against"] = "integrated"
    d["measures"] = MEASURES
    # Both instruments, not one. `vit_harness.py` and `vit_mutate.py` stamp
    # loop_rev AND vit_rev; this artifact carried only loop_rev, so the one
    # measurement of the INTEGRATION WRAPPER -- the thing VIT generates -- was
    # the one that did not say which VIT generated it. That is exactly backwards.
    # Read as pins because neither repo has git inside the container that
    # produced the run, and never dressed up as a verified read.
    base = Path.home() / "Artifacts/vit_translation"
    for key, pin in (("loop_rev", base / "translation-loop/.loop_rev"),
                     ("vit_rev", base / "vit/.vit_rev")):
        if pin.is_file() and pin.read_text().strip():
            d[key] = pin.read_text().strip() + "-pinned"
    if red is not None:
        # A red run INVERTS the verdict: failing is the result, and passing is
        # the finding -- a harness that stays green under a deliberate
        # perturbation cannot be told from one that cannot fail.
        d["red_test"] = {
            "perturbation": red,
            "result": f"{d['failed']} of {d['checked']} case(s) failed",
            "why": "a harness whose green was never observed going red cannot be "
                   "told from one that cannot fail",
        }
        p.write_text(json.dumps(d, indent=1) + "\n")
        ok = d["failed"] > 0 and d["checked"] > 0
        print(f"POST-INTEGRATION RED TEST {'OK (went red)' if ok else 'FAILED (stayed green)'}: "
              f"checked {d['checked']}  failed {d['failed']}")
        return 0 if ok else 1

    p.write_text(json.dumps(d, indent=1) + "\n")
    ok = d["failed"] == 0 and d["checked"] > 0
    print(f"POST-INTEGRATION {'PASS' if ok else 'FAIL'}: "
          f"checked {d['checked']}  failed {d['failed']}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
