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
    if len(argv) != 1:
        print("usage: _harness_stamp.py <artifact.json>", file=sys.stderr)
        return 2
    p = Path(argv[0])
    d = json.loads(p.read_text())
    d["against"] = "integrated"
    d["measures"] = MEASURES
    pin = Path.home() / "Artifacts/vit_translation/translation-loop/.loop_rev"
    if pin.is_file():
        d["loop_rev"] = pin.read_text().strip() + "-pinned"
    p.write_text(json.dumps(d, indent=1) + "\n")
    ok = d["failed"] == 0 and d["checked"] > 0
    print(f"POST-INTEGRATION {'PASS' if ok else 'FAIL'}: "
          f"checked {d['checked']}  failed {d['failed']}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
