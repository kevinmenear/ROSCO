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


def _rev(root: Path) -> str:
    """What revision `root` is at: measured if possible, claimed if not.

    Three reads, in decreasing order of what they can see, and each says which
    one it was so nobody has to guess:

      `<sha>` / `<sha>-dirty`  git ran; it can also see an edited tree
      `<sha>-nogit`            `.git/HEAD` read as text; the revision is right
                               and a local edit is invisible
      `<sha>-pinned`           a hand-written pin file; it can see nothing, and
                               this campaign has twice found it stale
      `unknown`                nothing to claim, said rather than invented
    """
    import subprocess
    try:
        r = subprocess.run(["git", "-C", str(root), "rev-parse", "--short", "HEAD"],
                           capture_output=True, text=True, timeout=10)
        if rev := r.stdout.strip():
            s = subprocess.run(["git", "-C", str(root), "status", "--porcelain"],
                               capture_output=True, text=True, timeout=10)
            return rev + ("-dirty" if s.stdout.strip() else "")
    except Exception:
        pass
    head = root / ".git" / "HEAD"
    if head.is_file():
        try:
            text = head.read_text().strip()
            if text.startswith("ref:"):
                p = root / ".git" / text.split(None, 1)[1].strip()
                if p.is_file():
                    return p.read_text().strip()[:7] + "-nogit"
            elif text:
                return text[:7] + "-nogit"
        except Exception:
            pass
    for pin in (root / ".loop_rev", root / ".vit_rev"):
        if pin.is_file() and pin.read_text().strip():
            return pin.read_text().strip() + "-pinned"
    return "unknown"


def main(argv: list[str]) -> int:
    # `--pre` stamps a PRE-integration run, which needs the red-test record and
    # nothing else: its `against` is already what vit_harness.py wrote, and
    # MEASURES is a sentence about the integrated build that would be false
    # here. Added at unit #4, where `harness.sh --red-test` in pre mode was
    # ACCEPTED AND DROPPED -- the pre path returns before the stamping block --
    # so a red artifact came back saying only `failed: 27` with nothing about
    # what was perturbed. harness.sh's own header says a red artifact that
    # cannot say what it measured is not evidence about the instrument.
    red, pre = None, False
    argv = list(argv)
    if "--pre" in argv:
        pre = True
        argv.remove("--pre")
    if len(argv) == 3 and argv[1] == "--red-test":
        red, argv = argv[2], argv[:1]
    if len(argv) != 1:
        print("usage: _harness_stamp.py <artifact.json> [--pre] "
              "[--red-test <what was perturbed>]", file=sys.stderr)
        return 2
    p = Path(argv[0])
    d = json.loads(p.read_text())
    if not pre:
        d["against"] = "integrated"
        d["measures"] = MEASURES
    # Both instruments, not one. `vit_harness.py` and `vit_mutate.py` stamp
    # loop_rev AND vit_rev; this artifact carried only loop_rev, so the one
    # measurement of the INTEGRATION WRAPPER -- the thing VIT generates -- was
    # the one that did not say which VIT generated it. That is exactly backwards.
    #
    # ONLY WHERE THE RUN DID NOT ALREADY SAY, and that is a correction made at
    # unit #8. This block used to overwrite unconditionally, from the two PIN
    # FILES -- which are hand-written, gitignored and stale by construction
    # (`vit` commit ae7a2d8 says so in its subject). So a run that had correctly
    # read `57c6fe3` had it replaced by a claimed `6d13949-pinned`, and the
    # campaign's red-test artifacts named an instrument two commits behind the
    # one that produced them. A measured value must never be clobbered by a
    # claimed one; this now fills a MISSING or `unknown` key and nothing else.
    #
    # And it runs on the MAC, where git exists -- so the pin is the last resort
    # here too, not the first. Third site of this same read, after
    # `vit_harness.py` and `vit_mutate.py`; not imported because those two are
    # deliberately standalone and this one is the campaign's, not the loop's.
    base = Path.home() / "Artifacts/vit_translation"
    for key, root in (("loop_rev", base / "translation-loop"),
                      ("vit_rev", base / "vit")):
        if d.get(key) and d[key] != "unknown":
            continue
        d[key] = _rev(root)
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
        where = "PRE-INTEGRATION" if pre else "POST-INTEGRATION"
        print(f"{where} RED TEST {'OK (went red)' if ok else 'FAILED (stayed green)'}: "
              f"checked {d['checked']}  failed {d['failed']}")
        return 0 if ok else 1

    if pre:
        # Nothing to add to a green pre-integration run: vit_harness.py already
        # stamped it. Rewriting it here would only risk changing it.
        return 0
    p.write_text(json.dumps(d, indent=1) + "\n")
    ok = d["failed"] == 0 and d["checked"] > 0
    print(f"POST-INTEGRATION {'PASS' if ok else 'FAIL'}: "
          f"checked {d['checked']}  failed {d['failed']}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
