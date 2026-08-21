#!/usr/bin/env python3
"""Print the case index either side of each call, so a SIGSEGV names a case.

    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
        python3 evidence/WindSpeedEstimator/probe_case_trace.py --on"
    ... make test && ./test <stem>_cases.bin 2> /tmp/probe.err ...
    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
        python3 evidence/WindSpeedEstimator/probe_case_trace.py --off"

Unit #45's shape: `vit_harness.py` prints its JSON payload AFTER the loop, so a
crash inside the loop produces an empty stdout and the run reports `harness
produced no JSON` -- which reads as a build or an emitter problem and is
neither. Two `fprintf`s to STDERR turn it into a case index and a side.

STDERR, not stdout: `vit_mutate.py` and `vit_harness.py` both read a JSON
payload out of `./test`'s stdout, precisely because the reference may PRINT.

`--off` restores the file from the copy `--on` made, which is unit #47's rule:
a later `--no-generate` keeps the test source, and a mutation sweep over an
instrumented one would print two lines per case per mutant.
"""
import shutil
import sys
from pathlib import Path

ROOT = Path("/workspace/ROSCO-r2")
if not ROOT.is_dir():
    ROOT = Path(__file__).resolve().parents[2]

TEST = ROOT / "translations/ControllerBlocks/windspeedestimator_test/windspeedestimator_test.cpp"
KEEP = TEST.with_suffix(".cpp.pretrace")

CPP_CALL = "        WindSpeedEstimator(&LocalVar_a,"
F90_CALL = "        windspeedestimator_f90(&LocalVar_b.iStatus"


def on() -> None:
    text = TEST.read_text()
    if "CASE %d CPP" in text:
        print("probe_case_trace: already instrumented")
        return
    shutil.copyfile(TEST, KEEP)
    for anchor, side in ((CPP_CALL, "CPP"), (F90_CALL, "F90")):
        if anchor not in text:
            raise SystemExit(f"probe_case_trace: anchor not found: {anchor!r}")
        line = f'        std::fprintf(stderr, "CASE %d {side}\\n", c);\n'
        text = text.replace(anchor, line + anchor, 1)
    TEST.write_text(text)
    print(f"probe_case_trace: instrumented; original kept at {KEEP.name}")


def off() -> None:
    if not KEEP.is_file():
        raise SystemExit(f"probe_case_trace: no {KEEP.name} to restore from")
    shutil.copyfile(KEEP, TEST)
    KEEP.unlink()
    print("probe_case_trace: restored")


if __name__ == "__main__":
    if "--on" in sys.argv:
        on()
    elif "--off" in sys.argv:
        off()
    else:
        raise SystemExit(__doc__)
