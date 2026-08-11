#!/usr/bin/env python3
"""The same probe as `probe_ext_mode_1.py`, with the FIXTURE the tree was missing.

`probe_ext_mode_1.py` measured that forcing `Ext_Mode = 1` makes the original
Fortran die on SIGSEGV, and unit #5 read that as "there is no runnable oracle for
ExtController anywhere in this campaign". The measurement was right. The reading
was one step too far, and this file is the step back.

The crash is not a property of `ExtController`. It is a property of the INPUT:
`DLL_FileName` is the literal string `"unused"` in all 14 Examples inputs, no
external Bladed-style library was shipped anywhere in the tree, `dlopen` fails,
`ExtController` does not check `ErrVar%ErrStat`, and the CALL through the
still-null `ProcAddr(1)` two statements later is the signal. Supply a library
that loads and exports `DISCON`, and every one of those steps succeeds.

So this probe changes exactly one thing from its predecessor: it also patches
`DLL_FileName` to `fixtures/bladed_stub/libdiscon_stub.so`. It is an ADDITION
next to the original probe rather than an edit of it (P5) -- the SIGSEGV result
is still true of the campaign's own inputs and its artifact still stands. What
this one measures is a different question: whether an oracle CAN be constructed,
not whether one already existed.

The two artifacts are meant to be read together:

    probe_ext_mode_1.json              Ext_Mode=1, DLL_FileName="unused"   -> signal 11
    probe_ext_mode_1_with_oracle.json  Ext_Mode=1, DLL_FileName=the stub   -> ?

TWO PROCESSES, for the reason the first probe records: a restore that must
survive a crash belongs in the parent. This run is expected NOT to crash, which
is exactly when a `finally` looks like it would have been fine -- and the run
that proves it is the one where it is not. `Examples/DISCON.IN` is gitignored,
so nothing downstream would catch a botched restore.

    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
        bash fixtures/bladed_stub/build.sh && \
        python3 evidence/ExtController/probe_ext_mode_1_with_oracle.py"

Writes its result as JSON on stdout.
"""
import json
import os
import subprocess
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
EXAMPLES = os.path.join(ROOT, "Examples")
PARAM = os.path.join(EXAMPLES, "DISCON.IN")
STUB = os.path.join(ROOT, "fixtures", "bladed_stub", "libdiscon_stub.so")


def child() -> int:
    """Runs in its own process. Restores nothing -- the parent owns that."""
    if not os.path.isfile(STUB):
        print("MISSING the stub library does not exist; run fixtures/bladed_stub/build.sh",
              flush=True)
        return 2

    sys.path.insert(0, EXAMPLES)
    os.chdir(EXAMPLES)
    import vit_sim

    turbine, controller, cp_filename = vit_sim.load_turbine_and_controller()
    # The quotes matter: `ParseInput` reads DLL_FileName as a quoted string, and
    # write_discon's patcher substitutes the value verbatim.
    vit_sim.write_discon(turbine, controller, cp_filename, PARAM,
                         patches={"Ext_Mode": 1, "DLL_FileName": f'"{STUB}"'})
    for line in open(PARAM):
        if ("! Ext_Mode" in line or "! DLL_FileName" in line
                or "! DLL_ProcName" in line):
            print("CONFIG " + line.strip(), flush=True)

    # Constructing ControllerInterface performs the iStatus == 0 call into
    # DISCON, which is the call that reaches ExtController's init branch --
    # LoadDynamicLib, the ALLOCATE, and the call through ProcAddr(1).
    vit_sim.ROSCO_ci.ControllerInterface(
        vit_sim.lib_name, param_filename=PARAM,
        sim_name="vit_probe_extcontroller_oracle")
    print("RETURNED the first call returned without terminating", flush=True)
    return 0


def parent() -> int:
    snapshot = open(PARAM, "rb").read() if os.path.isfile(PARAM) else None
    result = {"probe": "ExtController oracle availability, WITH the stub fixture",
              "stub": os.path.relpath(STUB, ROOT),
              "stub_exists": os.path.isfile(STUB)}
    try:
        proc = subprocess.run([sys.executable, __file__, "--child"],
                              capture_output=True, text=True)
        result["exit_status"] = proc.returncode
        if proc.returncode < 0:
            result["signal"] = -proc.returncode
        result["config"] = [l[len("CONFIG "):] for l in proc.stdout.splitlines()
                            if l.startswith("CONFIG ")]
        result["returned_normally"] = "RETURNED" in proc.stdout
        result["stdout_tail"] = [l for l in proc.stdout.splitlines() if l.strip()][-12:]
        result["stderr_tail"] = [l for l in proc.stderr.splitlines() if l.strip()][-6:]
    except BaseException:
        result["error"] = traceback.format_exc()
    finally:
        restored = False
        if snapshot is not None and open(PARAM, "rb").read() != snapshot:
            with open(PARAM, "wb") as f:
                f.write(snapshot)
            restored = True
        result["discon_in_restored_by_parent"] = restored
    print(json.dumps(result, indent=1))
    return 0


if __name__ == "__main__":
    if "--child" in sys.argv:
        raise SystemExit(child())
    raise SystemExit(parent())
