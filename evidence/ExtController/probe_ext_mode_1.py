#!/usr/bin/env python3
"""Does a runnable ORACLE exist for ExtController on this campaign's inputs?

P7 says the oracle is the original source. For every other unit in this
campaign that has been a question about *which* input to run the original on.
Here it is a question about whether the original can be run at all.

`Ext_Mode` is 0 in all 14 `Examples/*.IN`, `vit_sim.py` never patches it, and
`ExtControl.f90` measures 0/28 executable lines in all 27 scenarios. The obvious
next move is to turn the guard on and capture state from the original. This
measures what happens when you do.

The two facts it tests against, both read out of the source first:

  * `DLL_FileName` is the literal string `"unused"` in all 14 inputs, and no
    external Bladed-style library is shipped anywhere in the tree.
  * `ExtController` does NOT check `ErrVar%ErrStat` after `LoadDynamicLib`.
    On a failed `dlopen`, `DLL_Ext%ProcAddr(1)` stays `C_NULL_FUNPTR`, and the
    next two statements are `C_F_PROCPOINTER` on it and a CALL through it.

TWO PROCESSES ON PURPOSE. The first version put the snapshot/restore of
`Examples/DISCON.IN` in a `finally` inside the process that makes the call, and
that restore CANNOT RUN: the call dies on SIGSEGV, which does not unwind. The
run left `Ext_Mode = 1` in the gate's own input file, and because
`Examples/DISCON.IN` is GITIGNORED (.gitignore:78) `git status` stayed clean and
`done.py`'s clean-tree predicate could not have caught it -- the next gate run
would have measured a silently reconfigured controller. So the parent snapshots
and restores, the child crashes, and the restore happens either way. Same
lesson as `gate.py`'s `inputs_restored`, one failure mode further along.

    docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2 && \
        python3 evidence/ExtController/probe_ext_mode_1.py"

Writes its result as JSON on stdout. Exit status is the CHILD's, so a caller
can record the signal.
"""
import json
import os
import subprocess
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLES = os.path.abspath(os.path.join(HERE, "..", "..", "Examples"))
PARAM = os.path.join(EXAMPLES, "DISCON.IN")


def child() -> int:
    """Runs in its own process. Assumed to die; restores nothing."""
    sys.path.insert(0, EXAMPLES)
    os.chdir(EXAMPLES)
    import vit_sim

    turbine, controller, cp_filename = vit_sim.load_turbine_and_controller()
    vit_sim.write_discon(turbine, controller, cp_filename, PARAM,
                         patches={"Ext_Mode": 1})
    for line in open(PARAM):
        if "! Ext_Mode" in line or "! DLL_FileName" in line:
            print("CONFIG " + line.strip(), flush=True)

    # Constructing ControllerInterface performs the iStatus == 0 call into
    # DISCON, which is the call that reaches ExtController's init branch.
    vit_sim.ROSCO_ci.ControllerInterface(
        vit_sim.lib_name, param_filename=PARAM, sim_name="vit_probe_extcontroller")
    print("RETURNED the first call returned without terminating", flush=True)
    return 0


def parent() -> int:
    snapshot = open(PARAM, "rb").read() if os.path.isfile(PARAM) else None
    result = {"probe": "ExtController oracle availability"}
    try:
        proc = subprocess.run([sys.executable, __file__, "--child"],
                              capture_output=True, text=True)
        result["exit_status"] = proc.returncode
        if proc.returncode < 0:
            result["signal"] = -proc.returncode
        result["config"] = [l[len("CONFIG "):] for l in proc.stdout.splitlines()
                            if l.startswith("CONFIG ")]
        result["returned_normally"] = "RETURNED" in proc.stdout
        tail = [l for l in proc.stdout.splitlines() if l.strip()][-12:]
        result["stdout_tail"] = tail
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
