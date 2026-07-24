"""
vit_restart_test
----------------
Validation harness for ROSCO's checkpoint/restart path (avrSWAP status
iStatus == -8 / -9), which the 27-scenario simulation gate never exercises
(the standard driver only ever issues iStatus 0/1/-1). This closes the
behavioral-evidence gap for WriteRestartFile / ReadRestartFile flagged in
PROJECT_PLAN_ROSCO.md (§ Stage C, "Restart I/O is only triggered by
iStatus==-8/-9 ... Verified indirectly").

FINDING (2026-07-23): this harness FOUND, then verified the FIX for, a real bug in
the C++ restart. Under a clean apples-to-apples comparison — integrated pure-C++
vs modified-Fortran (reset_to_clean.sh), both provenance-verified — the BUGGY C++
restart diverged from Fortran by ~4.2e6 (6/13 arrays) while the continuous path was
bit-identical, i.e. WriteRestartFile/ReadRestartFile did not reproduce Fortran
restart. Root cause: discon.cpp reset the objInst filter/limiter instance counters
BEFORE the iStatus==-9 ReadRestartFile, so the restored (checkpointed, high) counter
values clobbered the reset and the controllers indexed the wrong FP/piP/rlP state
slots on restart. Fortran resets them in SetParameters, AFTER ReadRestartFile.
Fix: move the objInst reset after the -9 restore in discon.cpp. See dev note
202607232319. A SILENT bug — the 27-scenario gate never issues iStatus -8/-9.

(An even earlier manual run had reported C++ restart == Fortran at 0.0; that was
WRONG — it compared against an ambiguous/stale libdiscon.so, because ROSCO's real
C++ DISCON entry lives only on the `integrated` branch while `main`'s discon.cpp is
a stub. Always verify binary provenance before trusting a restart pass.)

This harness GATES on Test B: the current build's continuous and restart traces must
be bit-identical to the committed reference in restart_baseline/ (generate with
--make-reference against the MODIFIED-Fortran build, scripts/reset_to_clean.sh).
With the fix, Test B PASSES (both paths bit-identical). Test A (roundtrip) is
informational and differs in both languages.

Mechanism (confirmed against source):
  * iStatus == -8  -> DISCON writes a checkpoint (WriteRestartFile), tagged with
    NINT(Time/DT), to <RootName><tag>.RO.chkp   (ROSCO_IO.f90:29-30).
  * iStatus == -9  -> DISCON treats the call as a first call, reads the checkpoint
    tagged NINT(avrSWAP_t / avrSWAP_dt) (ROSCO_IO.f90:375-376), then — because
    DISCON.F90:99 admits iStatus <= -8 into the control block — computes control
    for that step. So the -9 call IS the resumed step, not a bare state load.
  * RootName == avcOUTNAME == the ControllerInterface sim_name, so pointing
    sim_name at restart_test_scratch/ keeps every .RO.chkp out of the tree.

Test A — roundtrip self-consistency (INFORMATIONAL only; not the gate — it
differs in both languages because restart is checkpoint-restore + re-init):

  1. CONTINUOUS run: init(0), step 1..N with iStatus=1, and at step K issue
     iStatus=-8 (writes the checkpoint; control output is unchanged because the
     -8 call still runs the control block). Record outputs[0..N].
  2. RESTART run: a fresh controller whose FIRST call is iStatus=-9 at t_K
     (reads the checkpoint and computes step K), seeded with the continuous
     run's plant state at K-1, then step K+1..N with iStatus=1. Record
     outputs[K..N].
  3. Assert continuous[K..N] == restart[K..N], per array. Any divergence means a
     state field was dropped or written out of order relative to the read — the
     exact silent-corruption risk PROJECT_PLAN_ROSCO.md warns about.

Run INSIDE the ROSCO container, against the built libdiscon.so under test
(the branch's C++ build, or the Fortran baseline build to validate the harness):

    python3 Examples/vit_restart_test.py --checkpoint-frac 0.5

Every artifact (DISCON.IN copy, *.RO.chkp, *.npz, diff report) is written under
restart_test_scratch/. Nothing here touches the 27-scenario baselines.
"""

import argparse
import os
from ctypes import (cdll, POINTER, c_float, c_int32, c_char_p,
                    create_string_buffer)

import numpy as np

# Reuse the deterministic turbine/controller loader, the DISCON writer, the
# extra-avrSWAP index map, and — importantly — the FITPACK stack-scrubber
# monkey-patch, all applied at import of vit_sim. Importing does not run its
# main() (guarded by __name__ == '__main__').
from vit_sim import load_turbine_and_controller, write_discon, EXTRA_AVRSWAP

from rosco import discon_lib_path as LIB_NAME
from rosco.toolbox import control_interface as ROSCO_ci

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(THIS_DIR)
SCRATCH = os.path.join(REPO_ROOT, 'restart_test_scratch')
os.makedirs(SCRATCH, exist_ok=True)

# Committed, platform-specific Fortran reference traces (the Test B gate compares
# this build against them). Generate/refresh with --make-reference run against the
# modified-Fortran build; see scripts/reset_to_clean.sh.
BASELINE_DIR = os.path.join(THIS_DIR, 'restart_baseline')

RPM2RAD = np.pi / 30.0


class RestartControllerInterface(ROSCO_ci.ControllerInterface):
    """A ControllerInterface whose FIRST DLL call is iStatus=-9 (restart read)
    rather than iStatus=0 (fresh init). This mirrors how OpenFAST resumes a run:
    the very first controller call after a restart is -9, never 0-then-(-9).

    We override init_discon to perform all of the base setup EXCEPT the initial
    iStatus=0 call; the -9 call is then issued explicitly via restart_first_call.
    """

    def init_discon(self):
        # --- mirrors ControllerInterface.init_discon() up to, but NOT including,
        #     the first call_discon() (base lines 92-147) ---
        self.torque = 0
        self.discon = cdll.LoadLibrary(self.lib_name)
        self.avrSWAP = np.zeros(self.avr_size)

        self.avrSWAP[2] = self.DT
        self.avrSWAP[60] = self.num_blade
        self.avrSWAP[19] = 1.0
        self.avrSWAP[20] = 1.0
        self.avrSWAP[82] = 0
        self.avrSWAP[26] = 10
        self.avrSWAP[3] = self.pitch * np.deg2rad(1)
        self.avrSWAP[32] = self.pitch * np.deg2rad(1)
        self.avrSWAP[33] = self.pitch * np.deg2rad(1)
        self.avrSWAP[27] = 1
        self.avrSWAP[22] = 0

        self.aviFAIL = c_int32()
        self.accINFILE = self.param_name.encode("utf-8")
        self.avcOUTNAME = (self.sim_name).encode("utf-8")
        self.avcMSG = create_string_buffer(1000)
        self.discon.DISCON.argtypes = [
            POINTER(c_float), POINTER(c_int32), c_char_p, c_char_p, c_char_p,
        ]
        self.avrSWAP[48] = self.char_buffer
        self.avrSWAP[49] = len(self.param_name)
        self.avrSWAP[50] = len(self.avcOUTNAME)
        self.avrSWAP[51] = self.char_buffer
        # Deliberately NO first call here — see restart_first_call().

    def restart_first_call(self, turbine_state):
        """Issue the resumed step as an iStatus=-9 call: DISCON reads the
        checkpoint (tagged NINT(t/dt)) and computes control for this step."""
        ts = dict(turbine_state)
        ts["iStatus"] = -9
        return self.call_controller(ts)


def _turbine_state(i, t, dt, ws, bld_pitch, gen_torque, gen_speed, rot_speed,
                   turbine, istatus):
    """Assemble the per-step turbine_state dict (mirrors vit_sim manual loops)."""
    return {
        'iStatus': istatus,
        't': t[i], 'dt': dt, 'ws': ws[i],
        'bld_pitch': bld_pitch[i - 1],
        'gen_torque': gen_torque[i - 1],
        'gen_speed': gen_speed[i],
        'gen_eff': turbine.GenEff / 100,
        'rot_speed': rot_speed[i],
        'Yaw_fromNorth': 0.0, 'Y_MeasErr': 0.0,
    }


def _advance_plant(i, dt, ws, bld_pitch, gen_torque, rot_speed, gen_speed,
                   turbine, GBRatio, R):
    """1-DOF drivetrain step (same model as vit_sim scenario 2/7)."""
    tsr = rot_speed[i - 1] * R / ws[i]
    cp = turbine.Cp.interp_surface(bld_pitch[i - 1], tsr)
    aero_torque = 0.5 * turbine.rho * (np.pi * R ** 3) * (cp / tsr) * ws[i] ** 2
    rot_speed[i] = rot_speed[i - 1] + (dt / turbine.J) * (
        aero_torque - GBRatio * gen_torque[i - 1] / (turbine.GBoxEff / 100))
    gen_speed[i] = rot_speed[i] * GBRatio


def simulate(controller_int, turbine, t, ws, dt, GBRatio, R,
             start, ic, checkpoint_step=None, restart=False):
    """Run the manual 1-DOF loop from `start`..N.

    ic = (rot_speed[start-1], gen_torque[start-1], bld_pitch[start-1]) seeds the
    plant so a restart run resumes from the continuous run's state at start-1.
    If checkpoint_step is set, that step is issued with iStatus=-8 (write).
    If restart is True, the FIRST step (== start) is the -9 resumed call.
    """
    n = len(t)
    rot_speed = np.zeros(n)
    gen_speed = np.zeros(n)
    gen_torque = np.zeros(n)
    bld_pitch = np.zeros(n)
    nac_yaw = np.zeros(n)
    gen_power = np.zeros(n)
    extra = {name: np.zeros(n) for name in EXTRA_AVRSWAP}

    rs0, gt0, bp0 = ic
    rot_speed[start - 1] = rs0
    gen_speed[start - 1] = rs0 * GBRatio
    gen_torque[start - 1] = gt0
    bld_pitch[start - 1] = bp0

    for i in range(start, n):
        _advance_plant(i, dt, ws, bld_pitch, gen_torque, rot_speed, gen_speed,
                       turbine, GBRatio, R)

        is_last = (i == n - 1)
        istatus = -1 if is_last else 1
        if checkpoint_step is not None and i == checkpoint_step:
            istatus = -8  # write checkpoint (still runs the control block)

        ts = _turbine_state(i, t, dt, ws, bld_pitch, gen_torque, gen_speed,
                            rot_speed, turbine, istatus)

        if restart and i == start:
            gen_torque[i], bld_pitch[i], nac_yaw[i] = \
                controller_int.restart_first_call(ts)
        else:
            gen_torque[i], bld_pitch[i], nac_yaw[i] = \
                controller_int.call_controller(ts)

        gen_power[i] = gen_torque[i] * gen_speed[i] * (turbine.GenEff / 100)
        for name, idx in EXTRA_AVRSWAP.items():
            extra[name][i] = controller_int.avrSWAP[idx]

    controller_int.kill_discon()
    out = {'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
           'gen_speed': gen_speed, 'gen_power': gen_power, 'nac_yaw': nac_yaw}
    out.update(extra)
    return out


def compare_traces(cont, restarted, k, n):
    """Exact per-array comparison over the overlap window [k, n)."""
    report, all_ok = [], True
    for name in sorted(cont):
        a = cont[name][k:n]
        b = restarted[name][k:n]
        ok = np.array_equal(a, b)
        if not ok:
            diff = np.abs(a - b)
            j = int(np.argmax(diff))
            report.append(f"  FAIL {name}: {int((diff != 0).sum())}/{len(a)} differ, "
                          f"max |Δ|={diff[j]:.3e} at step {k + j}")
            all_ok = False
        else:
            report.append(f"  PASS {name}: {len(a)} values identical")
    return all_ok, report


def _run_current_build(args):
    """Run the currently-loaded libdiscon.so through continuous + restart, return
    (cont, restarted, n, K)."""
    turbine, controller, cp_filename = load_turbine_and_controller()
    GBRatio = turbine.Ng
    R = turbine.rotor_radius

    param_filename = os.path.join(SCRATCH, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename)

    dt = args.dt
    t = np.arange(0, args.tlen, dt)
    ws = np.ones_like(t) * args.ws
    n = len(t)
    K = max(2, int(n * args.checkpoint_frac))
    print(f"Restart test: N={n} steps, dt={dt}s, checkpoint at step K={K} "
          f"(t={t[K]:.3f}s), wind={args.ws} m/s")

    root = os.path.join(SCRATCH, 'vit_restart')
    init_ic = (4.0 * RPM2RAD, 0.0, 0.0)  # rot_speed, gen_torque, bld_pitch at step 0

    # Continuous run (writes checkpoint at K via iStatus=-8).
    ci = ROSCO_ci.ControllerInterface(LIB_NAME, param_filename=param_filename,
                                      sim_name=root, DT=dt)
    cont = simulate(ci, turbine, t, ws, dt, GBRatio, R,
                    start=1, ic=init_ic, checkpoint_step=K)
    # Restart run (first call iStatus=-9 at t_K, seeded with continuous state at K-1).
    ci_r = RestartControllerInterface(LIB_NAME, param_filename=param_filename,
                                      sim_name=root, DT=dt)
    restart_ic = (cont['gen_speed'][K - 1] / GBRatio,
                  cont['gen_torque'][K - 1], cont['bld_pitch'][K - 1])
    restarted = simulate(ci_r, turbine, t, ws, dt, GBRatio, R,
                         start=K, ic=restart_ic, restart=True)
    np.savez(os.path.join(SCRATCH, 'continuous.npz'), **cont)
    np.savez(os.path.join(SCRATCH, 'restarted.npz'), **restarted)
    return cont, restarted, n, K


def main():
    ap = argparse.ArgumentParser(
        description="ROSCO checkpoint/restart validation — Test B gate: this build's "
                    "restart output must be bit-identical to the committed Fortran reference.")
    # Defaults MUST match the committed reference in restart_baseline/.
    ap.add_argument('--tlen', type=float, default=60.0, help='sim length (s)')
    ap.add_argument('--dt', type=float, default=0.025, help='timestep (s)')
    ap.add_argument('--checkpoint-frac', type=float, default=0.5,
                    help='fraction of the run at which to checkpoint/restart')
    ap.add_argument('--ws', type=float, default=11.0, help='wind speed (m/s)')
    ap.add_argument('--make-reference', action='store_true',
                    help='write this run as the committed Fortran reference — run ONLY '
                         'against a modified-Fortran build (scripts/reset_to_clean.sh)')
    args = ap.parse_args()

    cont, restarted, n, K = _run_current_build(args)

    if args.make_reference:
        os.makedirs(BASELINE_DIR, exist_ok=True)
        np.savez(os.path.join(BASELINE_DIR, 'continuous_fortran.npz'), **cont)
        np.savez(os.path.join(BASELINE_DIR, 'restarted_fortran.npz'), **restarted)
        print(f"\nWrote Fortran reference to {BASELINE_DIR}. Commit ONLY if generated "
              f"against the modified-Fortran build.")
        return 0

    # Test A (informational, NOT the gate): roundtrip self-consistency. This is EXPECTED to
    # differ — ROSCO restart is checkpoint-restore + SetParameters re-init, so a restarted run
    # does not reproduce the uninterrupted run bit-for-bit, identically in Fortran and C++.
    okA, _ = compare_traces(cont, restarted, K, n)
    print(f"\n[Test A] roundtrip self-consistency (informational): "
          f"{'identical' if okA else 'differs, as expected in both languages'}")

    # Test B (THE GATE): this build's traces vs the committed Fortran reference. The checkpoint
    # file formats differ across languages by design, so we compare controller OUTPUTS, not the
    # .chkp bytes: a faithful C++ restart reproduces the Fortran restart's outputs exactly.
    cf = os.path.join(BASELINE_DIR, 'continuous_fortran.npz')
    rf = os.path.join(BASELINE_DIR, 'restarted_fortran.npz')
    if not (os.path.exists(cf) and os.path.exists(rf)):
        print(f"\nNo Fortran reference in {BASELINE_DIR}. Generate it with --make-reference "
              f"against a Fortran build, then commit it.")
        return 2
    ref_cont, ref_restart = np.load(cf), np.load(rf)
    okc, rc = compare_traces(cont, ref_cont, 1, n)          # continuous path
    okr, rr = compare_traces(restarted, ref_restart, K, n)  # restart path
    print("\n[Test B] continuous vs Fortran reference:")
    print("\n".join(rc))
    print("[Test B] restart vs Fortran reference:")
    print("\n".join(rr))
    gate = okc and okr
    print(f"\nRESTART GATE (Test B — C++ restart ≡ Fortran restart): "
          f"{'PASS' if gate else 'FAIL'} ({n - 1} continuous + {n - K} restart steps)")
    return 0 if gate else 1


if __name__ == '__main__':
    raise SystemExit(main())
