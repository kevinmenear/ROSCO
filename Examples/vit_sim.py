"""
vit_sim
-------
VIT-specific simulation script for kernel extraction and baseline capture.

Runs multiple simulation scenarios to exercise all controller code paths needed
for VIT translation verification:

  1. Standard 1-DOF step-wind simulation (same as 04_simple_sim.py)
     - Exercises: saturate, wrap_180, interp1d, LPFilter, HPFilter, SecLPFilter
  2. Yaw-by-IPC simulation with Y_ControlMode=2
     - Exercises: wrap_360 (via synthetic NacHeading/NacVane signals)
  3. Filter coverage simulation with multi-mode flags enabled
     - Exercises: NotchFilter, SecLPFilter_Vel, ForeAftDamping, FloatingFeedback,
       FlapControl, YawRateControl, StructuralControl, CableControl
  4. Flap control simulation with Flp_Mode=2
     - Exercises: PIIController (dual-integral flap controller)
  5. Active wake control simulation with AWC_Mode=4
     - Exercises: ResController (proportional-resonant controller), ActiveWakeControl
  6. IPC simulation with IPC_ControlMode=1
     - Exercises: IPC, NotchFilterSlopes
  7. Synthetic inputs for under-exercised functions (manual sim loop)
     - Exercises: YawRateControl, ForeAftDamping, FloatingFeedback,
       StructuralControl, CableControl, FlapControl with non-zero inputs
  8. IPC + AWC with real blade moments (manual sim loop)
     - Exercises: IPC (real gains), ActiveWakeControl (rootMOOP feedback),
       NotchFilterSlopes (non-zero rootMOOP)

All scenarios use the same compiled libdiscon.so, so KGen instrumentation
captures state from whichever function is being extracted.

Usage:
    python3 vit_sim.py              # Run all scenarios
    python3 vit_sim.py --scenario 1 # Standard sim only
    python3 vit_sim.py --scenario 2 # Yaw-by-IPC sim only
    python3 vit_sim.py --scenario 3 # Filter/mode coverage sim only
    python3 vit_sim.py --scenario 4 # Flap control sim only
    python3 vit_sim.py --scenario 5 # Active wake control sim only
    python3 vit_sim.py --scenario 6 # IPC sim only
    python3 vit_sim.py --scenario 7 # Synthetic inputs sim only
    python3 vit_sim.py --scenario 8 # IPC + AWC sim only
"""

import argparse
import hashlib
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np


def save_and_print_results(arrays, scenario_num, output_dir):
    """Save simulation arrays to .npz and print MD5 checksums.

    Args:
        arrays: dict of {name: numpy_array} — the simulation outputs
        scenario_num: int — scenario number (1-6)
        output_dir: str or None — directory to save .npz files (skip if None)
    """
    # Always print checksums to stdout
    for name, arr in sorted(arrays.items()):
        md5 = hashlib.md5(arr.tobytes()).hexdigest()
        print(f"  scenario_{scenario_num} {name}: md5={md5} n={len(arr)}")

    # Save .npz if output_dir specified
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        path = os.path.join(output_dir, f'scenario_{scenario_num}.npz')
        np.savez(path, **arrays)
        print(f"  Saved: {path}")

from rosco import discon_lib_path as lib_name
from rosco.toolbox import controller as ROSCO_controller
from rosco.toolbox import turbine as ROSCO_turbine
from rosco.toolbox import sim as ROSCO_sim
from rosco.toolbox import control_interface as ROSCO_ci
from rosco.toolbox.utilities import write_DISCON
from rosco.toolbox.inputs.validation import load_rosco_yaml


this_dir = os.path.dirname(os.path.abspath(__file__))
example_out_dir = os.path.join(this_dir, 'examples_out')
os.makedirs(example_out_dir, exist_ok=True)

# Workaround for scipy FITPACK bispev non-determinism (dev note 202603261512).
# FITPACK's fpbisp reads an uninitialized stack variable whose value depends on
# residual stack contents from prior Fortran calls (e.g., ROSCO's CableControl).
# We cache the RectBivariateSpline (so the FITPACK constructor doesn't refill
# the stack before evaluation) and scrub 64KB of stack before each evaluation.
#
# Confirmed in scipy 1.17.1 on aarch64 (QEMU/Colima). To test if a future scipy
# version fixes this, comment out the monkey-patch below and run Scenario 3
# thirty times — all runs should produce the same gen_torque MD5.
# See dev note 202603261512 for the full bug report with reproduction steps.
import ctypes as _ctypes
from scipy import interpolate as _interpolate
_scrub_lib_path = os.path.join(os.path.dirname(this_dir), 'rosco', 'lib', 'libscrub.so')
if os.path.exists(_scrub_lib_path):
    _scrub_lib = _ctypes.CDLL(_scrub_lib_path)
    def _scrubbed_interp_surface(self, pitch, TSR):
        if not hasattr(self, '_vit_surface_spline'):
            self._vit_surface_spline = _interpolate.RectBivariateSpline(
                self.pitch_initial_rad, self.TSR_initial, self.performance_table.T)
        _scrub_lib.scrub_stack()
        return np.squeeze(self._vit_surface_spline(pitch, TSR).T)
    def _scrubbed_interp_gradient(self, pitch, TSR):
        if not hasattr(self, '_vit_grad_pitch_spline'):
            self._vit_grad_pitch_spline = _interpolate.RectBivariateSpline(
                self.pitch_initial_rad, self.TSR_initial, self.gradient_pitch.T)
            self._vit_grad_TSR_spline = _interpolate.RectBivariateSpline(
                self.pitch_initial_rad, self.TSR_initial, self.gradient_TSR.T)
        _scrub_lib.scrub_stack()
        grad = np.array([self._vit_grad_pitch_spline(pitch, TSR).T,
                         self._vit_grad_TSR_spline(pitch, TSR).T])
        return np.ndarray.flatten(grad)
    ROSCO_turbine.RotorPerformance.interp_surface = _scrubbed_interp_surface
    ROSCO_turbine.RotorPerformance.interp_gradient = _scrubbed_interp_gradient


def load_turbine_and_controller():
    """Load the NREL5MW turbine and tune a ROSCO controller. Returns (turbine, controller)."""
    tune_dir = os.path.join(this_dir, 'Tune_Cases')
    parameter_filename = os.path.join(tune_dir, 'NREL5MW.yaml')
    inps = load_rosco_yaml(parameter_filename)
    path_params = inps['path_params']
    controller_params = inps['controller_params']

    turbine = ROSCO_turbine.Turbine
    turbine = turbine.load(os.path.join(example_out_dir, '01_NREL5MW_saved.p'))

    cp_filename = os.path.join(tune_dir, path_params['rotor_performance_filename'])
    turbine.load_from_fast(
        path_params['FAST_InputFile'],
        os.path.join(tune_dir, path_params['FAST_directory']),
        rot_source='txt', txt_filename=cp_filename
    )

    controller = ROSCO_controller.Controller(controller_params)
    controller.tune_controller(turbine)

    return turbine, controller, cp_filename


def write_discon(turbine, controller, cp_filename, param_filename, patches=None):
    """Write DISCON.IN and optionally patch parameter values.

    Args:
        patches: dict of {param_name: new_value} to replace in the generated file.
                 e.g. {'Y_ControlMode': 2}
    """
    write_DISCON(turbine, controller, param_file=param_filename, txt_filename=cp_filename)

    if patches:
        with open(param_filename, 'r') as f:
            text = f.read()
        for param, value in patches.items():
            # Match lines like "0                   ! Y_ControlMode   - description"
            # Also handles multi-value lines like "0.0 0.0   ! AWC_CntrGains ..."
            pattern = rf'^(.+?)(\s+! {param}\b.*)$'
            replacement = rf'{value}\2'
            text, count = re.subn(pattern, replacement, text, flags=re.MULTILINE)
            if count == 0:
                print(f"WARNING: Could not patch {param} in {param_filename}")
        with open(param_filename, 'w') as f:
            f.write(text)


# ---------------------------------------------------------------------------
# Scenario 1: Standard step-wind simulation (same as 04_simple_sim.py)
# ---------------------------------------------------------------------------
def run_scenario_1(turbine, controller, cp_filename, output_dir=None):
    """Standard 1-DOF sim. Exercises saturate, wrap_180, interp1d, etc."""
    print("=" * 60)
    print("Scenario 1: Standard step-wind simulation")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON.IN')
    write_discon(turbine, controller, cp_filename, param_filename)

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim1'
    )

    sim_1 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 1000
    ws0 = 7
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_1.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False)

    # Second run to check deallocation (same as 04_simple_sim.py)
    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim1b'
    )
    sim_1b = ROSCO_sim.Sim(turbine, controller_int)
    sim_1b.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False)

    np.testing.assert_almost_equal(sim_1.gen_speed, sim_1b.gen_speed)
    save_and_print_results({
        'gen_torque': sim_1.gen_torque, 'bld_pitch': sim_1.bld_pitch,
        'gen_speed': sim_1.gen_speed, 'gen_power': sim_1.gen_power,
        'nac_yaw': sim_1.nac_yaw,
    }, 1, output_dir)
    print("Scenario 1: PASSED (deterministic, deallocated correctly)")


# ---------------------------------------------------------------------------
# Scenario 2: Yaw-by-IPC simulation (Y_ControlMode=2 → wrap_360)
# ---------------------------------------------------------------------------
def run_scenario_2(turbine, controller, cp_filename, output_dir=None):
    """Sim with Y_ControlMode=2 to exercise wrap_360.

    Sets synthetic NacHeading and NacVane values on avrSWAP so that
    wrap_360 receives inputs spanning <0, 0-360, and >=360 ranges.
    """
    print("=" * 60)
    print("Scenario 2: Yaw-by-IPC simulation (wrap_360)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_yaw.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'Y_ControlMode': 2,
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim2'
    )

    # Shorter simulation — we just need wrap_360 to be called enough times
    # for KGen to capture 20 invocations
    dt = 0.025
    tlen = 100
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0

    # Synthetic NacHeading and NacVane signals (in radians, as avrSWAP expects).
    # The controller converts to degrees via * R2D, then wrap_360 wraps to [0, 360].
    # We vary the sum so it exercises all 3 branches: <0, 0-360, >=360.
    deg2rad = np.pi / 180.0
    nac_heading_deg = 350.0  # Near 360 so that heading + vane can cross both boundaries
    nac_heading_rad = nac_heading_deg * deg2rad

    # 1-DOF rotor model variables
    R = turbine.rotor_radius
    GBRatio = turbine.Ng
    rpm2RadSec = 2.0 * np.pi / 60.0

    bld_pitch = np.zeros_like(t)
    rot_speed = np.ones_like(t) * 4.0 * rpm2RadSec
    gen_speed = rot_speed * GBRatio
    gen_torque = np.zeros_like(t)

    for i, ti in enumerate(t):
        if i == 0:
            continue

        ws_i = ws[i]
        tsr = rot_speed[i-1] * R / ws_i
        cp = turbine.Cp.interp_surface(bld_pitch[i-1], tsr)
        aero_torque = 0.5 * turbine.rho * (np.pi * R**3) * (cp / tsr) * ws_i**2
        rot_speed[i] = rot_speed[i-1] + (dt / turbine.J) * (
            aero_torque - GBRatio * gen_torque[i-1] / (turbine.GBoxEff / 100)
        )
        gen_speed[i] = rot_speed[i] * GBRatio

        # Synthetic NacVane: oscillates so heading+vane crosses 0 and 360
        nac_vane_deg = 20.0 * np.sin(2 * np.pi * ti / 50.0)  # +/- 20 deg
        nac_vane_rad = nac_vane_deg * deg2rad

        # Synthetic rotor azimuth (just accumulate based on rot speed)
        azimuth_rad = (rot_speed[i] * ti) % (2 * np.pi)

        # Set standard turbine state
        turbine_state = {}
        turbine_state['iStatus'] = 1 if i < len(t) - 1 else -1
        turbine_state['t'] = ti
        turbine_state['dt'] = dt
        turbine_state['ws'] = ws_i
        turbine_state['bld_pitch'] = bld_pitch[i-1]
        turbine_state['gen_torque'] = gen_torque[i-1]
        turbine_state['gen_speed'] = gen_speed[i]
        turbine_state['gen_eff'] = turbine.GenEff / 100
        turbine_state['rot_speed'] = rot_speed[i]
        turbine_state['Yaw_fromNorth'] = 0.0
        turbine_state['Y_MeasErr'] = 0.0

        # Set extra avrSWAP values that call_controller doesn't handle.
        # These must be set BEFORE call_controller (which calls call_discon).
        # Python index = Fortran index - 1.
        controller_int.avrSWAP[23] = nac_vane_rad    # avrSWAP(24) = NacVane [rad]
        controller_int.avrSWAP[36] = nac_heading_rad  # avrSWAP(37) = NacHeading [rad]
        controller_int.avrSWAP[59] = azimuth_rad      # avrSWAP(60) = Azimuth [rad]

        # Call controller
        gen_torque[i], bld_pitch[i], _ = controller_int.call_controller(turbine_state)

    controller_int.kill_discon()
    save_and_print_results({
        'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
        'gen_speed': gen_speed,
    }, 2, output_dir)
    print("Scenario 2: PASSED (wrap_360 exercised)")


# ---------------------------------------------------------------------------
# Scenario 3: Filter coverage (IPC + notch + cable control)
# ---------------------------------------------------------------------------
def run_scenario_3(turbine, controller, cp_filename, output_dir=None):
    """Sim with notch filters, cable control, and multiple mode flags enabled.

    Exercises filter/controller functions not active in the standard config:
    - NotchFilter: F_GenSpdNotch_N=1 adds a notch at 1.0 rad/s on gen speed
    - SecLPFilter_Vel: CC_Group_N=1 enables the cable control filter loop
    - FlapControl: Flp_Mode=1 (IPC-based flap control)
    - Plus: TD_Mode, Fl_Mode, Y_ControlMode, StC_Mode, CC_Mode

    Note: NotchFilterSlopes requires IPC_ControlMode=1, which conflicts with
    Flp_Mode > 0 (ROSCO mutual exclusion). Use Scenario 6 for NotchFilterSlopes.
    """
    print("=" * 60)
    print("Scenario 3: Mode coverage (notch + cable + flap + structural)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_filters.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        # NotchFilter: 1 notch filter on generator speed
        'F_NumNotchFilts': 1,
        'F_NotchFreqs': '1.0000',
        'F_NotchBetaNum': '0.0000',
        'F_NotchBetaDen': '0.2500',
        'F_GenSpdNotch_N': 1,
        'F_GenSpdNotch_Ind': '1',
        # Note: NotchFilterSlopes requires IPC_ControlMode=1, but ROSCO rejects
        # IPC_ControlMode > 0 AND Flp_Mode > 0 simultaneously. Since this scenario
        # needs Flp_Mode=1, NotchFilterSlopes must use Scenario 6 instead.

        # SecLPFilter_Vel: enable cable control loop
        # CC_Mode=1 required so DISCON calls CableControl (gated by CC_Mode > 0).
        # CC_DesiredL is fixed-size(12) initialized to 0; with CC_Group_N=1 and
        # tlen=400 < 500, the filter processes zero input the entire run.
        'CC_Mode': 1,
        'CC_Group_N': 1,
        'CC_GroupIndex': '2601',
        # ForeAftDamping: enable tower damper (TD_Mode=1).
        # FA_AccHPF is 0 in 1-DOF sim, so output is 0 — fine for view type verification.
        'TD_Mode': 1,
        # FloatingFeedback: enable floating-specific feedback (Fl_Mode=1).
        # NacIMU_FA_Acc is 0 in 1-DOF sim — output is 0, fine for verification.
        'Fl_Mode': 1,
        # YawRateControl: enable yaw rate control (Y_ControlMode=1).
        # NacHeading/NacVane are 0 in 1-DOF — yaw error is 0, fine for verification.
        'Y_ControlMode': 1,
        # StructuralControl: enable structural control (StC_Mode=1).
        # Uses fixed hardcoded step — processes zero input in 1-DOF sim.
        'StC_Mode': 1,
        'StC_Group_N': 1,
        'StC_GroupIndex': '2801',
        # FlapControl: enable IPC-based flap control (Flp_Mode=1).
        # Blade root bending moments are 0 in 1-DOF sim — fine for extraction.
        'Flp_Mode': 1,
        'F_FlpCornerFreq': '1.0 0.7',  # Required when Flp_Mode > 0
        # FloatingFeedback: required when Fl_Mode > 0
        'F_FlCornerFreq': '1.0 0.7',  # Filter corner freq + damping for floating feedback
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim3'
    )

    sim_3 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_3.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False)
    save_and_print_results({
        'gen_torque': sim_3.gen_torque, 'bld_pitch': sim_3.bld_pitch,
        'gen_speed': sim_3.gen_speed, 'gen_power': sim_3.gen_power,
        'nac_yaw': sim_3.nac_yaw,
    }, 3, output_dir)
    print("Scenario 3: PASSED (NotchFilter + SecLPFilter_Vel + mode flags exercised)")


# ---------------------------------------------------------------------------
# Scenario 4: Flap control (Flp_Mode=2 → PIIController)
# ---------------------------------------------------------------------------
def run_scenario_4(turbine, controller, cp_filename, output_dir=None):
    """Sim with Flp_Mode=2 to exercise PIIController.

    Flp_Mode=2 enables proportional flap control using PIIController
    (dual-integral variant of PIController). IPC must be disabled
    (mutual exclusion: IPC_ControlMode=0 when Flp_Mode > 0).

    In the 1-DOF sim, blade root moments are near-zero, so the flap
    controller processes small signals. This is fine for state capture.
    """
    print("=" * 60)
    print("Scenario 4: Flap control (Flp_Mode=2, PIIController)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_flp.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'Flp_Mode': 2,
        'IPC_ControlMode': 0,  # Mutual exclusion with Flp_Mode > 0
        # F_FlpCornerFreq(1) must be > 0 when Flp_Mode > 0
        # (validation in CheckInputs sets aviFAIL=-1 otherwise)
        'F_FlpCornerFreq': '0.5000  0.7000',
        # Small nonzero gains so PIIController runs without instability
        'Flp_Kp': '-0.001',
        'Flp_Ki': '-0.0005',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim4'
    )

    sim_4 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 100
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_4.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False)
    save_and_print_results({
        'gen_torque': sim_4.gen_torque, 'bld_pitch': sim_4.bld_pitch,
        'gen_speed': sim_4.gen_speed, 'gen_power': sim_4.gen_power,
        'nac_yaw': sim_4.nac_yaw,
    }, 4, output_dir)
    print("Scenario 4: PASSED (PIIController exercised)")


# ---------------------------------------------------------------------------
# Scenario 5: Active wake control (AWC_Mode=4 → ResController)
# ---------------------------------------------------------------------------
def run_scenario_5(turbine, controller, cp_filename, output_dir=None):
    """Sim with AWC_Mode=4 to exercise ResController.

    AWC_Mode=4 enables closed-loop proportional-resonant active wake
    control. The ResController uses Tustin-discretized resonant filter
    equations. AWC_Mode > 1 requires individual pitch control
    (avrSWAP(28)=1, already set by ControllerInterface).

    In the 1-DOF sim, ColemanTransformed blade root moments are
    near-zero, so the resonant controller processes small error signals.
    """
    print("=" * 60)
    print("Scenario 5: Active wake control (AWC_Mode=4, ResController)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_awc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'AWC_Mode': 4,
        # Nonzero gains so ResController produces nontrivial output
        'AWC_CntrGains': '0.0100 0.0050',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim5'
    )

    sim_5 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_5.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False)
    save_and_print_results({
        'gen_torque': sim_5.gen_torque, 'bld_pitch': sim_5.bld_pitch,
        'gen_speed': sim_5.gen_speed, 'gen_power': sim_5.gen_power,
        'nac_yaw': sim_5.nac_yaw,
    }, 5, output_dir)
    print("Scenario 5: PASSED (ResController exercised)")


# ---------------------------------------------------------------------------
# Scenario 6: IPC (IPC_ControlMode=1 → NotchFilterSlopes)
# ---------------------------------------------------------------------------
def run_scenario_6(turbine, controller, cp_filename, output_dir=None):
    """Sim with IPC_ControlMode=1 to exercise NotchFilterSlopes.

    IPC_ControlMode=1 enables Individual Pitch Control, which calls
    NotchFilterSlopes for 1P blade root bending moment filtering
    (gated by IPC_ControlMode > 0 in PreFilterMeasuredSignals).

    Flp_Mode must be 0: ROSCO rejects IPC_ControlMode > 0 AND Flp_Mode > 0
    simultaneously (ReadSetParameters.f90:1108).

    In the 1-DOF sim, blade root moments are 0, so IPC and
    NotchFilterSlopes process zero signals. Fine for state capture.
    """
    print("=" * 60)
    print("Scenario 6: IPC (IPC_ControlMode=1, NotchFilterSlopes)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_ipc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'IPC_ControlMode': 1,
        'Flp_Mode': 0,       # Mutual exclusion with IPC_ControlMode > 0
        'IPC_KI': '0.0 0.0',
        'IPC_KP': '0.0 0.0',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim6'
    )

    sim_6 = ROSCO_sim.Sim(turbine, controller_int)

    dt = 0.025
    tlen = 100
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i in range(len(t)):
        ws[i] = ws[i] + t[i] // 100

    sim_6.sim_ws_series(t, ws, rotor_rpm_init=4, make_plots=False)
    save_and_print_results({
        'gen_torque': sim_6.gen_torque, 'bld_pitch': sim_6.bld_pitch,
        'gen_speed': sim_6.gen_speed, 'gen_power': sim_6.gen_power,
        'nac_yaw': sim_6.nac_yaw,
    }, 6, output_dir)
    print("Scenario 6: PASSED (NotchFilterSlopes exercised)")


# ---------------------------------------------------------------------------
# Scenario 7: Synthetic inputs for under-exercised functions
# ---------------------------------------------------------------------------
def run_scenario_7(turbine, controller, cp_filename, output_dir=None):
    """Exercises functions that receive zero inputs in the 1-DOF sim.

    Uses a manual sim loop (like Scenario 2) to inject synthetic non-zero
    values into avrSWAP before each controller call:
    - NacVane/NacHeading: oscillating yaw error for YawRateControl
    - FA_Acc_TT: oscillating tower acceleration for ForeAftDamping
    - NacIMU_FA_RAcc: oscillating nacelle IMU for FloatingFeedback
    - rootMOOP: per-blade 1P sinusoidal moments for FlapControl
    - tlen=600s: exceeds Time>500 threshold for StructuralControl/CableControl

    Mode flags: Y_ControlMode=1, TD_Mode=1, Fl_Mode=1, StC_Mode=1,
    CC_Mode=1, Flp_Mode=1. IPC_ControlMode=0 (mutual exclusion with Flp_Mode).
    """
    print("=" * 60)
    print("Scenario 7: Synthetic inputs (yaw, tower, float, struct, cable, flap)")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_synth.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'Y_ControlMode': 1,
        'TD_Mode': 1,
        'Fl_Mode': 1,
        'StC_Mode': 1,
        'StC_Group_N': 1,
        'StC_GroupIndex': '2801',
        'CC_Mode': 1,
        'CC_Group_N': 1,
        'CC_GroupIndex': '2601',
        'Flp_Mode': 1,
        'F_FlpCornerFreq': '1.0 0.7',
        'F_FlCornerFreq': '1.0 0.7',
        'IPC_ControlMode': 0,
        'AWC_Mode': 0,
        'F_NumNotchFilts': 1,
        'F_NotchFreqs': '1.0000',
        'F_NotchBetaNum': '0.0000',
        'F_NotchBetaDen': '0.2500',
        'F_GenSpdNotch_N': 1,
        'F_GenSpdNotch_Ind': '1',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim7'
    )

    dt = 0.025
    tlen = 600  # Must exceed 500 for StructuralControl/CableControl step
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i_ws in range(len(t)):
        ws[i_ws] = ws[i_ws] + t[i_ws] // 100

    deg2rad = np.pi / 180.0
    R = turbine.rotor_radius
    GBRatio = turbine.Ng
    rpm2RadSec = 2.0 * np.pi / 60.0

    bld_pitch = np.zeros_like(t)
    rot_speed = np.ones_like(t) * 4.0 * rpm2RadSec
    gen_speed = rot_speed * GBRatio
    gen_torque = np.zeros_like(t)
    gen_power = np.zeros_like(t)
    nac_yaw = np.zeros_like(t)
    nac_yawrate = np.zeros_like(t)

    for i, ti in enumerate(t):
        if i == 0:
            continue

        ws_i = ws[i]
        tsr = rot_speed[i-1] * R / ws_i
        cp = turbine.Cp.interp_surface(bld_pitch[i-1], tsr)
        aero_torque = 0.5 * turbine.rho * (np.pi * R**3) * (cp / tsr) * ws_i**2
        rot_speed[i] = rot_speed[i-1] + (dt / turbine.J) * (
            aero_torque - GBRatio * gen_torque[i-1] / (turbine.GBoxEff / 100)
        )
        gen_speed[i] = rot_speed[i] * GBRatio

        # Synthetic yaw signals
        nac_vane_deg = 20.0 * np.sin(2 * np.pi * ti / 50.0)
        nac_vane_rad = nac_vane_deg * deg2rad
        nac_heading_rad = 350.0 * deg2rad

        # Synthetic rotor azimuth
        azimuth_rad = (rot_speed[i] * ti) % (2 * np.pi)

        # Synthetic tower fore-aft acceleration (~0.3 Hz tower mode)
        fa_acc_tt = 0.5 * np.sin(2 * np.pi * ti / 3.0)

        # Synthetic nacelle IMU acceleration
        nac_imu_fa_racc = 0.3 * np.sin(2 * np.pi * ti / 3.0)

        # Synthetic blade root moments (1P per-blade, 120 deg phase offset)
        if rot_speed[i] > 0.1:
            t_rotor = 2 * np.pi / rot_speed[i]
        else:
            t_rotor = 100.0
        rootMOOP = [
            1000.0 * np.sin(2 * np.pi * ti / t_rotor + k * 2 * np.pi / 3)
            for k in range(3)
        ]

        turbine_state = {}
        turbine_state['iStatus'] = 1 if i < len(t) - 1 else -1
        turbine_state['t'] = ti
        turbine_state['dt'] = dt
        turbine_state['ws'] = ws_i
        turbine_state['bld_pitch'] = bld_pitch[i-1]
        turbine_state['gen_torque'] = gen_torque[i-1]
        turbine_state['gen_speed'] = gen_speed[i]
        turbine_state['gen_eff'] = turbine.GenEff / 100
        turbine_state['rot_speed'] = rot_speed[i]
        turbine_state['Yaw_fromNorth'] = nac_yaw[i-1]
        turbine_state['Y_MeasErr'] = nac_vane_rad
        turbine_state['FA_Acc_TT'] = fa_acc_tt
        turbine_state['NacIMU_FA_RAcc'] = nac_imu_fa_racc

        # Inject avrSWAP values not handled by call_controller
        controller_int.avrSWAP[23] = nac_vane_rad      # avrSWAP(24) NacVane
        controller_int.avrSWAP[36] = nac_heading_rad    # avrSWAP(37) NacHeading
        controller_int.avrSWAP[59] = azimuth_rad         # avrSWAP(60) Azimuth
        controller_int.avrSWAP[29] = rootMOOP[0]         # avrSWAP(30) rootMOOP(1)
        controller_int.avrSWAP[30] = rootMOOP[1]         # avrSWAP(31) rootMOOP(2)
        controller_int.avrSWAP[31] = rootMOOP[2]         # avrSWAP(32) rootMOOP(3)

        gen_torque[i], bld_pitch[i], nac_yawrate[i] = controller_int.call_controller(turbine_state)
        gen_power[i] = gen_speed[i] * gen_torque[i] * turbine.GenEff / 100
        nac_yaw[i] = nac_yaw[i-1] + nac_yawrate[i] * dt

    controller_int.kill_discon()
    save_and_print_results({
        'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
        'gen_speed': gen_speed, 'gen_power': gen_power,
        'nac_yaw': nac_yaw,
    }, 7, output_dir)
    print("Scenario 7: PASSED (yaw, tower, float, struct, cable, flap exercised)")


# ---------------------------------------------------------------------------
# Scenario 8: IPC with real gains + blade moments + ActiveWakeControl
# ---------------------------------------------------------------------------
def run_scenario_8(turbine, controller, cp_filename, output_dir=None):
    """Exercises IPC and ActiveWakeControl with non-zero blade root moments.

    IPC_ControlMode=1 with non-zero gains + AWC_Mode=4 with rootMOOP feedback.
    Also exercises NotchFilterSlopes (triggered by IPC_ControlMode > 0 in
    PreFilterMeasuredSignals).

    Flp_Mode=0 (mutual exclusion with IPC_ControlMode > 0).
    """
    print("=" * 60)
    print("Scenario 8: IPC + AWC with blade moments")
    print("=" * 60)

    param_filename = os.path.join(this_dir, 'DISCON_ipc_awc.IN')
    write_discon(turbine, controller, cp_filename, param_filename, patches={
        'IPC_ControlMode': 1,
        'IPC_KP': '0.1 0.1',
        'IPC_KI': '0.01 0.01',
        'AWC_Mode': 4,
        'AWC_NumModes': 1,
        'AWC_n': '1',
        'AWC_clockangle': '0.0',
        'AWC_freq': '0.05',
        'AWC_amp': '0.0',
        'AWC_CntrGains': '0.0100 0.0050',
        'Flp_Mode': 0,
        'F_NumNotchFilts': 1,
        'F_NotchFreqs': '1.0000',
        'F_NotchBetaNum': '0.0000',
        'F_NotchBetaDen': '0.2500',
        'F_GenSpdNotch_N': 1,
        'F_GenSpdNotch_Ind': '1',
    })

    controller_int = ROSCO_ci.ControllerInterface(
        lib_name, param_filename=param_filename, sim_name='vit_sim8'
    )

    dt = 0.025
    tlen = 400
    ws0 = 9
    t = np.arange(0, tlen, dt)
    ws = np.ones_like(t) * ws0
    for i_ws in range(len(t)):
        ws[i_ws] = ws[i_ws] + t[i_ws] // 100

    deg2rad = np.pi / 180.0
    R = turbine.rotor_radius
    GBRatio = turbine.Ng
    rpm2RadSec = 2.0 * np.pi / 60.0

    bld_pitch = np.zeros_like(t)
    rot_speed = np.ones_like(t) * 4.0 * rpm2RadSec
    gen_speed = rot_speed * GBRatio
    gen_torque = np.zeros_like(t)
    gen_power = np.zeros_like(t)
    nac_yaw = np.zeros_like(t)

    for i, ti in enumerate(t):
        if i == 0:
            continue

        ws_i = ws[i]
        tsr = rot_speed[i-1] * R / ws_i
        cp = turbine.Cp.interp_surface(bld_pitch[i-1], tsr)
        aero_torque = 0.5 * turbine.rho * (np.pi * R**3) * (cp / tsr) * ws_i**2
        rot_speed[i] = rot_speed[i-1] + (dt / turbine.J) * (
            aero_torque - GBRatio * gen_torque[i-1] / (turbine.GBoxEff / 100)
        )
        gen_speed[i] = rot_speed[i] * GBRatio

        azimuth_rad = (rot_speed[i] * ti) % (2 * np.pi)

        # Synthetic blade root moments (1P per-blade)
        if rot_speed[i] > 0.1:
            t_rotor = 2 * np.pi / rot_speed[i]
        else:
            t_rotor = 100.0
        rootMOOP = [
            1000.0 * np.sin(2 * np.pi * ti / t_rotor + k * 2 * np.pi / 3)
            for k in range(3)
        ]

        turbine_state = {}
        turbine_state['iStatus'] = 1 if i < len(t) - 1 else -1
        turbine_state['t'] = ti
        turbine_state['dt'] = dt
        turbine_state['ws'] = ws_i
        turbine_state['bld_pitch'] = bld_pitch[i-1]
        turbine_state['gen_torque'] = gen_torque[i-1]
        turbine_state['gen_speed'] = gen_speed[i]
        turbine_state['gen_eff'] = turbine.GenEff / 100
        turbine_state['rot_speed'] = rot_speed[i]
        turbine_state['Yaw_fromNorth'] = 0.0
        turbine_state['Y_MeasErr'] = 0.0

        # Inject blade root moments and azimuth
        controller_int.avrSWAP[29] = rootMOOP[0]   # avrSWAP(30) rootMOOP(1)
        controller_int.avrSWAP[30] = rootMOOP[1]   # avrSWAP(31) rootMOOP(2)
        controller_int.avrSWAP[31] = rootMOOP[2]   # avrSWAP(32) rootMOOP(3)
        controller_int.avrSWAP[59] = azimuth_rad    # avrSWAP(60) Azimuth

        gen_torque[i], bld_pitch[i], _ = controller_int.call_controller(turbine_state)
        gen_power[i] = gen_speed[i] * gen_torque[i] * turbine.GenEff / 100

    controller_int.kill_discon()
    save_and_print_results({
        'gen_torque': gen_torque, 'bld_pitch': bld_pitch,
        'gen_speed': gen_speed, 'gen_power': gen_power,
        'nac_yaw': nac_yaw,
    }, 8, output_dir)
    print("Scenario 8: PASSED (IPC + AWC with blade moments exercised)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='VIT simulation runner')
    parser.add_argument('--scenario', type=int, default=0,
                        help='Run specific scenario (1-8). Default 0 = run all.')
    parser.add_argument('--output-dir', type=str, default=None,
                        help='Save simulation output arrays to .npz files in this directory.')
    args = parser.parse_args()

    turbine, controller, cp_filename = load_turbine_and_controller()
    od = args.output_dir

    # Scenario 3 runs first so KGen's early invocations (1-20) capture all
    # mode-gated code paths: CC_Mode=1 (CableControl), StC_Mode=1
    # (StructuralControl), Fl_Mode=1 (FloatingFeedback), Flp_Mode=1
    # (FlapControl), Y_ControlMode=1 (YawRateControl), TD_Mode=1
    # (ForeAftDamping). Scenarios 4 and 5 follow for PIIController
    # (Flp_Mode=2) and ResController (AWC_Mode=4).
    if args.scenario == 0 or args.scenario == 3:
        run_scenario_3(turbine, controller, cp_filename, od)

    if args.scenario == 0 or args.scenario == 4:
        run_scenario_4(turbine, controller, cp_filename, od)

    if args.scenario == 0 or args.scenario == 5:
        run_scenario_5(turbine, controller, cp_filename, od)

    if args.scenario == 0 or args.scenario == 1:
        run_scenario_1(turbine, controller, cp_filename, od)

    if args.scenario == 0 or args.scenario == 2:
        run_scenario_2(turbine, controller, cp_filename, od)

    if args.scenario == 0 or args.scenario == 6:
        run_scenario_6(turbine, controller, cp_filename, od)

    if args.scenario == 0 or args.scenario == 7:
        run_scenario_7(turbine, controller, cp_filename, od)

    if args.scenario == 0 or args.scenario == 8:
        run_scenario_8(turbine, controller, cp_filename, od)

    print("\n" + "=" * 60)
    print("All scenarios complete.")
    print("=" * 60)


if __name__ == '__main__':
    main()
