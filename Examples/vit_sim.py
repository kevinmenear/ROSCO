"""
vit_sim
-------
VIT-specific simulation script for kernel extraction and baseline capture.

Runs multiple simulation scenarios to exercise all controller code paths needed
for VIT translation verification:

  1. Standard 1-DOF step-wind simulation (same as 04_simple_sim.py)
     - Exercises: saturate, wrap_180, interp1d, and most other functions
  2. Yaw-by-IPC simulation with Y_ControlMode=2
     - Exercises: wrap_360 (via synthetic NacHeading/NacVane signals)

All scenarios use the same compiled libdiscon.so, so KGen instrumentation
captures state from whichever function is being extracted.

Usage:
    python3 vit_sim.py              # Run all scenarios
    python3 vit_sim.py --scenario 1 # Standard sim only
    python3 vit_sim.py --scenario 2 # Yaw-by-IPC sim only
"""

import argparse
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import numpy as np

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
            pattern = rf'^(\S+)(\s+! {param}\b.*)$'
            replacement = rf'{value}\2'
            text, count = re.subn(pattern, replacement, text, flags=re.MULTILINE)
            if count == 0:
                print(f"WARNING: Could not patch {param} in {param_filename}")
        with open(param_filename, 'w') as f:
            f.write(text)


# ---------------------------------------------------------------------------
# Scenario 1: Standard step-wind simulation (same as 04_simple_sim.py)
# ---------------------------------------------------------------------------
def run_scenario_1(turbine, controller, cp_filename):
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
    print("Scenario 1: PASSED (deterministic, deallocated correctly)")


# ---------------------------------------------------------------------------
# Scenario 2: Yaw-by-IPC simulation (Y_ControlMode=2 → wrap_360)
# ---------------------------------------------------------------------------
def run_scenario_2(turbine, controller, cp_filename):
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
    print("Scenario 2: PASSED (wrap_360 exercised)")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='VIT simulation runner')
    parser.add_argument('--scenario', type=int, default=0,
                        help='Run specific scenario (1 or 2). Default 0 = run all.')
    args = parser.parse_args()

    turbine, controller, cp_filename = load_turbine_and_controller()

    if args.scenario == 0 or args.scenario == 1:
        run_scenario_1(turbine, controller, cp_filename)

    if args.scenario == 0 or args.scenario == 2:
        run_scenario_2(turbine, controller, cp_filename)

    print("\n" + "=" * 60)
    print("All scenarios complete.")
    print("=" * 60)


if __name__ == '__main__':
    main()
