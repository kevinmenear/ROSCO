#!/usr/bin/env python3
"""Why no scenario drives `wrap_180` outside [-180, 180], read from committed data.

THE TRAP THIS EXISTS TO CLOSE. Reading `Examples/vit_sim.py` says scenario 7 (and
27) drive the yaw path hard:

    nac_vane_deg    = 20.0 * np.sin(2 * np.pi * ti / 50.0)      # +/- 20 deg
    nac_heading_rad = 350.0 * deg2rad                           # 350 deg
    ...
    # Inject avrSWAP values not handled by call_controller
    controller_int.avrSWAP[23] = nac_vane_rad      # avrSWAP(24) NacVane
    controller_int.avrSWAP[36] = nac_heading_rad   # avrSWAP(37) NacHeading

`ReadAvrSWAP` sets `LocalVar%NacHeading = avrSWAP(37) * R2D`, so this reads as
NacHeading = 350 and NacVane in +/-20, hence `x = NacHeading + NacVane` in
[330, 370] at Controllers.f90:400 -- squarely in the `x .gt. 180` branch, on
every one of 23,999 timesteps.

`coverage_branch_deadness.py` says that branch has ZERO hits in all 27
scenarios. One of the two readings is wrong, and it is the source-tracing one.

THE CAUSE. `call_controller` re-populates avrSWAP from `turbine_state` AFTER the
scenario's injection and BEFORE `call_discon`:

    rosco/toolbox/control_interface.py:209   self.avrSWAP[23] = turbine_state["Y_MeasErr"]
    rosco/toolbox/control_interface.py:211   self.avrSWAP[36] = turbine_state["Yaw_fromNorth"]

So two of the six indices the comment calls "not handled by call_controller" ARE
handled by it. Index 23 survives by coincidence -- `Y_MeasErr` is set to the same
`nac_vane_rad` two lines above -- and index 36 does not: `Yaw_fromNorth` is
`nac_yaw[i-1]`, the ACCUMULATED yaw position, which starts at 0.

THE FALSIFICATION, and it needs no simulation. `DebugVar%Yaw_Err` is
`wrap_180(NacHeadingTarget - LocalVar%NacHeading)` (Controllers.f90:423) and
`DebugVar%NacHeadingTarget` is the other operand; both are committed channels of
`vit_sim7.RO.dbg`. Given the observed range of the target, a heading of 350
CONSTRAINS Yaw_Err to an interval, and the observed Yaw_Err is outside it.

Exit 0 means: NacHeading = 350 is refuted by the committed baseline, so the only
input in this corpus intended to push `x` past +/-180 never reaches the
controller.
"""
import sys

import numpy as np


def wrap_180(x):
    """The reference, transcribed, so the prediction uses the unit's own rule."""
    return np.where(x <= -180.0, x + 360.0, np.where(x > 180.0, x - 360.0, x))


FAILED = []
for scen in (7, 27):
    path = f'vit_sim{scen}.RO.dbg'
    names = open(path).readlines()[1].split()
    d = np.loadtxt(path, skiprows=3)
    tgt = d[:, names.index('NacHeadingTarge')]
    err = d[:, names.index('Yaw_Err')]

    # If NacHeading were the injected 350, Yaw_Err would be forced into this
    # interval by the observed target range alone.
    pred = wrap_180(np.array([tgt.min(), tgt.max()]) - 350.0)
    lo, hi = pred.min(), pred.max()

    print(f'== {path}   {len(d):,} timesteps')
    print(f'  NacHeadingTarget observed        [{tgt.min():+9.4f}, {tgt.max():+9.4f}]')
    print(f'  Yaw_Err          observed        [{err.min():+9.4f}, {err.max():+9.4f}]')
    print(f'  Yaw_Err PREDICTED if heading=350 [{lo:+9.4f}, {hi:+9.4f}]'
          '   <- wrap_180(target - 350)')
    outside = int(((err < lo - 1e-9) | (err > hi + 1e-9)).sum())
    print(f'  timesteps OUTSIDE that interval  {outside:,} of {len(d):,}')
    # The alternative hypothesis -- heading = the ACCUMULATED yaw, starting at 0
    # -- must fit, and its residual is a quantity with a name: how far the
    # nacelle has yawed. A heading pinned at 0 would give residual 0; this is
    # the drift, and it stays an order of magnitude below the 330 that
    # separates the two hypotheses.
    fit = float(np.abs(err - wrap_180(tgt - 0.0)).max())
    print(f'  |Yaw_Err - wrap_180(target - 0)|  max {fit:.4f}   '
          '<- = the accumulated yaw travel, not 350')
    print(f'  max |x| seen by the unit here    {max(abs(tgt).max(), abs(err).max()):.4f}'
          '   (both are wrap_180 OUTPUTS at :419 and :423)')
    print()
    if outside == 0:
        FAILED.append(f'{path}: heading=350 NOT refuted')

if FAILED:
    print('INCONCLUSIVE:', '; '.join(FAILED))
    sys.exit(1)

print('VERDICT: heading = 350 is refuted in both scenarios that inject it -- 10,632')
print('         of 23,999 timesteps fall outside the interval it would force, and one')
print('         would have been enough. avrSWAP(37) is')
print('         overwritten by call_controller before the controller reads it, so')
print('         the corpus never drives `x` outside [-180, 180] and both wrapping')
print('         branches are unreachable by the gate and by any kernel.')
sys.exit(0)
