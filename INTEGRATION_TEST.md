# Integration Test: Baseline vs Integrated Array Comparison

This document captures the exact steps to validate that C++ translations produce byte-identical simulation results to the original Fortran.

## Prerequisites

- Docker container `vit-dev` running (mount: `~/Artifacts/vit_translation/` → `/workspace/`)
- VIT and ROSCO installed in the container
- Golden kernel fixtures in `kernel/` (for KGen verification, not used here)
- Translation files in `translations/` for all functions being integrated
- FITPACK stack scrubber built: `gcc -shared -fPIC -o rosco/lib/libscrub.so rosco/controller/src/scrub_stack.c` (required for Scenario 3 determinism — see dev note `202603261512`)

## Overview

The test compares simulation output arrays (gen_torque, bld_pitch, gen_speed, gen_power, nac_yaw) between a pure-Fortran baseline and the integrated Fortran/C++ build. Each of 8 scenarios runs in a **separate OS process** to avoid DLL SAVE variable contamination across scenarios.

**What is compared:** 612,000 float64 values across 8 scenarios, using `np.array_equal()` (byte-identical, not tolerance-based).

**What this proves:** The Fortran wrappers, C struct passing, view types, C++→C++ call paths, and library loading all produce correct results in the real ROSCO build. Every controller output at every timestep matches the pure-Fortran original.

## Step 1: Reset Sources to Clean Fortran

Run on the **Mac** (not in the container):

```bash
cd ~/Artifacts/vit_translation/ROSCO

# Functions.f90 + Filters.f90: dba7738 (restart fix + notch filter bug fix, no wrappers)
git checkout dba7738 -- rosco/controller/src/Functions.f90 rosco/controller/src/Filters.f90

# Controllers.f90 + ControllerBlocks.f90: fde0498 (Phase 4C, no wrappers)
git checkout fde0498 -- rosco/controller/src/Controllers.f90 rosco/controller/src/ControllerBlocks.f90

# DISCON.F90: upstream (no wrappers)
git checkout e8010f0 -- rosco/controller/src/DISCON.F90

# Create C++ stubs (CMakeLists.txt references these)
for f in rosco/controller/src/*_cpp.cpp; do echo "// stub" > "$f"; done
```

### Apply FlapControl K init bug fix

fde0498 does NOT include this fix. Without it, Scenario 4 (Flp_Mode=2) segfaults.

In `rosco/controller/src/Controllers.f90`, find line 669 (inside FlapControl, `IF (CntrPar%Flp_Mode == 2) THEN` block) and wrap the PIIController call in a DO loop:

```fortran
! BEFORE (broken — K is uninitialized):
                IF (CntrPar%Flp_Mode == 2) THEN
                    LocalVar%Flp_Angle(K) = PIIController(...)
                ENDIF

! AFTER (fixed):
                IF (CntrPar%Flp_Mode == 2) THEN
                    DO K = 1,LocalVar%NumBl
                        LocalVar%Flp_Angle(K) = PIIController(...)
                    END DO
                ENDIF
```

### Verify clean state

```bash
# Must all be 0:
grep -c '_c(' rosco/controller/src/Functions.f90
grep -c '_c(' rosco/controller/src/Filters.f90
grep -c '_c(' rosco/controller/src/Controllers.f90
grep -c '_c(' rosco/controller/src/ControllerBlocks.f90

# Must be "// stub":
cat rosco/controller/src/saturate_cpp.cpp
```

## Step 2: Build Clean Fortran Baseline

Run in the **container**:

```bash
cd /workspace/ROSCO/rosco/controller/build
rm -rf *
cmake .. -DCMAKE_INSTALL_PREFIX=install
cmake --build .
cp libdiscon.so ../../lib/libdiscon.so
cd /workspace/ROSCO
```

## Step 3: Capture Baseline Arrays

Each scenario runs in a **separate `docker exec` call** from the Mac. This guarantees a fresh OS process for each scenario — no DLL SAVE variable persistence between scenarios.

```bash
docker exec vit-dev bash -c "rm -rf /workspace/ROSCO/baseline_final"

docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 1 --output-dir /workspace/ROSCO/baseline_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 2 --output-dir /workspace/ROSCO/baseline_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 3 --output-dir /workspace/ROSCO/baseline_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 4 --output-dir /workspace/ROSCO/baseline_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 5 --output-dir /workspace/ROSCO/baseline_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 6 --output-dir /workspace/ROSCO/baseline_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 7 --output-dir /workspace/ROSCO/baseline_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 8 --output-dir /workspace/ROSCO/baseline_final"
```

Verify: 8 `.npz` files in `baseline_final/`, all 8 scenarios print "PASSED".

## Step 4: Integrate All Functions

Run in the **container**:

```bash
cd /workspace/ROSCO

vit integrate saturate translations/Functions/saturate.cpp -f rosco/controller/src/Functions.f90 --apply
vit integrate wrap_180 translations/Functions/wrap_180.cpp -f rosco/controller/src/Functions.f90 --apply
vit integrate wrap_360 translations/Functions/wrap_360.cpp -f rosco/controller/src/Functions.f90 --apply
vit integrate ratelimit translations/Functions/ratelimit.cpp -f rosco/controller/src/Functions.f90 --apply
vit integrate ColemanTransform translations/Functions/colemantransform.cpp -f rosco/controller/src/Functions.f90 --apply
vit integrate ColemanTransformInverse translations/Functions/colemantransforminverse.cpp -f rosco/controller/src/Functions.f90 --apply
vit integrate identity translations/Functions/identity.cpp -f rosco/controller/src/Functions.f90 --apply
vit integrate sigma translations/Functions/sigma.cpp -f rosco/controller/src/Functions.f90 --apply
vit integrate interp1d translations/Functions/interp1d.cpp -f rosco/controller/src/Functions.f90 --apply

vit integrate LPFilter translations/Filters/lpfilter.cpp -f rosco/controller/src/Filters.f90 --apply
vit integrate HPFilter translations/Filters/hpfilter.cpp -f rosco/controller/src/Filters.f90 --apply
vit integrate SecLPFilter translations/Filters/seclpfilter.cpp -f rosco/controller/src/Filters.f90 --apply
vit integrate SecLPFilter_Vel translations/Filters/seclpfilter_vel.cpp -f rosco/controller/src/Filters.f90 --apply
vit integrate NotchFilter translations/Filters/notchfilter.cpp -f rosco/controller/src/Filters.f90 --apply
vit integrate NotchFilterSlopes translations/Filters/notchfilterslopes.cpp -f rosco/controller/src/Filters.f90 --apply

vit integrate PIController translations/Controllers/picontroller.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate PIIController translations/Controllers/piicontroller.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate ResController translations/Controllers/rescontroller.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate ForeAftDamping translations/Controllers/foreaftdamping.cpp -f rosco/controller/src/Controllers.f90 --apply

vit integrate FloatingFeedback translations/Controllers/floatingfeedback.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate StructuralControl translations/Controllers/structuralcontrol.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate CableControl translations/Controllers/cablecontrol.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate FlapControl translations/Controllers/flapcontrol.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate YawRateControl translations/Controllers/yawratecontrol.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate VariableSpeedControl translations/Controllers/variablespeedcontrol.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate IPC translations/Controllers/ipc.cpp -f rosco/controller/src/Controllers.f90 --apply
vit integrate ActiveWakeControl translations/Controllers/activewakecontrol.cpp -f rosco/controller/src/Controllers.f90 --apply
```

Each must print "Integration applied successfully".

### Verify integration was applied

```bash
# Should show non-zero counts:
grep -c '_c(' rosco/controller/src/Functions.f90     # expect 18
grep -c '_c(' rosco/controller/src/Filters.f90       # expect 12
grep -c '_c(' rosco/controller/src/Controllers.f90   # expect 24

# Should show 27:
grep -c '_cpp.cpp' rosco/controller/CMakeLists.txt

# Should show real C++ code, not "// stub":
head -3 rosco/controller/src/saturate_cpp.cpp
```

**Note:** PitchControl is excluded. It depends on PitchSaturation (ControllerBlocks.f90, Phase 8), which is not yet translated. See dev note `202603241718`.

## Step 5: Clean Rebuild with C++

Run in the **container**:

```bash
cd /workspace/ROSCO/rosco/controller/build
rm -rf *
cmake .. -DCMAKE_INSTALL_PREFIX=install
cmake --build .
cp libdiscon.so ../../lib/libdiscon.so
cd /workspace/ROSCO
```

## Step 6: Capture Integrated Arrays

Same per-scenario isolation as baseline — each scenario in a **separate `docker exec` call**:

```bash
docker exec vit-dev bash -c "rm -rf /workspace/ROSCO/integrated_final"

docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 1 --output-dir /workspace/ROSCO/integrated_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 2 --output-dir /workspace/ROSCO/integrated_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 3 --output-dir /workspace/ROSCO/integrated_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 4 --output-dir /workspace/ROSCO/integrated_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 5 --output-dir /workspace/ROSCO/integrated_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 6 --output-dir /workspace/ROSCO/integrated_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 7 --output-dir /workspace/ROSCO/integrated_final"
docker exec vit-dev bash -c "cd /workspace/ROSCO/Examples && python3 vit_sim.py --scenario 8 --output-dir /workspace/ROSCO/integrated_final"
```

Verify: 8 `.npz` files in `integrated_final/`, all 8 scenarios print "PASSED".

## Step 7: Compare All Arrays

Run in the **container** (or from Mac via `docker exec`):

```bash
docker exec vit-dev bash -c 'cd /workspace/ROSCO && python3 -c "
import numpy as np, sys

print(\"========================================\")
print(\"BASELINE vs INTEGRATED: Full Comparison\")
print(\"========================================\")
print()

ok = True
total = 0

for s in [1, 2, 3, 4, 5, 6, 7, 8]:
    b = np.load(f\"baseline_final/scenario_{s}.npz\")
    i = np.load(f\"integrated_final/scenario_{s}.npz\")

    if set(b.files) != set(i.files):
        print(f\"scenario_{s}: KEY MISMATCH baseline={sorted(b.files)} integrated={sorted(i.files)}\")
        ok = False
        continue

    for key in sorted(b.files):
        nb = len(b[key])
        ni = len(i[key])
        if nb != ni:
            print(f\"  scenario_{s} {key}: LENGTH MISMATCH baseline={nb} integrated={ni}\")
            ok = False
            continue
        total += nb
        if np.array_equal(b[key], i[key]):
            print(f\"  scenario_{s} {key}: IDENTICAL ({nb} values)\")
        else:
            diff = np.abs(b[key] - i[key])
            idx = diff.argmax()
            first = np.where(b[key] != i[key])[0][0]
            print(f\"  scenario_{s} {key}: MISMATCH max_diff={diff.max():.2e} at idx {idx}, first_diff at idx {first}\")
            ok = False

print()
if ok:
    print(f\"RESULT: ALL IDENTICAL — {total} total float64 values compared\")
else:
    print(f\"RESULT: DIFFERENCES FOUND\")
    sys.exit(1)
"'
```

**Expected output:** ALL IDENTICAL — 612,000 total float64 values compared.

## Step 8: Cleanup

```bash
docker exec vit-dev bash -c "rm -rf /workspace/ROSCO/integrated_final"
```

Keep `baseline_final/` as reference for future tests (or regenerate from clean Fortran as needed).

## Why Per-Scenario Isolation Is Required

Running all 6 scenarios in a single Python process causes Fortran SAVE variables (LocalVar, CntrPar, objInst, etc.) to persist between scenarios via DLL memory that `dlclose()` does not reliably release. This cross-scenario contamination is:

- **Consistent with clean Fortran** (same contaminated result every run)
- **Non-deterministic with the integrated binary** (varies between runs due to Python Cp interpolation sensitivity to shared library memory layout)

Per-scenario isolation (separate `docker exec` = separate OS process = guaranteed fresh library load) eliminates both issues. Each scenario starts with zeroed SAVE variables and independent Python/scipy state.

## Scenario Coverage

| Scenario | Duration | Timesteps | Mode Flags | Functions Exercised |
|----------|----------|-----------|------------|-------------------|
| 1 | 1000s | 40,000 | Default | saturate, wrap_180, interp1d, LPFilter, HPFilter, SecLPFilter, PIController, VariableSpeedControl |
| 2 | 100s | 4,000 | Y_ControlMode=2 | wrap_360 |
| 3 | 400s | 16,000 | CC_Mode=1, TD_Mode=1, Fl_Mode=1, Y_ControlMode=1, StC_Mode=1, Flp_Mode=1, F_GenSpdNotch_N=1 | NotchFilter, SecLPFilter_Vel, ForeAftDamping, FloatingFeedback, FlapControl, YawRateControl, StructuralControl, CableControl |
| 4 | 100s | 4,000 | Flp_Mode=2 | PIIController |
| 5 | 400s | 16,000 | AWC_Mode=4 | ResController, ActiveWakeControl |
| 6 | 100s | 4,000 | IPC_ControlMode=1 | IPC, NotchFilterSlopes |
| 7 | 600s | 24,000 | Y_ControlMode=1, TD_Mode=1, Fl_Mode=1, StC_Mode=1, CC_Mode=1, Flp_Mode=1 | YawRateControl, ForeAftDamping, FloatingFeedback, StructuralControl, CableControl, FlapControl (all with synthetic non-zero inputs: NacVane, NacHeading, FA_Acc_TT, NacIMU_FA_RAcc, rootMOOP) |
| 8 | 400s | 16,000 | IPC_ControlMode=1 (KP=0.1, KI=0.01), AWC_Mode=4 | IPC (real gains), ActiveWakeControl (rootMOOP feedback), NotchFilterSlopes (non-zero rootMOOP) |

## Last Validated

2026-03-26: 27 functions, 612,000 values across 8 scenarios, ALL IDENTICAL.
