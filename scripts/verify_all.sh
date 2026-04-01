#!/bin/bash
# Verify all 41 C++ translations against golden kernel fixtures.
# Run from the ROSCO repo root inside the Docker container.
#
# Usage:
#   bash scripts/verify_all.sh          # Run all 41 sequentially
#   bash scripts/verify_all.sh 1        # Batch 1 only (14 functions)
#   bash scripts/verify_all.sh 2        # Batch 2 only (14 functions)
#   bash scripts/verify_all.sh 3        # Batch 3 only (13 functions)
#
# For parallel execution, launch 3 separate docker exec calls:
#   docker exec vit-dev bash -c "cd /workspace/ROSCO && bash scripts/verify_all.sh 1" &
#   docker exec vit-dev bash -c "cd /workspace/ROSCO && bash scripts/verify_all.sh 2" &
#   docker exec vit-dev bash -c "cd /workspace/ROSCO && bash scripts/verify_all.sh 3" &
#   wait
#
# Prerequisites:
#   - Golden kernel fixtures exist in kernel/ (run scripts/extract_all.sh first)
#   - C++ translations exist in translations/
#   - ROSCO source must be CLEAN (no integration wrappers). Run scripts/reset_to_clean.sh first.
#     vit verify scans the source to detect translated callees for bridge generation.
#     Integrated source has _c() wrappers that break callee detection.

BATCH=${1:-0}

# --- Check: source must be clean (no integration wrappers) ---
if grep -q '_c(' rosco/controller/src/Controllers.f90 2>/dev/null; then
    echo "ERROR: Source files have integration wrappers (_c calls detected)."
    echo "Run: bash scripts/reset_to_clean.sh"
    exit 1
fi

# --- Step 0: Clean stale build artifacts from kernel directories ---
# Only remove compiler outputs (.o, .mod, .exe) and verification logs (.csv).
# Do NOT remove .h, .f90, *.cpp, or *.hpp — these are tracked golden
# fixture files that vit verify depends on, and git is not available in the
# container to restore them.
echo "Cleaning kernel build artifacts..."
for d in kernel/*/; do
    rm -f "${d}"*.o "${d}"*.mod "${d}"*.exe "${d}"*.csv 2>/dev/null
done
echo "Done."
echo ""

PASS=0
FAIL=0

verify() {
    local name=$1
    local cpp=$2
    local fortran=$3
    result=$(vit verify "$name" "$cpp" -f "$fortran" --kernel-dir "kernel/$name" 2>&1 | grep -oE '[0-9]+/[0-9]+ passed' || true)
    if [ -n "$result" ]; then
        echo "  OK $name: $result"
        PASS=$((PASS + 1))
    else
        echo "  FAIL $name"
        FAIL=$((FAIL + 1))
    fi
}

# --- Batch 1: Functions + Filters + ReadSetParameters (14 functions) ---
if [ "$BATCH" = "0" ] || [ "$BATCH" = "1" ]; then
    echo "--- Functions ---"
    verify saturate                translations/Functions/saturate.cpp                rosco/controller/src/Functions.f90
    verify wrap_180                translations/Functions/wrap_180.cpp                rosco/controller/src/Functions.f90
    verify wrap_360                translations/Functions/wrap_360.cpp                rosco/controller/src/Functions.f90
    verify ratelimit               translations/Functions/ratelimit.cpp               rosco/controller/src/Functions.f90
    verify sigma                   translations/Functions/sigma.cpp                   rosco/controller/src/Functions.f90
    verify interp1d                translations/Functions/interp1d.cpp                rosco/controller/src/Functions.f90
    verify interp2d                translations/Functions/interp2d.cpp                rosco/controller/src/Functions.f90
    verify ColemanTransform        translations/Functions/colemantransform.cpp        rosco/controller/src/Functions.f90
    verify ColemanTransformInverse translations/Functions/colemantransforminverse.cpp rosco/controller/src/Functions.f90
    verify identity                translations/Functions/identity.cpp                rosco/controller/src/Functions.f90
    echo "--- ReadSetParameters ---"
    verify ReadAvrSWAP             translations/ReadSetParameters/readavrswap.cpp     rosco/controller/src/ReadSetParameters.f90
    echo "--- Filters ---"
    verify LPFilter                translations/Filters/lpfilter.cpp                  rosco/controller/src/Filters.f90
    verify HPFilter                translations/Filters/hpfilter.cpp                  rosco/controller/src/Filters.f90
    verify SecLPFilter             translations/Filters/seclpfilter.cpp               rosco/controller/src/Filters.f90
fi

# --- Batch 2: Filters (cont) + Controllers (14 functions) ---
if [ "$BATCH" = "0" ] || [ "$BATCH" = "2" ]; then
    echo "--- Filters (cont) ---"
    verify NotchFilter       translations/Filters/notchfilter.cpp       rosco/controller/src/Filters.f90
    verify NotchFilterSlopes translations/Filters/notchfilterslopes.cpp rosco/controller/src/Filters.f90
    verify SecLPFilter_Vel   translations/Filters/seclpfilter_vel.cpp   rosco/controller/src/Filters.f90
    echo "--- Controllers ---"
    verify PIController        translations/Controllers/picontroller.cpp        rosco/controller/src/Controllers.f90
    verify PIIController       translations/Controllers/piicontroller.cpp       rosco/controller/src/Controllers.f90
    verify ResController       translations/Controllers/rescontroller.cpp       rosco/controller/src/Controllers.f90
    verify ForeAftDamping      translations/Controllers/foreaftdamping.cpp      rosco/controller/src/Controllers.f90
    verify FloatingFeedback    translations/Controllers/floatingfeedback.cpp    rosco/controller/src/Controllers.f90
    verify StructuralControl   translations/Controllers/structuralcontrol.cpp   rosco/controller/src/Controllers.f90
    verify CableControl        translations/Controllers/cablecontrol.cpp        rosco/controller/src/Controllers.f90
    verify FlapControl         translations/Controllers/flapcontrol.cpp         rosco/controller/src/Controllers.f90
    verify YawRateControl      translations/Controllers/yawratecontrol.cpp      rosco/controller/src/Controllers.f90
    verify VariableSpeedControl translations/Controllers/variablespeedcontrol.cpp rosco/controller/src/Controllers.f90
    verify PIDController       translations/Controllers/pidcontroller.cpp       rosco/controller/src/Controllers.f90
fi

# --- Batch 3: Controllers (cont) + ControllerBlocks (13 functions) ---
if [ "$BATCH" = "0" ] || [ "$BATCH" = "3" ]; then
    echo "--- Controllers (cont) ---"
    verify IPC                 translations/Controllers/ipc.cpp                 rosco/controller/src/Controllers.f90
    verify ActiveWakeControl   translations/Controllers/activewakecontrol.cpp   rosco/controller/src/Controllers.f90
    verify PitchControl        translations/Controllers/pitchcontrol.cpp        rosco/controller/src/Controllers.f90
    echo "--- ControllerBlocks ---"
    verify PitchSaturation     translations/ControllerBlocks/pitchsaturation.cpp     rosco/controller/src/ControllerBlocks.f90
    verify AeroDynTorque       translations/Functions/aerodyntorque.cpp               rosco/controller/src/Functions.f90
    verify StateMachine        translations/ControllerBlocks/statemachine.cpp         rosco/controller/src/ControllerBlocks.f90
    verify SetpointSmoother    translations/ControllerBlocks/setpointsmoother.cpp     rosco/controller/src/ControllerBlocks.f90
    verify PowerControlSetpoints translations/ControllerBlocks/powercontrolsetpoints.cpp rosco/controller/src/ControllerBlocks.f90
    verify RefSpeedExclusion   translations/ControllerBlocks/refspeedexclusion.cpp    rosco/controller/src/ControllerBlocks.f90
    verify ComputeVariablesSetpoints translations/ControllerBlocks/computevariablessetpoints.cpp rosco/controller/src/ControllerBlocks.f90
    verify Shutdown            translations/ControllerBlocks/shutdown.cpp              rosco/controller/src/ControllerBlocks.f90
    verify Startup             translations/ControllerBlocks/startup.cpp               rosco/controller/src/ControllerBlocks.f90
    verify WindSpeedEstimator  translations/ControllerBlocks/windspeedestimator.cpp   rosco/controller/src/ControllerBlocks.f90
fi

echo ""
if [ "$BATCH" = "0" ]; then
    TOTAL=41
else
    TOTAL=$((PASS + FAIL))
fi
echo "=== Verification complete: $PASS/$TOTAL passed, $FAIL failed ==="

if [ $FAIL -gt 0 ]; then
    exit 1
fi
