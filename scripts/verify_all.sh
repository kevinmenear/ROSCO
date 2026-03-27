#!/bin/bash
# Verify all 28 C++ translations against golden kernel fixtures.
# Run from the ROSCO repo root inside the Docker container.
#
# Usage: bash scripts/verify_all.sh
#
# Prerequisites:
#   - Golden kernel fixtures exist in kernel/ (run scripts/extract_all.sh first)
#   - C++ translations exist in translations/

PASS=0
FAIL=0
TOTAL=28

verify() {
    local name=$1
    local cpp=$2
    local fortran=$3
    result=$(vit verify "$name" "$cpp" -f "$fortran" --kernel-dir "kernel/$name" 2>&1 | grep -oE '[0-9]+/[0-9]+ passed' || true)
    if [ -n "$result" ]; then
        echo "  OK $name: $result"
        PASS=$((PASS + 1))
    else
        error=$(vit verify "$name" "$cpp" -f "$fortran" --kernel-dir "kernel/$name" 2>&1 | tail -3)
        echo "FAIL $name"
        echo "     $error"
        FAIL=$((FAIL + 1))
    fi
}

echo "=== Verifying 28 functions ==="
echo ""

# Functions
echo "--- Functions ---"
verify saturate                translations/Functions/saturate.cpp                rosco/controller/src/Functions.f90
verify wrap_180                translations/Functions/wrap_180.cpp                rosco/controller/src/Functions.f90
verify wrap_360                translations/Functions/wrap_360.cpp                rosco/controller/src/Functions.f90
verify ratelimit               translations/Functions/ratelimit.cpp               rosco/controller/src/Functions.f90
verify sigma                   translations/Functions/sigma.cpp                   rosco/controller/src/Functions.f90
verify interp1d                translations/Functions/interp1d.cpp                rosco/controller/src/Functions.f90
verify ColemanTransform        translations/Functions/colemantransform.cpp        rosco/controller/src/Functions.f90
verify ColemanTransformInverse translations/Functions/colemantransforminverse.cpp rosco/controller/src/Functions.f90
verify identity                translations/Functions/identity.cpp                rosco/controller/src/Functions.f90

# Filters
echo "--- Filters ---"
verify LPFilter          translations/Filters/lpfilter.cpp          rosco/controller/src/Filters.f90
verify HPFilter          translations/Filters/hpfilter.cpp          rosco/controller/src/Filters.f90
verify SecLPFilter       translations/Filters/seclpfilter.cpp       rosco/controller/src/Filters.f90
verify NotchFilter       translations/Filters/notchfilter.cpp       rosco/controller/src/Filters.f90
verify NotchFilterSlopes translations/Filters/notchfilterslopes.cpp rosco/controller/src/Filters.f90
verify SecLPFilter_Vel   translations/Filters/seclpfilter_vel.cpp   rosco/controller/src/Filters.f90

# Controllers
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
verify IPC                 translations/Controllers/ipc.cpp                 rosco/controller/src/Controllers.f90
verify ActiveWakeControl   translations/Controllers/activewakecontrol.cpp   rosco/controller/src/Controllers.f90
verify PitchControl        translations/Controllers/pitchcontrol.cpp        rosco/controller/src/Controllers.f90

echo ""
echo "=== Verification complete: $PASS/$TOTAL passed, $FAIL failed ==="

if [ $FAIL -gt 0 ]; then
    exit 1
fi
