#!/bin/bash
# Integrate all 29 C++ translations into the ROSCO codebase.
# Run from the ROSCO repo root inside the Docker container.
#
# Usage: bash scripts/integrate_all.sh
#
# Prerequisites:
#   - Clean source (bash scripts/reset_to_clean.sh)
#   - Translation files in translations/

set -e

PASS=0
FAIL=0
TOTAL=29

integrate() {
    local name=$1
    local cpp=$2
    local fortran=$3
    result=$(vit integrate "$name" "$cpp" -f "$fortran" --apply 2>&1)
    if echo "$result" | grep -q "Integration applied successfully"; then
        echo "  OK $name"
        PASS=$((PASS + 1))
    else
        error=$(echo "$result" | tail -1)
        echo "FAIL $name: $error"
        FAIL=$((FAIL + 1))
    fi
}

echo "=== Integrating 29 functions ==="
echo ""

# Functions
echo "--- Functions ---"
integrate saturate                translations/Functions/saturate.cpp                rosco/controller/src/Functions.f90
integrate wrap_180                translations/Functions/wrap_180.cpp                rosco/controller/src/Functions.f90
integrate wrap_360                translations/Functions/wrap_360.cpp                rosco/controller/src/Functions.f90
integrate ratelimit               translations/Functions/ratelimit.cpp               rosco/controller/src/Functions.f90
integrate ColemanTransform        translations/Functions/colemantransform.cpp        rosco/controller/src/Functions.f90
integrate ColemanTransformInverse translations/Functions/colemantransforminverse.cpp rosco/controller/src/Functions.f90
integrate identity                translations/Functions/identity.cpp                rosco/controller/src/Functions.f90
integrate sigma                   translations/Functions/sigma.cpp                   rosco/controller/src/Functions.f90
integrate interp1d                translations/Functions/interp1d.cpp                rosco/controller/src/Functions.f90

# Filters
echo "--- Filters ---"
integrate LPFilter          translations/Filters/lpfilter.cpp          rosco/controller/src/Filters.f90
integrate HPFilter          translations/Filters/hpfilter.cpp          rosco/controller/src/Filters.f90
integrate SecLPFilter       translations/Filters/seclpfilter.cpp       rosco/controller/src/Filters.f90
integrate SecLPFilter_Vel   translations/Filters/seclpfilter_vel.cpp   rosco/controller/src/Filters.f90
integrate NotchFilter       translations/Filters/notchfilter.cpp       rosco/controller/src/Filters.f90
integrate NotchFilterSlopes translations/Filters/notchfilterslopes.cpp rosco/controller/src/Filters.f90

# Controllers
echo "--- Controllers ---"
integrate PIController        translations/Controllers/picontroller.cpp        rosco/controller/src/Controllers.f90
integrate PIIController       translations/Controllers/piicontroller.cpp       rosco/controller/src/Controllers.f90
integrate ResController       translations/Controllers/rescontroller.cpp       rosco/controller/src/Controllers.f90
integrate ForeAftDamping      translations/Controllers/foreaftdamping.cpp      rosco/controller/src/Controllers.f90
integrate FloatingFeedback    translations/Controllers/floatingfeedback.cpp    rosco/controller/src/Controllers.f90
integrate StructuralControl   translations/Controllers/structuralcontrol.cpp   rosco/controller/src/Controllers.f90
integrate CableControl        translations/Controllers/cablecontrol.cpp        rosco/controller/src/Controllers.f90
integrate FlapControl         translations/Controllers/flapcontrol.cpp         rosco/controller/src/Controllers.f90
integrate YawRateControl      translations/Controllers/yawratecontrol.cpp      rosco/controller/src/Controllers.f90
integrate VariableSpeedControl translations/Controllers/variablespeedcontrol.cpp rosco/controller/src/Controllers.f90
integrate IPC                 translations/Controllers/ipc.cpp                 rosco/controller/src/Controllers.f90
integrate ActiveWakeControl   translations/Controllers/activewakecontrol.cpp   rosco/controller/src/Controllers.f90

# ControllerBlocks
echo "--- ControllerBlocks ---"
integrate PitchSaturation    translations/ControllerBlocks/pitchsaturation.cpp rosco/controller/src/ControllerBlocks.f90

# PitchControl (depends on PitchSaturation above)
echo "--- PitchControl ---"
integrate PitchControl       translations/Controllers/pitchcontrol.cpp        rosco/controller/src/Controllers.f90

echo ""
echo "=== Integration complete: $PASS/$TOTAL passed, $FAIL failed ==="

if [ $FAIL -gt 0 ]; then
    exit 1
fi
