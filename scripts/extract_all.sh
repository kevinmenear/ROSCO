#!/bin/bash
# Extract all 40 functions sequentially.
# Each extraction uses the specific scenario needed for that function's call site.
# Run from the ROSCO repo root inside the Docker container.
#
# Usage: bash scripts/extract_all.sh
#
# Prerequisites:
#   - Clean source (bash scripts/reset_to_clean.sh)
#   - Clean build (cd rosco/controller/build && rm -rf * && cmake .. -DCMAKE_INSTALL_PREFIX=install && cmake --build . && cp libdiscon.so ../../lib/libdiscon.so)
#   - No stale kernel/ directory (rm -rf kernel/)

set -e

PASS=0
FAIL=0
TOTAL=40

extract() {
    local name=$1; shift
    rm -rf model/ model.ini kgen.log elapsedtime/ state/
    output=$(vit extract "$@" 2>&1)
    cases=$(echo "$output" | grep -oP 'Cases:\s+\K\d+' || true)
    error=$(echo "$output" | grep -iE 'ERROR|FAILED|exception' | head -1 || true)
    if [ -n "$error" ]; then
        echo "FAIL $name: $error"
        FAIL=$((FAIL + 1))
    elif [ -n "$cases" ] && [ "$cases" -gt 0 ]; then
        echo "  OK $name: $cases cases"
        PASS=$((PASS + 1))
    else
        echo "FAIL $name: 0 cases captured"
        FAIL=$((FAIL + 1))
    fi
}

echo "=== Extracting 40 functions ==="
echo ""

# Functions (Scenario 3)
extract saturate       saturate -f rosco/controller/src/Controllers.f90 -l 102 --run-args '--scenario 3'
extract wrap_180       wrap_180 -f rosco/controller/src/Filters.f90 -l 432 --run-args '--scenario 3'
extract wrap_360       wrap_360 -f rosco/controller/src/Controllers.f90 -l 517 --run-args '--scenario 2'
extract ratelimit      ratelimit -f rosco/controller/src/Controllers.f90 -l 103 --run-args '--scenario 3'
extract ColemanTransform ColemanTransform -f rosco/controller/src/Controllers.f90 -l 510 --run-args '--scenario 8'
extract ColemanTransformInverse ColemanTransformInverse -f rosco/controller/src/Controllers.f90 -l 561 --run-args '--scenario 8'
extract identity       identity -f rosco/controller/src/ControllerBlocks.f90 -l 454 --run-args '--scenario 3'
extract sigma          sigma -f rosco/controller/src/Controllers.f90 -l 528 --run-args '--scenario 8'
extract interp1d       interp1d -f rosco/controller/src/Controllers.f90 -l 62 --run-args '--scenario 3'

# Filters
extract LPFilter       LPFilter -f rosco/controller/src/Filters.f90 -l 347 --run-args '--scenario 3'
extract HPFilter       HPFilter -f rosco/controller/src/Filters.f90 -l 375 --run-args '--scenario 3'
extract SecLPFilter    SecLPFilter -f rosco/controller/src/Filters.f90 -l 371 --run-args '--scenario 3'
extract NotchFilter    NotchFilter -f rosco/controller/src/Filters.f90 -l 356 --run-args '--scenario 3'
extract NotchFilterSlopes NotchFilterSlopes -f rosco/controller/src/Filters.f90 -l 405 --run-args '--scenario 6'
extract SecLPFilter_Vel SecLPFilter_Vel -f rosco/controller/src/Controllers.f90 -l 941 --run-args '--scenario 3'

# Controllers (Scenario 3)
extract PIController   PIController -f rosco/controller/src/Controllers.f90 -l 68 --run-args '--scenario 3'
extract PIIController  PIIController -f rosco/controller/src/Controllers.f90 -l 682 --run-args '--scenario 4'
extract ResController  ResController -f rosco/controller/src/Controllers.f90 -l 813 --run-args '--scenario 8'

# Controllers (Scenario 7 — synthetic non-zero inputs)
extract ForeAftDamping ForeAftDamping -f rosco/controller/src/Controllers.f90 -l 80 --run-args '--scenario 7'
extract FloatingFeedback FloatingFeedback -f rosco/controller/src/Controllers.f90 -l 96 --run-args '--scenario 7'

# Controllers (Scenario 8 — IPC + AWC with blade moments)
extract IPC            IPC -f rosco/controller/src/Controllers.f90 -l 73 --run-args '--scenario 8'
extract ActiveWakeControl ActiveWakeControl -f rosco/controller/src/Controllers.f90 -l 144 --run-args '--scenario 8'

# DISCON.F90 (Scenario 3)
extract VariableSpeedControl VariableSpeedControl -f rosco/controller/src/DISCON.F90 -l 117 --run-args '--scenario 3'
extract PitchControl   PitchControl -f rosco/controller/src/DISCON.F90 -l 119 --run-args '--scenario 3'

# DISCON.F90 (Scenario 7 — synthetic non-zero inputs)
extract YawRateControl YawRateControl -f rosco/controller/src/DISCON.F90 -l 123 --run-args '--scenario 7'
extract FlapControl    FlapControl -f rosco/controller/src/DISCON.F90 -l 127 --run-args '--scenario 7'
extract CableControl   CableControl -f rosco/controller/src/DISCON.F90 -l 132 --run-args '--scenario 7'
extract StructuralControl StructuralControl -f rosco/controller/src/DISCON.F90 -l 137 --run-args '--scenario 7'

# ControllerBlocks (Scenario 3)
extract PitchSaturation PitchSaturation -f rosco/controller/src/Controllers.f90 -l 87 --run-args '--scenario 3'
extract interp2d       interp2d -f rosco/controller/src/ControllerBlocks.f90 -l 425 --run-args '--scenario 3'
extract AeroDynTorque  AeroDynTorque -f rosco/controller/src/ControllerBlocks.f90 -l 389 --run-args '--scenario 3'
extract RefSpeedExclusion RefSpeedExclusion -f rosco/controller/src/ControllerBlocks.f90 -l 157 --run-args '--scenario 3'

# DISCON.F90 ControllerBlocks (Scenario 3)
extract WindSpeedEstimator WindSpeedEstimator -f rosco/controller/src/DISCON.F90 -l 109 --run-args '--scenario 3'
extract PowerControlSetpoints PowerControlSetpoints -f rosco/controller/src/DISCON.F90 -l 110 --run-args '--scenario 3'
extract ComputeVariablesSetpoints ComputeVariablesSetpoints -f rosco/controller/src/DISCON.F90 -l 114 --run-args '--scenario 3'
extract StateMachine   StateMachine -f rosco/controller/src/DISCON.F90 -l 115 --run-args '--scenario 3'
extract SetpointSmoother SetpointSmoother -f rosco/controller/src/DISCON.F90 -l 116 --run-args '--scenario 3'

# DISCON.F90 ControllerBlocks (Scenario 9 — SD_Mode=1, SU_Mode=1)
extract Shutdown       Shutdown -f rosco/controller/src/DISCON.F90 -l 107 --run-args '--scenario 9'
extract Startup        Startup -f rosco/controller/src/DISCON.F90 -l 112 --run-args '--scenario 9'

# ReadSetParameters (Scenario 3)
# Note: ReadAvrSWAP extraction requires patching ROSCO_Helpers.f90 to work around
# KGen's parser failing on backslash characters in PathIsRelative/GetPath functions.
# ReadSetParameters.f90 USEs ROSCO_Helpers, triggering KGen to parse the file.
echo "--- ReadSetParameters (patching ROSCO_Helpers.f90 for KGen) ---"
cp rosco/controller/src/ROSCO_Helpers.f90 rosco/controller/src/ROSCO_Helpers.f90.bak
sed -i "s|INDEX( GivenFil, '\\\\\\\\', BACK=.TRUE. )|INDEX( GivenFil, '/', BACK=.TRUE. )|" rosco/controller/src/ROSCO_Helpers.f90
sed -i 's|INDEX( GivenFil, ":\\\\"|INDEX( GivenFil, ":/"|' rosco/controller/src/ROSCO_Helpers.f90
sed -i 's|INDEX( "/\\\\"|INDEX( "//"|' rosco/controller/src/ROSCO_Helpers.f90
extract ReadAvrSWAP    ReadAvrSWAP -f rosco/controller/src/DISCON.F90 -l 81 --run-args '--scenario 3'
cp rosco/controller/src/ROSCO_Helpers.f90.bak rosco/controller/src/ROSCO_Helpers.f90
rm rosco/controller/src/ROSCO_Helpers.f90.bak

echo ""
echo "=== Extraction complete: $PASS/$TOTAL passed, $FAIL failed ==="

if [ $FAIL -gt 0 ]; then
    exit 1
fi
