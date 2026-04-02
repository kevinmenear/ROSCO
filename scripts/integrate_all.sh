#!/bin/bash
# Integrate all 45 C++ translations into the ROSCO codebase.
# (39 algorithm functions from Phases 1-9, ReadAvrSWAP + PIDController from Phase 10A,
#  unwrap, and 3 Stage B functions: CheckInputs, ReadCpFile, ReadControlParameterFileSub)
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
TOTAL=45

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

echo "=== Integrating $TOTAL functions ==="
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
integrate interp2d                translations/Functions/interp2d.cpp                rosco/controller/src/Functions.f90
integrate AeroDynTorque           translations/Functions/aerodyntorque.cpp           rosco/controller/src/Functions.f90
integrate unwrap                 translations/Functions/unwrap.cpp                 rosco/controller/src/Functions.f90

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

# ControllerBlocks (order matters: callees before callers)
echo "--- ControllerBlocks ---"
integrate PitchSaturation           translations/ControllerBlocks/pitchsaturation.cpp           rosco/controller/src/ControllerBlocks.f90
integrate StateMachine              translations/ControllerBlocks/statemachine.cpp              rosco/controller/src/ControllerBlocks.f90
integrate SetpointSmoother          translations/ControllerBlocks/setpointsmoother.cpp          rosco/controller/src/ControllerBlocks.f90
integrate PowerControlSetpoints     translations/ControllerBlocks/powercontrolsetpoints.cpp     rosco/controller/src/ControllerBlocks.f90
integrate RefSpeedExclusion         translations/ControllerBlocks/refspeedexclusion.cpp         rosco/controller/src/ControllerBlocks.f90
integrate ComputeVariablesSetpoints translations/ControllerBlocks/computevariablessetpoints.cpp rosco/controller/src/ControllerBlocks.f90
integrate Shutdown                  translations/ControllerBlocks/shutdown.cpp                  rosco/controller/src/ControllerBlocks.f90
integrate Startup                   translations/ControllerBlocks/startup.cpp                   rosco/controller/src/ControllerBlocks.f90
integrate WindSpeedEstimator        translations/ControllerBlocks/windspeedestimator.cpp        rosco/controller/src/ControllerBlocks.f90

integrate PIDController       translations/Controllers/pidcontroller.cpp       rosco/controller/src/Controllers.f90

# PitchControl (depends on PitchSaturation above)
echo "--- PitchControl ---"
integrate PitchControl       translations/Controllers/pitchcontrol.cpp        rosco/controller/src/Controllers.f90

# ReadSetParameters
echo "--- ReadSetParameters ---"
integrate ReadAvrSWAP        translations/ReadSetParameters/readavrswap.cpp   rosco/controller/src/ReadSetParameters.f90

# Stage B manual integration: CheckInputs (pure validation, no KGen)
# Replace CheckInputs body with C++ wrapper. The C++ translation is in
# rosco/controller/src/checkinputs.cpp (copied from translations/ReadSetParameters/).
echo "--- Stage B: CheckInputs (manual) ---"
cp translations/ReadSetParameters/checkinputs.cpp rosco/controller/src/checkinputs.cpp
python3 -c "
import re, sys
with open('rosco/controller/src/ReadSetParameters.f90', 'r') as f:
    content = f.read()
# Find the CheckInputs subroutine body and replace with wrapper
pattern = r'(    SUBROUTINE CheckInputs\(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG\)).*?(    END SUBROUTINE CheckInputs)'
wrapper = '''    SUBROUTINE CheckInputs(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters

        IMPLICIT NONE

        ! Inputs
        TYPE(ControlParameters),    INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT), TARGET :: LocalVar
        TYPE(ErrorVariables),       INTENT(INOUT), TARGET :: ErrVar
        INTEGER(IntKi),             INTENT(IN   )         :: size_avcMSG
        REAL(ReKi),                 INTENT(IN   )         :: avrSWAP(*)

        ! VIT: C++ wrapper
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view

        INTERFACE
            SUBROUTINE checkinputs_c(LocalVar, CntrPar, avrSWAP, ErrVar, size_avcMSG) BIND(C, NAME='checkinputs_c')
                USE ISO_C_BINDING
                TYPE(C_PTR), VALUE :: LocalVar
                TYPE(C_PTR), VALUE :: CntrPar
                REAL(C_FLOAT), INTENT(IN) :: avrSWAP(*)
                TYPE(C_PTR), VALUE :: ErrVar
                INTEGER(C_INT32_T), VALUE :: size_avcMSG
            END SUBROUTINE checkinputs_c
        END INTERFACE

        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL checkinputs_c(C_LOC(LocalVar), C_LOC(CntrPar_view), avrSWAP, C_LOC(ErrVar), INT(size_avcMSG, C_INT32_T))

    END SUBROUTINE CheckInputs'''
result = re.sub(pattern, wrapper, content, flags=re.DOTALL)
if result == content:
    print('FAIL CheckInputs: pattern not found in ReadSetParameters.f90')
    sys.exit(1)
with open('rosco/controller/src/ReadSetParameters.f90', 'w') as f:
    f.write(result)
print('  OK CheckInputs')
" 2>&1
if [ $? -eq 0 ]; then
    PASS=$((PASS + 1))
else
    FAIL=$((FAIL + 1))
fi

# Stage B manual integration: ReadCpFile (performance table reader, no KGen)
# Replace ReadCpFile body with C++ wrapper that pre-allocates arrays.
echo "--- Stage B: ReadCpFile (manual) ---"
cp translations/ReadSetParameters/readcpfile.cpp rosco/controller/src/readcpfile.cpp
python3 -c "
import re, sys
with open('rosco/controller/src/ReadSetParameters.f90', 'r') as f:
    content = f.read()
pattern = r'(    SUBROUTINE ReadCpFile\(CntrPar\s*,\s*PerfData\s*,\s*ErrVar\)).*?(    END SUBROUTINE ReadCpFile)'
wrapper = '''    SUBROUTINE ReadCpFile(CntrPar, PerfData, ErrVar)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : PerformanceData, ControlParameters, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        USE vit_performancedata_view, ONLY: performancedata_view_t, vit_populate_performancedata

        IMPLICIT NONE

        TYPE(ControlParameters),    INTENT(INOUT), TARGET :: CntrPar
        TYPE(PerformanceData),      INTENT(INOUT), TARGET :: PerfData
        TYPE(ErrorVariables),       INTENT(INOUT), TARGET :: ErrVar

        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(performancedata_view_t), TARGET :: PerfData_view

        INTERFACE
            SUBROUTINE readcpfile_c(CntrPar, PerfData, ErrVar) BIND(C, NAME=\\\"readcpfile_c\\\")
                USE ISO_C_BINDING
                TYPE(C_PTR), VALUE :: CntrPar
                TYPE(C_PTR), VALUE :: PerfData
                TYPE(C_PTR), VALUE :: ErrVar
            END SUBROUTINE readcpfile_c
        END INTERFACE

        ALLOCATE(PerfData%Beta_vec(CntrPar%PerfTableSize(1)))
        ALLOCATE(PerfData%TSR_vec(CntrPar%PerfTableSize(2)))
        ALLOCATE(PerfData%Cp_mat(CntrPar%PerfTableSize(2), CntrPar%PerfTableSize(1)))
        ALLOCATE(PerfData%Ct_mat(CntrPar%PerfTableSize(2), CntrPar%PerfTableSize(1)))
        ALLOCATE(PerfData%Cq_mat(CntrPar%PerfTableSize(2), CntrPar%PerfTableSize(1)))

        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_performancedata(PerfData, PerfData_view)

        CALL readcpfile_c(C_LOC(CntrPar_view), C_LOC(PerfData_view), C_LOC(ErrVar))

    END SUBROUTINE ReadCpFile'''
result = re.sub(pattern, wrapper, content, flags=re.DOTALL)
if result == content:
    print('FAIL ReadCpFile: pattern not found in ReadSetParameters.f90')
    sys.exit(1)
with open('rosco/controller/src/ReadSetParameters.f90', 'w') as f:
    f.write(result)
print('  OK ReadCpFile')
" 2>&1
if [ $? -eq 0 ]; then
    PASS=$((PASS + 1))
else
    FAIL=$((FAIL + 1))
fi

# Stage B manual integration: ReadControlParameterFileSub (config parser, no KGen)
# Replace entire ReadControlParameterFileSub with two-pass C++ wrapper.
# Wrapper template stored in translations/ (too large to embed in shell script).
echo "--- Stage B: ReadControlParameterFileSub (manual) ---"
cp translations/ReadSetParameters/readcontrolparameterfilesub.cpp rosco/controller/src/readcontrolparameterfilesub.cpp
python3 -c "
import re, sys
with open('rosco/controller/src/ReadSetParameters.f90', 'r') as f:
    content = f.read()
with open('translations/ReadSetParameters/readcontrolparameterfilesub_wrapper.f90', 'r') as f:
    wrapper = f.read()
pattern = r'(    SUBROUTINE ReadControlParameterFileSub\(CntrPar, LocalVar, accINFILE, accINFILE_size, RootName, ErrVar\)).*?(    END SUBROUTINE ReadControlParameterFileSub)'
result = re.sub(pattern, wrapper.rstrip(), content, flags=re.DOTALL)
if result == content:
    print('FAIL ReadControlParameterFileSub: pattern not found in ReadSetParameters.f90')
    sys.exit(1)
with open('rosco/controller/src/ReadSetParameters.f90', 'w') as f:
    f.write(result)
print('  OK ReadControlParameterFileSub')
" 2>&1
if [ $? -eq 0 ]; then
    PASS=$((PASS + 1))
else
    FAIL=$((FAIL + 1))
fi

echo ""
echo "=== Integration complete: $PASS/$TOTAL passed, $FAIL failed ==="

if [ $FAIL -gt 0 ]; then
    exit 1
fi
