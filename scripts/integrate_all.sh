#!/bin/bash
# Integrate all 52 C++ translations + DISCON entry point + Fortran removal.
# (39 algorithm functions from Phases 1-9, ReadAvrSWAP + PIDController from Phase 10A,
#  unwrap, 3 Stage B functions, 2 Stage D functions, 3 Stage C functions,
#  2 Stage E functions, DISCON entry point from Phase 11A,
#  and Fortran removal from Phase 11B)
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
TOTAL=54

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
integrate NotchFilterSlopes         translations/Filters/notchfilterslopes.cpp         rosco/controller/src/Filters.f90
integrate PreFilterMeasuredSignals  translations/Filters/prefiltermeasuredsignals.cpp  rosco/controller/src/Filters.f90

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

# Stage D: Platform modules (manual wrappers — #ifdef and conditional ALLOCATE)
echo "--- Stage D: UpdateZeroMQ (manual) ---"
# ZeroMQInterface.f90 has #ifdef preprocessor directives that break VIT's parser.
# Use pre-built wrapper directly.
cp translations/IO/zmq_interface.cpp rosco/controller/src/updatezeromq.cpp
python3 -c "
import sys
wrapper = '''module ZeroMQInterface
   USE, INTRINSIC :: ISO_C_BINDING
   IMPLICIT NONE

    ! Auto-generated interface for C++ implementation of UpdateZeroMQ
    INTERFACE
        SUBROUTINE updatezeromq_c(LocalVar, CntrPar, ErrVar) BIND(C, NAME=\\\"updatezeromq_c\\\")
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE updatezeromq_c
    END INTERFACE

CONTAINS
    SUBROUTINE UpdateZeroMQ(LocalVar, CntrPar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
        IMPLICIT NONE
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view struct from Fortran type
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL updatezeromq_c(C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(ErrVar))
    END SUBROUTINE UpdateZeroMQ
end module ZeroMQInterface
'''
with open('rosco/controller/src/ZeroMQInterface.f90', 'w') as f:
    f.write(wrapper)
print('  OK UpdateZeroMQ')
" 2>&1
if [ $? -eq 0 ]; then
    PASS=$((PASS + 1))
else
    FAIL=$((FAIL + 1))
fi

echo "--- Stage D: ExtController ---"
# ExtController needs a guarded ALLOCATE before the view populate.
integrate ExtController translations/IO/ext_control.cpp rosco/controller/src/ExtControl.f90
# Add guarded ALLOCATE for ExtDLL%avrSWAP (conditional, not auto-allocatable)
python3 -c "
with open('rosco/controller/src/ExtControl.f90', 'r') as f:
    content = f.read()
content = content.replace(
    '        ! Populate view structs from Fortran types\n        CALL vit_populate_controlparameters',
    '        ! Pre-allocate ExtDLL%avrSWAP if not yet allocated (first call)\n        IF (.NOT. ALLOCATED(ExtDLL%avrSWAP)) ALLOCATE(ExtDLL%avrSWAP(2000))\n        ! Populate view structs from Fortran types\n        CALL vit_populate_controlparameters'
)
with open('rosco/controller/src/ExtControl.f90', 'w') as f:
    f.write(content)
" 2>&1

# Stage C: ROSCO_IO — restart I/O + debug logger
# WriteRestartFile and Debug use vit integrate (CHARACTER(variable) now supported).
# ReadRestartFile stays manual (needs callee dispatch for ReadControlParameterFileSub/ReadCpFile).
# Copy shared restart header (needed by both writerestartfile.cpp and readrestartfile.cpp)
cp translations/IO/restart_fields.h rosco/controller/src/restart_fields.h

echo "--- Stage C: WriteRestartFile ---"
integrate WriteRestartFile  translations/IO/writerestartfile.cpp  rosco/controller/src/ROSCO_IO.f90

echo "--- Stage C: Debug ---"
integrate Debug             translations/IO/debug_logger.cpp      rosco/controller/src/ROSCO_IO.f90

echo "--- Stage C: ReadRestartFile (manual — callee dispatch) ---"
cp translations/IO/readrestartfile.cpp rosco/controller/src/readrestartfile.cpp
python3 -c "
import re, sys

with open('rosco/controller/src/ROSCO_IO.f90', 'r') as f:
    content = f.read()

read_wrapper = '''SUBROUTINE ReadRestartFile(avrSWAP, LocalVar, CntrPar, objInst, PerfData, RootName, size_avcOUTNAME, ErrVar)
    USE, INTRINSIC :: ISO_C_Binding
    USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables, ObjectInstances, PerformanceData
    USE ReadSetParameters, ONLY : ReadControlParameterFileSub, ReadCpFile
    USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters
    USE vit_performancedata_view, ONLY: performancedata_view_t, vit_populate_performancedata
    IMPLICIT NONE

    REAL(ReKi), INTENT(IN)                         :: avrSWAP(*)
    TYPE(LocalVariables), INTENT(INOUT), TARGET    :: LocalVar
    TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
    TYPE(ObjectInstances), INTENT(INOUT), TARGET   :: objInst
    TYPE(PerformanceData), INTENT(INOUT), TARGET   :: PerfData
    TYPE(ErrorVariables), INTENT(INOUT), TARGET    :: ErrVar
    INTEGER(IntKi), INTENT(IN)                     :: size_avcOUTNAME
    CHARACTER(size_avcOUTNAME), INTENT(IN)         :: RootName

    TYPE(controlparameters_view_t), TARGET :: CntrPar_view
    TYPE(performancedata_view_t), TARGET :: PerfData_view
    CHARACTER(KIND=C_CHAR) :: RootName_c(size_avcOUTNAME)
    INTEGER :: i

    INTERFACE
        SUBROUTINE readrestartfile_c(avrSWAP, LocalVar, CntrPar, objInst, PerfData, RootName, size_avcOUTNAME, ErrVar) &
                BIND(C, NAME=\\\"readrestartfile_c\\\")
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(IN) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: PerfData
            CHARACTER(KIND=C_CHAR), INTENT(IN) :: RootName(*)
            INTEGER(C_INT), VALUE :: size_avcOUTNAME
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE readrestartfile_c
    END INTERFACE

    DO i = 1, size_avcOUTNAME
        RootName_c(i) = RootName(i:i)
    END DO

    CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
    CALL vit_populate_performancedata(PerfData, PerfData_view)
    CALL readrestartfile_c(avrSWAP, C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(objInst), &
                           C_LOC(PerfData_view), RootName_c, INT(size_avcOUTNAME, C_INT), C_LOC(ErrVar))

    ! Read parameter files (same as original Fortran)
    CALL ReadControlParameterFileSub(CntrPar, LocalVar, LocalVar%ACC_INFILE, LocalVar%ACC_INFILE_SIZE, RootName, ErrVar)
    IF (CntrPar%WE_Mode > 0) THEN
        CALL ReadCpFile(CntrPar, PerfData, ErrVar)
    ENDIF
END SUBROUTINE ReadRestartFile'''

pattern = r'SUBROUTINE ReadRestartFile\(avrSWAP, LocalVar, CntrPar, objInst, PerfData, RootName, size_avcOUTNAME, ErrVar\).*?END SUBROUTINE ReadRestartFile'
result = re.sub(pattern, read_wrapper, content, flags=re.DOTALL)
if result == content:
    print('FAIL ReadRestartFile: pattern not found')
    sys.exit(1)
with open('rosco/controller/src/ROSCO_IO.f90', 'w') as f:
    f.write(result)
print('  OK ReadRestartFile')
" 2>&1
if [ $? -eq 0 ]; then
    PASS=$((PASS + 1))
else
    FAIL=$((FAIL + 1))
fi

# Add readrestartfile.cpp to CMakeLists.txt if not already present
python3 -c "
with open('rosco/controller/CMakeLists.txt', 'r') as f:
    content = f.read()
if 'readrestartfile.cpp' not in content:
    content = content.replace('src/updatezeromq.cpp', 'src/updatezeromq.cpp\n    src/readrestartfile.cpp')
    with open('rosco/controller/CMakeLists.txt', 'w') as f:
        f.write(content)
"

# Stage E: SetParameters (manual — callee dispatch for ReadControlParameterFileSub/ReadCpFile)
# Per-timestep init + banner + file reading stay in Fortran wrapper.
# C++ handles LocalVar initialization, CheckInputs, OL_Index.
echo "--- Stage E: SetParameters (manual) ---"
cp translations/ReadSetParameters/setparameters.cpp rosco/controller/src/setparameters.cpp
python3 -c "
import re, sys
with open('rosco/controller/src/ReadSetParameters.f90', 'r') as f:
    content = f.read()
pattern = r'(    SUBROUTINE SetParameters\(avrSWAP, accINFILE, size_avcMSG, CntrPar, LocalVar, objInst, PerfData, RootName, ErrVar\)).*?(    END SUBROUTINE SetParameters)'
wrapper = '''    SUBROUTINE SetParameters(avrSWAP, accINFILE, size_avcMSG, CntrPar, LocalVar, objInst, PerfData, RootName, ErrVar)
        USE, INTRINSIC :: ISO_C_Binding
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, PerformanceData, ErrorVariables, rosco_version
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters

        IMPLICIT NONE

        REAL(ReKi),                 INTENT(INOUT), TARGET :: avrSWAP(*)
        CHARACTER(C_CHAR),          INTENT(IN   )         :: accINFILE(NINT(avrSWAP(50)))
        INTEGER(IntKi),             INTENT(IN   )         :: size_avcMSG
        TYPE(ControlParameters),    INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables),       INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances),      INTENT(INOUT), TARGET :: objInst
        TYPE(PerformanceData),      INTENT(INOUT), TARGET :: PerfData
        TYPE(ErrorVariables),       INTENT(INOUT), TARGET :: ErrVar
        CHARACTER(NINT(avrSWAP(50))-1), INTENT(IN)        :: RootName

        INTEGER(IntKi) :: I
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view

        INTERFACE
            SUBROUTINE setparameters_c(CntrPar, LocalVar, avrSWAP, objInst, ErrVar, size_avcMSG) BIND(C, NAME=\\\"setparameters_c\\\")
                USE ISO_C_BINDING
                TYPE(C_PTR), VALUE :: CntrPar
                TYPE(C_PTR), VALUE :: LocalVar
                REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
                TYPE(C_PTR), VALUE :: objInst
                TYPE(C_PTR), VALUE :: ErrVar
                INTEGER(C_INT32_T), VALUE :: size_avcMSG
            END SUBROUTINE setparameters_c
        END INTERFACE

        ! --- Per-timestep init (must happen before any callee calls) ---
        ErrVar%aviFAIL = 0
        ErrVar%size_avcMSG = size_avcMSG

        objInst%instLPF         = 1
        objInst%instSecLPF      = 1
        objInst%instSecLPFV     = 1
        objInst%instHPF         = 1
        objInst%instNotchSlopes = 1
        objInst%instNotch       = 1
        objInst%instPI          = 1
        objInst%instRes         = 1
        objInst%instRL          = 1

        avrSWAP(35) = 1.0
        avrSWAP(36) = 0.0
        avrSWAP(41) = 0.0
        avrSWAP(46) = 0.0
        avrSWAP(55) = 0.0
        avrSWAP(56) = 0.0
        avrSWAP(65) = 0.0
        avrSWAP(72) = 0.0
        avrSWAP(79) = 4.0
        avrSWAP(80) = 0.0
        avrSWAP(81) = 0.0

        ! --- iStatus==0: callee dispatch (Fortran handles ALLOCATE protocol) ---
        IF (LocalVar%iStatus == 0) THEN
            write (*,*) \\\"                                                                              \\\"//NEW_LINE(\\\"A\\\")// &
                        \\\"------------------------------------------------------------------------------\\\"//NEW_LINE(\\\"A\\\")// &
                        \\\"Running ROSCO-\\\"//TRIM(rosco_version)//NEW_LINE(\\\"A\\\")// &
                        \\\"A wind turbine controller framework for public use in the scientific field    \\\"//NEW_LINE(\\\"A\\\")// &
                        \\\"Developed in collaboration: National Renewable Energy Laboratory              \\\"//NEW_LINE(\\\"A\\\")// &
                        \\\"                            Delft University of Technology, The Netherlands   \\\"//NEW_LINE(\\\"A\\\")// &
                        \\\"------------------------------------------------------------------------------\\\"

            LocalVar%ACC_INFILE_SIZE = NINT(avrSWAP(50))
            LocalVar%ACC_INFILE = \\\" \\\"
            DO I = 1, MIN(LocalVar%ACC_INFILE_SIZE, 1024)
                LocalVar%ACC_INFILE(I:I) = accINFILE(I)
            END DO

            CALL ReadControlParameterFileSub(CntrPar, LocalVar, accINFILE, NINT(avrSWAP(50)), RootName, ErrVar)
            IF (ErrVar%aviFAIL < 0) THEN
                ErrVar%ErrMsg = \\\"SetParameters:\\\"//TRIM(ErrVar%ErrMsg)
                RETURN
            ENDIF

            IF (CntrPar%WE_Mode > 0) THEN
                CALL ReadCpFile(CntrPar, PerfData, ErrVar)
            ENDIF
        ENDIF

        ! --- Populate CntrPar view (arrays allocated after first call) ---
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)

        ! --- C++ handles: LocalVar init, CheckInputs, OL_Index ---
        CALL setparameters_c(C_LOC(CntrPar_view), C_LOC(LocalVar), avrSWAP, C_LOC(objInst), C_LOC(ErrVar), INT(size_avcMSG, C_INT32_T))

        ! --- Error prepend for CheckInputs errors ---
        IF (LocalVar%iStatus == 0 .AND. ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = \\\"SetParameters:\\\"//TRIM(ErrVar%ErrMsg)
        ENDIF

    END SUBROUTINE SetParameters'''
result = re.sub(pattern, wrapper, content, flags=re.DOTALL)
if result == content:
    print('FAIL SetParameters: pattern not found in ReadSetParameters.f90')
    sys.exit(1)
with open('rosco/controller/src/ReadSetParameters.f90', 'w') as f:
    f.write(result)
print('  OK SetParameters')
" 2>&1
if [ $? -eq 0 ]; then
    PASS=$((PASS + 1))
else
    FAIL=$((FAIL + 1))
fi

# Add setparameters.cpp to CMakeLists.txt if not already present
python3 -c "
with open('rosco/controller/CMakeLists.txt', 'r') as f:
    content = f.read()
if 'setparameters.cpp' not in content:
    content = content.replace('src/updatezeromq.cpp', 'src/updatezeromq.cpp\n    src/setparameters.cpp')
    with open('rosco/controller/CMakeLists.txt', 'w') as f:
        f.write(content)
"

# Phase 11A: Replace DISCON.F90 with C++ entry point
echo "--- Phase 11A: DISCON (C++ entry point) ---"
cp translations/discon.cpp rosco/controller/src/discon.cpp
python3 -c "
with open('rosco/controller/CMakeLists.txt', 'r') as f:
    content = f.read()
if 'src/discon.cpp' not in content:
    content = content.replace('src/DISCON.F90\n', 'src/discon.cpp\n')
    with open('rosco/controller/CMakeLists.txt', 'w') as f:
        f.write(content)
    print('  OK DISCON: swapped DISCON.F90 -> discon.cpp in CMakeLists.txt')
else:
    print('  OK DISCON: discon.cpp already in CMakeLists.txt')
" 2>&1
if [ $? -eq 0 ]; then
    PASS=$((PASS + 1))
else
    FAIL=$((FAIL + 1))
fi

# Phase 11B: Remove all Fortran from the build (pure C++ libdiscon.so)
echo "--- Phase 11B: Remove Fortran (pure C++ build) ---"
python3 -c "
import re, sys

with open('rosco/controller/CMakeLists.txt', 'r') as f:
    content = f.read()

# Scan src/ directory for all .cpp files (not CMakeLists.txt SOURCES —
# manual integration blocks copy .cpp files to src/ but may not add them
# to CMake when starting from upstream Fortran-only CMakeLists.txt).
import os
cpp_files = sorted(['src/' + f for f in os.listdir('rosco/controller/src') if f.endswith('.cpp')])
if not cpp_files:
    print('FAIL: No .cpp files found in src/')
    sys.exit(1)

sources_block = '\n'.join(f'    {f}' for f in cpp_files)

# Write a clean C/C++ CMakeLists.txt
cmake = '''cmake_minimum_required(VERSION 3.6)
project(ROSCO VERSION 2.9.0 LANGUAGES C CXX)

if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE \"RelWithDebInfo\" CACHE STRING \"Choose the build type: Debug RelWithDebInfo Release\" FORCE)
endif()

# VIT: -ffp-contract=off for bit-identical verification during translation phase.
set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} -ffp-contract=off\")

set(SOURCES
''' + sources_block + '''
)

# ZMQ Library
find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ libzmq)
if(PC_ZeroMQ_FOUND)
    find_path(ZeroMQ_INCLUDE_DIR NAMES zmq.h PATHS \${PC_ZeroMQ_INCLUDE_DIRS})
    find_library(ZeroMQ_LIBRARY NAMES zmq PATHS \${PC_ZeroMQ_LIBRARY_DIRS})
    include_directories(\${ZeroMQ_INCLUDE_DIR})
    add_compile_options(-I\${ZeroMQ_INCLUDE_DIR} -l\${ZeroMQ_LIBRARY} -fPIC)
    add_library(zmq_client OBJECT src/zmq_client.c)
    add_definitions(-DZMQ_CLIENT=\"TRUE\")
    set(SOURCES \${SOURCES} \$<TARGET_OBJECTS:zmq_client>)
endif()

add_library(discon SHARED \${SOURCES})
target_include_directories(discon PRIVATE \${CMAKE_CURRENT_SOURCE_DIR}/../../translations)
target_link_libraries(discon PRIVATE stdc++ dl)

if(PC_ZeroMQ_FOUND)
    target_include_directories(discon PUBLIC \${ZeroMQ_INCLUDE_DIR})
    target_link_libraries(discon PUBLIC \${ZeroMQ_LIBRARY})
endif()

install(TARGETS discon
  EXPORT \"\${CMAKE_PROJECT_NAME}Libraries\"
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

set(linuxDefault \${CMAKE_INSTALL_PREFIX} STREQUAL \"/usr/local\")
set(windowsDefault \${CMAKE_INSTALL_PREFIX} STREQUAL \"C:\\\\\\\\Program Files (x86)\")
if(\${linuxDefault} OR \${windowsDefault})
  set(CMAKE_INSTALL_PREFIX \"\${CMAKE_SOURCE_DIR}/../\")
endif()
'''

with open('rosco/controller/CMakeLists.txt', 'w') as f:
    f.write(cmake)
print('  OK: Removed all Fortran from CMakeLists.txt (' + str(len(cpp_files)) + ' C++ sources)')
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
