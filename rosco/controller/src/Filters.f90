! Copyright 2019 NREL

! Licensed under the Apache License, Version 2.0 (the "License"); you may not use
! this file except in compliance with the License. You may obtain a copy of the
! License at http://www.apache.org/licenses/LICENSE-2.0

! Unless required by applicable law or agreed to in writing, software distributed
! under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
! CONDITIONS OF ANY KIND, either express or implied. See the License for the
! specific language governing permissions and limitations under the License.
! -------------------------------------------------------------------------------------------
! This module contains all the filters and related subroutines

! Filters:
!       LPFilter: Low-pass filter
!       SecLPFilter: Second order low-pass filter
!       HPFilter: High-pass filter
!       NotchFilter: Notch filter
!       NotchFilterSlopes: Notch Filter with descending slopes
!       PreFilterMeasuredSignals: Pre-filter signals during each run iteration

MODULE Filters
!...............................................................................................................................
    USE Constants
    USE Functions
    USE ISO_C_BINDING
    IMPLICIT NONE


    ! Auto-generated interface for C++ implementation of HPFilter
    INTERFACE
        FUNCTION hpfilter_c(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='hpfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: hpfilter_c
        END FUNCTION hpfilter_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of LPFilter
    INTERFACE
        FUNCTION lpfilter_c(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='lpfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: lpfilter_c
        END FUNCTION lpfilter_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of NotchFilter
    INTERFACE
        FUNCTION notchfilter_c(InputSignal, DT, omega, betaNum, betaDen, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='notchfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: omega
            REAL(C_DOUBLE), VALUE :: betaNum
            REAL(C_DOUBLE), VALUE :: betaDen
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: notchfilter_c
        END FUNCTION notchfilter_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of NotchFilterSlopes
    INTERFACE
        FUNCTION notchfilterslopes_c(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, has_Moving, Moving, has_InitialValue, InitialValue) BIND(C, NAME='notchfilterslopes_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            REAL(C_DOUBLE), VALUE :: Damp
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_Moving
            INTEGER(C_INT), VALUE :: Moving
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: notchfilterslopes_c
        END FUNCTION notchfilterslopes_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of SecLPFilter
    INTERFACE
        FUNCTION seclpfilter_c(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='seclpfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            REAL(C_DOUBLE), VALUE :: Damp
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: seclpfilter_c
        END FUNCTION seclpfilter_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of SecLPFilter_Vel
    INTERFACE
        FUNCTION seclpfilter_vel_c(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='seclpfilter_vel_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            REAL(C_DOUBLE), VALUE :: Damp
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            INTEGER(C_INT), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: seclpfilter_vel_c
        END FUNCTION seclpfilter_vel_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of PreFilterMeasuredSignals
    INTERFACE
        SUBROUTINE prefiltermeasuredsignals_c(CntrPar, LocalVar, DebugVar, objInst, ErrVar) BIND(C, NAME='prefiltermeasuredsignals_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE prefiltermeasuredsignals_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION LPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue) RESULT(LPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: CornerFreq
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: LPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        LPFilter_result = REAL(lpfilter_c(InputSignal, DT, CornerFreq, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION LPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue) RESULT(SecLPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: CornerFreq
        REAL(8), INTENT(IN) :: Damp
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: SecLPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        SecLPFilter_result = REAL(seclpfilter_c(InputSignal, DT, CornerFreq, Damp, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION SecLPFilter

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION SecLPFilter_Vel(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue) RESULT(SecLPFilter_Vel_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: CornerFreq
        REAL(8), INTENT(IN) :: Damp
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: SecLPFilter_Vel_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        SecLPFilter_Vel_result = REAL(seclpfilter_vel_c(InputSignal, DT, CornerFreq, Damp, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION SecLPFilter_Vel

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION HPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, InitialValue) RESULT(HPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: CornerFreq
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: HPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        HPFilter_result = REAL(hpfilter_c(InputSignal, DT, CornerFreq, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION HPFilter
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION NotchFilterSlopes(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, Moving, InitialValue) RESULT(NotchFilterSlopes_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: CornerFreq
        REAL(8), INTENT(IN) :: Damp
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        LOGICAL, INTENT(IN), OPTIONAL :: Moving
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: NotchFilterSlopes_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_Moving_flag
        INTEGER(C_INT) :: Moving_val
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_Moving_flag = 0
        Moving_val = 0
        IF (PRESENT(Moving)) THEN
            has_Moving_flag = 1
            Moving_val = MERGE(1_C_INT, 0_C_INT, Moving)
        END IF
        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        NotchFilterSlopes_result = REAL(notchfilterslopes_c(InputSignal, DT, CornerFreq, Damp, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_Moving_flag, Moving_val, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION NotchFilterSlopes
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION NotchFilter(InputSignal, DT, omega, betaNum, betaDen, FP, iStatus, reset, inst, InitialValue) RESULT(NotchFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: InputSignal
        REAL(8), INTENT(IN) :: DT
        REAL(8), INTENT(IN) :: omega
        REAL(8), INTENT(IN) :: betaNum
        REAL(8), INTENT(IN) :: betaDen
        TYPE(FILTERPARAMETERS), INTENT(INOUT), TARGET :: FP
        INTEGER(4), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: InitialValue
        REAL(8) :: NotchFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        NotchFilter_result = REAL(notchfilter_c(InputSignal, DT, omega, betaNum, betaDen, C_LOC(FP), iStatus, MERGE(1_C_INT, 0_C_INT, reset), inst, has_InitialValue_flag, InitialValue_val), 8)
    END FUNCTION NotchFilter
!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE PreFilterMeasuredSignals(CntrPar, LocalVar, DebugVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, DebugVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(DEBUGVARIABLES), INTENT(INOUT), TARGET :: DebugVar
        TYPE(OBJECTINSTANCES), INTENT(INOUT), TARGET :: objInst
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL prefiltermeasuredsignals_c(C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(DebugVar), C_LOC(objInst), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE PreFilterMeasuredSignals
    END MODULE Filters
