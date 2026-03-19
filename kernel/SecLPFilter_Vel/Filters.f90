!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-19 15:14:31 
!KGEN version : 0.8.1 
  
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
    USE functions 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 
    USE ISO_C_BINDING
    IMPLICIT NONE 


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
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: seclpfilter_vel_c
        END FUNCTION seclpfilter_vel_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

    FUNCTION SecLPFilter_Vel(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue) RESULT(SecLPFilter_Vel_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: InputSignal
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: CornerFreq
        REAL(DbKi), INTENT(IN) :: Damp
        TYPE(FilterParameters), INTENT(INOUT), TARGET :: FP
        INTEGER(IntKi), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi), INTENT(IN), OPTIONAL :: InitialValue
        REAL(DbKi) :: SecLPFilter_Vel_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        SecLPFilter_Vel_result = REAL(seclpfilter_vel_c(REAL(InputSignal, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(CornerFreq, C_DOUBLE), REAL(Damp, C_DOUBLE), C_LOC(FP), INT(iStatus, C_INT), LOGICAL(reset, C_BOOL), inst, has_InitialValue_flag, InitialValue_val), DbKi)
    END FUNCTION SecLPFilter_Vel
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


    END MODULE Filters