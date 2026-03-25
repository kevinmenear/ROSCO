!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-18 12:19:00 
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
! This module contains basic control-related functions
! Functions:
!       AeroDynTorque: Calculate aerodynamic torque
!       ColemanTransform: Perform Colemant transform
!       ColemanTransformInverse: Perform inverse Colemant transform
!       CPFunction: Find Cp using parameterized surface
!       Debug: Debug the controller
!       DFController: DF control
!       identity: Make identity matrix
!       interp1d: 1-d interpolation
!       interp2d: 2-d interpolation
!       matinv3: 3x3 matrix inverse
!       PIDController: implement a PID controller
!       ratelimit: Rate limit signal
!       saturate: Saturate signal


MODULE Functions

    USE constants 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 

    USE ISO_C_BINDING
    IMPLICIT NONE 


    ! Auto-generated interface for C++ implementation of ratelimit
    INTERFACE
        FUNCTION ratelimit_c(inputSignal, minRate, maxRate, DT, reset, rlP, inst, has_ResetValue, ResetValue) BIND(C, NAME='ratelimit_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: inputSignal
            REAL(C_DOUBLE), VALUE :: minRate
            REAL(C_DOUBLE), VALUE :: maxRate
            REAL(C_DOUBLE), VALUE :: DT
            LOGICAL(C_BOOL), VALUE :: reset
            TYPE(C_PTR), VALUE :: rlP
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_ResetValue
            REAL(C_DOUBLE), VALUE :: ResetValue
            REAL(C_DOUBLE) :: ratelimit_c
        END FUNCTION ratelimit_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION saturate(inputValue, minValue, maxValue)
    ! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue

        IMPLICIT NONE

        REAL(DbKi), INTENT(IN)     :: inputValue
        REAL(DbKi), INTENT(IN)     :: minValue
        REAL(DbKi), INTENT(IN)     :: maxValue

        saturate = REAL(MIN(MAX(inputValue,minValue), maxValue),DbKi)

    END FUNCTION saturate
!-------------------------------------------------------------------------------------------------------------------------------
    
    FUNCTION ratelimit(inputSignal, minRate, maxRate, DT, reset, rlP, inst, ResetValue) RESULT(ratelimit_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : rlParams
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: inputSignal
        REAL(DbKi), INTENT(IN) :: minRate
        REAL(DbKi), INTENT(IN) :: maxRate
        REAL(DbKi), INTENT(IN) :: DT
        LOGICAL, INTENT(IN) :: reset
        TYPE(rlParams), INTENT(INOUT), TARGET :: rlP
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi), INTENT(IN), OPTIONAL :: ResetValue
        REAL(DbKi) :: ratelimit_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_ResetValue_flag
        REAL(C_DOUBLE) :: ResetValue_val

        has_ResetValue_flag = 0
        ResetValue_val = 0.0D0
        IF (PRESENT(ResetValue)) THEN
            has_ResetValue_flag = 1
            ResetValue_val = REAL(ResetValue, C_DOUBLE)
        END IF
        ratelimit_result = REAL(ratelimit_c(REAL(inputSignal, C_DOUBLE), REAL(minRate, C_DOUBLE), REAL(maxRate, C_DOUBLE), REAL(DT, C_DOUBLE), LOGICAL(reset, C_BOOL), C_LOC(rlP), inst, has_ResetValue_flag, ResetValue_val), DbKi)
    END FUNCTION ratelimit


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


END MODULE Functions