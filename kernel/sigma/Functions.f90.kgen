!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 20:36:27 
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


    ! Auto-generated interface for C++ implementation of sigma
    INTERFACE
        FUNCTION sigma_c(x, x0, x1, y0, y1, ErrVar) BIND(C, NAME='sigma_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: x
            REAL(C_DOUBLE), VALUE :: x0
            REAL(C_DOUBLE), VALUE :: x1
            REAL(C_DOUBLE), VALUE :: y0
            REAL(C_DOUBLE), VALUE :: y1
            TYPE(C_PTR), VALUE :: ErrVar
            REAL(C_DOUBLE) :: sigma_c
        END FUNCTION sigma_c
    END INTERFACE

CONTAINS
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
    FUNCTION sigma(x, x0, x1, y0, y1, ErrVar) RESULT(sigma_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: x
        REAL(DbKi), INTENT(IN) :: x0
        REAL(DbKi), INTENT(IN) :: x1
        REAL(DbKi), INTENT(IN) :: y0
        REAL(DbKi), INTENT(IN) :: y1
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        REAL(DbKi) :: sigma_result
        sigma_result = REAL(sigma_c(REAL(x, C_DOUBLE), REAL(x0, C_DOUBLE), REAL(x1, C_DOUBLE), REAL(y0, C_DOUBLE), REAL(y1, C_DOUBLE), C_LOC(ErrVar)), DbKi)
    END FUNCTION sigma
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


END MODULE Functions