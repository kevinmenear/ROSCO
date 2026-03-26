!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 20:37:03 
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


    ! Auto-generated interface for C++ implementation of interp1d
    INTERFACE
        FUNCTION interp1d_c(xData, n_xData, yData, n_yData, xq, ErrVar) BIND(C, NAME='interp1d_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), INTENT(IN) :: xData(*)
            INTEGER(C_INT), VALUE :: n_xData
            REAL(C_DOUBLE), INTENT(IN) :: yData(*)
            INTEGER(C_INT), VALUE :: n_yData
            REAL(C_DOUBLE), VALUE :: xq
            TYPE(C_PTR), VALUE :: ErrVar
            REAL(C_DOUBLE) :: interp1d_c
        END FUNCTION interp1d_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------
    


    FUNCTION interp1d(xData, yData, xq, ErrVar) RESULT(interp1d_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: xData(:)
        REAL(DbKi), INTENT(IN) :: yData(:)
        REAL(DbKi), INTENT(IN) :: xq
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        REAL(DbKi) :: interp1d_result
        interp1d_result = REAL(interp1d_c(xData, SIZE(xData), yData, SIZE(yData), REAL(xq, C_DOUBLE), C_LOC(ErrVar)), DbKi)
    END FUNCTION interp1d
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