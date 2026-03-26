!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 23:31:53 
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


    ! Auto-generated interface for C++ implementation of identity
    INTERFACE
        SUBROUTINE identity_c(n, identity_result) BIND(C, NAME='identity_c')
            USE ISO_C_BINDING
            INTEGER(C_INT), VALUE :: n
            REAL(C_DOUBLE), INTENT(OUT) :: identity_result(*)
        END SUBROUTINE identity_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------
    


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

    FUNCTION identity(n) RESULT(A)
        USE ISO_C_BINDING
        IMPLICIT NONE
        INTEGER, INTENT(IN) :: n
        REAL(DbKi), DIMENSION(N, N) :: A
        CALL identity_c(n, A)
    END FUNCTION identity
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