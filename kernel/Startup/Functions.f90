!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-28 08:53:42 
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

    IMPLICIT NONE 

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
    REAL(DbKi) FUNCTION sigma(x, x0, x1, y0, y1, ErrVar)
    ! Generic sigma function
        USE rosco_types, ONLY: errorvariables 
        USE rosco_types, ONLY: kr_rosco_types_errorvariables 
        USE rosco_types, ONLY: kv_rosco_types_errorvariables 
        IMPLICIT NONE
        ! Inputs
    
        TYPE(ErrorVariables), INTENT(INOUT) :: ErrVar

        REAL(DbKi), Intent(IN)  :: x, x0, x1
        REAL(DbKi), Intent(IN)  :: y0, y1
        ! Local
            
        REAL(DbKi) :: a3, a2, a1, a0

        CHARACTER(*), PARAMETER                 :: RoutineName = 'sigma'

        a3 = 2/(x0-x1)**3
        a2 = -3*(x0+x1)/(x0-x1)**3
        a1 = 6*x1*x0/(x0-x1)**3
        a0 = (x0-3*x1)*x0**2/(x0-x1)**3

        IF (x < x0) THEN
            sigma = y0
        ELSEIF (x > x1) THEN
            sigma = y1
        ELSE
            sigma = (a3*x**3 + a2*x**2 + a1*x + a0)*(y1-y0) + y0
        ENDIF 
        ! Add RoutineName to error message

        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
        
    END FUNCTION sigma
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


END MODULE Functions