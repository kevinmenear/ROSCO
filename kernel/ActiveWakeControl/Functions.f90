!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 20:49:48 
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
    REAL(DbKi) FUNCTION saturate(inputValue, minValue, maxValue)
    ! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue

        IMPLICIT NONE

        REAL(DbKi), INTENT(IN)     :: inputValue
        REAL(DbKi), INTENT(IN)     :: minValue
        REAL(DbKi), INTENT(IN)     :: maxValue

        saturate = REAL(MIN(MAX(inputValue,minValue), maxValue),DbKi)

    END FUNCTION saturate
!-------------------------------------------------------------------------------------------------------------------------------
    


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

    SUBROUTINE ColemanTransform(rootMOOP, aziAngle, nHarmonic, axTOut, axYOut)
    ! The Coleman or d-q axis transformation transforms the root out of plane bending moments of each turbine blade
    ! to a direct axis and a quadrature axis

        IMPLICIT NONE
        ! Inputs
        REAL(DbKi), INTENT(IN)     :: rootMOOP(3)                      ! Root out of plane bending moments of each blade
        REAL(DbKi), INTENT(IN)     :: aziAngle                         ! Rotor azimuth angle
        INTEGER(IntKi), INTENT(IN)  :: nHarmonic                        ! The harmonic number, nP
        ! Outputs
        REAL(DbKi), INTENT(OUT)    :: axTOut, axYOut               ! Direct axis and quadrature axis outputted by this transform
        ! Local
        REAL(DbKi), PARAMETER      :: phi2 = 2.0/3.0*PI                ! Phase difference from first to second blade
        REAL(DbKi), PARAMETER      :: phi3 = 4.0/3.0*PI                ! Phase difference from first to third blade
        ! Body

        axTOut  = 2.0/3.0 * (cos(nHarmonic*(aziAngle))*rootMOOP(1) + cos(nHarmonic*(aziAngle+phi2))*rootMOOP(2) + cos(nHarmonic*(aziAngle+phi3))*rootMOOP(3))
        axYOut  = 2.0/3.0 * (sin(nHarmonic*(aziAngle))*rootMOOP(1) + sin(nHarmonic*(aziAngle+phi2))*rootMOOP(2) + sin(nHarmonic*(aziAngle+phi3))*rootMOOP(3))
        
    END SUBROUTINE ColemanTransform
!-------------------------------------------------------------------------------------------------------------------------------

    SUBROUTINE ColemanTransformInverse(axTIn, axYIn, aziAngle, nHarmonic, aziOffset, PitComIPC)
    ! The inverse Coleman or d-q axis transformation transforms the direct axis and quadrature axis
    ! back to root out of plane bending moments of each turbine blade
        IMPLICIT NONE
        ! Inputs
        REAL(DbKi), INTENT(IN)     :: axTIn, axYIn         ! Direct axis and quadrature axis
        REAL(DbKi), INTENT(IN)     :: aziAngle                     ! Rotor azimuth angle
        REAL(DbKi), INTENT(IN)     :: aziOffset                    ! Phase shift added to the azimuth angle
        INTEGER(IntKi), INTENT(IN)  :: nHarmonic                    ! The harmonic number, nP
        ! Outputs
        REAL(DbKi), INTENT(OUT)    :: PitComIPC(3)                   ! Commanded individual pitch (deg)
        ! Local
        REAL(DbKi), PARAMETER      :: phi2 = 2.0/3.0*PI                ! Phase difference from first to second blade
        REAL(DbKi), PARAMETER      :: phi3 = 4.0/3.0*PI                ! Phase difference from first to third blade
        ! Body

        PitComIPC(1) = cos(nHarmonic*(aziAngle+aziOffset))*axTIn + sin(nHarmonic*(aziAngle+aziOffset))*axYIn
        PitComIPC(2) = cos(nHarmonic*(aziAngle+aziOffset+phi2))*axTIn + sin(nHarmonic*(aziAngle+aziOffset+phi2))*axYIn
        PitComIPC(3) = cos(nHarmonic*(aziAngle+aziOffset+phi3))*axTIn + sin(nHarmonic*(aziAngle+aziOffset+phi3))*axYIn

    END SUBROUTINE ColemanTransformInverse
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION wrap_360(x) 
    ! Function modifies input angle, x, such that 0<=x<=360, preventing windup
        REAL(DbKi), INTENT(IN) :: x         ! angle, degrees

        IF (x .lt. 0.0) THEN
            wrap_360 = x + 360.0
        ELSEIF (x .ge. 360.0) THEN
            wrap_360 = x - 360.0
        ELSE
            wrap_360 = x
        ENDIF

    END FUNCTION wrap_360
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


END MODULE Functions