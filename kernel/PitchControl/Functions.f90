!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-27 18:27:59 
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
    USE kgen_utils_mod, ONLY: kgen_dp, kgen_array_sumcheck 
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
    
    REAL(DbKi) FUNCTION ratelimit(inputSignal, minRate, maxRate, DT, reset, rlP, inst, ResetValue)
    ! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue
        USE rosco_types, ONLY: rlparams 
        USE rosco_types, ONLY: kr_rosco_types_rlparams 
        USE rosco_types, ONLY: kv_rosco_types_rlparams 

        
        IMPLICIT NONE

        REAL(DbKi), INTENT(IN)          :: inputSignal
        REAL(DbKi), INTENT(IN)          :: minRate
        REAL(DbKi), INTENT(IN)          :: maxRate
        REAL(DbKi), INTENT(IN)          :: DT
        LOGICAL,    INTENT(IN)         :: reset  
        TYPE(rlParams), INTENT(INOUT)   :: rlP
        INTEGER(IntKi), INTENT(INOUT)   :: inst
        REAL(DbKi), OPTIONAL,  INTENT(IN)          :: ResetValue           ! Value to base rate limit off if restarting
        ! Local variables

        REAL(DbKi)                 :: rate
        REAL(DbKi)                 :: ResetValue_

        ResetValue_ = inputSignal
        IF (PRESENT(ResetValue)) ResetValue_ = ResetValue   

        IF (reset) THEN
            rlP%LastSignal(inst) = ResetValue_
            ratelimit = ResetValue_
            
        ELSE
            rate = (inputSignal - rlP%LastSignal(inst))/DT                       ! Signal rate (unsaturated)
            rate = saturate(rate, minRate, maxRate)                 ! Saturate the signal rate

            ratelimit = rlP%LastSignal(inst) + rate*DT  

            rlP%LastSignal(inst) = ratelimit

        ENDIF
        ! Increment instance

        inst = inst + 1                     ! Saturate the overall command using the rate limit

    END FUNCTION ratelimit


    REAL(DbKi) FUNCTION interp1d(xData, yData, xq, ErrVar)
    ! interp1d 1-D interpolation (table lookup), xData should be strictly increasing
        
        USE rosco_types, ONLY: errorvariables 
        USE rosco_types, ONLY: kr_rosco_types_errorvariables 
        USE rosco_types, ONLY: kv_rosco_types_errorvariables 
        IMPLICIT NONE
        ! Inputs

        REAL(DbKi), DIMENSION(:), INTENT(IN)       :: xData        ! Provided x data (vector), to be interpolated
        REAL(DbKi), DIMENSION(:), INTENT(IN)       :: yData        ! Provided y data (vector), to be interpolated
        REAL(DbKi), INTENT(IN)                     :: xq           ! x-value for which the y value has to be interpolated
        INTEGER(IntKi)                              :: I            ! Iteration index
        ! Error Catching

        TYPE(ErrorVariables), INTENT(INOUT)     :: ErrVar
        INTEGER(IntKi)                              :: I_DIFF

        CHARACTER(*), PARAMETER                 :: RoutineName = 'interp1d'
        ! Catch Errors
        ! Are xData and yData the same size?

        
        IF (SIZE(xData) .NE. SIZE(yData)) THEN
            ErrVar%aviFAIL = -1
            ErrVar%ErrMsg  = ' xData and yData are not the same size'
            WRITE(ErrVar%ErrMsg,"(A,I2,A,I2,A)") " SIZE(xData) =", SIZE(xData), & 
            ' and SIZE(yData) =', SIZE(yData),' are not the same'
        END IF
        ! Is xData non decreasing

        DO I_DIFF = 1, size(xData) - 1
            IF (xData(I_DIFF + 1) - xData(I_DIFF) <= 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = ' xData is not strictly increasing'
                EXIT 
            END IF
        END DO
        ! Interpolate
        
        IF (xq <= MINVAL(xData)) THEN
            interp1d = yData(1)
        ELSEIF (xq >= MAXVAL(xData)) THEN
            interp1d = yData(SIZE(xData))
        ELSE
            DO I = 1, SIZE(xData)
                IF (xq <= xData(I)) THEN
                    interp1d = yData(I-1) + (yData(I) - yData(I-1))/(xData(I) - xData(I-1))*(xq - xData(I-1))
                    EXIT
                ELSE
                    CONTINUE
                END IF
            END DO
        END IF
        ! Add RoutineName to error message

        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
        
    END FUNCTION interp1d
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