!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 20:50:11 
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


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


    FUNCTION unwrap(x, ErrVar) result(y)
    ! Unwrap function
    ! If difference between signal elements is < -pi, add 2 pi to reset of signal, and the opposite
    ! Someday, generalize period and check difference is less than period/2
        USE rosco_types, ONLY: errorvariables 
        USE rosco_types, ONLY: kr_rosco_types_errorvariables 
        USE rosco_types, ONLY: kv_rosco_types_errorvariables 
        IMPLICIT NONE
        ! Inputs
    
        TYPE(ErrorVariables), INTENT(INOUT) :: ErrVar
        REAL(DbKi), DIMENSION(:), Intent(IN)  :: x
        ! Output

        REAL(DbKi), DIMENSION(SIZE(x)) :: y
        ! Local
            
        INTEGER(IntKi) :: i

        CHARACTER(*), PARAMETER                 :: RoutineName = 'unwrap'

        y = x ! set initial
        DO i = 2, SIZE(x)
            DO while (y(i) - y(i-1) .LE. -PI)
                y(i:SIZE(x)) = y(i:SIZE(x)) + 2 * PI
            END DO

            DO while (y(i) - y(i-1) .GE. PI)
                y(i:SIZE(x)) = y(i:SIZE(x)) - 2 * PI
            END DO
        END DO
        ! Add RoutineName to error message

        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
        ! Debug
        ! write(400,*) x
        ! write(401,*) y

        
    END FUNCTION unwrap
!-------------------------------------------------------------------------------------------------------------------------------


END MODULE Functions