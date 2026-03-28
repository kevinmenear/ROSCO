!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-28 09:16:16 
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


    ! Auto-generated interface for C++ implementation of interp2d
    INTERFACE
        FUNCTION interp2d_c(xData, n_xData, yData, n_yData, zData, n_zData_rows, n_zData_cols, xq, yq, ErrVar) BIND(C, NAME='interp2d_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), INTENT(IN) :: xData(*)
            INTEGER(C_INT), VALUE :: n_xData
            REAL(C_DOUBLE), INTENT(IN) :: yData(*)
            INTEGER(C_INT), VALUE :: n_yData
            REAL(C_DOUBLE), INTENT(IN) :: zData(*)
            INTEGER(C_INT), VALUE :: n_zData_rows
            INTEGER(C_INT), VALUE :: n_zData_cols
            REAL(C_DOUBLE), VALUE :: xq
            REAL(C_DOUBLE), VALUE :: yq
            TYPE(C_PTR), VALUE :: ErrVar
            REAL(C_DOUBLE) :: interp2d_c
        END FUNCTION interp2d_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------
    


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

    FUNCTION interp2d(xData, yData, zData, xq, yq, ErrVar) RESULT(interp2d_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: xData(:)
        REAL(DbKi), INTENT(IN) :: yData(:)
        REAL(DbKi), INTENT(IN) :: zData(:,:)
        REAL(DbKi), INTENT(IN) :: xq
        REAL(DbKi), INTENT(IN) :: yq
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        REAL(DbKi) :: interp2d_result
        interp2d_result = REAL(interp2d_c(xData, SIZE(xData), yData, SIZE(yData), zData, SIZE(zData, 1), SIZE(zData, 2), REAL(xq, C_DOUBLE), REAL(yq, C_DOUBLE), C_LOC(ErrVar)), DbKi)
    END FUNCTION interp2d
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