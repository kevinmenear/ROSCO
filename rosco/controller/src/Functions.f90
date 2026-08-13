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

USE Constants

USE ISO_C_BINDING
IMPLICIT NONE


    ! Auto-generated interface for C++ implementation of ColemanTransform
    INTERFACE
        SUBROUTINE colemantransform_c(rootMOOP, aziAngle, nHarmonic, axTOut, axYOut) BIND(C, NAME='colemantransform_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), INTENT(IN) :: rootMOOP(*)
            REAL(C_DOUBLE), VALUE :: aziAngle
            INTEGER(C_INT), VALUE :: nHarmonic
            REAL(C_DOUBLE), INTENT(OUT) :: axTOut
            REAL(C_DOUBLE), INTENT(OUT) :: axYOut
        END SUBROUTINE colemantransform_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of ColemanTransformInverse
    INTERFACE
        SUBROUTINE colemantransforminverse_c(axTIn, axYIn, aziAngle, nHarmonic, aziOffset, PitComIPC) BIND(C, NAME='colemantransforminverse_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: axTIn
            REAL(C_DOUBLE), VALUE :: axYIn
            REAL(C_DOUBLE), VALUE :: aziAngle
            INTEGER(C_INT), VALUE :: nHarmonic
            REAL(C_DOUBLE), VALUE :: aziOffset
            REAL(C_DOUBLE), INTENT(OUT) :: PitComIPC(*)
        END SUBROUTINE colemantransforminverse_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of identity
    INTERFACE
        SUBROUTINE identity_c(n, identity_result) BIND(C, NAME='identity_c')
            USE ISO_C_BINDING
            INTEGER(C_INT), VALUE :: n
            REAL(C_DOUBLE), INTENT(OUT) :: identity_result(*)
        END SUBROUTINE identity_c
    END INTERFACE


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


    ! Auto-generated interface for C++ implementation of saturate
    INTERFACE
        FUNCTION saturate_c(inputValue, minValue, maxValue) BIND(C, NAME='saturate_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: inputValue
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE) :: saturate_c
        END FUNCTION saturate_c
    END INTERFACE


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


    ! Auto-generated interface for C++ implementation of unwrap
    INTERFACE
        SUBROUTINE unwrap_c(x, n_x, ErrVar, unwrap_result) BIND(C, NAME='unwrap_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), INTENT(IN) :: x(*)
            INTEGER(C_INT), VALUE :: n_x
            TYPE(C_PTR), VALUE :: ErrVar
            REAL(C_DOUBLE), INTENT(OUT) :: unwrap_result(*)
        END SUBROUTINE unwrap_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION saturate(inputValue, minValue, maxValue) RESULT(saturate_result)
        USE ISO_C_BINDING
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: inputValue
        REAL(8), INTENT(IN) :: minValue
        REAL(8), INTENT(IN) :: maxValue
        REAL(8) :: saturate_result
        saturate_result = REAL(saturate_c(inputValue, minValue, maxValue), 8)
    END FUNCTION saturate
    
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION ratelimit(inputSignal, minRate, maxRate, DT, reset, rlP, inst, ResetValue)
    ! Saturates inputValue. Makes sure it is not smaller than minValue and not larger than maxValue
        USE ROSCO_Types, ONLY : rlParams

        
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


    FUNCTION interp1d(xData, yData, xq, ErrVar) RESULT(interp1d_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: xData(:)
        REAL(8), INTENT(IN) :: yData(:)
        REAL(8), INTENT(IN) :: xq
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        REAL(8) :: interp1d_result
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        interp1d_result = REAL(interp1d_c(xData, SIZE(xData), yData, SIZE(yData), xq, C_LOC(ErrVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END FUNCTION interp1d

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION interp2d(xData, yData, zData, xq, yq, ErrVar)
    ! interp2d 2-D interpolation (table lookup). Query done using bilinear interpolation. 
    ! Note that the interpolated matrix with associated query vectors may be different than "standard", - zData should be formatted accordingly
    ! - xData follows the matrix from left to right
    ! - yData follows the matrix from top to bottom
    ! A simple case: xData = [1 2 3], yData = [4 5 6]
    !        | 1    2   3
    !       -------------
    !       4| a    b   c
    !       5| d    e   f
    !       6| g    H   i

        USE ROSCO_Types, ONLY : ErrorVariables
        USE ieee_arithmetic
        
        IMPLICIT NONE
    
        ! Inputs
        REAL(DbKi), DIMENSION(:),   INTENT(IN)     :: xData        ! Provided x data (vector), to find query point (should be strictly increasing)
        REAL(DbKi), DIMENSION(:),   INTENT(IN)     :: yData        ! Provided y data (vector), to find query point (should be strictly increasing)
        REAL(DbKi), DIMENSION(:,:), INTENT(IN)     :: zData        ! Provided z data (vector), to be interpolated
        REAL(DbKi),                 INTENT(IN)     :: xq           ! x-value for which the z value has to be interpolated
        REAL(DbKi),                 INTENT(IN)     :: yq           ! y-value for which the z value has to be interpolated

        ! Allocate variables
        INTEGER(IntKi)                              :: i            ! Iteration index & query index, x-direction
        INTEGER(IntKi)                              :: ii           ! Iteration index & second que .  ry index, x-direction
        INTEGER(IntKi)                              :: j            ! Iteration index & query index, y-direction
        INTEGER(IntKi)                              :: jj           ! Iteration index & second query index, y-direction
        REAL(DbKi), DIMENSION(2,2)                 :: fQ           ! zData value at query points for bilinear interpolation            
        REAL(DbKi), DIMENSION(1)                   :: fxy           ! Interpolated z-data point to be returned
        REAL(DbKi)                                 :: fxy1          ! zData value at query point for bilinear interpolation
        REAL(DbKi)                                 :: fxy2          ! zData value at query point for bilinear interpolation       
        LOGICAL                                 :: edge     

        ! Error Catching
        TYPE(ErrorVariables), INTENT(INOUT)     :: ErrVar
        INTEGER(IntKi)                              :: I_DIFF

        CHARACTER(*), PARAMETER                 :: RoutineName = 'interp2d'
        
        ! Error catching
        ! Are xData and zData(:,1) the same size?
        IF (SIZE(xData) .NE. SIZE(zData,2)) THEN
            ErrVar%aviFAIL = -1
            WRITE(ErrVar%ErrMsg,"(A,I4,A,I4,A)") " SIZE(xData) =", SIZE(xData), & 
            ' and SIZE(zData,1) =', SIZE(zData,2),' are not the same'
        END IF

        ! Are yData and zData(1,:) the same size?
        IF (SIZE(yData) .NE. SIZE(zData,1)) THEN
            ErrVar%aviFAIL = -1
            WRITE(ErrVar%ErrMsg,"(A,I4,A,I4,A)") " SIZE(yData) =", SIZE(yData), & 
            ' and SIZE(zData,2) =', SIZE(zData,1),' are not the same'
        END IF

        ! Is xData non decreasing
        DO I_DIFF = 1, size(xData) - 1
            IF (xData(I_DIFF + 1) - xData(I_DIFF) <= 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = ' xData is not strictly increasing'
                EXIT 
            END IF
        END DO

        ! Is yData non decreasing
        DO I_DIFF = 1, size(yData) - 1
            IF (yData(I_DIFF + 1) - yData(I_DIFF) <= 0) THEN
                ErrVar%aviFAIL = -1
                ErrVar%ErrMsg  = ' yData is not strictly increasing'
                EXIT 
            END IF
        END DO

        ! ---- Find corner indices surrounding desired interpolation point -----
            ! x-direction
        IF (xq <= MINVAL(xData) .OR. (ieee_is_nan(xq))) THEN       ! On lower x-bound, just need to find zData(yq)
            j = 1
            jj = 1
            interp2d = interp1d(yData,zData(:,j),yq,ErrVar)     
            RETURN
        ELSEIF (xq >= MAXVAL(xData)) THEN   ! On upper x-bound, just need to find zData(yq)
            j = size(xData)
            jj = size(xData)
            interp2d = interp1d(yData,zData(:,j),yq,ErrVar)
            RETURN
        ELSE
            DO j = 1,size(xData)            
                IF (xq == xData(j)) THEN ! On axis, just need 1d interpolation
                    jj = j
                    interp2d = interp1d(yData,zData(:,j),yq,ErrVar)  
                    RETURN
                ELSEIF (xq < xData(j)) THEN
                    jj = j
                    EXIT
                ELSE
                    CONTINUE
                END IF
            END DO
        ENDIF
        j = j-1 ! Move j back one
            ! y-direction
        IF (yq <= MINVAL(yData) .OR. (ieee_is_nan(yq))) THEN       ! On lower y-bound, just need to find zData(xq)
            i = 1
            ii = 1
            interp2d = interp1d(xData,zData(i,:),xq,ErrVar)     
            RETURN
        ELSEIF (yq >= MAXVAL(yData)) THEN   ! On upper y-bound, just need to find zData(xq)
            i = size(yData)
            ii = size(yData)
            interp2d = interp1d(xData,zData(i,:),xq,ErrVar)      
            RETURN
        ELSE
            DO i = 1,size(yData)
                IF (yq == yData(i)) THEN    ! On axis, just need 1d interpolation
                    ii = i
                    interp2d = interp1d(xData,zData(i,:),xq,ErrVar)        
                    RETURN
                ELSEIF (yq < yData(i)) THEN
                    ii = i
                    EXIT
                ELSE
                    CONTINUE
                END IF
            END DO
        ENDIF
        i = i-1 ! move i back one
        
        ! ---- Do bilinear interpolation ----
        ! Find values at corners 
        fQ(1,1) = zData(i,j)
        fQ(2,1) = zData(ii,j)
        fQ(1,2) = zData(i,jj)
        fQ(2,2) = zData(ii,jj)
        ! Interpolate
        fxy1 = (xData(jj) - xq)/(xData(jj) - xData(j))*fQ(1,1) + (xq - xData(j))/(xData(jj) - xData(j))*fQ(1,2)
        fxy2 = (xData(jj) - xq)/(xData(jj) - xData(j))*fQ(2,1) + (xq - xData(j))/(xData(jj) - xData(j))*fQ(2,2)
        fxy = (yData(ii) - yq)/(yData(ii) - yData(i))*fxy1 + (yq - yData(i))/(yData(ii) - yData(i))*fxy2

        interp2d = fxy(1)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF

    END FUNCTION interp2d

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION matinv3(A) RESULT(B)
    ! Performs a direct calculation of the inverse of a 3×3 matrix.
    ! Source: http://fortranwiki.org/fortran/show/Matrix+inversion
        REAL(DbKi), INTENT(IN) :: A(3,3)   !! Matrix
        REAL(DbKi)             :: B(3,3)   !! Inverse matrix
        REAL(DbKi)             :: detinv

        ! Calculate the inverse determinant of the matrix
        detinv = 1/(A(1,1)*A(2,2)*A(3,3) - A(1,1)*A(2,3)*A(3,2)&
                - A(1,2)*A(2,1)*A(3,3) + A(1,2)*A(2,3)*A(3,1)&
                + A(1,3)*A(2,1)*A(3,2) - A(1,3)*A(2,2)*A(3,1))

        ! Calculate the inverse of the matrix
        B(1,1) = +detinv * (A(2,2)*A(3,3) - A(2,3)*A(3,2))
        B(2,1) = -detinv * (A(2,1)*A(3,3) - A(2,3)*A(3,1))
        B(3,1) = +detinv * (A(2,1)*A(3,2) - A(2,2)*A(3,1))
        B(1,2) = -detinv * (A(1,2)*A(3,3) - A(1,3)*A(3,2))
        B(2,2) = +detinv * (A(1,1)*A(3,3) - A(1,3)*A(3,1))
        B(3,2) = -detinv * (A(1,1)*A(3,2) - A(1,2)*A(3,1))
        B(1,3) = +detinv * (A(1,2)*A(2,3) - A(1,3)*A(2,2))
        B(2,3) = -detinv * (A(1,1)*A(2,3) - A(1,3)*A(2,1))
        B(3,3) = +detinv * (A(1,1)*A(2,2) - A(1,2)*A(2,1))
    END FUNCTION matinv3

!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION identity(n) RESULT(A)
        USE ISO_C_BINDING
        IMPLICIT NONE
        INTEGER, INTENT(IN) :: n
        REAL(8), DIMENSION(n, n) :: A
        CALL identity_c(n, A)
    END FUNCTION identity

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ColemanTransform(rootMOOP, aziAngle, nHarmonic, axTOut, axYOut)
        USE ISO_C_BINDING
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: rootMOOP(3)
        REAL(8), INTENT(IN) :: aziAngle
        INTEGER(4), INTENT(IN) :: nHarmonic
        REAL(8), INTENT(OUT) :: axTOut
        REAL(8), INTENT(OUT) :: axYOut
        CALL colemantransform_c(rootMOOP, aziAngle, nHarmonic, axTOut, axYOut)
    END SUBROUTINE ColemanTransform

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ColemanTransformInverse(axTIn, axYIn, aziAngle, nHarmonic, aziOffset, PitComIPC)
        USE ISO_C_BINDING
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: axTIn
        REAL(8), INTENT(IN) :: axYIn
        REAL(8), INTENT(IN) :: aziAngle
        INTEGER(4), INTENT(IN) :: nHarmonic
        REAL(8), INTENT(IN) :: aziOffset
        REAL(8), INTENT(OUT) :: PitComIPC(3)
        CALL colemantransforminverse_c(axTIn, axYIn, aziAngle, nHarmonic, aziOffset, PitComIPC)
    END SUBROUTINE ColemanTransformInverse

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION CPfunction(CP, lambda)
    ! Paremeterized Cp(lambda) function for a fixed pitch angle. Circumvents the need of importing a look-up table
        IMPLICIT NONE
        
        ! Inputs
        REAL(DbKi), INTENT(IN) :: CP(4)    ! Parameters defining the parameterizable Cp(lambda) function
        REAL(DbKi), INTENT(IN) :: lambda    ! Estimated or measured tip-speed ratio input
        
        ! Lookup
        CPfunction = exp(-CP(1)/lambda)*(CP(2)/lambda-CP(3))+CP(4)*lambda
        CPfunction = saturate(CPfunction, 0.001_DbKi, 1.0_DbKi)
        
    END FUNCTION CPfunction

!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION AeroDynTorque(RotSpeed, BldPitch, LocalVar, CntrPar, PerfData, ErrVar)
    ! Function for computing the aerodynamic torque, divided by the effective rotor torque of the turbine, for use in wind speed estimation
        
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, PerformanceData, ErrorVariables
        IMPLICIT NONE
    
        ! Inputs
        TYPE(ControlParameters), INTENT(IN) :: CntrPar
        TYPE(LocalVariables), INTENT(IN) :: LocalVar
        TYPE(PerformanceData), INTENT(IN) :: PerfData
        TYPE(ErrorVariables), INTENT(INOUT) :: ErrVar

        REAL(DbKi), INTENT(IN)  :: RotSpeed
        REAL(DbKi), INTENT(IN)  :: BldPitch
            
        ! Local
        REAL(DbKi) :: RotorArea
        REAL(DbKi) :: Cp
        REAL(DbKi) :: Lambda
        REAL(DbKi) :: WindSpeed

        CHARACTER(*), PARAMETER                 :: RoutineName = 'AeroDynTorque'

        ! Find Torque
        RotorArea = PI*CntrPar%WE_BladeRadius**2
        WindSpeed = MAX(LocalVar%WE_Vw,EPSILON(1.0_DbKi))
        Lambda = RotSpeed*CntrPar%WE_BladeRadius/WindSpeed

        ! Compute Cp
        Cp = interp2d(PerfData%Beta_vec,PerfData%TSR_vec,PerfData%Cp_mat, BldPitch*R2D, Lambda, ErrVar)
        
        AeroDynTorque = 0.5*(CntrPar%WE_RhoAir*RotorArea)*(LocalVar%WE_Vw**3/RotSpeed)*Cp
        AeroDynTorque = MAX(AeroDynTorque, 0.0_DbKi)

        ! Add RoutineName to error message
        IF (ErrVar%aviFAIL < 0) THEN
            ErrVar%ErrMsg = RoutineName//':'//TRIM(ErrVar%ErrMsg)
        ENDIF
        
    END FUNCTION AeroDynTorque
!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION wrap_180(x) 
    ! Function modifies input angle, x, such that -180<=x<=180, preventing windup
        REAL(DbKi), INTENT(IN) :: x         ! angle, degrees

        IF (x .le. -180.0) THEN
            wrap_180 = x + 360.0
        ELSEIF (x .gt. 180.0) THEN
            wrap_180 = x - 360.0
        ELSE
            wrap_180 = x
        ENDIF

    END FUNCTION wrap_180
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
    FUNCTION sigma(x, x0, x1, y0, y1, ErrVar) RESULT(sigma_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: x
        REAL(8), INTENT(IN) :: x0
        REAL(8), INTENT(IN) :: x1
        REAL(8), INTENT(IN) :: y0
        REAL(8), INTENT(IN) :: y1
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        REAL(8) :: sigma_result
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        sigma_result = REAL(sigma_c(x, x0, x1, y0, y1, C_LOC(ErrVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END FUNCTION sigma


!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION unwrap(x, ErrVar) RESULT(y)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: x(:)
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        REAL(8), DIMENSION(SIZE(x)) :: y
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL unwrap_c(x, SIZE(x), C_LOC(ErrVar_view), y)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END FUNCTION unwrap


!-------------------------------------------------------------------------------------------------------------------------------
END MODULE Functions
