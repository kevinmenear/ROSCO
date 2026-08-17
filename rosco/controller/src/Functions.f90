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


    ! Auto-generated interface for C++ implementation of wrap_180
    INTERFACE
        FUNCTION wrap_180_c(x) BIND(C, NAME='wrap_180_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: x
            REAL(C_DOUBLE) :: wrap_180_c
        END FUNCTION wrap_180_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of wrap_360
    INTERFACE
        FUNCTION wrap_360_c(x) BIND(C, NAME='wrap_360_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: x
            REAL(C_DOUBLE) :: wrap_360_c
        END FUNCTION wrap_360_c
    END INTERFACE


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


    ! Auto-generated interface for C++ implementation of ratelimit
    INTERFACE
        FUNCTION ratelimit_c(inputSignal, minRate, maxRate, DT, reset, rlP, inst, has_ResetValue, ResetValue) BIND(C, NAME='ratelimit_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: inputSignal
            REAL(C_DOUBLE), VALUE :: minRate
            REAL(C_DOUBLE), VALUE :: maxRate
            REAL(C_DOUBLE), VALUE :: DT
            INTEGER(C_INT), VALUE :: reset
            TYPE(C_PTR), VALUE :: rlP
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_ResetValue
            REAL(C_DOUBLE), VALUE :: ResetValue
            REAL(C_DOUBLE) :: ratelimit_c
        END FUNCTION ratelimit_c
    END INTERFACE


    ! Auto-generated interface for C++ implementation of AeroDynTorque
    INTERFACE
        FUNCTION aerodyntorque_c(RotSpeed, BldPitch, LocalVar, CntrPar, PerfData, ErrVar) BIND(C, NAME='aerodyntorque_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: RotSpeed
            REAL(C_DOUBLE), VALUE :: BldPitch
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: PerfData
            TYPE(C_PTR), VALUE :: ErrVar
            REAL(C_DOUBLE) :: aerodyntorque_c
        END FUNCTION aerodyntorque_c
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
    FUNCTION ratelimit(inputSignal, minRate, maxRate, DT, reset, rlP, inst, ResetValue) RESULT(ratelimit_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : rlParams
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: inputSignal
        REAL(8), INTENT(IN) :: minRate
        REAL(8), INTENT(IN) :: maxRate
        REAL(8), INTENT(IN) :: DT
        LOGICAL, INTENT(IN) :: reset
        TYPE(RLPARAMS), INTENT(INOUT), TARGET :: rlP
        INTEGER(4), INTENT(INOUT) :: inst
        REAL(8), INTENT(IN), OPTIONAL :: ResetValue
        REAL(8) :: ratelimit_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_ResetValue_flag
        REAL(C_DOUBLE) :: ResetValue_val

        has_ResetValue_flag = 0
        ResetValue_val = 0.0D0
        IF (PRESENT(ResetValue)) THEN
            has_ResetValue_flag = 1
            ResetValue_val = REAL(ResetValue, C_DOUBLE)
        END IF
        ratelimit_result = REAL(ratelimit_c(inputSignal, minRate, maxRate, DT, MERGE(1_C_INT, 0_C_INT, reset), C_LOC(rlP), inst, has_ResetValue_flag, ResetValue_val), 8)
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
    FUNCTION interp2d(xData, yData, zData, xq, yq, ErrVar) RESULT(interp2d_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: xData(:)
        REAL(8), INTENT(IN) :: yData(:)
        REAL(8), INTENT(IN) :: zData(:,:)
        REAL(8), INTENT(IN) :: xq
        REAL(8), INTENT(IN) :: yq
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        REAL(8) :: interp2d_result
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        interp2d_result = REAL(interp2d_c(xData, SIZE(xData), yData, SIZE(yData), zData, SIZE(zData, 1), SIZE(zData, 2), xq, yq, C_LOC(ErrVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
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
    FUNCTION AeroDynTorque(RotSpeed, BldPitch, LocalVar, CntrPar, PerfData, ErrVar) RESULT(AeroDynTorque_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, PerformanceData, ErrorVariables
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_performancedata_view, ONLY: performancedata_view_t, vit_populate_performancedata, vit_copy_scalars_to_performancedata
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: RotSpeed
        REAL(8), INTENT(IN) :: BldPitch
        TYPE(LOCALVARIABLES), INTENT(IN), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(IN), TARGET :: CntrPar
        TYPE(PERFORMANCEDATA), INTENT(IN), TARGET :: PerfData
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        REAL(8) :: AeroDynTorque_result
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(performancedata_view_t), TARGET :: PerfData_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_performancedata(PerfData, PerfData_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        AeroDynTorque_result = REAL(aerodyntorque_c(RotSpeed, BldPitch, C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(PerfData_view), C_LOC(ErrVar_view)), 8)
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END FUNCTION AeroDynTorque
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION wrap_180(x) RESULT(wrap_180_result)
        USE ISO_C_BINDING
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: x
        REAL(8) :: wrap_180_result
        wrap_180_result = REAL(wrap_180_c(x), 8)
    END FUNCTION wrap_180
!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION wrap_360(x) RESULT(wrap_360_result)
        USE ISO_C_BINDING
        IMPLICIT NONE
        REAL(8), INTENT(IN) :: x
        REAL(8) :: wrap_360_result
        wrap_360_result = REAL(wrap_360_c(x), 8)
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
