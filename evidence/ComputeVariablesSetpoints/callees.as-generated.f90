! VIT: callee bridges for the ComputeVariablesSetpoints test-validate harness.
! Each `<callee>_c` calls the ORIGINAL Fortran, so both sides of the
! comparison share one callee implementation and a mismatch is
! attributable to the unit under test.

! VIT: Kernel callee bridge for LPFilter
! Allows C++ translations to call the original Fortran function.
    FUNCTION lpfilter_bridge(InputSignal, DT, CornerFreq, FP, iStatus, reset, inst, has_InitialValue, InitialValue) &
        BIND(C, NAME='lpfilter_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Filters, ONLY : LPFilter
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: InputSignal
        REAL(C_DOUBLE), VALUE :: DT
        REAL(C_DOUBLE), VALUE :: CornerFreq
        TYPE(C_PTR), VALUE :: FP
        INTEGER(C_INT), VALUE :: iStatus
        INTEGER(C_INT), VALUE :: reset
        INTEGER(C_INT), INTENT(INOUT) :: inst
        INTEGER(C_INT), VALUE :: has_InitialValue
        REAL(C_DOUBLE), VALUE :: InitialValue
        REAL(C_DOUBLE) :: bridge_result
        TYPE(FilterParameters), POINTER :: FP_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(FP, FP_f)
        IF (has_InitialValue /= 0) THEN
        bridge_result = REAL(LPFilter(InputSignal, DT, CornerFreq, FP_f, iStatus, (reset /= 0), inst, InitialValue), C_DOUBLE)
        ELSE
        bridge_result = REAL(LPFilter(InputSignal, DT, CornerFreq, FP_f, iStatus, (reset /= 0), inst), C_DOUBLE)
        END IF
    END FUNCTION lpfilter_bridge
! VIT: Kernel callee bridge for interp1d
! Allows C++ translations to call the original Fortran function.
    FUNCTION interp1d_bridge(xData, n_xData, yData, n_yData, xq, ErrVar) &
        BIND(C, NAME='interp1d_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY : interp1d
        USE ROSCO_Types, ONLY : ErrorVariables
        USE vit_errorvariables_view, ONLY: vit_original_errorvariables, errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables, &
            vit_direct_errorvariables, vit_view_in_errorvariables, vit_view_out_errorvariables
        IMPLICIT NONE
        REAL(C_DOUBLE), INTENT(IN) :: xData(*)
        INTEGER(C_INT), VALUE :: n_xData
        REAL(C_DOUBLE), INTENT(IN) :: yData(*)
        INTEGER(C_INT), VALUE :: n_yData
        REAL(C_DOUBLE), VALUE :: xq
        TYPE(C_PTR), VALUE :: ErrVar
        REAL(C_DOUBLE) :: bridge_result
        TYPE(errorvariables_view_t), POINTER :: ErrVar_view
        LOGICAL :: vit_direct_ErrVar
        ! Flush C++ modifications to Fortran before callee call
        CALL C_F_POINTER(ErrVar, ErrVar_view)
        vit_direct_ErrVar = .NOT. ASSOCIATED(vit_original_errorvariables)
        IF (vit_direct_ErrVar) THEN
            vit_original_errorvariables => vit_direct_errorvariables
            CALL vit_view_in_errorvariables(ErrVar_view, vit_original_errorvariables)
        ELSE
            CALL vit_copy_scalars_to_errorvariables(ErrVar_view, vit_original_errorvariables)
        END IF
        bridge_result = REAL(interp1d(xData(1:n_xData), yData(1:n_yData), xq, vit_original_errorvariables), C_DOUBLE)
        ! Re-sync view struct from Fortran type after callee modified it
        CALL C_F_POINTER(ErrVar, ErrVar_view)
        IF (vit_direct_ErrVar) THEN
            CALL vit_view_out_errorvariables(vit_original_errorvariables, ErrVar_view)
            NULLIFY(vit_original_errorvariables)
        ELSE
            CALL vit_populate_errorvariables(vit_original_errorvariables, ErrVar_view)
        END IF
    END FUNCTION interp1d_bridge
! VIT: Kernel callee bridge for RefSpeedExclusion
! Allows C++ translations to call the original Fortran function.
    SUBROUTINE refspeedexclusion_bridge(LocalVar, CntrPar, objInst, DebugVar) &
        BIND(C, NAME='refspeedexclusion_c')
        USE ISO_C_BINDING
        USE ControllerBlocks, ONLY : RefSpeedExclusion
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, DebugVariables, ObjectInstances
        USE vit_localvariables_view, ONLY: vit_original_localvariables, localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables, &
            vit_direct_localvariables, vit_view_in_localvariables, vit_view_out_localvariables
        USE vit_controlparameters_view, ONLY: vit_original_controlparameters
        IMPLICIT NONE
        TYPE(C_PTR), VALUE :: LocalVar
        TYPE(C_PTR), VALUE :: CntrPar
        TYPE(C_PTR), VALUE :: objInst
        TYPE(C_PTR), VALUE :: DebugVar
        TYPE(ObjectInstances), POINTER :: objInst_f
        TYPE(DebugVariables), POINTER :: DebugVar_f
        TYPE(localvariables_view_t), POINTER :: LocalVar_view
        LOGICAL :: vit_direct_LocalVar
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(objInst, objInst_f)
        CALL C_F_POINTER(DebugVar, DebugVar_f)
        ! Flush C++ modifications to Fortran before callee call
        CALL C_F_POINTER(LocalVar, LocalVar_view)
        vit_direct_LocalVar = .NOT. ASSOCIATED(vit_original_localvariables)
        IF (vit_direct_LocalVar) THEN
            vit_original_localvariables => vit_direct_localvariables
            CALL vit_view_in_localvariables(LocalVar_view, vit_original_localvariables)
        ELSE
            CALL vit_copy_scalars_to_localvariables(LocalVar_view, vit_original_localvariables)
        END IF
        CALL RefSpeedExclusion(vit_original_localvariables, vit_original_controlparameters, objInst_f, DebugVar_f)
        ! Re-sync view struct from Fortran type after callee modified it
        CALL C_F_POINTER(LocalVar, LocalVar_view)
        IF (vit_direct_LocalVar) THEN
            CALL vit_view_out_localvariables(vit_original_localvariables, LocalVar_view)
            NULLIFY(vit_original_localvariables)
        ELSE
            CALL vit_populate_localvariables(vit_original_localvariables, LocalVar_view)
        END IF
    END SUBROUTINE refspeedexclusion_bridge
! VIT: Kernel callee bridge for saturate
! Allows C++ translations to call the original Fortran function.
    FUNCTION saturate_bridge(inputValue, minValue, maxValue) &
        BIND(C, NAME='saturate_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Functions, ONLY : saturate
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: inputValue
        REAL(C_DOUBLE), VALUE :: minValue
        REAL(C_DOUBLE), VALUE :: maxValue
        REAL(C_DOUBLE) :: bridge_result
        bridge_result = REAL(saturate(inputValue, minValue, maxValue), C_DOUBLE)
    END FUNCTION saturate_bridge
