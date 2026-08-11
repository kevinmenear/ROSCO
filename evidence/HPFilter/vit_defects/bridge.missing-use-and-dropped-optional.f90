! VIT: Test-validate bridge for HPFilter
! Allows C++ test harness to call the original Fortran function.
! Handles C↔Fortran type conversions for derived types and CHARACTER.
    SUBROUTINE hpfilter_f90(InputSignal, DT, CornerFreq, FP_ptr, iStatus, reset, inst, &
        InitialValue, vit_result) &
        BIND(C, NAME='hpfilter_f90')
        USE ISO_C_BINDING
        USE Filters
        USE Constants
        USE Functions
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE, INTENT(IN) :: InputSignal
        REAL(C_DOUBLE), VALUE, INTENT(IN) :: DT
        REAL(C_DOUBLE), VALUE, INTENT(IN) :: CornerFreq
        TYPE(C_PTR), VALUE :: FP_ptr
        INTEGER(C_INT), VALUE, INTENT(IN) :: iStatus
        INTEGER(C_INT), VALUE, INTENT(IN) :: reset
        INTEGER(C_INT), INTENT(INOUT) :: inst
        REAL(C_DOUBLE), VALUE, INTENT(IN) :: InitialValue
        REAL(C_DOUBLE), INTENT(OUT) :: vit_result
        TYPE(FilterParameters), POINTER :: FP
        LOGICAL :: reset_f
        CALL C_F_POINTER(FP_ptr, FP)
        reset_f = (reset /= 0)
        vit_result = HPFilter(InputSignal, DT, CornerFreq, FP, iStatus, reset_f, inst, InitialValue)
    END SUBROUTINE hpfilter_f90