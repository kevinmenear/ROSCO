! VIT: Kernel callee bridge for PIDController
! Allows C++ translations to call the original Fortran function.
    FUNCTION pidcontroller_bridge(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP, reset, objInst, LocalVar) &
        BIND(C, NAME='pidcontroller_c') RESULT(bridge_result)
        USE ISO_C_BINDING
        USE Controllers, ONLY: PIDController
        USE ROSCO_Types, ONLY : piParams, LocalVariables, ObjectInstances
        IMPLICIT NONE
        REAL(C_DOUBLE), VALUE :: error
        REAL(C_DOUBLE), VALUE :: kp
        REAL(C_DOUBLE), VALUE :: ki
        REAL(C_DOUBLE), VALUE :: kd
        REAL(C_DOUBLE), VALUE :: tf
        REAL(C_DOUBLE), VALUE :: minValue
        REAL(C_DOUBLE), VALUE :: maxValue
        REAL(C_DOUBLE), VALUE :: DT
        REAL(C_DOUBLE), VALUE :: I0
        TYPE(C_PTR), VALUE :: piP
        LOGICAL(C_BOOL), VALUE :: reset
        TYPE(C_PTR), VALUE :: objInst
        TYPE(C_PTR), VALUE :: LocalVar
        REAL(C_DOUBLE) :: bridge_result
        TYPE(piParams), POINTER :: piP_f
        TYPE(ObjectInstances), POINTER :: objInst_f
        TYPE(LocalVariables), POINTER :: LocalVar_f
        ! Convert C pointers to Fortran pointers
        CALL C_F_POINTER(piP, piP_f)
        CALL C_F_POINTER(objInst, objInst_f)
        CALL C_F_POINTER(LocalVar, LocalVar_f)
        bridge_result = REAL(PIDController(error, kp, ki, kd, tf, minValue, maxValue, DT, I0, piP_f, LOGICAL(reset), objInst_f, LocalVar_f), C_DOUBLE)
    END FUNCTION pidcontroller_bridge