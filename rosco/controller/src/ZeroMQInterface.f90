module ZeroMQInterface
   USE, INTRINSIC :: ISO_C_BINDING, only: C_CHAR, C_DOUBLE, C_NULL_CHAR
   IMPLICIT NONE
   ! 


    ! Auto-generated interface for C++ implementation of UpdateZeroMQ
    INTERFACE
        SUBROUTINE updatezeromq_c(LocalVar, CntrPar, ErrVar) BIND(C, NAME='updatezeromq_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE updatezeromq_c
    END INTERFACE

CONTAINS
    SUBROUTINE UpdateZeroMQ(LocalVar, CntrPar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables, ControlParameters, ErrorVariables
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL updatezeromq_c(C_LOC(LocalVar_view), C_LOC(CntrPar_view), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE UpdateZeroMQ
end module ZeroMQInterface
