!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-24 18:24:23 
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
! This module contains the primary controller routines


MODULE Controllers

    USE controllerblocks 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 

    USE ISO_C_BINDING
    IMPLICIT NONE 


    ! Auto-generated interface for C++ implementation of StructuralControl
    INTERFACE
        SUBROUTINE structuralcontrol_c(avrSWAP, CntrPar, LocalVar, objInst, ErrVar) BIND(C, NAME='structuralcontrol_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE structuralcontrol_c
    END INTERFACE

CONTAINS
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

    SUBROUTINE StructuralControl(avrSWAP, CntrPar, LocalVar, objInst, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_original_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        ! Stash original Fortran pointers for callee bridges
        vit_original_controlparameters => CntrPar
        BLOCK
            REAL(C_DOUBLE) :: vit_tmp_avrswap(2000)
            vit_tmp_avrswap = REAL(avrSWAP(1:2000), C_DOUBLE)
            CALL structuralcontrol_c(vit_tmp_avrswap, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(ErrVar))
            avrSWAP(1:2000) = REAL(vit_tmp_avrswap, KIND(avrSWAP))
        END BLOCK
    END SUBROUTINE StructuralControl
!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


        !-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers