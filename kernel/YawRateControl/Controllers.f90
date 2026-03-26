!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 20:50:55 
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


    ! Auto-generated interface for C++ implementation of YawRateControl
    INTERFACE
        SUBROUTINE yawratecontrol_c(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar) BIND(C, NAME='yawratecontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
            TYPE(C_PTR), VALUE :: DebugVar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE yawratecontrol_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------  


!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE YawRateControl(avrSWAP, CntrPar, LocalVar, objInst, DebugVar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances, DebugVariables, ErrorVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_original_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(DebugVariables), INTENT(INOUT), TARGET :: DebugVar
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        ! Stash original Fortran pointers for callee bridges
        vit_original_controlparameters => CntrPar
        CALL yawratecontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst), C_LOC(DebugVar), C_LOC(ErrVar))
    END SUBROUTINE YawRateControl
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


!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers