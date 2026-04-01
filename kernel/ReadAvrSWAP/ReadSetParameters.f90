!KGEN-generated Fortran source file 
  
!Generated at : 2026-04-01 19:32:32 
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
! Read and set the parameters used by the controller


MODULE ReadSetParameters


    USE functions 
    USE rosco_helpers 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 
    USE ISO_C_BINDING
    IMPLICIT NONE 



    ! Auto-generated interface for C++ implementation of ReadAvrSWAP
    INTERFACE
        SUBROUTINE readavrswap_c(avrSWAP, LocalVar, CntrPar, ErrVar) BIND(C, NAME='readavrswap_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE readavrswap_c
    END INTERFACE

CONTAINS
 ! -----------------------------------------------------------------------------------
    ! Read avrSWAP array passed from ServoDyn    
    SUBROUTINE ReadAvrSWAP(avrSWAP, LocalVar, CntrPar, ErrVar)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : LocalVariables
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_original_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ControlParameters), INTENT(IN), TARGET :: CntrPar
        TYPE(ErrorVariables), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        ! Stash original Fortran pointers for callee bridges
        vit_original_controlparameters => CntrPar
        CALL readavrswap_c(avrSWAP, C_LOC(LocalVar), C_LOC(CntrPar_view), C_LOC(ErrVar))
    END SUBROUTINE ReadAvrSWAP
! -----------------------------------------------------------------------------------
    ! Define parameters for control actions


    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file
    ! Also, all computed CntrPar%* parameters should be computed in this subroutine
    


    ! -----------------------------------------------------------------------------------
    ! Read all constant control parameters from DISCON.IN parameter file


    ! -----------------------------------------------------------------------------------
    ! Check for errors before any execution


    
END MODULE ReadSetParameters