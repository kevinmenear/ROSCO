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

! Subroutines:
!           PitchControl: Blade pitch control high level subroutine
!           VariableSpeedControl: Variable speed generator torque control
!           YawRateControl: Nacelle yaw control
!           IPC: Individual pitch control
!           ForeAftDamping: Tower fore-aft damping control
!           FloatingFeedback: Tower fore-aft feedback for floating offshore wind turbines

MODULE ExtControl

    USE, INTRINSIC :: ISO_C_Binding
    USE Functions
    USE ROSCO_Types
    USE SysSubs
    USE Constants

    IMPLICIT NONE



    ABSTRACT INTERFACE
    SUBROUTINE BladedDLL_Legacy_Procedure ( avrSWAP, aviFAIL, accINFILE, avcOUTNAME, avcMSG )  BIND(C)
       USE, INTRINSIC :: ISO_C_Binding

       USE Constants

       REAL(ReKi),             INTENT(INOUT) :: avrSWAP   (*)  !< DATA
       INTEGER(C_INT),         INTENT(INOUT) :: aviFAIL        !< FLAG  (Status set in DLL and returned to simulation code)
       CHARACTER(KIND=C_CHAR), INTENT(IN)    :: accINFILE (*)  !< INFILE
       CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: avcOUTNAME(*)  !< OUTNAME (in:Simulation RootName; out:Name:Units; of logging channels)
       CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: avcMSG    (*)  !< MESSAGE (Message from DLL to simulation code [ErrMsg])
    END SUBROUTINE BladedDLL_Legacy_Procedure

    END INTERFACE


    ! Auto-generated interface for C++ implementation of ExtController
    INTERFACE
        SUBROUTINE extcontroller_c(avrSWAP, CntrPar, LocalVar, ExtDLL, ErrVar) BIND(C, NAME='extcontroller_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: ExtDLL
            TYPE(C_PTR), VALUE :: ErrVar
        END SUBROUTINE extcontroller_c
    END INTERFACE

CONTAINS

    SUBROUTINE ExtController(avrSWAP, CntrPar, LocalVar, ExtDLL, ErrVar)
        USE ISO_C_BINDING
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_copy_scalars_to_controlparameters
        USE vit_localvariables_view, ONLY: localvariables_view_t, vit_populate_localvariables, vit_copy_scalars_to_localvariables
        USE vit_extcontroltype_view, ONLY: extcontroltype_view_t, vit_populate_extcontroltype, vit_copy_scalars_to_extcontroltype
        USE vit_errorvariables_view, ONLY: errorvariables_view_t, vit_populate_errorvariables, vit_copy_scalars_to_errorvariables
        IMPLICIT NONE
        REAL(4), INTENT(INOUT) :: avrSWAP(*)
        TYPE(CONTROLPARAMETERS), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LOCALVARIABLES), INTENT(INOUT), TARGET :: LocalVar
        TYPE(EXTCONTROLTYPE), INTENT(INOUT), TARGET :: ExtDLL
        TYPE(ERRORVARIABLES), INTENT(INOUT), TARGET :: ErrVar
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        TYPE(localvariables_view_t), TARGET :: LocalVar_view
        TYPE(extcontroltype_view_t), TARGET :: ExtDLL_view
        TYPE(errorvariables_view_t), TARGET :: ErrVar_view
        INTEGER(IntKi), PARAMETER :: max_avr_entries = 2000

        ! THE ONE STATEMENT THE C++ CANNOT PERFORM, transcribed verbatim from
        ! the original and placed BEFORE the populators so that C_LOC has real
        ! memory to point at.
        !
        ! `ALLOCATE(ExtDLL%avrSWAP(max_avr_entries))` allocates FORTRAN storage.
        ! A view struct carries a raw pointer and a count; C++ can write through
        ! that pointer and can report a different count, but it cannot create
        ! the allocation. On the first call `ExtDLL%avrSWAP` is unallocated, so
        ! the populator hands the translation a NULL pointer -- and the
        ! translation's first act is to fill 2000 elements.
        !
        ! `vit integrate --auto-allocate` exists for exactly this and CANNOT BE
        ! USED HERE. Three defects, measured, with the artifact kept at
        ! evidence/ExtController/vit_defects/integrate_auto_allocate.wrapper.f90:
        !   1. Its copy-call scan matches ANY `CALL x(..., arg%field, ...)`, so
        !      it hoisted `LoadDynamicLib` AND the external DLL call itself into
        !      this wrapper -- which would call both a second time.
        !   2. The size expression is the local PARAMETER `max_avr_entries`, so
        !      it classifies the allocation `local` and declines it anyway.
        !   3. Its error handling emits `SetErrStat(...)` and `CHARACTER(ErrMsgLen)`,
        !      which are OpenFAST names that do not exist anywhere in ROSCO.
        ! Recorded for the Driver in DECISIONS.md rather than worked around in
        ! the tool's output silently.
        IF (LocalVar%iStatus == 0) THEN
            ALLOCATE(ExtDLL%avrSWAP(max_avr_entries))
        END IF

        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        CALL vit_populate_localvariables(LocalVar, LocalVar_view)
        CALL vit_populate_extcontroltype(ExtDLL, ExtDLL_view)
        CALL vit_populate_errorvariables(ErrVar, ErrVar_view)
        CALL extcontroller_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar_view), C_LOC(ExtDLL_view), C_LOC(ErrVar_view))
        ! Copy modified scalars back from view to Fortran type
        CALL vit_copy_scalars_to_controlparameters(CntrPar_view, CntrPar)
        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)
        CALL vit_copy_scalars_to_extcontroltype(ExtDLL_view, ExtDLL)
        CALL vit_copy_scalars_to_errorvariables(ErrVar_view, ErrVar)
    END SUBROUTINE ExtController


!=================================================================================================================
!=================================================================================================================
!=================================================================================================================


END MODULE ExtControl
