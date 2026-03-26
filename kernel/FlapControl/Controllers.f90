!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 20:53:08 
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


    ! Auto-generated interface for C++ implementation of FlapControl
    INTERFACE
        SUBROUTINE flapcontrol_c(avrSWAP, CntrPar, LocalVar, objInst) BIND(C, NAME='flapcontrol_c')
            USE ISO_C_BINDING
            REAL(C_FLOAT), INTENT(INOUT) :: avrSWAP(*)
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE flapcontrol_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------  


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE FlapControl(avrSWAP, CntrPar, LocalVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_original_controlparameters
        IMPLICIT NONE
        REAL(ReKi), INTENT(INOUT) :: avrSWAP(*)
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        ! Stash original Fortran pointers for callee bridges
        vit_original_controlparameters => CntrPar
        CALL flapcontrol_c(avrSWAP, C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst))
    END SUBROUTINE FlapControl
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
    REAL(DbKi) FUNCTION PIController(error, kp, ki, minValue, maxValue, DT, I0, piP, reset, inst)
        USE rosco_types, ONLY: piparams 
    ! PI controller, with output saturation
        USE rosco_types, ONLY: kr_rosco_types_piparams 
        USE rosco_types, ONLY: kv_rosco_types_piparams 


        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi),    INTENT(IN)         :: error
        REAL(DbKi),    INTENT(IN)         :: kp
        REAL(DbKi),    INTENT(IN)         :: ki
        REAL(DbKi),    INTENT(IN)         :: minValue
        REAL(DbKi),    INTENT(IN)         :: maxValue
        REAL(DbKi),    INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)      :: inst
        REAL(DbKi),    INTENT(IN)         :: I0
        TYPE(piParams), INTENT(INOUT)  :: piP
        LOGICAL,    INTENT(IN)         :: reset     
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                         :: PTerm                                        ! Proportional term
        ! Initialize persistent variables/arrays, and set inital condition for integrator term

        IF (reset) THEN
            piP%ITerm(inst) = I0
            piP%ITermLast(inst) = I0
            
            PIController = I0
        ELSE
            PTerm = kp*error
            piP%ITerm(inst) = piP%ITerm(inst) + DT*ki*error
            piP%ITerm(inst) = saturate(piP%ITerm(inst), minValue, maxValue)
            PIController = saturate(PTerm + piP%ITerm(inst), minValue, maxValue)
        
            piP%ITermLast(inst) = piP%ITerm(inst)
        END IF
        inst = inst + 1
        
    END FUNCTION PIController
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

    REAL(DbKi) FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst)
    ! PI controller, with output saturation. 
    ! Added error2 term for additional integral control input
        USE rosco_types, ONLY: piparams 
        USE rosco_types, ONLY: kr_rosco_types_piparams 
        USE rosco_types, ONLY: kv_rosco_types_piparams 
        
        IMPLICIT NONE
        ! Allocate Inputs
        REAL(DbKi), INTENT(IN)         :: error
        REAL(DbKi), INTENT(IN)         :: error2
        REAL(DbKi), INTENT(IN)         :: kp
        REAL(DbKi), INTENT(IN)         :: ki2
        REAL(DbKi), INTENT(IN)         :: ki
        REAL(DbKi), INTENT(IN)         :: minValue
        REAL(DbKi), INTENT(IN)         :: maxValue
        REAL(DbKi), INTENT(IN)         :: DT
        INTEGER(IntKi), INTENT(INOUT)   :: inst
        REAL(DbKi), INTENT(IN)         :: I0
        TYPE(piParams), INTENT(INOUT) :: piP
        LOGICAL, INTENT(IN)         :: reset     
        ! Allocate local variables
        INTEGER(IntKi)                      :: i                                            ! Counter for making arrays
        REAL(DbKi)                         :: PTerm                                        ! Proportional term
        ! Initialize persistent variables/arrays, and set inital condition for integrator term

        IF (reset) THEN
            piP%ITerm(inst) = I0
            piP%ITermLast(inst) = I0
            piP%ITerm2(inst) = I0
            piP%ITermLast2(inst) = I0
            
            PIIController = I0
        ELSE
            PTerm = kp*error
            piP%ITerm(inst) = piP%ITerm(inst) + DT*ki*error
            piP%ITerm2(inst) = piP%ITerm2(inst) + DT*ki2*error2
            piP%ITerm(inst) = saturate(piP%ITerm(inst), minValue, maxValue)
            piP%ITerm2(inst) = saturate(piP%ITerm2(inst), minValue, maxValue)
            PIIController = PTerm + piP%ITerm(inst) + piP%ITerm2(inst)
            PIIController = saturate(PIIController, minValue, maxValue)
        
            piP%ITermLast(inst) = piP%ITerm(inst)
        END IF
        inst = inst + 1
        
    END FUNCTION PIIController
        !-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers