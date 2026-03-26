!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 20:41:14 
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
! VIT: removed invalid PUBLIC statement

    ! Auto-generated interface for C++ implementation of ForeAftDamping
    INTERFACE
        SUBROUTINE foreaftdamping_c(CntrPar, LocalVar, objInst) BIND(C, NAME='foreaftdamping_c')
            USE ISO_C_BINDING
            TYPE(C_PTR), VALUE :: CntrPar
            TYPE(C_PTR), VALUE :: LocalVar
            TYPE(C_PTR), VALUE :: objInst
        END SUBROUTINE foreaftdamping_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------
SUBROUTINE pitchcontrol(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, cntrpar, localvar, objinst) 
    ! Blade pitch controller, generally maximizes rotor speed below rated (region 2) and regulates rotor speed above rated (region 3)
    !       PC_State = PC_State_Disabled (0), fix blade pitch to fine pitch angle (PC_FinePit)
    !       PC_State = PC_State_Disabled (1), is gain scheduled PI controller 
    USE rosco_types, ONLY: controlparameters, localvariables, objectinstances 
        ! Inputs
    USE kgen_utils_mod
    USE kgen_utils_mod
    USE rosco_types, ONLY: kr_rosco_types_controlparameters 
    USE rosco_types, ONLY: kr_rosco_types_localvariables 
    USE rosco_types, ONLY: kr_rosco_types_objectinstances 
    USE rosco_types, ONLY: kv_rosco_types_controlparameters 
    USE rosco_types, ONLY: kv_rosco_types_localvariables 
    USE rosco_types, ONLY: kv_rosco_types_objectinstances 
        
    TYPE(controlparameters), INTENT(INOUT) :: cntrpar 
    TYPE(localvariables), INTENT(INOUT) :: localvar 
    TYPE(objectinstances), INTENT(INOUT) :: objinst 
        ! Allocate Variables:


        ! ------- Blade Pitch Controller --------
        ! Load PC State
    INTEGER, INTENT(IN) :: kgen_unit 
    REAL(KIND=kgen_dp), INTENT(OUT) :: kgen_measure 
    LOGICAL, INTENT(OUT) :: kgen_isverified 
    CHARACTER(LEN=*), INTENT(IN) :: kgen_filepath 
    LOGICAL :: kgen_istrue 
    REAL(KIND=8) :: kgen_array_sum 
    INTEGER :: kgen_intvar, kgen_ierr 
    INTEGER :: kgen_mpirank, kgen_openmptid, kgen_kernelinvoke 
    LOGICAL :: kgen_evalstage, kgen_warmupstage, kgen_mainstage 
    COMMON / state / kgen_mpirank, kgen_openmptid, kgen_kernelinvoke, kgen_evalstage, kgen_warmupstage, kgen_mainstage 
    INTEGER, PARAMETER :: KGEN_MAXITER = 1 
      
    TYPE(check_t) :: check_status 
    INTEGER*8 :: kgen_start_clock, kgen_stop_clock, kgen_rate_clock 
    REAL(KIND=kgen_dp) :: gkgen_measure 
    TYPE(controlparameters) :: kgenref_cntrpar 
    TYPE(localvariables) :: kgenref_localvar 
    TYPE(objectinstances) :: kgenref_objinst 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
      
    !extern output variables 
      
    !local output variables 
    CALL kr_rosco_types_controlparameters(kgenref_cntrpar, kgen_unit, "kgenref_cntrpar", .FALSE.) 
    CALL kr_rosco_types_localvariables(kgenref_localvar, kgen_unit, "kgenref_localvar", .FALSE.) 
    CALL kr_rosco_types_objectinstances(kgenref_objinst, kgen_unit, "kgenref_objinst", .FALSE.) 


        ! Hold blade pitch at last value
        ! If:
        !   In pre-startup mode (before freewheeling)


        ! Compute (interpolate) the gains based on previously commanded blade pitch angles and lookup table:
        
        ! Compute the collective pitch command associated with the proportional and integral gains:
        
        ! Find individual pitch control contribution


        ! Include tower fore-aft tower vibration damping control
        
    IF (kgen_evalstage) THEN 
    END IF   
    IF (kgen_warmupstage) THEN 
    END IF   
    IF (kgen_mainstage) THEN 
    END IF   
      
    !Uncomment following call statement to turn on perturbation experiment. 
    !Adjust perturbation value and/or kind parameter if required. 
    !CALL kgen_perturb_real( your_variable, 1.0E-15_8 ) 
      
      
    !call to kgen kernel 
            CALL ForeAftDamping(CntrPar, LocalVar, objInst)
            IF (kgen_mainstage) THEN 
                  
                !verify init 
                CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
                CALL kgen_init_check(check_status, rank=kgen_mpirank) 
                  
                !extern verify variables 
                  
                !local verify variables 
                CALL kv_rosco_types_controlparameters("cntrpar", check_status, cntrpar, kgenref_cntrpar) 
                CALL kv_rosco_types_localvariables("localvar", check_status, localvar, kgenref_localvar) 
                CALL kv_rosco_types_objectinstances("objinst", check_status, objinst, kgenref_objinst) 
                IF (check_status%rank == 0) THEN 
                    WRITE (*, *) "" 
                END IF   
                IF (kgen_verboseLevel > 0) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "Number of output variables: ", check_status%numTotal 
                        WRITE (*, *) "Number of identical variables: ", check_status%numIdentical 
                        WRITE (*, *) "Number of non-identical variables within tolerance: ", check_status%numInTol 
                        WRITE (*, *) "Number of non-identical variables out of tolerance: ", check_status%numOutTol 
                        WRITE (*, *) "Tolerance: ", kgen_tolerance 
                    END IF   
                END IF   
                IF (check_status%rank == 0) THEN 
                    WRITE (*, *) "" 
                END IF   
                IF (check_status%numOutTol > 0) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "Verification FAILED with" // TRIM(ADJUSTL(kgen_filepath)) 
                    END IF   
                    check_status%Passed = .FALSE. 
                    kgen_isverified = .FALSE. 
                ELSE 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "Verification PASSED with " // TRIM(ADJUSTL(kgen_filepath)) 
                    END IF   
                    check_status%Passed = .TRUE. 
                    kgen_isverified = .TRUE. 
                END IF   
                IF (check_status%rank == 0) THEN 
                    WRITE (*, *) "" 
                END IF   
                CALL SYSTEM_CLOCK(kgen_start_clock, kgen_rate_clock) 
                DO kgen_intvar = 1, KGEN_MAXITER 
            CALL ForeAftDamping(CntrPar, LocalVar, objInst)
                END DO   
                CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
                kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
                IF (check_status%rank==0) THEN 
                    WRITE (*, *) "ForeAftDamping : Time per call (usec): ", kgen_measure 
                END IF   
            END IF   
            IF (kgen_warmupstage) THEN 
            END IF   
            IF (kgen_evalstage) THEN 
            END IF   
        ! Pitch Saturation
        

        ! FloatingFeedback
        

        ! Saturate collective pitch commands:
        
        ! Combine and saturate all individual pitch commands in software


        ! Open Loop control, use if
        !   Open loop mode active         Using OL blade pitch control      


        ! Active wake control


        ! Shutdown


        ! Place pitch actuator here, so it can be used with or without open-loop
        
       

        ! Hardware saturation: using CntrPar%PC_MinPit


        ! Add pitch actuator fault for blade K


        ! Command the pitch demanded from the last
        ! call to the controller (See Appendix A of Bladed User's Guide):

        ! Add RoutineName to error message


END SUBROUTINE pitchcontrol 
!-------------------------------------------------------------------------------------------------------------------------------  


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE ForeAftDamping(CntrPar, LocalVar, objInst)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : ControlParameters, LocalVariables, ObjectInstances
        USE vit_controlparameters_view, ONLY: controlparameters_view_t, vit_populate_controlparameters, vit_original_controlparameters
        IMPLICIT NONE
        TYPE(ControlParameters), INTENT(INOUT), TARGET :: CntrPar
        TYPE(LocalVariables), INTENT(INOUT), TARGET :: LocalVar
        TYPE(ObjectInstances), INTENT(INOUT), TARGET :: objInst
        TYPE(controlparameters_view_t), TARGET :: CntrPar_view
        ! Populate view structs from Fortran types
        CALL vit_populate_controlparameters(CntrPar, CntrPar_view)
        ! Stash original Fortran pointers for callee bridges
        vit_original_controlparameters => CntrPar
        CALL foreaftdamping_c(C_LOC(CntrPar_view), C_LOC(LocalVar), C_LOC(objInst))
    END SUBROUTINE ForeAftDamping
!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


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


        !-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers