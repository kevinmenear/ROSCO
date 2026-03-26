!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 23:44:31 
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

    ! Auto-generated interface for C++ implementation of PIIController
    INTERFACE
        FUNCTION piicontroller_c(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst) BIND(C, NAME='piicontroller_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: error
            REAL(C_DOUBLE), VALUE :: error2
            REAL(C_DOUBLE), VALUE :: kp
            REAL(C_DOUBLE), VALUE :: ki
            REAL(C_DOUBLE), VALUE :: ki2
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: I0
            TYPE(C_PTR), VALUE :: piP
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: piicontroller_c
        END FUNCTION piicontroller_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------  


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------
SUBROUTINE flapcontrol(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, cntrpar, localvar, objinst) 
        ! Yaw rate controller
        !       Y_ControlMode = 0, No yaw control
        !       Y_ControlMode = 1, Simple yaw rate control using yaw drive
        !       Y_ControlMode = 2, Yaw by IPC (accounted for in IPC subroutine)
    USE rosco_types, ONLY: controlparameters, localvariables, objectinstances 
    USE kgen_utils_mod
    USE kgen_utils_mod
    USE rosco_types, ONLY: kr_rosco_types_localvariables 
    USE rosco_types, ONLY: kr_rosco_types_controlparameters 
    USE rosco_types, ONLY: kr_rosco_types_objectinstances 
    USE rosco_types, ONLY: kv_rosco_types_localvariables 
    USE rosco_types, ONLY: kv_rosco_types_controlparameters 
    USE rosco_types, ONLY: kv_rosco_types_objectinstances 
    
    
    TYPE(controlparameters), INTENT(INOUT) :: cntrpar 
    TYPE(localvariables), INTENT(INOUT) :: localvar 
    TYPE(objectinstances), INTENT(INOUT) :: objinst 
        ! Internal Variables
    INTEGER(KIND=intki) :: k 
        ! Flap control
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
    TYPE(localvariables) :: kgenref_localvar 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
    READ (UNIT = kgen_unit) k 
      
    !extern output variables 
      
    !local output variables 
    CALL kr_rosco_types_localvariables(kgenref_localvar, kgen_unit, "kgenref_localvar", .FALSE.) 

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
                    LocalVar%Flp_Angle(K) = PIIController(-LocalVar%rootMOOPF(K), 0 - LocalVar%Flp_Angle(K), CntrPar%Flp_Kp, CntrPar%Flp_Ki, 0.05_DbKi, -CntrPar%Flp_MaxPit , CntrPar%Flp_MaxPit , LocalVar%DT, 0.0_DbKi, LocalVar%piP, (LocalVar%restart /= 0), objInst%instPI)
                    IF (kgen_mainstage) THEN 
                          
                        !verify init 
                        CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
                        CALL kgen_init_check(check_status, rank=kgen_mpirank) 
                          
                        !extern verify variables 
                          
                        !local verify variables 
                        CALL kv_rosco_types_localvariables("localvar", check_status, localvar, kgenref_localvar) 
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
                    LocalVar%Flp_Angle(K) = PIIController(-LocalVar%rootMOOPF(K), 0 - LocalVar%Flp_Angle(K), CntrPar%Flp_Kp, CntrPar%Flp_Ki, 0.05_DbKi, -CntrPar%Flp_MaxPit , CntrPar%Flp_MaxPit , LocalVar%DT, 0.0_DbKi, LocalVar%piP, (LocalVar%restart /= 0), objInst%instPI)
                        END DO   
                        CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
                        kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
                        IF (check_status%rank==0) THEN 
                            WRITE (*, *) "PIIController : Time per call (usec): ", kgen_measure 
                        END IF   
                    END IF   
                    IF (kgen_warmupstage) THEN 
                    END IF   
                    IF (kgen_evalstage) THEN 
                    END IF   
END SUBROUTINE flapcontrol 
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

    FUNCTION PIIController(error, error2, kp, ki, ki2, minValue, maxValue, DT, I0, piP, reset, inst) RESULT(PIIController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : piParams
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: error
        REAL(DbKi), INTENT(IN) :: error2
        REAL(DbKi), INTENT(IN) :: kp
        REAL(DbKi), INTENT(IN) :: ki
        REAL(DbKi), INTENT(IN) :: ki2
        REAL(DbKi), INTENT(IN) :: minValue
        REAL(DbKi), INTENT(IN) :: maxValue
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: I0
        TYPE(piParams), INTENT(INOUT), TARGET :: piP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi) :: PIIController_result
        PIIController_result = REAL(piicontroller_c(REAL(error, C_DOUBLE), REAL(error2, C_DOUBLE), REAL(kp, C_DOUBLE), REAL(ki, C_DOUBLE), REAL(ki2, C_DOUBLE), REAL(minValue, C_DOUBLE), REAL(maxValue, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(I0, C_DOUBLE), C_LOC(piP), LOGICAL(reset, C_BOOL), inst), DbKi)
    END FUNCTION PIIController
        !-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers