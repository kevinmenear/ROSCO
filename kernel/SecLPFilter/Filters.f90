!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-27 18:20:10 
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
! This module contains all the filters and related subroutines
! Filters:
!       LPFilter: Low-pass filter
!       SecLPFilter: Second order low-pass filter
!       HPFilter: High-pass filter
!       NotchFilter: Notch filter
!       NotchFilterSlopes: Notch Filter with descending slopes
!       PreFilterMeasuredSignals: Pre-filter signals during each run iteration


MODULE Filters
!...............................................................................................................................
    USE functions 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 
    USE ISO_C_BINDING
    IMPLICIT NONE 
! VIT: removed invalid PUBLIC statement

    ! Auto-generated interface for C++ implementation of SecLPFilter
    INTERFACE
        FUNCTION seclpfilter_c(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, has_InitialValue, InitialValue) BIND(C, NAME='seclpfilter_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: InputSignal
            REAL(C_DOUBLE), VALUE :: DT
            REAL(C_DOUBLE), VALUE :: CornerFreq
            REAL(C_DOUBLE), VALUE :: Damp
            TYPE(C_PTR), VALUE :: FP
            INTEGER(C_INT), VALUE :: iStatus
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            INTEGER(C_INT), VALUE :: has_InitialValue
            REAL(C_DOUBLE), VALUE :: InitialValue
            REAL(C_DOUBLE) :: seclpfilter_c
        END FUNCTION seclpfilter_c
    END INTERFACE

CONTAINS
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
    FUNCTION SecLPFilter(InputSignal, DT, CornerFreq, Damp, FP, iStatus, reset, inst, InitialValue) RESULT(SecLPFilter_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : FilterParameters
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: InputSignal
        REAL(DbKi), INTENT(IN) :: DT
        REAL(DbKi), INTENT(IN) :: CornerFreq
        REAL(DbKi), INTENT(IN) :: Damp
        TYPE(FilterParameters), INTENT(INOUT), TARGET :: FP
        INTEGER(IntKi), INTENT(IN) :: iStatus
        LOGICAL(4), INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi), INTENT(IN), OPTIONAL :: InitialValue
        REAL(DbKi) :: SecLPFilter_result

        ! Local variables for OPTIONAL args
        INTEGER(C_INT) :: has_InitialValue_flag
        REAL(C_DOUBLE) :: InitialValue_val

        has_InitialValue_flag = 0
        InitialValue_val = 0.0D0
        IF (PRESENT(InitialValue)) THEN
            has_InitialValue_flag = 1
            InitialValue_val = REAL(InitialValue, C_DOUBLE)
        END IF
        SecLPFilter_result = REAL(seclpfilter_c(REAL(InputSignal, C_DOUBLE), REAL(DT, C_DOUBLE), REAL(CornerFreq, C_DOUBLE), REAL(Damp, C_DOUBLE), C_LOC(FP), INT(iStatus, C_INT), LOGICAL(reset, C_BOOL), inst, has_InitialValue_flag, InitialValue_val), DbKi)
    END FUNCTION SecLPFilter
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------
    SUBROUTINE prefiltermeasuredsignals(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, cntrpar, localvar, objinst) 
    ! Prefilter shared measured wind turbine signals

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
        ! If there's an error, don't even try to run
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
          
        !extern output variables 
          
        !local output variables 
        CALL kr_rosco_types_localvariables(kgenref_localvar, kgen_unit, "kgenref_localvar", .FALSE.) 


        ! Filter the HSS (generator) and LSS (rotor) speed measurement:
        ! Apply Low-Pass Filter (choice between first- and second-order low-pass filter)

        ! Apply Notch Fitler to Gen Speed 
        

        ! Filtering the tower fore-aft acceleration signal 
        ! Force to start at 0


        ! Low pass
        !$kgen begin_callsite SecLPFilter

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
        LocalVar%NacIMU_FA_AccF = SecLPFilter(LocalVar%NacIMU_FA_RAcc, LocalVar%DT, CntrPar%F_FlCornerFreq(1), CntrPar%F_FlCornerFreq(2), LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instSecLPF) ! Fixed Damping
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
        LocalVar%NacIMU_FA_AccF = SecLPFilter(LocalVar%NacIMU_FA_RAcc, LocalVar%DT, CntrPar%F_FlCornerFreq(1), CntrPar%F_FlCornerFreq(2), LocalVar%FP, LocalVar%iStatus, (LocalVar%restart /= 0), objInst%instSecLPF) ! Fixed Damping
            END DO   
            CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
            kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
            IF (check_status%rank==0) THEN 
                WRITE (*, *) "SecLPFilter : Time per call (usec): ", kgen_measure 
            END IF   
        END IF   
        IF (kgen_warmupstage) THEN 
        END IF   
        IF (kgen_evalstage) THEN 
        END IF   
        !$kgen end_callsite
        ! High pass
        
        ! Notch filters
        

        ! FA acc for ForeAft damping, condition matches whether it's used in Controllers.f90
        

        ! Filter Wind Speed Estimator Signal
        
        ! Blade root bending moment for IPC


        ! Control commands (used by WSE, mostly)


        ! Wind vane signal

        ! Debug Variables

    END SUBROUTINE prefiltermeasuredsignals 
    END MODULE Filters