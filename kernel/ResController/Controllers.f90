!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-25 23:35:28 
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

    ! Auto-generated interface for C++ implementation of ResController
    INTERFACE
        FUNCTION rescontroller_c(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst) BIND(C, NAME='rescontroller_c')
            USE ISO_C_BINDING
            REAL(C_DOUBLE), VALUE :: error
            REAL(C_DOUBLE), VALUE :: kp
            REAL(C_DOUBLE), VALUE :: ki
            REAL(C_DOUBLE), VALUE :: freq
            REAL(C_DOUBLE), VALUE :: minValue
            REAL(C_DOUBLE), VALUE :: maxValue
            REAL(C_DOUBLE), VALUE :: DT
            TYPE(C_PTR), VALUE :: resP
            LOGICAL(C_BOOL), VALUE :: reset
            INTEGER(C_INT), INTENT(INOUT) :: inst
            REAL(C_DOUBLE) :: rescontroller_c
        END FUNCTION rescontroller_c
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


SUBROUTINE activewakecontrol(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, cntrpar, localvar, objinst) 
        ! Active wake controller
        !       AWC_Mode = 0, No active wake control
        !       AWC_Mode = 1, SNL active wake control
        !       AWC_Mode = 2, Coleman Transform-based active wake control
        !       AWC_Mode = 3, Closed-loop Proportional-integral (PI) active wake control
        !       AWC_Mode = 4, Closed-loop Proportional-resonant (PR) active wake control
        !       AWC_Mode = 5, Strouhal transformation based closed-loop active wake control
    USE rosco_types, ONLY: controlparameters, localvariables, objectinstances 
    USE kgen_utils_mod
    USE kgen_utils_mod
    USE rosco_types, ONLY: kr_rosco_types_controlparameters 
    USE rosco_types, ONLY: kr_rosco_types_localvariables 
    USE rosco_types, ONLY: kr_rosco_types_objectinstances 
    USE rosco_types, ONLY: kv_rosco_types_controlparameters 
    USE rosco_types, ONLY: kv_rosco_types_localvariables 
    USE rosco_types, ONLY: kv_rosco_types_objectinstances 
    USE kgen_utils_mod

    TYPE(controlparameters), INTENT(INOUT) :: cntrpar 
    TYPE(localvariables), INTENT(INOUT) :: localvar 
    TYPE(objectinstances), INTENT(INOUT) :: objinst 
        ! Local vars

        ! COMPLEX(DbKi)              :: complexI = (0.0, 1.0)  ! VIT: removed (complex split)
    INTEGER(KIND=intki) :: imode 
    REAL(KIND=dbki), dimension(2) :: awc_tiltyaw = [0.0, 0.0] 
    REAL(KIND=dbki), dimension(2) :: error = [0.0, 0.0] 
        ! Compute the AWC pitch settings, complex number approach
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
    REAL(KIND=dbki), dimension(2) :: kgenref_awc_tiltyaw 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
    READ (UNIT = kgen_unit) imode 
    READ (UNIT = kgen_unit) kgen_istrue 
    IF (kgen_istrue) THEN 
        READ (UNIT = kgen_unit) kgen_array_sum 
        READ (UNIT = kgen_unit) awc_tiltyaw 
        CALL kgen_array_sumcheck("awc_tiltyaw", kgen_array_sum, DBLE(SUM(awc_tiltyaw, mask=(awc_tiltyaw .eq. awc_tiltyaw))), &
        &.TRUE.) 
    END IF   
    READ (UNIT = kgen_unit) kgen_istrue 
    IF (kgen_istrue) THEN 
        READ (UNIT = kgen_unit) kgen_array_sum 
        READ (UNIT = kgen_unit) error 
        CALL kgen_array_sumcheck("error", kgen_array_sum, DBLE(SUM(error, mask=(error .eq. error))), .TRUE.) 
    END IF   
      
    !extern output variables 
      
    !local output variables 
    READ (UNIT = kgen_unit) kgen_istrue 
    IF (kgen_istrue) THEN 
        READ (UNIT = kgen_unit) kgen_array_sum 
        READ (UNIT = kgen_unit) kgenref_awc_tiltyaw 
        CALL kgen_array_sumcheck("kgenref_awc_tiltyaw", kgen_array_sum, DBLE(SUM(kgenref_awc_tiltyaw, mask=(kgenref_awc_tiltyaw &
        &.eq. kgenref_awc_tiltyaw))), .TRUE.) 
    END IF   


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
                        AWC_TiltYaw(Imode) = ResController(Error(Imode), CntrPar%AWC_CntrGains(1), CntrPar%AWC_CntrGains(2), CntrPar%AWC_freq(Imode), & 
                                                            CntrPar%PC_MinPit, CntrPar%PC_MaxPit, LocalVar%DT, LocalVar%resP, (LocalVar%restart /= 0), objInst%instRes)
                        IF (kgen_mainstage) THEN 
                              
                            !verify init 
                            CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
                            CALL kgen_init_check(check_status, rank=kgen_mpirank) 
                              
                            !extern verify variables 
                              
                            !local verify variables 
                            CALL kv_activewakecontrol_real__dbki_dim1("awc_tiltyaw", check_status, awc_tiltyaw, &
                            &kgenref_awc_tiltyaw) 
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
                        AWC_TiltYaw(Imode) = ResController(Error(Imode), CntrPar%AWC_CntrGains(1), CntrPar%AWC_CntrGains(2), CntrPar%AWC_freq(Imode), & 
                                                            CntrPar%PC_MinPit, CntrPar%PC_MaxPit, LocalVar%DT, LocalVar%resP, (LocalVar%restart /= 0), objInst%instRes)
                            END DO   
                            CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
                            kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
                            IF (check_status%rank==0) THEN 
                                WRITE (*, *) "ResController : Time per call (usec): ", kgen_measure 
                            END IF   
                        END IF   
                        IF (kgen_warmupstage) THEN 
                        END IF   
                        IF (kgen_evalstage) THEN 
                        END IF   


                          
                        CONTAINS 
                          

                        !verify state subroutine for kv_activewakecontrol_real__dbki_dim1 
                        RECURSIVE SUBROUTINE kv_activewakecontrol_real__dbki_dim1(varname, check_status, var, kgenref_var) 
                            CHARACTER(LEN=*), INTENT(IN) :: varname 
                            TYPE(check_t), INTENT(INOUT) :: check_status 
                            REAL(KIND=dbki), INTENT(IN), DIMENSION(:) :: var, kgenref_var 
                            INTEGER :: check_result 
                            LOGICAL :: is_print = .FALSE. 
                              
                            INTEGER :: idx1 
                            INTEGER :: n 
                            real(KIND=dbki) :: nrmsdiff, rmsdiff 
                            real(KIND=dbki), ALLOCATABLE :: buf1(:), buf2(:) 
                              
                            check_status%numTotal = check_status%numTotal + 1 
                              
                            IF (ALL(var == kgenref_var)) THEN 
                                check_status%numIdentical = check_status%numIdentical + 1 
                                IF (kgen_verboseLevel > 1) THEN 
                                    IF (check_status%rank == 0) THEN 
                                        WRITE (*, *) trim(adjustl(varname)), " is IDENTICAL." 
                                    END IF   
                                END IF   
                                check_result = CHECK_IDENTICAL 
                                WRITE(*, *) "[VIT_ARRAY] ", trim(adjustl(varname)), " | IDENTICAL | size=", SIZE(var)
                            ELSE 
                                ALLOCATE (buf1(SIZE(var,dim=1))) 
                                ALLOCATE (buf2(SIZE(var,dim=1))) 
                                n = SIZE(var) 
                                WHERE ( ABS(kgenref_var) > kgen_minvalue ) 
                                    buf1 = ((var-kgenref_var)/kgenref_var)**2 
                                    buf2 = (var-kgenref_var)**2 
                                ELSEWHERE 
                                    buf1 = (var-kgenref_var)**2 
                                    buf2 = buf1 
                                END WHERE   
                                nrmsdiff = SQRT(SUM(buf1)/DBLE(n)) 
                                rmsdiff = SQRT(SUM(buf2)/DBLE(n)) 
                                IF (rmsdiff > kgen_tolerance) THEN 
                                    check_status%numOutTol = check_status%numOutTol + 1 
                                    IF (kgen_verboseLevel > 0) THEN 
                                        IF (check_status%rank == 0) THEN 
                                            WRITE (*, *) trim(adjustl(varname)), " is NOT IDENTICAL(out of tolerance)." 
                                        END IF   
                                    END IF   
                                    check_result = CHECK_OUT_TOL 
                                    WRITE(*, *) "[VIT_ARRAY] ", trim(adjustl(varname)), " | OUT_TOL | n_diff=", n, " | rms=", rmsdiff
                                ELSE 
                                    check_status%numInTol = check_status%numInTol + 1 
                                    IF (kgen_verboseLevel > 1) THEN 
                                        IF (check_status%rank == 0) THEN 
                                            WRITE (*, *) trim(adjustl(varname)), " is NOT IDENTICAL(within tolerance)." 
                                        END IF   
                                    END IF   
                                    check_result = CHECK_IN_TOL 
                                    WRITE(*, *) "[VIT_ARRAY] ", trim(adjustl(varname)), " | IN_TOL | n_diff=", n, " | rms=", rmsdiff
                                END IF   
                            END IF   
                            IF (check_result == CHECK_IDENTICAL) THEN 
                                IF (kgen_verboseLevel > 2) THEN 
                                    IF (check_status%rank == 0) THEN 
                                        WRITE (*, *) count( var /= kgenref_var), " of ", size( var ), " elements are different." 
                                        WRITE (*, *) "Average - kernel ", sum(var)/real(size(var)) 
                                        WRITE (*, *) "Average - reference ", sum(kgenref_var)/real(size(kgenref_var)) 
                                        WRITE (*, *) "RMS of difference is ", 0 
                                        WRITE (*, *) "Normalized RMS of difference is ", 0 
                                        WRITE (*, *) "" 
                                    END IF   
                                END IF   
                            ELSE IF (check_result == CHECK_OUT_TOL) THEN 
                                IF (kgen_verboseLevel > 0) THEN 
                                    IF (check_status%rank == 0) THEN 
                                        WRITE (*, *) count( var /= kgenref_var), " of ", size( var ), " elements are different." 
                                        WRITE (*, *) "Average - kernel ", sum(var)/real(size(var)) 
                                        WRITE (*, *) "Average - reference ", sum(kgenref_var)/real(size(kgenref_var)) 
                                        WRITE (*, *) "RMS of difference is ", rmsdiff 
                                        WRITE (*, *) "Normalized RMS of difference is ", nrmsdiff 
                                        WRITE (*, *) "" 
                                    END IF   
                                END IF   
                            ELSE IF (check_result == CHECK_IN_TOL) THEN 
                                IF (kgen_verboseLevel > 1) THEN 
                                    IF (check_status%rank == 0) THEN 
                                        WRITE (*, *) count( var /= kgenref_var), " of ", size( var ), " elements are different." 
                                        WRITE (*, *) "Average - kernel ", sum(var)/real(size(var)) 
                                        WRITE (*, *) "Average - reference ", sum(kgenref_var)/real(size(kgenref_var)) 
                                        WRITE (*, *) "RMS of difference is ", rmsdiff 
                                        WRITE (*, *) "Normalized RMS of difference is ", nrmsdiff 
                                        WRITE (*, *) "" 
                                    END IF   
                                END IF   
                            END IF   
                              
                        END SUBROUTINE kv_activewakecontrol_real__dbki_dim1 
                          
END SUBROUTINE activewakecontrol 
!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------

!-------------------------------------------------------------------------------------------------------------------------------


!-------------------------------------------------------------------------------------------------------------------------------


        !-------------------------------------------------------------------------------------------------------------------------------

    FUNCTION ResController(error, kp, ki, freq, minValue, maxValue, DT, resP, reset, inst) RESULT(ResController_result)
        USE ISO_C_BINDING
        USE ROSCO_Types, ONLY : resParams
        IMPLICIT NONE
        REAL(DbKi), INTENT(IN) :: error
        REAL(DbKi), INTENT(IN) :: kp
        REAL(DbKi), INTENT(IN) :: ki
        REAL(DbKi), INTENT(IN) :: freq
        REAL(DbKi), INTENT(IN) :: minValue
        REAL(DbKi), INTENT(IN) :: maxValue
        REAL(DbKi), INTENT(IN) :: DT
        TYPE(resParams), INTENT(INOUT), TARGET :: resP
        LOGICAL, INTENT(IN) :: reset
        INTEGER(IntKi), INTENT(INOUT) :: inst
        REAL(DbKi) :: ResController_result
        ResController_result = REAL(rescontroller_c(REAL(error, C_DOUBLE), REAL(kp, C_DOUBLE), REAL(ki, C_DOUBLE), REAL(freq, C_DOUBLE), REAL(minValue, C_DOUBLE), REAL(maxValue, C_DOUBLE), REAL(DT, C_DOUBLE), C_LOC(resP), LOGICAL(reset, C_BOOL), inst), DbKi)
    END FUNCTION ResController
!-------------------------------------------------------------------------------------------------------------------------------

END MODULE Controllers