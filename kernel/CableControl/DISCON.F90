!KGEN-generated Fortran source file 
  
!Generated at : 2026-03-27 18:30:06 
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
! High level run script
!=======================================================================


SUBROUTINE discon(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, avrswap) 
! DO NOT REMOVE or MODIFY LINES starting with "!DEC$" or "!GCC$"
! !DEC$ specifies attributes for IVF and !GCC$ specifies attributes for gfortran

    USE controllers 
    USE functions 
    USE extcontrol 
    USE kgen_utils_mod, ONLY: kgen_dp, kgen_array_sumcheck 
    USE kgen_utils_mod, ONLY: kgen_perturb_real 
    USE kgen_utils_mod, ONLY: check_t, kgen_init_check, kgen_init_verify, kgen_tolerance, kgen_minvalue, kgen_verboselevel, &
    &CHECK_IDENTICAL, CHECK_IN_TOL, CHECK_OUT_TOL 

    IMPLICIT NONE 
! Enable .dll export
!DEC$ ATTRIBUTES DLLEXPORT :: DISCON
!GCC$ ATTRIBUTES DLLEXPORT :: DISCON
!------------------------------------------------------------------------------------------------------------------------------
! Variable declaration and initialization
!------------------------------------------------------------------------------------------------------------------------------
! Passed Variables:
!REAL(ReKi), INTENT(IN)      :: from_SC(*)       ! DATA from the super controller
!REAL(ReKi), INTENT(INOUT)   :: to_SC(*)         ! DATA to the super controller


    REAL(KIND=reki), INTENT(INOUT) :: avrswap(*) 

    TYPE(controlparameters) :: cntrpar 
    TYPE(localvariables) :: localvar 
    TYPE(objectinstances) :: objinst 
    TYPE(errorvariables) :: errvar 


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
      
    PUBLIC discon 
    TYPE(check_t) :: check_status 
    INTEGER*8 :: kgen_start_clock, kgen_stop_clock, kgen_rate_clock 
    REAL(KIND=kgen_dp) :: gkgen_measure 
    REAL(KIND=reki), dimension(:), allocatable :: kgenref_avrswap 
    TYPE(controlparameters) :: kgenref_cntrpar 
    TYPE(localvariables) :: kgenref_localvar 
    TYPE(objectinstances) :: kgenref_objinst 
    TYPE(errorvariables) :: kgenref_errvar 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
    CALL kr_rosco_types_controlparameters(cntrpar, kgen_unit, "cntrpar", .FALSE.) 
    CALL kr_rosco_types_localvariables(localvar, kgen_unit, "localvar", .FALSE.) 
    CALL kr_rosco_types_objectinstances(objinst, kgen_unit, "objinst", .FALSE.) 
    CALL kr_rosco_types_errorvariables(errvar, kgen_unit, "errvar", .FALSE.) 
      
    !extern output variables 
      
    !local output variables 
    CALL kr_discon_real__reki_dim1(kgenref_avrswap, kgen_unit, "kgenref_avrswap", .FALSE.) 
    CALL kr_rosco_types_controlparameters(kgenref_cntrpar, kgen_unit, "kgenref_cntrpar", .FALSE.) 
    CALL kr_rosco_types_localvariables(kgenref_localvar, kgen_unit, "kgenref_localvar", .FALSE.) 
    CALL kr_rosco_types_objectinstances(kgenref_objinst, kgen_unit, "kgenref_objinst", .FALSE.) 
    CALL kr_rosco_types_errorvariables(kgenref_errvar, kgen_unit, "kgenref_errvar", .FALSE.) 

!------------------------------------------------------------------------------------------------------------------------------
! Main control calculations
!------------------------------------------------------------------------------------------------------------------------------
! Check for restart


! Read avrSWAP array into derived types/variables

! Set Control Parameters


! Call external controller, if desired


! Filter signals


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
        CALL CableControl(avrSWAP,CntrPar,LocalVar, objInst, ErrVar)
        IF (kgen_mainstage) THEN 
              
            !verify init 
            CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=1) 
            CALL kgen_init_check(check_status, rank=kgen_mpirank) 
              
            !extern verify variables 
              
            !local verify variables 
            CALL kv_discon_real__reki_dim1("avrswap", check_status, avrswap, kgenref_avrswap) 
            CALL kv_rosco_types_controlparameters("cntrpar", check_status, cntrpar, kgenref_cntrpar) 
            CALL kv_rosco_types_localvariables("localvar", check_status, localvar, kgenref_localvar) 
            CALL kv_rosco_types_objectinstances("objinst", check_status, objinst, kgenref_objinst) 
            CALL kv_rosco_types_errorvariables("errvar", check_status, errvar, kgenref_errvar) 
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
        CALL CableControl(avrSWAP,CntrPar,LocalVar, objInst, ErrVar)
            END DO   
            CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
            kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
            IF (check_status%rank==0) THEN 
                WRITE (*, *) "CableControl : Time per call (usec): ", kgen_measure 
            END IF   
        END IF   
        IF (kgen_warmupstage) THEN 
        END IF   
        IF (kgen_evalstage) THEN 
        END IF   


! Add RoutineName to error message


          
        CONTAINS 
          
        !read state subroutine for kr_discon_real__reki_dim1 
        SUBROUTINE kr_discon_real__reki_dim1(var, kgen_unit, printname, printvar) 
            REAL(KIND=reki), INTENT(INOUT), ALLOCATABLE, DIMENSION(:) :: var 
            INTEGER, INTENT(IN) :: kgen_unit 
            CHARACTER(LEN=*), INTENT(IN) :: printname 
            LOGICAL, INTENT(IN), OPTIONAL :: printvar 
            LOGICAL :: kgen_istrue 
            REAL(KIND=8) :: kgen_array_sum 
            INTEGER :: idx1 
            INTEGER, DIMENSION(2,1) :: kgen_bound 
              
            READ (UNIT = kgen_unit) kgen_istrue 
            IF (kgen_istrue) THEN 
                IF (ALLOCATED( var )) THEN 
                    DEALLOCATE (var) 
                END IF   
                READ (UNIT = kgen_unit) kgen_array_sum 
                READ (UNIT = kgen_unit) kgen_bound(1, 1) 
                READ (UNIT = kgen_unit) kgen_bound(2, 1) 
                ALLOCATE (var(kgen_bound(1,1):kgen_bound(2,1))) 
                READ (UNIT = kgen_unit) var 
                CALL kgen_array_sumcheck(printname, kgen_array_sum, DBLE(SUM(var, mask=(var .eq. var))), .TRUE.) 
                IF (PRESENT( printvar ) .AND. printvar) THEN 
                    WRITE (*, *) "KGEN DEBUG: DBLE(SUM(" // printname // ")) = ", DBLE(SUM(var, mask=(var .eq. var))) 
                END IF   
            END IF   
        END SUBROUTINE kr_discon_real__reki_dim1 
          
        !verify state subroutine for kv_discon_real__reki_dim1 
        RECURSIVE SUBROUTINE kv_discon_real__reki_dim1(varname, check_status, var, kgenref_var) 
            CHARACTER(LEN=*), INTENT(IN) :: varname 
            TYPE(check_t), INTENT(INOUT) :: check_status 
            REAL(KIND=reki), INTENT(IN), DIMENSION(:) :: var, kgenref_var 
            INTEGER :: check_result 
            LOGICAL :: is_print = .FALSE. 
              
            INTEGER :: idx1 
            INTEGER :: n 
            real(KIND=reki) :: nrmsdiff, rmsdiff 
            real(KIND=reki), ALLOCATABLE :: buf1(:), buf2(:) 
              
            check_status%numTotal = check_status%numTotal + 1 
              
            IF (ALL(var == kgenref_var)) THEN 
                check_status%numIdentical = check_status%numIdentical + 1 
                IF (kgen_verboseLevel > 1) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) trim(adjustl(varname)), " is IDENTICAL." 
                    END IF   
                END IF   
                check_result = CHECK_IDENTICAL 
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
                ELSE 
                    check_status%numInTol = check_status%numInTol + 1 
                    IF (kgen_verboseLevel > 1) THEN 
                        IF (check_status%rank == 0) THEN 
                            WRITE (*, *) trim(adjustl(varname)), " is NOT IDENTICAL(within tolerance)." 
                        END IF   
                    END IF   
                    check_result = CHECK_IN_TOL 
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
              
        END SUBROUTINE kv_discon_real__reki_dim1 
          
END SUBROUTINE discon 