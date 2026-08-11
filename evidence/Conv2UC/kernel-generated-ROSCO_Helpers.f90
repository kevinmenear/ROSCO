!KGEN-generated Fortran source file 
  
!Generated at : 2026-08-11 01:15:37 
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
! Helper functions, primarily borrowed from NWTC_IO, for reading inputs and carrying out other helpful tasks


MODULE ROSCO_Helpers

    USE syssubs 
    USE kgen_utils_mod
    USE tprof_mod, ONLY: tstart, tstop, tnull, tprnt 


    USE ISO_C_BINDING
    IMPLICIT NONE 
    ! Global Variables

    


    INTEGER(IntKi), PARAMETER       :: MaxParamLength   = 200         ! characters, file paths can be long
    PUBLIC findline 


    ! Auto-generated interface for C++ implementation of Conv2UC
    INTERFACE
        SUBROUTINE conv2uc_c(Str, len_Str) BIND(C, NAME='conv2uc_c')
            USE ISO_C_BINDING
            CHARACTER(KIND=C_CHAR), INTENT(INOUT) :: Str(*)
            INTEGER(C_INT), VALUE :: len_Str
        END SUBROUTINE conv2uc_c
    END INTERFACE

CONTAINS
    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors


    !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors


     !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors


        !=======================================================================
    ! Parse integer input: read line, check that variable name is in line, handle errors


    !=======================================================================
    ! Parse double input, this is a copy of ParseInput_Int and a change in the variable definitions


    !=======================================================================
    ! Parse string input, this is a copy of ParseInput_Int and a change in the variable definitions


!=======================================================================
!> This subroutine parses the specified line of text for AryLen REAL values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


  !=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


!=======================================================================
!=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


!=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


!=======================================================================
 !> This subroutine checks the data to be parsed to make sure it finds
    !! the expected variable name and an associated value.


SUBROUTINE findline(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath) 
    USE kgen_utils_mod
    USE kgen_utils_mod
    USE kgen_utils_mod
    
    

    CHARACTER(LEN=maxparamlength) :: filelineuc 
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
    CHARACTER(LEN=maxparamlength) :: kgenref_filelineuc 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
    READ (UNIT = kgen_unit) filelineuc 
      
    !extern output variables 
      
    !local output variables 
    READ (UNIT = kgen_unit) kgenref_filelineuc 


    ! Make name uppercase
    
    ! Search for line in FileLines
    
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
        CALL Conv2UC(FileLineUC)
        IF (kgen_mainstage) THEN 
              
            !verify init 
            CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
            CALL kgen_init_check(check_status, rank=kgen_mpirank) 
              
            !extern verify variables 
              
            !local verify variables 
            CALL kv_findline_character_maxparamlength_("filelineuc", check_status, filelineuc, kgenref_filelineuc) 
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
        CALL Conv2UC(FileLineUC)
            END DO   
            CALL SYSTEM_CLOCK(kgen_stop_clock, kgen_rate_clock) 
            kgen_measure = 1.0D6*(kgen_stop_clock - kgen_start_clock)/DBLE(kgen_rate_clock*KGEN_MAXITER) 
            IF (check_status%rank==0) THEN 
                WRITE (*, *) "Conv2UC : Time per call (usec): ", kgen_measure 
            END IF   
        END IF   
        IF (kgen_warmupstage) THEN 
        END IF   
        IF (kgen_evalstage) THEN 
        END IF   

          
        CONTAINS 
          

        !verify state subroutine for kv_findline_character_maxparamlength_ 
        RECURSIVE SUBROUTINE kv_findline_character_maxparamlength_(varname, check_status, var, kgenref_var) 
            CHARACTER(LEN=*), INTENT(IN) :: varname 
            TYPE(check_t), INTENT(INOUT) :: check_status 
            CHARACTER(LEN=maxparamlength), INTENT(IN) :: var, kgenref_var 
            INTEGER :: check_result 
            LOGICAL :: is_print = .FALSE. 
              
            character(LEN=maxparamlength) :: diff 
              
            check_status%numTotal = check_status%numTotal + 1 
              
            IF ((var == kgenref_var) .OR. ((var /= var) .AND. (kgenref_var /= kgenref_var))) THEN
        IF (var /= var) WRITE(*, *) trim(adjustl(varname))," is IDENTICAL (both NaN, uninitialized)." 
                check_status%numIdentical = check_status%numIdentical + 1 
                IF (kgen_verboseLevel > 1) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) trim(adjustl(varname)), " is IDENTICAL." 
                    END IF   
                END IF   
                check_result = CHECK_IDENTICAL 
                WRITE(*, *) "[VIT_FIELD] ", trim(adjustl(varname)), " | IDENTICAL | ", var, " | ", kgenref_var
            ELSE 
                check_status%numOutTol = check_status%numOutTol + 1 
                IF (kgen_verboseLevel > 1) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) trim(adjustl(varname)), " is NOT IDENTICAL." 
                    END IF   
                END IF   
                check_result = CHECK_OUT_TOL 
                WRITE(*, *) "[VIT_FIELD] ", trim(adjustl(varname)), " | OUT_TOL | ", var, " | ", kgenref_var, " | ", diff
            END IF   
            IF (check_result == CHECK_IDENTICAL) THEN 
                IF (kgen_verboseLevel > 2) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "NOT IMPLEMENTED" 
                        WRITE (*, *) "" 
                    END IF   
                END IF   
            ELSE IF (check_result == CHECK_OUT_TOL) THEN 
                IF (kgen_verboseLevel > 0) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "NOT IMPLEMENTED" 
                        WRITE (*, *) "" 
                    END IF   
                END IF   
            ELSE IF (check_result == CHECK_IN_TOL) THEN 
                IF (kgen_verboseLevel > 1) THEN 
                    IF (check_status%rank == 0) THEN 
                        WRITE (*, *) "NOT IMPLEMENTED" 
                        WRITE (*, *) "" 
                    END IF   
                END IF   
            END IF   
              
        END SUBROUTINE kv_findline_character_maxparamlength_ 
          
END SUBROUTINE findline 
!=======================================================================


!=======================================================================
!> This subroutine is used to get the NumWords "words" from a line of text.
!! It uses spaces, tabs, commas, semicolons, single quotes, and double quotes ("whitespace")
!! as word separators. If there aren't NumWords in the line, the remaining array elements will remain empty.
!! Use CountWords (nwtc_io::countwords) to count the number of words in a line.


!=======================================================================
!> Let's parse the path name from the name of the given file.
!! We'll count everything before (and including) the last "\" or "/".

!=======================================================================
!> Let's parse the root file name from the name of the given file.
!! We'll count everything after the last period as the extension.
!! Borrowed from NWTC_IO...thanks!


!=======================================================================
!> This routine determines if the given file name is absolute or relative.
!! We will consider an absolute path one that satisfies one of the
!! following four criteria:
!!     1. It contains ":/"
!!     2. It contains ":\"
!!     3. It starts with "/"
!!     4. It starts with "\"
!!   
!! All others are considered relative.

!=======================================================================
! ------------------------------------------------------
    ! Read Open Loop Control Inputs
    ! Timeseries or lookup tables of the form
    ! index (time or wind speed)   channel_1 \t channel_2 \t channel_3 ...
    ! This could be used to read any group of data of unspecified length ...
    ! 


!=======================================================================
!> This routine returns the next unit number greater than 9 that is not currently in use.
!! If it cannot find any unit between 10 and 99 that is available, it either aborts or returns an appropriate error status/message.   


!=======================================================================
!> This function returns a character string encoded with the time in the form "hh:mm:ss".


!=======================================================================
! This function checks whether an array is non-decreasing


!=======================================================================
!> This routine converts all the text in a string to upper case.

    SUBROUTINE Conv2UC(Str)
        USE ISO_C_BINDING
        IMPLICIT NONE
        CHARACTER(*), INTENT(INOUT) :: Str
        CHARACTER(KIND=C_CHAR) :: Str_c(LEN(Str))
        INTEGER :: vit_i_Str
        ! Convert CHARACTER args to C_CHAR arrays
        DO vit_i_Str = 1, LEN(Str)
            Str_c(vit_i_Str) = Str(vit_i_Str:vit_i_Str)
        END DO
        CALL conv2uc_c(Str_c, LEN(Str))
        ! Copy C_CHAR arrays back to CHARACTER args (INTENT OUT/INOUT)
        DO vit_i_Str = 1, LEN(Str)
            Str(vit_i_Str:vit_i_Str) = Str_c(vit_i_Str)
        END DO
    END SUBROUTINE Conv2UC
!=======================================================================
     !> This function returns a left-adjusted string representing the passed numeric value. 
    !! It eliminates trailing zeroes and even the decimal point if it is not a fraction. \n
    !! Use Num2LStr (nwtc_io::num2lstr) instead of directly calling a specific routine in the generic interface.   


!=======================================================================


    !-------------------------------------------------------------------------------------------------------------------------------
    ! Copied from NWTC_IO.f90
    !> This function returns a character string encoded with today's date in the form dd-mmm-ccyy.


END MODULE ROSCO_Helpers