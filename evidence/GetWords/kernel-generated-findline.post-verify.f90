!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


!=======================================================================
!> This subroutine parses the specified line of text for AryLen INTEGER values.
!! Generate an error message if the value is the wrong type.
!! Use ParseAry (nwtc_io::parseary) instead of directly calling a specific routine in the generic interface.   


!=======================================================================
 !> This subroutine checks the data to be parsed to make sure it finds
    !! the expected variable name and an associated value.


SUBROUTINE findline(kgen_unit, kgen_measure, kgen_isverified, kgen_filepath, filelines) 
    USE kgen_utils_mod
    USE kgen_utils_mod
    USE kgen_utils_mod
    CHARACTER(LEN=*), dimension(:), INTENT(INOUT) :: filelines 
    
    
    CHARACTER(LEN=maxparamlength), allocatable :: words(:) 

    INTEGER(KIND=intki) :: i, wordind 
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
    CHARACTER(LEN=maxparamlength), allocatable, dimension(:) :: kgenref_words 
      
    !parent block preprocessing 
    kgen_mpirank = 0 
      
    !local input variables 
    CALL kr_findline_character_maxparamlength__dim1(words, kgen_unit, "words", .FALSE.) 
    READ (UNIT = kgen_unit) i 
    READ (UNIT = kgen_unit) wordind 
      
    !extern output variables 
      
    !local output variables 
    CALL kr_findline_character_maxparamlength__dim1(kgenref_words, kgen_unit, "kgenref_words", .FALSE.) 


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
        CALL GetWords ( FileLines(I), Words, WordInd )  
        IF (kgen_mainstage) THEN 
              
            !verify init 
            CALL kgen_init_verify(tolerance=1.D-14, minvalue=1.D-14, verboseLevel=100) 
            CALL kgen_init_check(check_status, rank=kgen_mpirank) 
              
            !extern verify variables 
              
            !local verify variables 
            CALL kv_findline_character_maxparamlength__dim1("words", check_status, words, kgenref_words) 
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
