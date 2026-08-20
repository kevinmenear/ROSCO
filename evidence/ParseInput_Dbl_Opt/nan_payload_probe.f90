PROGRAM nan_payload_probe
    IMPLICIT NONE
    INTEGER, PARAMETER :: MaxParamLength = 200
    CHARACTER(MaxParamLength) :: Words(2)
    REAL(8) :: Variable
    INTEGER :: ios, L
    INTEGER(8) :: bits
    DO L = 0, 196
        Words(1) = 'nan('//REPEAT('a', L)//')'
        Words(2) = 'Aa'
        Variable = -987.654D0
        READ (Words(1), *, IOSTAT=ios) Variable
        bits = TRANSFER(Variable, bits)
        WRITE (*, '(A,I5,A,I6,A,Z16.16)') 'REF ', L, ' iostat=', ios, ' bits=', bits
    END DO
END PROGRAM
