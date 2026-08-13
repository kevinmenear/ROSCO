! The list-directed layout of CheckInputs' three PRINT statements, measured.
!
! The one that matters carries a value:
!   PRINT *, "Note: PRC Mode = ", CntrPar%PRC_Mode, ", which will affect ..."
!
!   gfortran -ffree-line-length-0 -fdefault-real-8 -fdefault-double-8 -o pt pt.f90
!   ./pt | cat -A
!    Note: PRC Mode =            2 , which will affect VS_RefSpeed, VS_TSRopt, and PC_RefSpeed$
!    Note: PRC Mode =          -17 , tail$
!    ROSCO Warning: Note that frequency avoidance control (TRA_Mode > 1) will affect PRC set points$
!
! The record opens with one blank; a CHARACTER item is written verbatim with no
! separator; a default INTEGER occupies a 12-column right-justified field (2 ->
! eleven blanks then "2"; -17 -> nine blanks then "-17"); the item after it is
! preceded by one blank. So the C++ is
!
!   std::printf(" Note: PRC Mode = %12d , which will affect ...\n", PRC_Mode)
!
! Neither the gate nor the differential harness compares stdout, so this is
! transcribed for faithfulness rather than because a layer would catch it.
! ExtController's five PRINTs are the reason all three readers of stdout in this
! pipeline take the LAST parseable JSON object rather than the whole stream.
PROGRAM pt
  INTEGER(4) :: m
  m = 2
  PRINT *, "Note: PRC Mode = ", m, ", which will affect VS_RefSpeed, VS_TSRopt, and PC_RefSpeed"
  m = -17
  PRINT *, "Note: PRC Mode = ", m, ", tail"
  PRINT *, "ROSCO Warning: Note that frequency avoidance control (TRA_Mode > 1) will affect PRC set points"
END PROGRAM
