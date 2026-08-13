! Does gfortran REALLOCATE the left-hand side of `All_OL_Indices = (/.../)`?
!
! CheckInputs writes, at ReadSetParameters.f90:1577-1583 of the clean source:
!
!     ALLOCATE(All_OL_Indices(5))   ! Will need to increase to 5 when IPC
!     All_OL_Indices = (/CntrPar%Ind_BldPitch, &
!                        CntrPar%Ind_GenTq, CntrPar%Ind_YawRate, &
!                        CntrPar%Ind_R_Speed, CntrPar%Ind_R_Torque, &
!                        CntrPar%Ind_R_Pitch/)
!
! The constructor's FIRST item is the whole ALLOCATABLE array Ind_BldPitch, so
! the right-hand side has SIZE(Ind_BldPitch) + 5 = EIGHT elements against the
! five just allocated. Whether the translation must produce a 5-element or an
! 8-element array decides the answers of `ANY(All_OL_Indices < 0)` and
! `ALL(All_OL_Indices < 1)` three statements later, and reading the ALLOCATE
! gives the wrong one.
!
! rosco/controller/CMakeLists.txt passes neither -fno-realloc-lhs nor -std=f95,
! so gfortran is at F2003 defaults. Measured rather than read:
!
!   gfortran -ffree-line-length-0 -fdefault-real-8 -fdefault-double-8 -o rl rl.f90
!   ./rl
!    SIZE(A) =           8  LBOUND =           1
!             11          12          13           4           5           6           7           8
!
! Eight, lower bound 1, in constructor order. The ALLOCATE(5) decides nothing.
PROGRAM rl
  INTEGER(4), ALLOCATABLE :: A(:), bp(:)
  INTEGER(4) :: g, y, s, t, p
  ALLOCATE(bp(3)); bp = (/11,12,13/)
  g=4; y=5; s=6; t=7; p=8
  ALLOCATE(A(5))
  A = (/bp, g, y, s, t, p/)
  PRINT *, "SIZE(A) =", SIZE(A), " LBOUND =", LBOUND(A,1)
  PRINT *, A
END PROGRAM
