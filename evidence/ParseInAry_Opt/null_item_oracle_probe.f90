! Does a NULL ITEM leave a DEFINED value behind when `Ary` arrives ALLOCATED?
!
! THIS IS THE ONE CLAIM THE FIX FOR SURVIVOR 07b5ee72 RESTS ON, and it is the
! claim that decides whether the differential harness can carry a record with a
! `;` in it at all.
!
! `07b5ee72` negates `if (is_eol(rec[p]))` inside `eat_separator`'s blank scan,
! which can only change an answer where the semicolon guard reads `sep`
! (parseinary_opt.cpp:298). So the only records that separate the mutant from
! the shipped translation are records in which a `;` starts a NON-FIRST item and
! was reached across blanks -- and on those the shipped program treats the `;`
! as LEGAL, that is, as a null value that leaves its element UNCHANGED.
!
! Unchanged is only usable as an ORACLE if the element had a value to keep. On
! the arm the corpus takes today it does not: `Ary` arrives unallocated, the
! subroutine ALLOCATEs it, and an element the READ does not assign is recycled
! heap -- which is why six mutants in that region are carried as `unreachable`
! and why the fourth dispatch built every new record form so that a successful
! read assigns every element it consumes.
!
! THE OTHER ARM IS THE WAY OUT, and this probe is whether it works: hand the
! subroutine an ALREADY-ALLOCATED `Ary`. The ALLOCATE then FAILS (allocating an
! allocated object is an error in Fortran), the array keeps the caller's own
! values, and the READ writes into a DEFINED array. If gfortran leaves the null
! item's element at the value it arrived with, then a `;` record on such a case
! has a perfect oracle on every output, and the corpus can carry it.
!
!   gfortran -o null_item_oracle_probe null_item_oracle_probe.f90
program null_item_oracle_probe
  implicit none
  integer, allocatable :: Ary(:)
  integer :: st, ios, i
  character(len=64) :: line

  allocate(Ary(3))
  Ary = (/ 7, 8, 9 /)
  print *, "on entry           Ary =", Ary

  ! What the subroutine does with an Ary that is already allocated.
  allocate(Ary(3), STAT=st)
  print *, "ALLOCATE STAT      =", st, " (non-zero: the allocation FAILED)"
  print *, "after the failure  Ary =", Ary, " ALLOCATED =", allocated(Ary)

  ! The record the corpus cannot state today. Values first, key last, which is
  ! the shape FindLine hands the READ.
  line = "1 ; 3 KEYNAME"
  read(line, *, IOSTAT=ios) Ary
  print *, "record             = '", trim(line), "'"
  print *, "READ IOSTAT        =", ios, " (0: the reference calls this record LEGAL)"
  print *, "after the READ     Ary =", Ary

  ! And the same record with the array FRESHLY allocated, for contrast: the
  ! element the null item skips is whatever the allocator handed back.
  deallocate(Ary)
  allocate(Ary(3))
  read(line, *, IOSTAT=ios) Ary
  print *, "--- fresh ALLOCATE, same record"
  print *, "READ IOSTAT        =", ios
  print *, "after the READ     Ary =", Ary, " <- element 2 is UNDEFINED, not an oracle"

  ! A comma is the null item everyone knows; the semicolon is the one this
  ! parser's separator rules make legal only after blanks and only when it is
  ! not the first item. Both are shown so the reader can see they agree.
  Ary = (/ 7, 8, 9 /)
  line = "1 , 3 KEYNAME"
  read(line, *, IOSTAT=ios) Ary
  print *, "--- the comma form, for comparison"
  print *, "READ IOSTAT        =", ios, "  Ary =", Ary
end program
