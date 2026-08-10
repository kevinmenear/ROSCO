! Standalone replay of ROSCO's ratelimit final update, from exact operand bits.
! Args (hex Z16): inputSignal LastSignal DT minRate maxRate
program probe
  implicit none
  integer, parameter :: DbKi = 8
  character(len=32) :: a(5)
  integer(8) :: z(5), zr, zo
  real(DbKi) :: inp, L, DT, minR, maxR, rate, res
  integer :: i
  do i=1,5
    call get_command_argument(i, a(i)); read(a(i),'(Z16)') z(i)
  end do
  inp=transfer(z(1),inp); L=transfer(z(2),L); DT=transfer(z(3),DT)
  minR=transfer(z(4),minR); maxR=transfer(z(5),maxR)
  rate = (inp - L)/DT
  rate = real(min(max(rate,minR),maxR), DbKi)   ! ROSCO saturate()
  res  = L + rate*DT
  zr=transfer(rate,zr); zo=transfer(res,zo)
  write(*,'(A,Z16.16,A,Z16.16)') 'FORTRAN rate=0x',zr,'  result=0x',zo
end program
