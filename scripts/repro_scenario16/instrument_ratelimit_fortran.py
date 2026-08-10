#!/usr/bin/env python3
"""Insert an operand-capture dump into ROSCO's ratelimit (Functions.f90).

Idempotent. Dumps, for every ratelimit call whose result is near the
zero-crossing (|result| < 1e-9), the exact IEEE bit patterns of the true
operands *as they enter the compiled final update* — captured after
`ratelimit = LastSignal + rate*DT` and before LastSignal is overwritten,
so rlP%LastSignal(inst) still holds the operand used in the expression.

Usage: instrument_ratelimit.py <path-to-Functions.f90> <dump-file-path>
"""
import sys, re

src, dump = sys.argv[1], sys.argv[2]
s = open(src).read()
if "VIT_REPRO_DUMP" in s:
    print("already instrumented"); sys.exit(0)

anchor = "            ratelimit = rlP%LastSignal(inst) + rate*DT"
assert anchor in s, "anchor line not found (source shape changed?)"

block = anchor + "\n" + f'''
            ! --- VIT_REPRO_DUMP: capture true operands at the failing store ---
            IF (ABS(ratelimit) < 1.0d-9) THEN
              BLOCK
                INTEGER(8) :: zin, zls, zdt, zrt, zout, zmin, zmax
                INTEGER    :: urepro
                zin  = TRANSFER(inputSignal, zin)
                zls  = TRANSFER(rlP%LastSignal(inst), zls)
                zdt  = TRANSFER(DT, zdt)
                zrt  = TRANSFER(rate, zrt)
                zout = TRANSFER(ratelimit, zout)
                zmin = TRANSFER(minRate, zmin)
                zmax = TRANSFER(maxRate, zmax)
                OPEN(NEWUNIT=urepro, FILE="{dump}", POSITION="APPEND", ACTION="WRITE")
                WRITE(urepro,'(I0,7(1X,Z16.16))') inst, zin, zls, zdt, zrt, zout, zmin, zmax
                CLOSE(urepro)
              END BLOCK
            END IF
            ! --- end VIT_REPRO_DUMP ---'''
s = s.replace(anchor, block, 1)
open(src, "w").write(s)
print("instrumented", src)
