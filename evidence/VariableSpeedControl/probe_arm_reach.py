#!/usr/bin/env python3
"""How many cases of THIS corpus reach each arm a surviving mutant sits in.

    bash scripts/harness.sh VariableSpeedControl Controllers variablespeedcontrol \
         rosco/controller/src/Controllers.f90 --no-generate      # link for the tree
    python3 evidence/VariableSpeedControl/probe_arm_reach.py --patch
    docker exec vit-dev bash -lc "cd <unit_dir> && make test && ./test <bin> 2>probe.err"
    python3 evidence/VariableSpeedControl/probe_arm_reach.py --strip

WHY IT EXISTS. `unreachable` is a claim about the CORPUS, and this campaign
checks it harder than an equivalence precisely because it is cheaper to assert.
The claim each surviving mutant of this unit needs is "no case of the scored
corpus reaches the statement", and the only honest way to make it is to COUNT --
per arm, over the same `variablespeedcontrol_cases.bin` the sweep scored.

WHAT IT COUNTS, and every one of these is a guard the reference itself writes:

    tsr        VS_ControlMode in {2,3,4}          Controllers.f90:253-255
    tsr_fbp1   tsr .AND. VS_FBP == 1              :267
    kom        VS_ControlMode == 1                :271
    st1        kom .AND. VS_State == 1            :277
    st6        kom .AND. VS_State == 6            :285
    st6_fbp1   st6 .AND. VS_FBP == 1              :287
    st6_fbp23  st6 .AND. VS_FBP in {2,3}          :293
    fbp0_cp1   VS_FBP == 0 .AND. VS_ConstPower==1 :241-242
    ol         OL_Mode > 0 .AND. Ind_GenTq > 0    :322
    ol2        ol .AND. OL_Mode == 2              :331
    ol2_init   ol2 .AND. iStatus == 0             :335
    errmsg     aviFAIL < 0                        :361

IT READS ONLY INPUTS. Every quantity above is an input parameter of the case, so
the probe is about the QUESTIONS the corpus asks and not about either side's
ANSWERS -- which is what makes it valid evidence for a claim about the corpus
rather than about the translation (unit #47's rule).

THE POSITIVE CONTROL IS BUILT IN and it is the reason the list has both a live
and a dead member at the same nesting depth: `ol` and `ol2` must be NON-ZERO,
because `mutation/VariableSpeedControl.json` records two arith_op mutants at
variablespeedcontrol.cpp:541 -- inside `ol2` -- as KILLED. A probe that reported
zero everywhere would be measuring itself.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TEST = ROOT / "translations/Controllers/variablespeedcontrol_test/variablespeedcontrol_test.cpp"
MARK = "/*VITARMPROBE*/"

PROBE = r'''        { static long n=0,tsr=0,tsr_fbp1=0,kom=0,st1=0,st6=0,st6_fbp1=0,st6_fbp23=0,
                 fbp0_cp1=0,ol=0,ol2=0,ol2_init=0,errmsg=0;
          const double m=CntrPar_a.VS_ControlMode, f=CntrPar_a.VS_FBP, s=LocalVar_a.VS_State;
          const bool is_tsr=(m==2.0||m==3.0||m==4.0), is_kom=(m==1.0);
          const bool is_ol=(CntrPar_a.OL_Mode>0.0 && CntrPar_a.Ind_GenTq>0.0);
          n++;
          if(is_tsr){tsr++; if(f==1.0)tsr_fbp1++;}
          if(is_kom){kom++; if(s==1.0)st1++;
                     if(s==6.0){st6++; if(f==1.0)st6_fbp1++; if(f==2.0||f==3.0)st6_fbp23++;}}
          if(f==0.0 && CntrPar_a.VS_ConstPower==1.0)fbp0_cp1++;
          if(is_ol){ol++; if(CntrPar_a.OL_Mode==2.0){ol2++; if(LocalVar_a.iStatus==0)ol2_init++;}}
          if(ErrVar_a.aviFAIL<0)errmsg++;
          if(n==NCASES) fprintf(stderr,
            "ARMPROBE n=%ld tsr=%ld tsr_fbp1=%ld kom=%ld st1=%ld st6=%ld st6_fbp1=%ld "
            "st6_fbp23=%ld fbp0_cp1=%ld ol=%ld ol2=%ld ol2_init=%ld errmsg=%ld\n",
            n,tsr,tsr_fbp1,kom,st1,st6,st6_fbp1,st6_fbp23,fbp0_cp1,ol,ol2,ol2_init,errmsg);
        } ''' + MARK + "\n"

ANCHOR = "        int bad = 0;\n"


def main(argv: list[str]) -> int:
    if len(argv) != 2 or argv[1] not in ("--patch", "--strip"):
        print(__doc__)
        return 2
    src = TEST.read_text()
    if argv[1] == "--strip":
        out = "\n".join(ln for ln in src.splitlines() if MARK not in ln)
        # the block is multi-line; drop from its opening brace to the marker line
        out = re.sub(r"\n *\{ static long n=0,tsr=0.*?" + re.escape(MARK),
                     "", src, flags=re.S)
        TEST.write_text(out)
        print("stripped" if MARK not in out else "STRIP FAILED")
        return 0 if MARK not in out else 1
    if MARK in src:
        print("already patched")
        return 0
    m = re.search(r"for \(int c = 0; c < (\d+); c\+\+\) \{", src)
    if not m:
        print("cannot find the case loop -- is this the generated test?")
        return 1
    if src.count(ANCHOR) != 1:
        print(f"anchor appears {src.count(ANCHOR)} times, expected 1")
        return 1
    TEST.write_text(src.replace(ANCHOR, PROBE.replace("NCASES", m.group(1)) + ANCHOR))
    print(f"patched for {m.group(1)} cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
