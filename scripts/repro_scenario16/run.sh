#!/bin/bash
# Reproduce ROSCO Scenario 16's 2^-70 residual from clean builds and prove its
# true cause. Runs on the Mac host (needs git); build/run steps go through the
# `vit-dev` container (ARM64 Linux, gfortran/g++ 13.3.0). Isolated scratch trees
# are used so the tracked working tree is never modified.
#
# Usage:  bash scripts/repro_scenario16/run.sh
# See:    dev note 202607240344-scenario16-residual-reproduced-true-root-cause.md
set -euo pipefail

R="$(cd "$(dirname "$0")/../.." && pwd)"          # ROSCO repo root
D="$R/scripts/repro_scenario16"
CT=vit-dev                                         # container; /workspace -> vit_translation
CR=/workspace/ROSCO
FDUMP="$CR/repro_scratch/ratelimit_dump.txt"
CDUMP="$CR/repro_scratch_cpp/ratelimit_dump.txt"
dx(){ docker exec "$CT" bash -lc "$1"; }

echo "### 1. Extract isolated source trees (working tree untouched)"
rm -rf "$R/repro_scratch" "$R/repro_scratch_cpp"
mkdir -p "$R/repro_scratch" "$R/repro_scratch_cpp"
git -C "$R" archive e8010f0   rosco/controller | tar -x -C "$R/repro_scratch"     --strip-components=2
git -C "$R" archive integrated rosco/controller | tar -x -C "$R/repro_scratch_cpp" --strip-components=2

echo "### 2. Add -ffp-contract=off to the Fortran build; instrument both rate limiters"
python3 - "$R/repro_scratch/CMakeLists.txt" <<'PY'
import sys; p=sys.argv[1]; s=open(p).read()
if "ffp-contract=off" not in s:
    s=s.replace('-fdefault-double-8 -cpp"','-fdefault-double-8 -cpp -ffp-contract=off"')
open(p,"w").write(s)
PY
python3 "$D/instrument_ratelimit_fortran.py" "$R/repro_scratch/src/Functions.f90"          "$FDUMP"
python3 "$D/instrument_ratelimit_cpp.py"     "$R/repro_scratch_cpp/src/Functions/ratelimit.cpp" "$CDUMP"

echo "### 3. Build both libraries (-ffp-contract=off) and run Scenario 16 against each"
run_build () {  # $1=scratch subdir  $2=dumpfile
  local S="$CR/$1"
  dx "mkdir -p $S/build && cd $S/build && cmake .. -DCMAKE_BUILD_TYPE=Release >/tmp/cm.log 2>&1 && cmake --build . -j4 >/tmp/b.log 2>&1"
  dx "rm -f $2; L=$CR/rosco/lib/libdiscon.so; cp -f \$L $S/lib.bak; cp -f $S/build/libdiscon.so \$L; \
      cd $CR/Examples && python3 vit_sim.py --scenario 16 --output-dir $S/out16 >/tmp/s16.log 2>&1; \
      cp -f $S/lib.bak \$L"   # always restore the loaded library
}
run_build repro_scratch     "$FDUMP"
run_build repro_scratch_cpp "$CDUMP"

echo "### 4. Report bld_pitch[4444] and the captured operands at the store"
dx "python3 - <<'PY'
import numpy as np, struct
def h(x): return '0x%016X'%struct.unpack('<Q',struct.pack('<d',float(x)))[0]
for tag,p in [('FORTRAN','repro_scratch'),('C++','repro_scratch_cpp')]:
    a=np.load('$CR/'+p+'/out16/scenario_16.npz')['bld_pitch']
    print(tag,'bld_pitch[4444] =',h(a[4444]),'(2^-70)' if a[4444]==2.0**-70 else '(0.0)' if a[4444]==0 else '')
PY"
echo "--- Fortran operands at result==2^-70 ---"; dx "awk '\$6==\"3B90000000000000\"' $FDUMP | head -1"
echo "--- C++ operands at the same store (result==0) ---"; dx "grep '3EDF9C79' $CDUMP | head -1"

echo "### 5. Cross-language probes on identical operands"
dx "cd $CR/scripts/repro_scenario16 && \
    gfortran -O2 -ffp-contract=off -fdefault-real-8 -fdefault-double-8 probe.f90 -o /tmp/pf && \
    g++ -O2 -ffp-contract=off probe.cpp -o /tmp/pc && \
    echo 'TRUE operands (input LastSignal DT minRate maxRate):' && \
    /tmp/pf 0 3EDF9C7998447C18 3F999999A0000000 BFC65604189374BC 3FC65604189374BC && \
    /tmp/pc 0 3EDF9C7998447C18 3F999999A0000000 BFC65604189374BC 3FC65604189374BC && \
    echo 'OLD dev-notes operands (LastSignal=1.126e-6, DT=double 0.025):' && \
    /tmp/pf 0 3EB2E56267D20140 3F9999999999999A BFC65604189374BC 3FC65604189374BC && \
    /tmp/pc 0 3EB2E56267D20140 3F9999999999999A BFC65604189374BC 3FC65604189374BC"
echo "### done (2^-70 = 0x3B90000000000000)"
