#!/bin/bash
# Run evidence/PIDController/instlpf_probe.f90 against the CLEAN reference, one
# value per process, and report each exit status. Inside the container, from
# /workspace/ROSCO-r2, on the clean tree.
#
# Copied from evidence/FindLine/arylen_probe.sh with two names changed.
set -u
cd /workspace/ROSCO-r2
B=rosco/controller/build/CMakeFiles/discon.dir/src
gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off \
    -I rosco/controller/build/ftnmods -o /tmp/instlpf_probe \
    evidence/PIDController/instlpf_probe.f90 $B/*.o $B/SysFiles/*.o -lstdc++ -ldl 2>&1 | tail -5
[ -x /tmp/instlpf_probe ] || { echo "probe did not build"; exit 2; }
for a in -2147483648 -100000 -1000 -300 -2 -1 0 1 2 512 1024 1025 100000 2147483647; do
    out=$(/tmp/instlpf_probe "$a" 2>&1); rc=$?
    printf '%-12s exit=%-4s %s\n' "$a" "$rc" "$out"
done
