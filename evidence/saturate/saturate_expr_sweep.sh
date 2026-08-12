#!/bin/bash
# Run the sweep: gfortran's own MIN(MAX(...)) against the shipped C++ spelling
# and the two branch spellings it was chosen over. Bits, not values.
set -eu
cd "$(dirname "$0")"
T=saturate_expr_sweep_triples.txt
FFLAGS="-fdefault-real-8 -fdefault-double-8 -ffp-contract=off -O2"
gfortran $FFLAGS saturate_expr_sweep.f90 -o /tmp/sat_ref
g++ -ffp-contract=off -O2 saturate_expr_sweep.cpp -o /tmp/sat_cxx
/tmp/sat_ref < "$T" > /tmp/sat_out.ref
echo "triples: $(wc -l < "$T")"
for s in shipped branchA branchB; do
    /tmp/sat_cxx "$s" < "$T" > "/tmp/sat_out.$s"
    d=$(paste /tmp/sat_out.ref "/tmp/sat_out.$s" | awk '$1 != $2' | wc -l)
    echo "$s differs from gfortran on $d of $(wc -l < "$T") triples"
done
