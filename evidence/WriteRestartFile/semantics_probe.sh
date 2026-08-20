#!/bin/bash
# Semantics control for WriteRestartFile. Exit 0 == gfortran and the SHIPPED
# C++ helpers agree on all three questions the corpus cannot ask.
#
#   bash evidence/WriteRestartFile/semantics_probe.sh
#
# The C++ side INCLUDES rosco/controller/src/writerestartfile.cpp, so this
# measures the helpers that ship rather than a copy of them.
set -euo pipefail
CONTAINER="${VIT_CONTAINER:-vit-dev}"
docker exec "$CONTAINER" bash -lc '
set -e
cd /workspace/ROSCO-r2/evidence/WriteRestartFile
gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off \
    -o semantics_probe_f semantics_probe.f90
g++ -std=c++17 -ffp-contract=off \
    -I/workspace/ROSCO-r2/rosco/controller/src \
    -o semantics_probe_c semantics_probe.cpp
./semantics_probe_f
./semantics_probe_c
echo "--- the bytes gfortran wrote for .TRUE. / .FALSE. / .TRUE. ---"
od -An -tu1 semantics_probe.f.bin
echo "bin bytes: $(wc -c < semantics_probe.f.bin)   text records: $(wc -l < semantics_probe.f.txt)"
cmp semantics_probe.f.bin semantics_probe.c.bin && echo "BIN IDENTICAL"
cmp semantics_probe.f.txt semantics_probe.c.txt && echo "TXT IDENTICAL"
'
