#!/bin/bash
# Format-fidelity control for Debug. Exit 0 == the C++ record machinery and
# gfortran produce byte-identical output over the value set in fmt_probe.f90.
#
#   bash evidence/Debug/fmt_probe.sh
#
# The C++ side INCLUDES translations/ROSCO_IO/debug.cpp, so this measures the
# shipped helpers rather than a copy of them.
set -euo pipefail
CONTAINER="${VIT_CONTAINER:-vit-dev}"
docker exec "$CONTAINER" bash -lc '
set -e
cd /workspace/ROSCO-r2/evidence/Debug
gfortran -fdefault-real-8 -fdefault-double-8 -ffp-contract=off -o fmt_probe_f fmt_probe.f90
g++ -std=c++17 -ffp-contract=off \
    -I/workspace/ROSCO-r2/rosco/controller/src \
    -I/workspace/ROSCO-r2/translations/ROSCO_IO \
    -o fmt_probe_c fmt_probe.cpp -lgfortran
./fmt_probe_f
./fmt_probe_c
echo "records: $(wc -l < fmt_probe.f.out)  bytes: $(wc -c < fmt_probe.f.out)"
cmp fmt_probe.f.out fmt_probe.c.out && echo IDENTICAL
'
