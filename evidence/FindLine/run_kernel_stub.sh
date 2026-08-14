#!/bin/bash
# Measure the kernel's discriminating power for FindLine, which `vit verify`
# declined to measure: it builds a red test only for a by-value floating-point
# parameter, and this unit has none. A 20/20 IDENTICAL with no red test is a
# number, not evidence (X4), so each stub below is one.
#
#   bash evidence/FindLine/run_kernel_stub.sh <stub.cpp> <label>
#
# Run inside the container from /workspace/ROSCO-r2, on the CLEAN tree.
set -u
cd /workspace/ROSCO-r2
stub="$1"; label="$2"
real=evidence/FindLine/findline.final.cpp
live=translations/ROSCO_Helpers/findline.cpp

cp "$stub" "$live"
# The edit must have ARRIVED before anything is built (unit #23's bind-mount
# hazard, unit #30's silent no-op revert): compare hashes, do not assume.
if [ "$(md5sum < "$live")" = "$(md5sum < "$real")" ]; then
    echo "STUB IS IDENTICAL TO THE TRANSLATION -- this probe measures nothing"; exit 9
fi
vit verify FindLine "$live" -f rosco/controller/src/ROSCO_Helpers.f90 \
    --kernel-dir kernel/FindLine
rc=$?
cp kernel/FindLine/verify_fields.csv "evidence/FindLine/kernel.${label}.verify_fields.csv"
cp "$real" "$live"
# A stub `vit verify` writes `cases_passed` under translations.FindLine -- a
# machine-readable claim that the SHIPPED translation was verified by a run of
# something else (unit #11). Revert the file wholesale.
git checkout -- vit.yaml
echo "== $label: vit verify exit $rc"
python3 - "evidence/FindLine/kernel.${label}.verify_fields.csv" <<'PY'
import csv, collections, sys
rows = list(csv.DictReader(open(sys.argv[1])))
c = collections.Counter(r['status'] for r in rows)
per = collections.defaultdict(collections.Counter)
for r in rows:
    per[r['field']][r['status']] += 1
print('   fields:', dict(c))
for f in sorted(per):
    print(f'   {f:12s} {dict(per[f])}')
PY
