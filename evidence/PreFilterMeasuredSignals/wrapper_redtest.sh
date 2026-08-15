set -uo pipefail
cd /Users/kmenear/Artifacts/vit_translation/ROSCO-r2
F=rosco/controller/src/Filters.f90
cp "$F" /tmp/Filters.integrated.f90
restore() {
  cp /tmp/Filters.integrated.f90 "$F"
  docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j 8 >/dev/null 2>&1 && cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
  diff -q /tmp/Filters.integrated.f90 "$F" && echo "wrapper restored"
}
trap restore EXIT
python3 - <<'PY'
p='rosco/controller/src/Filters.f90'
old='        CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar)\n'
s=open(p).read()
assert s.count(old)==1, s.count(old)
open(p,'w').write(s.replace(old,'        ! RED TEST: the LocalVar copy-back deleted\n'))
print('perturbed')
PY
docker exec vit-dev bash -lc "cd /workspace/ROSCO-r2/rosco/controller/build && cmake --build . -j 8 >/dev/null 2>&1 && cp libdiscon.so /workspace/ROSCO-r2/rosco/lib/libdiscon.so"
bash scripts/harness.sh PreFilterMeasuredSignals Filters prefiltermeasuredsignals \
    rosco/controller/src/Filters.f90 --post-integration \
    --out harness/PreFilterMeasuredSignals.postintegration.redtest.nocopyback.json \
    --red-test "wrapper: CALL vit_copy_scalars_to_localvariables(LocalVar_view, LocalVar) deleted -- every scalar and fixed-size-array output this unit writes travels back through that one call, because the view holds them BY VALUE" 2>&1 | grep -E 'POST-INTEGRATION|RED'
