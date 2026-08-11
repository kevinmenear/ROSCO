#!/usr/bin/env bash
# Does AddToList's signature cross the C bridge?  Build three implementations
# against one driver and compare each against the oracle.
#
#   orig  ROSCO_Helpers.f90:1629 verbatim                    -- the oracle (P7)
#   vit   VIT d07a716's generated wrapper + interface block, with the best C++
#         its signature permits                              -- the prediction
#   cfi   a hand-written Fortran 2018 BIND(C) interface taking the ALLOCATABLE
#         dummy, and a C++ body using CFI_allocate/CFI_deallocate
#
#   docker exec vit-dev bash -lc \
#     "cd /workspace/ROSCO-r2/evidence/AddToList/bridge_probe && bash run.sh"
#
# RED TEST (X4 -- never take a green at face value on first use):
#
#   bash run.sh --red-test
#
# perturbs ONE token of the CFI translation (`element` -> `element + 1`),
# re-runs, and INVERTS the verdict: a red test that stays green is itself a
# failure.  It writes result.redtest.json and never edits addtolist_cfi.cpp.
#
# Both modes DELETE their output artifact before running.  A run that dies
# leaves no previous artifact behind to be read as this run's result.
set -uo pipefail
cd "$(dirname "$0")"

RED_TEST=""
OUT_JSON="result.json"
if [ "${1:-}" = "--red-test" ]; then
    RED_TEST="cfi: clist[isize] = element  ->  element + 1"
    OUT_JSON="result.redtest.json"
fi
rm -f "$OUT_JSON"

FFLAGS="-fdefault-real-8 -fdefault-double-8 -ffp-contract=off -cpp"
CXXFLAGS="-ffp-contract=off"

rm -rf out && mkdir -p out
: > out/build.log

CFI_SRC=addtolist_cfi.cpp
if [ -n "$RED_TEST" ]; then
    # The perturbation is fixed in this script, not passed in, so the
    # artifact's description cannot drift from what was compiled.
    CFI_SRC=out/addtolist_cfi.PERTURBED.cpp
    sed 's/clist\[isize\] = element;/clist[isize] = element + 1;/' \
        addtolist_cfi.cpp > "$CFI_SRC"
    grep -q 'element + 1' "$CFI_SRC" || {
        echo "red test: the perturbation did not apply -- refusing to report" >&2
        exit 2
    }
fi

declare -A BUILD_RC RUN_RC
for spec in "orig:mod_orig.f90:-:IMPL_ORIG" \
            "vit:mod_vit.f90:addtolist_vit.cpp:IMPL_VIT" \
            "cfi:mod_cfi.f90:${CFI_SRC}:IMPL_CFI"; do
    IFS=: read -r tag mod cpp def <<< "$spec"
    # each build needs its own module directory; the .mod names collide
    mkdir -p "out/mods_$tag"
    ( set -x
      gfortran $FFLAGS -c "$mod" -J "out/mods_$tag" -o "out/${tag}_mod.o"
      if [ "$cpp" != "-" ]; then g++ $CXXFLAGS -c "$cpp" -o "out/${tag}_impl.o"; fi
      gfortran $FFLAGS -D"$def" -I "out/mods_$tag" -J "out/mods_$tag" \
               -c driver.F90 -o "out/${tag}_drv.o"
      extra=""; [ "$cpp" != "-" ] && extra="out/${tag}_impl.o"
      gfortran "out/${tag}_mod.o" "out/${tag}_drv.o" $extra -lstdc++ -o "out/${tag}"
    ) >> out/build.log 2>&1
    BUILD_RC[$tag]=$?
    if [ "${BUILD_RC[$tag]}" -eq 0 ]; then
        "./out/${tag}" > "out/${tag}.txt" 2> "out/${tag}.stderr"
        RUN_RC[$tag]=$?
    else
        RUN_RC[$tag]=-1
        : > "out/${tag}.txt"
        echo "BUILD FAILED -- see out/build.log" > "out/${tag}.stderr"
    fi
done

echo "=================================================================="
for tag in orig vit cfi; do
    echo "--- $tag  (build rc=${BUILD_RC[$tag]}  run rc=${RUN_RC[$tag]}) ---"
    cat "out/${tag}.txt"
    [ -s "out/${tag}.stderr" ] && echo "  stderr: $(head -3 "out/${tag}.stderr")"
done
echo "=================================================================="

verdict_of() {
    local tag="$1"
    if   [ "${BUILD_RC[$tag]}" -ne 0 ]; then echo "BUILD_FAILED"
    elif [ "${RUN_RC[$tag]}"   -ne 0 ]; then echo "CRASHED_rc${RUN_RC[$tag]}"
    elif diff -q "out/orig.txt" "out/${tag}.txt" >/dev/null 2>&1; then echo "MATCHES_ORACLE"
    else echo "DIFFERS_FROM_ORACLE"; fi
}

# mod_vit.f90 is VIT's output, so the artifact must name which VIT produced it.
# `vit-dev` has no git; the pin file is the only reading available, and it is
# reported as a pin -- it cannot see whether the tree was edited after.
_vitrev() {
    for p in /workspace/vit/.vit_rev "$HOME/Artifacts/vit_translation/vit/.vit_rev"; do
        [ -f "$p" ] && { echo "$(tr -d '[:space:]' < "$p")-pinned"; return; }
    done
    echo unknown
}
VIT_REV="$(_vitrev)"
ORACLE_MD5="$(md5sum mod_orig.f90 | cut -c1-12)"

{
  echo "{"
  echo "  \"probe\": \"AddToList: does the signature cross the C bridge?\","
  echo "  \"oracle\": \"orig -- ROSCO_Helpers.f90:1629-1657 verbatim, mod_orig.f90 md5 ${ORACLE_MD5}\","
  echo "  \"vit_rev\": \"${VIT_REV}\","
  echo "  \"toolchain\": \"gfortran $(gfortran -dumpversion) / g++ $(g++ -dumpversion) $(uname -m)\","
  [ -n "$RED_TEST" ] && echo "  \"red_test\": \"${RED_TEST}\","
  echo "  \"scenarios\": \"A: allocated then appended 3x (the live ROSCO path). B: unallocated on entry (the else branch).\","
  echo "  \"implementations\": {"
  first=1
  for tag in orig vit cfi; do
      [ $first -eq 0 ] && echo ","
      first=0
      printf '    "%s": {"build_rc": %s, "run_rc": %s, "verdict": "%s", "lines_reported": %s}' \
             "$tag" "${BUILD_RC[$tag]}" "${RUN_RC[$tag]}" "$(verdict_of "$tag")" \
             "$(wc -l < "out/${tag}.txt")"
  done
  echo ""
  echo "  }"
  echo "}"
} > "$OUT_JSON"
cat "$OUT_JSON"

cfi_verdict="$(verdict_of cfi)"
if [ -n "$RED_TEST" ]; then
    # Inverted: the perturbed CFI translation MUST stop matching the oracle.
    if [ "$cfi_verdict" = "MATCHES_ORACLE" ]; then
        echo "RED TEST FAILED: the perturbation changed nothing the probe can see." >&2
        exit 1
    fi
    echo "RED TEST OK: perturbed cfi -> ${cfi_verdict}"
    exit 0
fi
exit 0
