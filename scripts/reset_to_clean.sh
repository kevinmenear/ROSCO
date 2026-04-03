#!/bin/bash
# Reset ROSCO source to clean Fortran for extraction, integration testing,
# or baseline capture.
#
# This script resets ALL source files and the build configuration to a
# buildable clean Fortran state (no integration wrappers, no C++ entry point)
# with all bug fixes applied. After running this script, ROSCO can be built
# as a pure Fortran project to capture baseline simulation arrays.
#
# What it resets:
#   - Controllers.f90 + ControllerBlocks.f90 → 46e8d4f (Phase 4C + K init fix)
#   - Functions.f90 + Filters.f90 → dba7738 (restart fix + notch filter bug fix)
#   - DISCON.F90 → e8010f0 (upstream, no wrappers)
#   - ReadSetParameters.f90 → HEAD (committed version has Phase 4A fixes, no wrappers)
#   - CMakeLists.txt → aef3155 (full Fortran build: DISCON.F90, all .f90 sources, gfortran)
#   - All *.cpp files → "// stub"
#
# What it does NOT reset (these have Phase 4A/4C fixes and no wrappers):
#   - ROSCO_Types.f90
#   - translations/, vit.yaml, kernel/
#
# Usage:
#   cd ~/Artifacts/vit_translation/ROSCO
#   bash scripts/reset_to_clean.sh

set -e

# Verify we're in the ROSCO repo root
if [ ! -d "rosco/controller/src" ]; then
    echo "ERROR: Must run from ROSCO repo root (rosco/controller/src/ not found)" >&2
    exit 1
fi

echo "Resetting ROSCO source to clean Fortran..."

# Controllers.f90 + ControllerBlocks.f90: Phase 4C with K init fix (no wrappers)
git checkout 46e8d4f -- rosco/controller/src/Controllers.f90 rosco/controller/src/ControllerBlocks.f90

# Functions.f90 + Filters.f90: restart fix + notch filter bug fix (no wrappers)
git checkout dba7738 -- rosco/controller/src/Functions.f90 rosco/controller/src/Filters.f90

# DISCON.F90: upstream (no wrappers)
git checkout e8010f0 -- rosco/controller/src/DISCON.F90

# CMakeLists.txt: full Fortran build configuration (pre-Phase 11A)
# This restores DISCON.F90 in SOURCES, all .f90 modules, gfortran flags,
# NWTC_SYS_FILE block, and -ffp-contract=off on both Fortran and C++.
git checkout aef3155 -- rosco/controller/CMakeLists.txt

# ReadSetParameters.f90: restore committed version (Phase 4A fixes, no wrappers)
git checkout -- rosco/controller/src/ReadSetParameters.f90

# Stage C/D files: restored to HEAD (clean, no wrappers — integrate_all.sh modifies them)
git checkout -- rosco/controller/src/ExtControl.f90 rosco/controller/src/ZeroMQInterface.f90 rosco/controller/src/ROSCO_IO.f90

# Create C++ stubs (CMakeLists.txt references these)
# Stub existing files
for f in rosco/controller/src/*.cpp; do
    [ -f "$f" ] && echo "// stub" > "$f"
done
# Also create stubs for any .cpp files listed in CMakeLists.txt that don't exist yet
grep -oE 'src/[^ ]+\.cpp' rosco/controller/CMakeLists.txt | while read f; do
    filepath="rosco/controller/$f"
    if [ ! -f "$filepath" ]; then
        echo "// stub" > "$filepath"
    fi
done

# Verify clean state (grep -c returns 1 when count is 0, so use || true)
echo ""
echo "Verification (all should be 0):"
grep -c '_c(' rosco/controller/src/Functions.f90 rosco/controller/src/Filters.f90 \
              rosco/controller/src/Controllers.f90 rosco/controller/src/ControllerBlocks.f90 \
              rosco/controller/src/ReadSetParameters.f90 || true

# Verify CMakeLists.txt has Fortran build configuration
if grep -q 'LANGUAGES Fortran C' rosco/controller/CMakeLists.txt && \
   grep -q 'src/DISCON.F90' rosco/controller/CMakeLists.txt; then
    echo "  CMakeLists.txt: Fortran build OK"
else
    echo "  CMakeLists.txt: WARNING — missing Fortran configuration"
fi

echo ""
echo "Reset complete."
