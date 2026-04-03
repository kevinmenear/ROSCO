#!/bin/bash
# Reset ROSCO source to a buildable clean Fortran state.
#
# Produces a pure Fortran build — no C++ compiler, no .cpp files, no VIT
# headers. After running this script, ROSCO can be built as a Fortran-only
# project to capture baseline simulation arrays.
#
# What it resets:
#   - Controllers.f90 + ControllerBlocks.f90 → 46e8d4f (Phase 4C + K init fix)
#   - Functions.f90 + Filters.f90 → dba7738 (restart fix + notch filter bug fix)
#   - DISCON.F90 → e8010f0 (upstream, no wrappers)
#   - ReadSetParameters.f90, ExtControl.f90, ZeroMQInterface.f90, ROSCO_IO.f90 → HEAD
#   - CMakeLists.txt → e8010f0 (upstream Fortran-only) + -ffp-contract=off
#   - Removes all .cpp and .h files from src/
#
# What it does NOT reset:
#   - ROSCO_Types.f90 (Phase 4A/4C type fixes, required by bug-fixed sources)
#   - translations/, vit.yaml, kernel/
#
# Usage:
#   cd ~/Artifacts/vit_translation/ROSCO
#   bash scripts/reset_to_clean.sh

set -e

if [ ! -d "rosco/controller/src" ]; then
    echo "ERROR: Must run from ROSCO repo root (rosco/controller/src/ not found)" >&2
    exit 1
fi

echo "Resetting ROSCO source to clean Fortran..."

# --- Restore .f90 source files ---

# Controllers.f90 + ControllerBlocks.f90: Phase 4C with K init fix (no wrappers)
git checkout 46e8d4f -- rosco/controller/src/Controllers.f90 rosco/controller/src/ControllerBlocks.f90

# Functions.f90 + Filters.f90: restart fix + notch filter bug fix (no wrappers)
git checkout dba7738 -- rosco/controller/src/Functions.f90 rosco/controller/src/Filters.f90

# DISCON.F90: upstream (no wrappers)
git checkout e8010f0 -- rosco/controller/src/DISCON.F90

# ReadSetParameters.f90: committed version (Phase 4A fixes, no wrappers)
git checkout -- rosco/controller/src/ReadSetParameters.f90

# Stage C/D files: committed version (clean, no wrappers)
git checkout -- rosco/controller/src/ExtControl.f90 rosco/controller/src/ZeroMQInterface.f90 rosco/controller/src/ROSCO_IO.f90

# --- Restore CMakeLists.txt to upstream Fortran-only build ---

# Upstream CMakeLists.txt: LANGUAGES Fortran C, 12 .f90 SOURCES only,
# no .cpp files, no CXX, no view populator .f90 files.
git checkout e8010f0 -- rosco/controller/CMakeLists.txt

# Add -ffp-contract=off to gfortran flags (not in upstream, needed for
# bit-identical verification against C++ translations)
sed -i '' 's/-fdefault-double-8 -cpp"/-fdefault-double-8 -cpp -ffp-contract=off"/' \
    rosco/controller/CMakeLists.txt

# --- Unstage everything ---
# git checkout <commit> -- <file> stages changes. Unstage to avoid
# phantom staged diffs that could be accidentally committed.
git reset HEAD -- rosco/controller/src/Controllers.f90 rosco/controller/src/ControllerBlocks.f90 \
    rosco/controller/src/Functions.f90 rosco/controller/src/Filters.f90 \
    rosco/controller/src/DISCON.F90 rosco/controller/CMakeLists.txt > /dev/null 2>&1

# --- Remove all C++ artifacts from src/ ---
# A clean Fortran build has no .cpp or .h files. These are created by
# integrate_all.sh during the translation/integration cycle.
rm -f rosco/controller/src/*.cpp rosco/controller/src/*.h

# --- Verification ---
echo ""
echo "Verification (all should be 0):"
grep -c '_c(' rosco/controller/src/Functions.f90 rosco/controller/src/Filters.f90 \
              rosco/controller/src/Controllers.f90 rosco/controller/src/ControllerBlocks.f90 \
              rosco/controller/src/ReadSetParameters.f90 || true

# Check no C++ artifacts remain
cpp_count=$(ls rosco/controller/src/*.cpp 2>/dev/null | wc -l | tr -d ' ')
h_count=$(ls rosco/controller/src/*.h 2>/dev/null | wc -l | tr -d ' ')
echo "  C++ files in src/: $cpp_count .cpp, $h_count .h (both should be 0)"

# Check CMakeLists.txt is pure Fortran
if grep -q 'LANGUAGES Fortran C)' rosco/controller/CMakeLists.txt && \
   grep -q 'src/DISCON.F90' rosco/controller/CMakeLists.txt && \
   ! grep -q '\.cpp' rosco/controller/CMakeLists.txt && \
   grep -q 'ffp-contract=off' rosco/controller/CMakeLists.txt; then
    echo "  CMakeLists.txt: pure Fortran build with -ffp-contract=off OK"
else
    echo "  CMakeLists.txt: WARNING — unexpected state"
fi

echo ""
echo "Reset complete."
