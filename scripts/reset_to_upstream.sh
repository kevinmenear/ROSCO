#!/bin/bash
# Reset ROSCO source to unmodified upstream state (e8010f0).
#
# Produces the original NREL/ROSCO v2.9.0 source with -ffp-contract=off
# added for bit-identical comparison. No VIT modifications — no BIND(C),
# no LOGICAL→INTEGER(C_INT), no bug fixes.
#
# Purpose: capture upstream baseline arrays for verifying that VIT's type
# modifications (Phase 4A/4C) are behaviorally neutral.
#
# What it resets:
#   - ALL files in rosco/controller/src/ → e8010f0 (upstream release)
#   - CMakeLists.txt → e8010f0 (Fortran-only) + -ffp-contract=off
#   - Removes all .cpp and .h files from src/
#
# WARNING: Scenario 4 (Flp_Mode=2) will segfault with upstream ROSCO due
# to uninitialized K in FlapControl init (ROSCO bug, fixed in our fork).
# Skip Scenario 4 when running simulations from this state.
#
# Usage:
#   cd ~/Artifacts/vit_translation/ROSCO
#   bash scripts/reset_to_upstream.sh

set -e

if [ ! -d "rosco/controller/src" ]; then
    echo "ERROR: Must run from ROSCO repo root (rosco/controller/src/ not found)" >&2
    exit 1
fi

echo "Resetting ROSCO source to upstream (e8010f0)..."

# --- Restore all source files to upstream ---
git checkout e8010f0 -- rosco/controller/src/
git checkout e8010f0 -- rosco/controller/CMakeLists.txt

# Add -ffp-contract=off to gfortran flags (not in upstream, needed for
# bit-identical comparison against our modified fork and C++ translations)
sed -i '' 's/-fdefault-double-8 -cpp"/-fdefault-double-8 -cpp -ffp-contract=off"/' \
    rosco/controller/CMakeLists.txt

# --- Unstage everything ---
git reset HEAD -- rosco/controller/ > /dev/null 2>&1

# --- Remove any C++ artifacts from src/ ---
rm -f rosco/controller/src/*.cpp rosco/controller/src/*.h

# --- Verification ---
echo ""
echo "Verification:"

# Check ROSCO_Types.f90 is upstream (no BIND(C), LOGICAL restart)
bind_c_count=$(grep -c 'BIND(C)' rosco/controller/src/ROSCO_Types.f90 || true)
if [ "$bind_c_count" -eq 0 ] && \
   grep -q 'LOGICAL.*restart' rosco/controller/src/ROSCO_Types.f90 && \
   grep -q 'COMPLEX.*AWC_complexangle' rosco/controller/src/ROSCO_Types.f90; then
    echo "  ROSCO_Types.f90: upstream (no BIND(C), LOGICAL restart, COMPLEX AWC) OK"
else
    echo "  ROSCO_Types.f90: WARNING — not upstream state"
fi

# Check no C++ artifacts
cpp_count=$(ls rosco/controller/src/*.cpp 2>/dev/null | wc -l | tr -d ' ')
h_count=$(ls rosco/controller/src/*.h 2>/dev/null | wc -l | tr -d ' ')
echo "  C++ files in src/: $cpp_count .cpp, $h_count .h (both should be 0)"

# Check CMakeLists.txt
if grep -q 'LANGUAGES Fortran C)' rosco/controller/CMakeLists.txt && \
   grep -q 'src/DISCON.F90' rosco/controller/CMakeLists.txt && \
   ! grep -q '\.cpp' rosco/controller/CMakeLists.txt && \
   grep -q 'ffp-contract=off' rosco/controller/CMakeLists.txt; then
    echo "  CMakeLists.txt: pure Fortran build with -ffp-contract=off OK"
else
    echo "  CMakeLists.txt: WARNING — unexpected state"
fi

echo ""
echo "Reset complete. (Reminder: skip Scenario 4 — upstream Flp_Mode=2 bug)"
