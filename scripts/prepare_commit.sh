#!/bin/bash
# Prepare the ROSCO working tree for committing on main.
#
# On main, the committed state is:
#   - .f90 files: clean Fortran (no integration wrappers)
#   - src/*.cpp + src/vit_translated.h: full integration content
#
# After a test cycle (reset_to_clean → integrate_all → sim comparison),
# the working tree has integration wrappers in .f90 files, regenerated
# .cpp files (may differ slightly from committed versions), date-stamped
# DISCON_*.IN files, and transient test artifacts.
#
# This script restores all integration artifacts to their committed state
# and removes transient files. Only genuinely NEW or CHANGED files
# (translations, kernels, scripts, docs) remain in git status.
#
# Workflow:
#   1. bash scripts/reset_to_clean.sh     (clean source for baseline)
#   2. build + capture baseline
#   3. bash scripts/integrate_all.sh      (integrate all functions)
#   4. build + compare against baseline
#   5. bash scripts/prepare_commit.sh     (clean up for commit)
#   6. For NEW functions: vit integrate NewFunc --apply (generates src/*.cpp)
#      then: git checkout -- rosco/controller/src/TheSourceFile.f90
#   7. git add ... && git commit
#
# Usage:
#   cd ~/Artifacts/vit_translation/ROSCO
#   bash scripts/prepare_commit.sh

set -e

if [ ! -d "rosco/controller/src" ]; then
    echo "ERROR: Must run from ROSCO repo root" >&2
    exit 1
fi

echo "Preparing working tree for commit..."

# 1. Restore .f90 source files to clean state (no wrappers)
echo "  Restoring .f90 files to clean Fortran..."
git checkout 46e8d4f -- rosco/controller/src/Controllers.f90 rosco/controller/src/ControllerBlocks.f90
git checkout dba7738 -- rosco/controller/src/Functions.f90 rosco/controller/src/Filters.f90
git checkout e8010f0 -- rosco/controller/src/DISCON.F90
git checkout -- rosco/controller/src/ReadSetParameters.f90

# 2. Restore tracked src/*.cpp files to committed state
#    (integrate_all.sh regenerates them, but output may differ slightly
#    from committed versions due to VIT code changes)
echo "  Restoring src/*.cpp to committed state..."
git checkout -- rosco/controller/src/*.cpp 2>/dev/null || true
git checkout -- rosco/controller/src/vit_translated.h 2>/dev/null || true

# 3. Restore tracked DISCON_*.IN date artifacts (vit_sim.py regenerates with today's date)
#    Must use git ls-files to avoid including untracked DISCON files in the glob,
#    which causes git checkout to fail on the untracked ones and skip the tracked ones.
echo "  Restoring DISCON_*.IN files..."
tracked_discon=$(git ls-files 'Examples/DISCON_*.IN')
if [ -n "$tracked_discon" ]; then
    echo "$tracked_discon" | xargs git checkout --
fi

# 4. Remove transient artifacts
echo "  Removing transient artifacts..."
rm -rf baseline_arrays/ integrated_arrays/
rm -f integrated_output.txt baseline_output.txt
rm -f Examples/core
rm -f rosco/controller/src/rosco_constants.h rosco/controller/src/vit_types.h

# 5. Create stubs for any NEW src/*.cpp files referenced in CMakeLists.txt
#    but not yet tracked (so cmake doesn't fail on clean builds)
grep -oE 'src/[^ ]+\.cpp' rosco/controller/CMakeLists.txt | while read f; do
    filepath="rosco/controller/$f"
    if [ ! -f "$filepath" ]; then
        echo "// stub" > "$filepath"
        echo "  Created stub: $filepath"
    fi
done

echo ""
echo "Verification:"
wrapper_count=$(grep -c '_c(' rosco/controller/src/Functions.f90 rosco/controller/src/Controllers.f90 rosco/controller/src/ControllerBlocks.f90 rosco/controller/src/ReadSetParameters.f90 2>/dev/null | awk -F: '{s+=$2} END {print s}')
echo "  .f90 wrapper calls: $wrapper_count (should be 0)"

cpp_stubs=$(grep -rl '// stub' rosco/controller/src/*.cpp 2>/dev/null | wc -l | tr -d ' ')
echo "  .cpp stub files: $cpp_stubs (should be 0)"

echo ""
if [ "$wrapper_count" -eq 0 ] && [ "$cpp_stubs" -eq 0 ]; then
    echo "Ready to commit. Run 'git status' to review."
else
    echo "WARNING: Unexpected state detected. Review before committing."
fi
