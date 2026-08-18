#!/bin/bash
# Build `parser_conformance.cpp` against the SHIPPED translation and replay all
# 113 gfortran records through it. With `--red-tests`, also replay three
# deliberately perturbed copies of the parser and assert each goes red.
#
#   bash evidence/ParseInAry_Opt/run_parser_replay.sh [--red-tests]
#
# X4: a green is not evidence on first use. The three perturbations are chosen
# to be the mistakes this parser could plausibly HAVE, not arbitrary damage:
#
#   1. ';' put into `is_value_terminator` -- which is unit #54's REAL separator
#      set, i.e. exactly what copying the sibling would have produced.
#   2. the `sep != Sep::Blank` half of the semicolon guard deleted -- the guard
#      that 31 measured records exist for.
#   3. `parse_int` made to store 0 for a field with no digits -- the REAL
#      reader's zero-store rule, i.e. the other half of the same wrong copy.
#
# THE PERTURBED COPY IS MADE ON THE HOST AND `docker cp`d IN, not written
# through the bind mount: unit #23's finding is that a host write can be read
# half-finished by the container. Each patch ASSERTS it changed the file (P10 --
# a red test that silently perturbed nothing is a green wearing a red's name).
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"
C=vit-dev
W=/workspace/ROSCO-r2
SRC=translations/ROSCO_Helpers/parseinary_opt.cpp

build_and_run() {   # $1 = CONTAINER directory holding parseinary_opt.cpp  $2 = label
    local rc=0
    docker exec "$C" bash -lc "
        set -e
        GCC_INC=\$(dirname \$(gfortran -print-file-name=include/ISO_Fortran_binding.h))
        g++ -std=c++17 -O2 -ffp-contract=off -I\"\$GCC_INC\" \
            -I$W/rosco/controller/src -I$1 \
            $W/evidence/ParseInAry_Opt/parser_conformance.cpp \
            -o /tmp/pc_replay -lgfortran
        /tmp/pc_replay $W/evidence/ParseInAry_Opt/parser_conformance.txt" || rc=$?
    echo "  [$2] exit $rc"
    return $rc
}

echo "=== the shipped translation ==="
build_and_run "$W/translations/ROSCO_Helpers" "green"

[ "${1:-}" = "--red-tests" ] || exit 0

perturb() {   # $1 = label   $2 = FROM   $3 = TO   $4 = description
    local label="$1" from="$2" to="$3" what="$4"
    local dir; dir=$(mktemp -d)
    python3 - "$SRC" "$dir/parseinary_opt.cpp" "$from" "$to" <<'PY'
import sys, pathlib
src, dst, frm, to = sys.argv[1:5]
s = pathlib.Path(src).read_text()
n = s.count(frm)
assert n == 1, f"the perturbation site occurs {n} times, expected exactly 1:\n  {frm}"
pathlib.Path(dst).write_text(s.replace(frm, to))
print(f"  patched 1 site: {frm.strip()}  ->  {to.strip()}")
PY
    docker exec "$C" bash -lc "rm -rf /tmp/red_$label && mkdir -p /tmp/red_$label"
    docker cp "$dir/parseinary_opt.cpp" "$C:/tmp/red_$label/parseinary_opt.cpp" > /dev/null
    rm -rf "$dir"
    echo "=== RED TEST: $what ==="
    if build_and_run "/tmp/red_$label" "red-$label"; then
        echo "  *** the replay stayed GREEN under a perturbation -- it is not an instrument"
        exit 1
    fi
    docker exec "$C" bash -lc "rm -rf /tmp/red_$label"
}

perturb semisep \
    "return is_blank(c) || is_eol(c) || c == ',' || c == '/';" \
    "return is_blank(c) || is_eol(c) || c == ',' || c == '/' || c == ';';" \
    "';' added to is_value_terminator -- the sibling unit's REAL separator set"

perturb semiguard \
    "if (first_item || sep != Sep::Blank) {" \
    "if (first_item) {" \
    "the sep != Sep::Blank half of the semicolon guard deleted"

# THE FIRST FORM OF THIS ONE WAS `out = 0;` INSIDE `parse_int`, AND IT STAYED
# GREEN -- correctly. `out` is discarded by the caller on a false return, so the
# edit changed the FILE and not the PROGRAM. The script's own assertion (the
# patch applied, exactly one site) cannot tell those apart. Moved to the caller,
# where the store reaches `v[i]`, it is the REAL reader's rule transplanted and
# it goes red. Recorded in DECISIONS.md: asserting that a perturbation changed
# the source is weaker than asserting it changed the answer.
perturb zerostore \
    "        if (!parse_int(rec, len, p, value)) {
            return 5010;
        }" \
    "        if (!parse_int(rec, len, p, value)) {
            v[i] = value;
            return 5010;
        }" \
    "the failing item stores 0 instead of keeping its value -- the REAL reader's rule"

echo "=== all three perturbations turned the replay red ==="
