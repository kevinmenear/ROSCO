#!/usr/bin/env python3
"""Turn one probe run's output into a RESULT ARTIFACT under harness/.

    probe_artifact.py <run-log> <out.json> <label> <sed-expr-or-empty>

WHY. `revcheck` and the done-condition read JSON under `harness/`, `mutation/`
and `gate/` and ask which instrument produced each one. This unit's only
instrument is a standalone probe whose output is text, so `revcheck` reported
NOT_EVALUABLE -- "no result artifact carries a revision to check". That is the
correct answer to the wrong artifact set: the measurement exists, it just was
not in a form anything could check the provenance of.

The file is named `<Unit>.probe.json`, NOT `<Unit>.json` or
`<Unit>.postintegration.json`, and the distinction is load-bearing: those two
names are what the done-condition reads for the campaign's GENERATED harness,
and this is not that harness. `revcheck` globs `harness/<Unit>.*` and will pick
this up; the done-condition's `harness_result_glob` will not, which is right --
the generated harness refused this unit and no file here should make it look
otherwise.

`loop_rev` and `vit_rev` come from `scripts/_harness_stamp.py`'s own `_rev`,
IMPORTED rather than re-implemented (P4). Its `main(--pre)` cannot be used for
this: on a GREEN pre-integration artifact it returns immediately without
writing, because "vit_harness.py already stamped it" -- true for its intended
caller and false here, since vit_harness.py never ran on this unit. That was
measured, not assumed: the first artifact came back with `loop_rev: None`.
`--pre --red-test` DOES write, so a red probe run is stamped by the tool.
"""
import json
import re
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts'))
from _harness_stamp import _rev  # noqa: E402

REPOS = Path.home() / 'Artifacts' / 'vit_translation'

# TYPE(ControlParameters) has 213 components; the probe also compares
# ErrVar%aviFAIL and ErrVar%ErrMsg. `checked` is cases x fields, which is the
# count a reader should compare against other units' `checked`.
FIELDS_PER_CASE = 213 + 2

INSTRUMENT = (
    "standalone differential probe -- NOT the campaign's generated harness, "
    "which REFUSED this unit (evidence/ReadControlParameterFileSub/"
    "harness.plan-dump.the-file-is-neither-supplied-nor-compared.txt). One "
    "Fortran program calls the reference and the translation on the SAME real "
    "DISCON.IN and compares all 213 components of TYPE(ControlParameters) -- "
    "the comparison generated from the type definition itself -- plus "
    "ErrVar%aviFAIL and ErrVar%ErrMsg. The C++ side reaches its Fortran type "
    "through vit_view_in_controlparameters, the copy-back routine `vit "
    "integrate` never emits a call to."
)

CANNOT_SEE = (
    "THE CORPUS VARIES NOTHING. 37 real DISCON.IN files are 37 points, not a "
    "corpus: no R2 flag ladder, no R5 shape pair, no R6 literal corpus and no "
    "R11 admissible state shapes them. RT2 measures what that costs -- "
    "a*(x*x) and (a*x)*x are indistinguishable over all 37, so the "
    "parenthesisation in VS_MinOMTq/VS_MaxOMTq is transcribed and untested. "
    "AND IT CANNOT BE MUTATION-SCORED: vit_mutate.py drives the generated "
    "harness's case file and its ./test, neither of which exists here, so P12 "
    "has no number for this unit. The echo comparison is 149 bytes and not 149 "
    "of ~15,000, because the ~193 per-parameter records belong to the "
    "Parse*_Opt callees and those five units are integrated and emit none."
)


POST_MEASURES = (
    "THE INTEGRATION WRAPPER'S MARSHALLING ONLY. After integration the Fortran "
    "body IS the translation, so both sides of this comparison run the same "
    "arithmetic and a difference means the wrapper corrupted an argument or "
    "threw an output away on the way through. That is not a weaker check here "
    "than it is for any other unit -- it is the ONE this unit needed. ~50 of "
    "its outputs are ALLOCATABLE components of a view-typed INOUT argument, "
    "and no `vit integrate` mode carried one back until `--copy-arrays`; the "
    "reference side of this run is the first wrapper that calls "
    "vit_view_in_controlparameters, so every field it compares is a field the "
    "integration could have dropped and did not. The arithmetic is covered by "
    "the pre-integration probe and by the mutation score taken over it."
)


def main(argv):
    if len(argv) not in (5, 6):
        print(__doc__.strip().split('\n')[2], file=sys.stderr)
        return 2
    log, out, label, sedx = argv[1:5]
    post = len(argv) == 6 and argv[5] == 'post' 
    text = open(log).read()
    m = re.search(r'^cases (\d+)\s+FIELD \(translation\) (\d+)\s+COPYBK[^0-9]*(\d+)',
                  text, re.M)
    if not m:
        # REFUSE rather than write a zero. A summary line that is missing means
        # the run did not finish, and an artifact saying `failed: 0` because
        # nothing was parsed is the exact shape of a green that measured
        # nothing.
        print('probe_artifact: no summary line in %s; refusing to write %s'
              % (log, out), file=sys.stderr)
        return 1
    cases, failed, copyback = (int(g) for g in m.groups())
    echo = re.search(r'IDENTICAL over all (\d+) bytes', text)
    doc = {
        "unit": "ReadControlParameterFileSub",
        # The done-condition reads this key and requires the exact string
        # `integrated` for a post-integration re-run (loop/done.py P11).
        "against": "integrated" if post else
                   "the clean Fortran reference, in the same process",
        "instrument": INSTRUMENT,
        "cases": cases,
        "fields_per_case": FIELDS_PER_CASE,
        "checked": cases * FIELDS_PER_CASE,
        # `failed` COUNTS FIELD REPORTS, NOT CASES, and the two differ here.
        # _harness_stamp.py's red-test template renders it as "N of <checked>
        # case(s) failed", which is its wording and not this artifact's: one
        # case can report several differing fields (RT8 reports two) and one
        # differing field can be reported by several cases (RT1 reports one
        # field on 36 of the 37).
        "failed": failed,
        "failed_counts": "differing FIELD reports, not cases",
        "copyback_unallocated_vs_empty": copyback,
        "echo_bytes_identical": int(echo.group(1)) if echo else 0,
        "cannot_see": CANNOT_SEE,
        "loop_rev": _rev(REPOS / 'translation-loop'),
        "vit_rev": _rev(REPOS / 'vit'),
    }
    if post:
        doc["measures"] = POST_MEASURES
    if sedx:
        doc["red_test"] = sedx
        doc["red_test_label"] = label
    with open(out, 'w') as f:
        json.dump(doc, f, indent=1)
        f.write('\n')
    print('probe_artifact: wrote %s -- %d case(s), %d checked, %d failed'
          % (out, cases, doc["checked"], failed))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
