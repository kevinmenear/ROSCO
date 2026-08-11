# ColemanTransform — the wrong artifacts, kept

C12: a defect that passed a layer it should have failed is recorded here with
the artifact that passed, before anything was changed to fix it.

## `kernel-window-1-20.*` — the kernel verification was vacuous

`vit.yaml` carried `kgen: invocation: 0:0:1-20` — the campaign's setup value,
derived from the first replication's setup commit. It captures the first 21
calls at the extracted call site. At `Controllers.f90:510` those 21 calls all
happen at simulation start, where `LocalVar%Azimuth` is 0 and `rootMOOPF` is 0.

So every one of the 21 cases fed ColemanTransform zero and expected zero back.

`kernel-window-1-20.stub.cpp` is a translation that reads none of its inputs and
writes `0.0` to both outputs. Against that window it produced:

    ✓ VERIFICATION PASSED: 21/21 passed
    4725 field entries, 4725 IDENTICAL

`kernel-window-1-20.stub-passes.verify_fields.csv` is that run's field log. The
four fields the function actually computes — `axistilt_1p`, `axisyaw_1p`,
`axistilt_2p`, `axisyaw_2p` — are `0.0000000000000000` in the reference column
for all 21 cases.

The green was real, the comparison was real, 4,725 values were compared, and it
constrained nothing about this function. `21/21 IDENTICAL` and `a stub passes`
were the same measurement.

Fixed by widening the window to a region where the rotor is turning, not by
changing what is compared. See DECISIONS.md.

## `kernel-window-27.*` — the working window, twice, from two VITs

`kernel-window-27.verify_fields.csv` and `.stub-fails.verify_fields.csv` are the
first pass, taken with VIT `d85b33b`.

The `.vit-d07a716.*` files are the SAME three measurements repeated on the
second pass, after VIT was reconciled with canonical (89 commits, DECISIONS.md).
They are kept beside the originals rather than replacing them, because the
question they answer is not "what does the kernel say" but "does it still say it
under a different instrument":

| file | run | result |
|---|---|---|
| `kernel-window-27.vit-d07a716.verify_fields.csv` | the translation | 63/63, 14,175 entries, all IDENTICAL |
| `kernel-window-27.vit-d07a716.stub-fails.verify_fields.csv` | the zero-writing stub | 124 of 14,175 `OUT_TOL` — the kernel discriminates |
| `kernel-window-27.vit-d07a716.statefiles.lst` | — | the 63 captured state files, by name |

The stub log is the one worth reading. 124 differing entries out of 14,175 is
what "this window is alive" looks like from the inside: 62 of 63 cases have a
non-zero `axistilt_1p` and `axisyaw_1p`, and those 124 values are the whole of
what separates the real translation from a function that ignores its arguments.
Under the `1-20` window the same stub produced zero differing entries.

Green was re-confirmed after the stub was reverted (63/63), so the red is
attributable to the stub and not to anything else that moved between runs.
