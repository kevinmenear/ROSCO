"""IS THE compare_record ADDITION ADDITIVE FOR EVERY UNIT THAT DOES NOT ASK FOR IT?

A control on the emitter, not on a unit: emit the SAME spec through the emit.py
this dispatch wrote and through the one it replaced, with compare_record absent,
and compare the generated C++ byte for byte. If a single line differs, every
other unit's harness has been changed by a change that claims to be opt-in.
"""
import importlib.util, sys, pathlib
ROOT = pathlib.Path("/workspace/translation-loop")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))

import harness.emit as new
spec_new = importlib.util.spec_from_file_location("emit_old", "/tmp/emitctl/emit_old.py")
old = importlib.util.module_from_spec(spec_new)
sys.modules['emit_old'] = old
spec_new.loader.exec_module(old)

import test_emit_decomposed as T
from harness.generate import generate

for mod, name in ((old, "before"), (new, "after")):
    pass

def build(mod):
    s = mod.CallSpec(
        "Widget", "Widget", "widget_f90",
        (mod.CArg("CntrPar", "controlparameters_view_t*", "struct",
                  struct_type="controlparameters_view_t",
                  fields=mod._struct_fields("Widget", "CntrPar", T.INFO, T.SIG)[0]),),
        None, "widget.hpp")
    gen = generate(T.SIG, seed=0, n_random=2)
    return mod.emit_source(s, T.SIG, len(gen.cases), "cases.bin",
                           ["  applied  R4_compare_all_outputs  return value + 1 out-parameter(s)"])

a, b = build(old), build(new)
print(f"emit.py BEFORE: {len(a)} bytes   AFTER: {len(b)} bytes")
print("byte-identical with compare_record absent:", a == b)
if a != b:
    import difflib
    for line in list(difflib.unified_diff(a.splitlines(), b.splitlines(), "before", "after", lineterm=""))[:40]:
        print(line)
c = build.__globals__  # noqa
# And the opposite control: with it ON, the source MUST differ, or the flag does nothing.
s = new.CallSpec("Widget", "Widget", "widget_f90",
                 (new.CArg("CntrPar", "controlparameters_view_t*", "struct",
                           struct_type="controlparameters_view_t",
                           fields=new._struct_fields("Widget", "CntrPar", T.INFO, T.SIG)[0]),),
                 None, "widget.hpp")
gen = generate(T.SIG, seed=0, n_random=2)
on = new.emit_source(s, T.SIG, len(gen.cases), "cases.bin", [], compare_record="because")
print("with compare_record ON the source differs:", on != b)
print("  and it carries the capture:", "vit_record.begin()" in on, "/ the counter:",
      "vit_record_nonempty" in on)
