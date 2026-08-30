#!/usr/bin/env python3
"""a later check: the units and frame declarations, checked across every layer.

WHY THIS IS ITS OWN CHECK. Every number the readback compares assumes metres
are metres and Z is up. Those are not properties of the robot -- they are
stage metadata, they are per-LAYER, and USD is perfectly happy to compose
layers that disagree. Nothing in a URDF states either of them, so the importer
decides, and if it decides differently from what your ROS 2 stack assumes then
every length in the readback agrees while the robot is the wrong size.

THE THREE FIELDS, AND WHY EACH ONE MATTERS HERE:

  metersPerUnit     ROS 2 is metres. Isaac's own assets are frequently
                    centimetres (metersPerUnit = 0.01), because that is the
                    Omniverse default for a lot of content. Import at 0.01 and
                    a 0.4 m chassis becomes 0.4 cm, or 40 m, depending which
                    way the mistake goes -- and the NUMBER in the file is
                    unchanged either way, so a value-by-value diff passes.

  kilogramsPerUnit  Same argument for mass. A 6 kg robot at the wrong scale is
                    6 grams or 6 tonnes, and it still reads "6".

  upAxis            ROS 2 and Gazebo are Z-up. A great deal of DCC content is
                    Y-up, and Isaac supports both. Import Y-up and gravity
                    pulls along what your URDF calls "forward".

EVERY LAYER, NOT JUST THE ROOT. The importer writes a layered stage -- root,
robot, base, materials, physics -- and each carries its own metadata block.
A stage whose root says metres while a payload says centimetres composes
without complaint. So this checks all of them and requires agreement.

Run:
    python3 scripts/check_units.py
    python3 scripts/check_units.py --selftest

Exit 1 if any layer disagrees with the expected ROS 2 convention.
"""
import re
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE_DIR = BASE / "sim-workspace/isaac/rover.usd/rover"

# THE ROS 2 CONVENTION, stated as the expectation rather than discovered.
EXPECT = {"metersPerUnit": 1.0, "kilogramsPerUnit": 1.0, "upAxis": "Z"}

NUM = re.compile(r'^\s*(metersPerUnit|kilogramsPerUnit)\s*=\s*([\d.eE+-]+)', re.M)
TOK = re.compile(r'^\s*(upAxis)\s*=\s*"([^"]+)"', re.M)


def layer_meta(path):
    """The metadata a single .usda layer declares. Missing is not the same as
    wrong: a layer that says nothing inherits, so report None and let the
    caller decide."""
    text = path.read_text()
    # Only the leading metadata block, between the first ( and its ).
    head = text[:text.index(")")] if ")" in text else text
    out = {k: None for k in EXPECT}
    for m in NUM.finditer(head):
        out[m.group(1)] = float(m.group(2))
    for m in TOK.finditer(head):
        out[m.group(1)] = m.group(2)
    return out


def check(stage_dir):
    layers = sorted(stage_dir.rglob("*.usda"))
    if not layers:
        return [], [f"no .usda layers under {stage_dir}"]
    rows, fails = [], []
    for p in layers:
        meta = layer_meta(p)
        rel = p.relative_to(stage_dir)
        for k, want in EXPECT.items():
            got = meta[k]
            if got is None:
                rows.append({"layer": str(rel), "field": k, "value": None,
                             "ok": True, "note": "inherits"})
                continue
            ok = (got == want)
            rows.append({"layer": str(rel), "field": k, "value": got,
                         "ok": ok, "note": ""})
            if not ok:
                fails.append(f"{rel}: {k} = {got!r}, expected {want!r}"
                             + (f"  -> every length is off by {want / got:g}x"
                                if k == "metersPerUnit" and got else "")
                             + (f"  -> gravity is not along the URDF's Z"
                                if k == "upAxis" else ""))
    return rows, fails


def main():
    if not STAGE_DIR.exists():
        sys.exit(f"no imported stage at {STAGE_DIR}; run scripts/import_rover.py")
    rows, fails = check(STAGE_DIR)
    print(f"UNITS AND FRAMES: {len({r['layer'] for r in rows})} layers, "
          f"{len(EXPECT)} fields each")
    print(f"expected (ROS 2 convention): "
          + ", ".join(f"{k}={v!r}" for k, v in EXPECT.items()) + "\n")
    w = max(len(r["layer"]) for r in rows)
    seen = set()
    for r in rows:
        if r["layer"] in seen:
            continue
        seen.add(r["layer"])
        vals = [x for x in rows if x["layer"] == r["layer"]]
        bad = [x["field"] for x in vals if not x["ok"]]
        detail = "  ".join(
            f"{x['field'].replace('PerUnit','')}="
            f"{'inherits' if x['value'] is None else x['value']}"
            for x in vals)
        print(f"  {r['layer']:<{w}}  {'DIFF' if bad else 'ok':<4}  {detail}")
    print()
    if fails:
        print(f"{len(fails)} unit/frame disagreement(s):")
        for f in fails:
            print("  -", f)
        return 1
    print("ALL LAYERS AGREE with the ROS 2 convention: "
          "metres, kilograms, Z up.")
    return 0


def selftest():
    """Assert BOTH directions on synthetic layers written to a temp dir.

    The real stage is currently correct, so a test that only ran against it
    would pass forever without proving the check can fail. Each case below
    plants one wrong field and requires it to be reported.
    """
    import tempfile
    bad = []
    good = '#usda 1.0\n(\n    kilogramsPerUnit = 1\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n'
    cases = [
        (good, 0, "a correct layer"),
        (good.replace("metersPerUnit = 1", "metersPerUnit = 0.01"), 1,
         "centimetres"),
        (good.replace("kilogramsPerUnit = 1", "kilogramsPerUnit = 1000"), 1,
         "tonnes"),
        (good.replace('upAxis = "Z"', 'upAxis = "Y"'), 1, "Y-up"),
        ('#usda 1.0\n(\n)\n', 0, "a layer that declares nothing"),
    ]
    for src, want_fails, what in cases:
        with tempfile.TemporaryDirectory() as td:
            d = Path(td) / "s"
            d.mkdir()
            (d / "layer.usda").write_text(src)
            _, fails = check(d)
            if bool(fails) != bool(want_fails):
                bad.append(f"  {what}: got {len(fails)} failures, "
                           f"expected {'>=1' if want_fails else '0'}")
    # And a MIXED stage: root correct, payload in centimetres. This is the case
    # the "every layer" rule exists for, and a root-only check would pass it.
    with tempfile.TemporaryDirectory() as td:
        d = Path(td) / "s"
        (d / "payloads").mkdir(parents=True)
        (d / "root.usda").write_text(good)
        (d / "payloads" / "geo.usda").write_text(
            good.replace("metersPerUnit = 1", "metersPerUnit = 0.01"))
        _, fails = check(d)
        if not fails:
            bad.append("  a mixed stage (root metres, payload centimetres) PASSED")
    if bad:
        print("SELFTEST FAILED:")
        print("\n".join(bad))
        return 1
    print(f"selftest ok: {len(cases)} single-layer cases and 1 mixed-layer "
          f"case asserted, both directions")
    return 0


if __name__ == "__main__":
    sys.exit(selftest() if "--selftest" in sys.argv else main())
