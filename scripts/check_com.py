#!/usr/bin/env python3
"""a later check: the centre of mass the URDF IMPLIES against what the stage holds.

WHY THIS IS SEPARATE FROM THE READBACK. `readback_isaac.py` compares mass, and
every mass on this robot matches exactly. Mass is how much there is; centre of
mass is where it is. They are different facts and only the first was checked.

WHAT THE IMPORTER ACTUALLY DOES, measured on Isaac Sim 6.0.1 and then confirmed
in its source (`_impl/link.py`, `if link.inertial.origin:`):

    physics:centerOfMass is written ONLY when the URDF link has an
    <inertial><origin>. Otherwise the attribute is absent entirely.

On this robot that means two links of seven carry it -- the wheels, which have
an inertial origin because of their 90 deg roll -- and five carry nothing.
Reading the CoM back off the LIVE rigid bodies (not the file) shows PhysX
defaults all of them to (0, 0, 0), the link origin.

THAT DEFAULT IS CORRECT HERE, BY CONSTRUCTION AND NOT BY LUCK-FREE DESIGN.
Every visual and collision element on this robot sits at its own link origin
with no offset, so "the middle of the link" and "the centre of the mass" are
the same point. A robot with a battery pack at one end does not have that
property, and if its URDF does not state the offset then PhysX puts the mass in
the middle: same total mass, same total inertia, wrong moment arm to the contact
patches. It drives straight correctly and pitches and corners wrong.

SO THE REPORT DISTINGUISHES TWO KINDS OF AGREEMENT. "stated" means the stage
holds an explicit value that matches. "default" means nothing was stated and
PhysX's default happens to equal what the URDF implies. Both pass. They are not
the same confidence, and collapsing them would hide exactly the case above.

Run:
    python3 scripts/check_com.py
    python3 scripts/check_com.py --selftest

Exit 1 if any link's centre of mass disagrees with what the URDF implies.
"""
import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
URDF = BASE / "data/rover.urdf"
PHYS = BASE / "sim-workspace/isaac/rover.usd/rover/payloads/Physics/physics.usda"

TOL = 1e-6      # metres. This is an exact-transfer field, not a simulation result.
PHYSX_DEFAULT = [0.0, 0.0, 0.0]


def _rb():
    """Reuse readback_isaac's block parser rather than re-deriving it.

    Its `_own_attr*` helpers already handle the trap that cost three false
    findings: USD nests links inside their parents, so a whole-subtree search
    returns a descendant's value.
    """
    spec = importlib.util.spec_from_file_location(
        "rb", BASE / "scripts/readback_isaac.py")
    m = importlib.util.module_from_spec(spec)
    argv, sys.argv = sys.argv, ["readback_isaac.py"]
    try:
        spec.loader.exec_module(m)
    finally:
        sys.argv = argv
    return m


def implied_com(link):
    """The centre of mass the URDF implies for a link.

    Explicit if <inertial><origin xyz> says so. Otherwise the centroid of the
    link's geometry: for a single primitive that is the geometry's own origin
    offset, which is (0,0,0) unless the element declares otherwise.

    DELIBERATELY NOT A FULL CENTROID SOLVER. A link with several offset
    primitives has a real centroid this does not compute, and it says so rather
    than guessing -- a check that quietly approximates is worse than one that
    declines.
    """
    inert = link.find("inertial")
    if inert is not None:
        o = inert.find("origin")
        if o is not None and o.get("xyz"):
            return [float(v) for v in o.get("xyz").split()], "stated"
    offsets = []
    for tag in ("collision", "visual"):
        for el in link.findall(tag):
            o = el.find("origin")
            xyz = [float(v) for v in o.get("xyz").split()] if (
                o is not None and o.get("xyz")) else [0.0, 0.0, 0.0]
            offsets.append(xyz)
    if not offsets:
        return None, "no geometry"
    first = offsets[0]
    if any(o != first for o in offsets[1:]):
        return None, "multiple offset primitives; not computed"
    return first, "implied by geometry"


def check(urdf_path, phys_path):
    m = _rb()
    root = ET.parse(urdf_path).getroot()
    blocks = m._blocks(phys_path.read_text())
    rows, fails = [], []
    for link in root.findall("link"):
        if link.find("inertial") is None:
            continue
        name = link.get("name")
        want, how = implied_com(link)
        b = next((x for x in blocks if x[3] == name and
                  m._own_attr1(x[5], x[0], "float physics:mass") is not None), None)
        got = m._own_attr3(b[5], b[0], "point3f physics:centerOfMass") if b else None
        source = "stated" if got is not None else "physx default"
        eff = got if got is not None else list(PHYSX_DEFAULT)
        if want is None:
            rows.append({"link": name, "implied": None, "effective": eff,
                         "source": source, "ok": True, "note": how})
            continue
        ok = all(abs(a - b2) <= TOL for a, b2 in zip(want, eff))
        rows.append({"link": name, "implied": want, "effective": eff,
                     "source": source, "ok": ok, "note": how})
        if not ok:
            fails.append(f"{name}: urdf implies {want}, stage uses {eff} "
                         f"({source})")
    return rows, fails


def main():
    if not URDF.exists() or not PHYS.exists():
        sys.exit("run scripts/import_rover.py first")
    rows, fails = check(URDF, PHYS)
    print(f"CENTRE OF MASS: {len(rows)} links with an inertial block, "
          f"tolerance {TOL} m\n")
    w = max(len(r["link"]) for r in rows)
    for r in rows:
        imp = "not computed" if r["implied"] is None else str(r["implied"])
        print(f"  {r['link']:<{w}}  {'ok  ' if r['ok'] else 'DIFF'}  "
              f"implied {imp:<18} effective {r['effective']}  "
              f"[{r['source']}]")
    n_stated = sum(1 for r in rows if r["source"] == "stated")
    n_def = len(rows) - n_stated
    print(f"\n{n_stated} stated explicitly, {n_def} by PhysX default")
    if fails:
        print(f"\n{len(fails)} disagreement(s):")
        for f in fails:
            print("  -", f)
        return 1
    print("all links agree with what the URDF implies")
    if n_def:
        print(f"NOTE: {n_def} of them agree BY DEFAULT, not by statement. That "
              f"holds only while every geometry sits at its link origin.")
    return 0


def selftest():
    """Both directions, on synthetic URDFs and synthetic physics layers."""
    import tempfile
    bad = []

    def phys(name, com=None):
        com_line = (f"                    point3f physics:centerOfMass = "
                    f"({com[0]}, {com[1]}, {com[2]})\n") if com else ""
        return ('#usda 1.0\n(\n)\n\nover "rover"\n{\n    over "Geometry"\n    {\n'
                f'                over "{name}" (\n'
                '                    prepend apiSchemas = ["PhysicsMassAPI"]\n'
                '                )\n                {\n'
                f'{com_line}'
                '                    float physics:mass = 1\n'
                '                }\n    }\n}\n')

    def urdf(origin=None, vis_origin=None):
        o = f'<origin xyz="{origin}"/>' if origin else ""
        v = f'<origin xyz="{vis_origin}"/>' if vis_origin else ""
        return (f'<robot name="t"><link name="a"><inertial>{o}'
                f'<mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" '
                f'iyz="0" izz="1"/></inertial>'
                f'<collision>{v}<geometry><box size="1 1 1"/></geometry>'
                f'</collision></link></robot>')

    with tempfile.TemporaryDirectory() as td:
        d = Path(td)
        cases = [
            # (urdf, physics, expect_fail, description)
            (urdf(), phys("a"), False, "nothing stated, default 0 matches"),
            (urdf(origin="0 0 0"), phys("a", [0, 0, 0]), False,
             "stated 0 and stage says 0"),
            (urdf(origin="0.1 0 0"), phys("a", [0.1, 0, 0]), False,
             "stated offset carried across"),
            (urdf(origin="0.1 0 0"), phys("a"), True,
             "urdf states an offset, stage defaults to 0"),
            (urdf(origin="0 0 0"), phys("a", [0.1, 0, 0]), True,
             "stage holds an offset the urdf does not"),
            (urdf(vis_origin="0.2 0 0"), phys("a"), True,
             "geometry offset implies a CoM the default misses"),
        ]
        for i, (u, p, want_fail, what) in enumerate(cases):
            up, pp = d / f"u{i}.urdf", d / f"p{i}.usda"
            up.write_text(u)
            pp.write_text(p)
            _, fails = check(up, pp)
            if bool(fails) != want_fail:
                bad.append(f"  {what}: got {len(fails)} failure(s), expected "
                           f"{'>=1' if want_fail else '0'}")
    if bad:
        print("SELFTEST FAILED:")
        print("\n".join(bad))
        return 1
    print("selftest ok: 6 cases asserted, both directions")
    return 0


if __name__ == "__main__":
    sys.exit(selftest() if "--selftest" in sys.argv else main())
