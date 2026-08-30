#!/usr/bin/env python3
"""a later check: is the inertia in the URDF physically sensible for the geometry?

A DIFFERENT KIND OF CHECK FROM EVERYTHING ELSE IN THIS REPO. Every other check
here is a TRANSFER check: does the stage hold what the file said. Those cannot
answer whether the file was right, and this robot's chassis is the proof --
notes 1.4 planted a solid-box tensor on what is described as a hollow shell,
and Gazebo, PhysX and the readback all carried it faithfully. A equals B says
nothing about A.

So this one never looks at the stage. It reads the URDF, computes the inertia
each link's geometry SHOULD have for its stated mass, and compares.

THE FORMULAE, stated so the numbers are checkable rather than magic. All for a
solid body of uniform density, about its own centre:

    box    (w, d, h)     Ixx = m(d^2 + h^2)/12,  and cyclically
    cylinder (r, l)      Iaxial = m r^2 / 2,  Itransverse = m(3r^2 + l^2)/12
    sphere  (r)          I = 2 m r^2 / 5, all three equal

UNIFORM DENSITY IS THE ASSUMPTION, AND IT IS OFTEN WRONG -- which is the point.
A real chassis is a shell with a battery in it, so its true tensor is NOT the
solid-box value. This check cannot tell you the true value; nothing can, short
of weighing the real robot. What it tells you is whether the number in your
file is the solid-body value, which is the single most common way a URDF
inertia gets written: somebody used a calculator, or copied a formula, and the
result is plausible-looking and wrong in a specific direction.

SO A "MATCH" HERE IS A WEAK SIGNAL AND A "MISMATCH" IS A STRONG ONE. Matching
the solid-body value means either the link really is solid, or somebody used
the same formula this script does. Not matching means somebody put real numbers
in, which is usually good. The report says which, and does not pretend the
first is a pass.

Run:
    python3 scripts/check_inertia_plausible.py
    python3 scripts/check_inertia_plausible.py --selftest

Exit code is 0 always: this reports, it does not gate. A plausibility finding
is a prompt to go and look, not a defect.
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
URDF = BASE / "data/rover.urdf"

REL = 0.05      # within 5 % counts as "matches the solid-body value"


def solid_inertia(geom, mass):
    """(Ixx, Iyy, Izz) for a uniform solid of this shape, about its centre."""
    box = geom.find("box")
    if box is not None:
        w, d, h = [float(v) for v in box.get("size").split()]
        return (mass * (d * d + h * h) / 12.0,
                mass * (w * w + h * h) / 12.0,
                mass * (w * w + d * d) / 12.0), f"box {w}x{d}x{h}"
    cyl = geom.find("cylinder")
    if cyl is not None:
        r, l = float(cyl.get("radius")), float(cyl.get("length"))
        trans = mass * (3 * r * r + l * l) / 12.0
        # URDF cylinders are Z-axial, so Izz is the axial moment.
        return (trans, trans, mass * r * r / 2.0), f"cylinder r={r} l={l}"
    sph = geom.find("sphere")
    if sph is not None:
        r = float(sph.get("radius"))
        i = 2.0 * mass * r * r / 5.0
        return (i, i, i), f"sphere r={r}"
    mesh = geom.find("mesh")
    if mesh is not None:
        return None, "mesh (no closed form)"
    return None, "unknown geometry"


def check(urdf_path):
    root = ET.parse(urdf_path).getroot()
    rows = []
    for link in root.findall("link"):
        inert = link.find("inertial")
        if inert is None:
            continue
        mass = float(inert.find("mass").get("value"))
        i = inert.find("inertia")
        stated = [float(i.get(k)) for k in ("ixx", "iyy", "izz")]
        # Prefer the COLLISION geometry: that is what the physics acts on, and
        # on this robot the lidar's visual is a mesh while its collision is the
        # analytic cylinder the inertia was written for.
        geom = None
        for tag in ("collision", "visual"):
            el = link.find(tag)
            if el is not None and el.find("geometry") is not None:
                geom = el.find("geometry")
                if tag == "collision":
                    break
        if geom is None:
            rows.append({"link": link.get("name"), "shape": "no geometry",
                         "stated": stated, "solid": None, "verdict": "skipped"})
            continue
        solid, shape = solid_inertia(geom, mass)
        if solid is None:
            rows.append({"link": link.get("name"), "shape": shape,
                         "stated": stated, "solid": None, "verdict": "skipped"})
            continue
        # REPORT THE MAGNITUDE, NOT JUST A VERDICT. camera_link trips a 5%
        # threshold on one axis (12%) purely because its inertia is written to
        # two significant figures -- 6e-06 against a solid value of 6.83e-06.
        # A bare "differs" reads as "somebody put real numbers in", which is
        # the opposite of the truth. The worst-axis percentage says which it is.
        worst = max(abs(a - b) / max(abs(b), 1e-12)
                    for a, b in zip(stated, solid))
        if worst <= REL:
            verdict = "solid-body value"
        elif worst <= 0.20:
            verdict = "solid, rounded"
        else:
            verdict = "differs"
        rows.append({"link": link.get("name"), "shape": shape, "mass": mass,
                     "stated": stated, "solid": list(solid),
                     "worst": worst, "verdict": verdict})
    return rows


def main():
    if not URDF.exists():
        sys.exit(f"no expanded URDF at {URDF}; run scripts/import_rover.py")
    rows = check(URDF)
    print(f"INERTIA PLAUSIBILITY: {len(rows)} links with an inertial block")
    print(f"comparing what is WRITTEN against the uniform-solid value "
          f"(within {REL:.0%})\n")
    w = max(len(r["link"]) for r in rows)
    for r in rows:
        if r["solid"] is None:
            print(f"  {r['link']:<{w}}  skipped   {r['shape']}")
            continue
        st = ", ".join(f"{v:.3g}" for v in r["stated"])
        so = ", ".join(f"{v:.3g}" for v in r["solid"])
        print(f"  {r['link']:<{w}}  {r['verdict']:<16}  "
              f"worst axis {r['worst']*100:5.1f} %   {r['shape']}")
        print(f"  {'':<{w}}    written  {st}")
        print(f"  {'':<{w}}    solid    {so}")
    n_solid = sum(1 for r in rows if r["verdict"] == "solid-body value")
    n_round = sum(1 for r in rows if r["verdict"] == "solid, rounded")
    n_diff = sum(1 for r in rows if r["verdict"] == "differs")
    print(f"\n{n_solid} exactly the uniform-solid value, {n_round} the same "
          f"value rounded, {n_diff} genuinely different")
    print("\nA MATCH IS A WEAK SIGNAL: it means the link really is solid, or")
    print("somebody used this same formula. A MISMATCH IS A STRONG ONE: it")
    print("means real numbers went in. Neither is a defect on its own.")
    return 0


def selftest():
    """Assert the formulae against hand-computable cases, both directions."""
    import tempfile
    bad = []

    def urdf(geom, ixx, iyy, izz, mass=1.0):
        return (f'<robot name="t"><link name="a"><inertial>'
                f'<mass value="{mass}"/><inertia ixx="{ixx}" ixy="0" ixz="0" '
                f'iyy="{iyy}" iyz="0" izz="{izz}"/></inertial>'
                f'<collision><geometry>{geom}</geometry></collision>'
                f'</link></robot>')

    # A 1 kg cube of side 1: I = m(1+1)/12 = 0.16667 on every axis.
    cube = '<box size="1 1 1"/>'
    # A 1 kg sphere of radius 1: I = 2/5 = 0.4 on every axis.
    sph = '<sphere radius="1"/>'
    # A 1 kg cylinder r=1 l=1: axial = 0.5, transverse = (3+1)/12 = 0.33333
    cyl = '<cylinder radius="1" length="1"/>'
    cases = [
        (urdf(cube, 0.166667, 0.166667, 0.166667), "solid-body value", "cube exact"),
        (urdf(cube, 0.10, 0.10, 0.10), "differs", "cube, real numbers"),
        (urdf(sph, 0.4, 0.4, 0.4), "solid-body value", "sphere exact"),
        (urdf(sph, 0.25, 0.25, 0.25), "differs", "sphere, hollow-ish"),
        (urdf(cyl, 0.333333, 0.333333, 0.5), "solid-body value", "cylinder exact"),
        (urdf(cyl, 0.5, 0.5, 0.333333), "differs", "cylinder, axes swapped"),
    ]
    with tempfile.TemporaryDirectory() as td:
        for i, (src, want, what) in enumerate(cases):
            p = Path(td) / f"u{i}.urdf"
            p.write_text(src)
            rows = check(p)
            got = rows[0]["verdict"]
            if got != want:
                bad.append(f"  {what}: got {got!r}, expected {want!r}")
        # a mesh collision must be SKIPPED, not guessed at
        p = Path(td) / "m.urdf"
        p.write_text(urdf('<mesh filename="x.stl"/>', 1, 1, 1))
        if check(p)[0]["verdict"] != "skipped":
            bad.append("  a mesh collision was not skipped")
    if bad:
        print("SELFTEST FAILED:")
        print("\n".join(bad))
        return 1
    print(f"selftest ok: {len(cases)} formula cases plus a mesh skip, "
          f"both directions")
    return 0


if __name__ == "__main__":
    sys.exit(selftest() if "--selftest" in sys.argv else main())
