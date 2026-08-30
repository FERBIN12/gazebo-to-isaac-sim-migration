#!/usr/bin/env python3
"""a later check: read the IMPORTED stage back, and diff it against the URDF.

This is the measurement notes 3.2 is built on, and it exists because the
alternative is looking at the viewport and saying it looks right. The defect
this catches is invisible in the viewport.

WHAT IT COMPARES. Five fields per link, taken from the URDF on one side and
from the imported USD on the other:

    mass                  kg
    inertia               the diagonal, kg m^2
    origin                the link's translation from its parent, m
    collision shape TYPE  sphere / cylinder / box / mesh
    collision dimensions  radius, length, extents, m

TYPES AS WELL AS VALUES, DELIBERATELY. A substitution preserves measurements
and changes behaviour: a convex hull that approximates a sphere has the right
radius by construction. Checking only numbers passes it. That is the general
principle this script demonstrates, and it is why the type column exists.

TOLERANCES, DECLARED BEFORE THE MEASUREMENT (G24). These are exact-transfer
fields, not simulation results -- the importer either carried the number across
or it did not -- so the tolerance is float32 storage noise, not physics. USD
stores mass and inertia as `float`, so a URDF double like 5.4e-5 comes back as
the nearest float32; 1e-6 RELATIVE covers that and nothing larger.

    mass, inertia, dimensions   1e-6 relative
    origin                      1e-6 absolute, m
    collision type              exact string match

Run (plain python3 -- reads USDA as text, so it needs no Isaac runtime):
    python3 scripts/readback_isaac.py

Writes data/readback.json, prints the diff, exits 1 if anything differs.
"""
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
URDF = BASE / "data/rover.urdf"
STAGE_DIR = BASE / "sim-workspace/isaac/rover.usd/rover"
GEOM = STAGE_DIR / "payloads/base.usda"
PHYS = STAGE_DIR / "payloads/Physics/physics.usda"
OUT = BASE / "data/readback.json"

REL_TOL = 1e-6
ABS_TOL = 1e-6
FIELDS = ["mass", "inertia", "origin", "collision_type", "collision_dims"]


def _rpy_to_link_frame(diag, rpy):
    """Rotate an inertia diagonal stated under <origin rpy> into the link frame.

    URDF rpy is fixed-axis XYZ (roll about x, then pitch about y, then yaw
    about z), i.e. R = Rz(yaw) @ Ry(pitch) @ Rx(roll). Returns the diagonal of
    R * diag * R^T, which is what the link frame sees.
    """
    import math
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    Ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    Rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]

    def mul(A, B):
        return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
                for i in range(3)]

    R = mul(Rz, mul(Ry, Rx))
    return [round(sum(R[i][k] * R[i][k] * diag[k] for k in range(3)), 12)
            for i in range(3)]


# ------------------------------------------------------------------ URDF side
def urdf_side():
    root = ET.parse(URDF).getroot()
    joint_of = {}
    for j in root.findall("joint"):
        child = j.find("child").get("link")
        origin = j.find("origin")
        xyz = [0.0, 0.0, 0.0]
        if origin is not None and origin.get("xyz"):
            xyz = [float(v) for v in origin.get("xyz").split()]
        joint_of[child] = {"type": j.get("type"), "origin": xyz}

    links = {}
    for l in root.findall("link"):
        name = l.get("name")
        rec = dict.fromkeys(FIELDS)
        inert = l.find("inertial")
        if inert is not None:
            rec["mass"] = float(inert.find("mass").get("value"))
            i = inert.find("inertia")
            diag = [float(i.get(k)) for k in ("ixx", "iyy", "izz")]
            # <inertial> CARRIES ITS OWN <origin rpy>, AND IT CHANGES THE FRAME.
            #
            # The wheels state ixx=iyy=0.000769, izz=0.001406 -- but under
            # `<origin rpy="1.5707963 0 0"/>`, so that tensor is expressed in a
            # frame rotated 90 deg about X, matching the cylinder's own axis.
            # Ignoring the rpy and comparing the raw numbers reported both
            # wheels as having a swapped inertia axis, which is a reader bug
            # that looks exactly like an importer bug: same three values, wrong
            # places. Isaac folds this rpy into physics:principalAxes, which is
            # why it has to be folded in here too, or the two sides are being
            # read in different frames.
            o = inert.find("origin")
            rpy = [0.0, 0.0, 0.0]
            if o is not None and o.get("rpy"):
                rpy = [float(v) for v in o.get("rpy").split()]
            rec["inertia"] = _rpy_to_link_frame(diag, rpy)
            rec["inertia_stated"] = diag
            rec["inertia_rpy"] = rpy
        col = l.find("collision")
        if col is not None:
            g = col.find("geometry")
            for kind in ("sphere", "cylinder", "box", "mesh"):
                e = g.find(kind)
                if e is None:
                    continue
                rec["collision_type"] = kind
                if kind == "sphere":
                    rec["collision_dims"] = {"radius": float(e.get("radius"))}
                elif kind == "cylinder":
                    rec["collision_dims"] = {"radius": float(e.get("radius")),
                                             "length": float(e.get("length"))}
                elif kind == "box":
                    rec["collision_dims"] = {
                        "size": [float(v) for v in e.get("size").split()]}
                else:
                    rec["collision_dims"] = {"filename": e.get("filename")}
                break
        if name in joint_of:
            rec["origin"] = joint_of[name]["origin"]
        links[name] = rec
    return links


# ------------------------------------------------------------------ USD side
#
# READ AS TEXT, ON PURPOSE. The alternative is opening the stage through pxr
# inside the Isaac runtime, which costs ~15 s of Kit startup for what is a
# parse. The importer writes .usda (ASCII). If a future version writes binary
# .usdc this fails loudly on a missing file rather than reporting zeros.
def _blocks(text):
    """(indent, kw, type, name, meta, body) for every def/over block.

    META IS SEPARATE FROM BODY, and that distinction is the point. A prim's
    apiSchemas -- which is what says whether a shape is the COLLISION shape or
    the visual twin beside it -- lives in a parenthesised metadata block BEFORE
    the brace body:

        def Sphere "sphere_1" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        { ... }

    A first version of this reader captured only the brace body, so every
    collision prim read as type None and the diff reported all nine links as
    differing in collision_type. That looked like a catastrophic importer bug
    and was a bug in the reader. When a measurement says everything is broken,
    suspect the measurement.
    """
    lines = text.splitlines()
    out = []
    for i, ln in enumerate(lines):
        m = re.match(r'^(\s*)(def|over)\s+(?:(\w+)\s+)?"([^"]+)"', ln)
        if not m:
            continue
        indent, kw, typ, name = m.group(1), m.group(2), m.group(3), m.group(4)
        meta, j = "", i
        if "(" in ln and ")" not in ln[ln.index("("):]:
            pdepth = 0
            for j in range(i, len(lines)):
                pdepth += lines[j].count("(") - lines[j].count(")")
                if pdepth == 0:
                    break
            meta = "\n".join(lines[i:j + 1])
        depth, start = 0, None
        for k in range(j, len(lines)):
            depth += lines[k].count("{") - lines[k].count("}")
            if start is None and "{" in lines[k]:
                start = k
            if start is not None and depth == 0:
                out.append((len(indent), kw, typ, name, meta,
                            "\n".join(lines[start:k + 1])))
                break
    return out


def _f3(body, attr):
    m = re.search(re.escape(attr) + r'\s*=\s*\(([^)]*)\)', body)
    return [float(v) for v in m.group(1).split(",")] if m else None


def _f1(body, attr):
    m = re.search(re.escape(attr) + r'\s*=\s*(-?[\d.eE+-]+)', body)
    return float(m.group(1)) if m else None


def _own_attr1(body, block_indent, attr):
    """A scalar attribute belonging to THIS block, not to a nested child."""
    want = " " * (block_indent + 4) + attr
    for ln in body.splitlines():
        if ln.startswith(want):
            m = re.search(r'=\s*(-?[\d.eE+-]+)', ln)
            if m:
                return float(m.group(1))
    return None


def _own_attr3(body, block_indent, attr):
    """A float3 attribute belonging to THIS block, not to a nested child.

    USD nests links inside their parents, so a naive search of a link's body
    finds the first descendant that happens to declare the attribute. Direct
    attributes of a block sit at block_indent + 4.
    """
    want = " " * (block_indent + 4) + attr
    for ln in body.splitlines():
        if ln.startswith(want):
            m = re.search(r'\(([^)]*)\)', ln)
            if m:
                return [float(v) for v in m.group(1).split(",")]
    return None


def _own_attr4(body, block_indent, attr):
    """A quaternion attribute belonging to THIS block. (w, x, y, z)."""
    want = " " * (block_indent + 4) + attr
    for ln in body.splitlines():
        if ln.startswith(want):
            m = re.search(r'\(([^)]*)\)', ln)
            if m:
                return [float(v) for v in m.group(1).split(",")]
    return None


def _to_link_frame(diag, quat):
    """Rotate a principal-frame inertia diagonal back into the link frame.

    I_link = R * diag(I_principal) * R^T, with R from the principalAxes
    quaternion. Returns the diagonal of that, which is what a URDF states.

    Pure python: this script deliberately runs on plain python3 with no numpy,
    so it can gate in the same place the other checks do.
    """
    if diag is None:
        return None
    if quat is None:
        return list(diag)
    w, x, y, z = quat
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    # diagonal of R * diag(d) * R^T is sum_k R[i][k]^2 * d[k]
    return [round(sum(R[i][k] * R[i][k] * diag[k] for k in range(3)), 12)
            for i in range(3)]


SHAPE = {"Sphere": "sphere", "Cylinder": "cylinder", "Cube": "box",
         "Mesh": "mesh"}


def usd_side(urdf_links):
    geom_blocks = _blocks(GEOM.read_text())
    phys_blocks = _blocks(PHYS.read_text())

    links = {}
    for name in urdf_links:
        rec = dict.fromkeys(FIELDS)

        gb = next((b for b in geom_blocks
                   if b[3] == name and b[1] == "def" and b[2] == "Xform"), None)
        if gb:
            gindent, body = gb[0], gb[5]
            # THIS LINK'S OWN translate, not a descendant's. base_footprint
            # declares no transform at all, so a plain search over its body
            # returned base_link's (0, 0, 0.075) and the diff called it an
            # origin mismatch on a link that has no origin. Only accept an
            # attribute at the block's own indent + 4.
            rec["origin"] = _own_attr3(body, gindent, "double3 xformOp:translate")
            # DIRECT CHILDREN ONLY. base_link nests left_wheel, right_wheel,
            # caster and mast inside it, so a search over the whole subtree
            # picks up a descendant's collision prim and files it under the
            # parent. A direct child's block indent is the link's indent + 4.
            for (ind, _kw, typ, _nm, imeta, ibody) in _blocks(body):
                if ind != gindent + 4:
                    continue
                if "PhysicsCollisionAPI" not in imeta or typ not in SHAPE:
                    continue
                rec["collision_type"] = SHAPE[typ]
                if typ == "Sphere":
                    rec["collision_dims"] = {
                        "radius": _f1(ibody, "double radius")}
                elif typ == "Cylinder":
                    rec["collision_dims"] = {
                        "radius": _f1(ibody, "double radius"),
                        "length": _f1(ibody, "double height")}
                elif typ == "Cube":
                    # A USD Cube HAS ONE `size`, so a non-cubic box is a unit
                    # cube with a non-uniform xformOp:scale. Reading `extent`
                    # alone reports (1, 1, 1) for a 0.4 x 0.28 x 0.1 chassis --
                    # a 10x error that looks like a catastrophic importer bug
                    # and is the reader ignoring the transform. Multiply.
                    side = _f1(ibody, "double size")
                    scale = _f3(ibody, "float3 xformOp:scale") or [1.0, 1.0, 1.0]
                    if side is not None:
                        rec["collision_dims"] = {
                            "size": [round(side * s, 12) for s in scale]}
                break

        # Physics layer: mass and inertia, and again OWN attributes only.
        # base_footprint is a bare structural `over` with no mass of its own,
        # and a whole-body search returned base_link's 6 kg and its inertia --
        # so a massless frame reported as the heaviest link on the robot.
        # Requiring "the block carries a mass" is not enough on its own,
        # because a nested child's mass satisfies it.
        pb = next((b for b in phys_blocks
                   if b[3] == name
                   and _own_attr1(b[5], b[0], "float physics:mass") is not None),
                  None)
        if pb:
            rec["mass"] = _own_attr1(pb[5], pb[0], "float physics:mass")
            diag = _own_attr3(pb[5], pb[0], "float3 physics:diagonalInertia")
            axes = _own_attr4(pb[5], pb[0], "quatf physics:principalAxes")
            # COMPARE IN THE SAME FRAME, OR THE COMPARISON IS MEANINGLESS.
            #
            # URDF states the inertia tensor in the LINK frame, with off-diagonal
            # terms allowed. PhysX stores a DIAGONAL plus the rotation that
            # diagonalised it (physics:principalAxes). Those are two
            # representations of the same tensor, so the raw diagonals do not
            # have to match and usually do not.
            #
            # Comparing them directly reported camera_link as a defect when the
            # importer was exactly right: rotating its stored diagonal back into
            # the link frame reproduces the URDF to 2e-12. Rotate first, then
            # diff. What survives that is a genuine disagreement.
            rec["inertia"] = _to_link_frame(diag, axes)
            rec["inertia_principal"] = diag
            rec["principal_axes"] = axes

        links[name] = rec
    return links


# ----------------------------------------------------------------- the joints
#
# A SEPARATE PASS, BECAUSE JOINTS ARE WHERE THE UNITS CHANGE.
#
# The link fields above are all unit-preserving: metres stay metres, kilograms
# stay kilograms. The joint limits are not. `physxJoint:maxJointVelocity` on a
# revolute joint is DEGREES per second, and the importer converts:
#
#   urdf_to_mjc_physx_conversion_utils.py:98
#       joint_max_velocity_deg = joint_max_velocity * 180 / 3.1415926
#
# so the URDF's 8 rad/s is stored as 458.36624. Read raw, that looks like a
# limit 57x too permissive, which is exactly the kind of number that gets
# called an importer bug and is not one. This pass converts back before
# comparing, and prints both, so the notes can show the raw value and the
# reason it is not what it looks like.
DEG = 180.0 / 3.1415926  # the importer's own constant, not math.pi


def joints_side():
    root = ET.parse(URDF).getroot()
    urdf_j = {}
    for j in root.findall("joint"):
        lim = j.find("limit")
        dyn = j.find("dynamics")
        urdf_j[j.get("name")] = {
            "type": j.get("type"),
            "effort": float(lim.get("effort")) if lim is not None
            and lim.get("effort") else None,
            "velocity": float(lim.get("velocity")) if lim is not None
            and lim.get("velocity") else None,
            "damping": float(dyn.get("damping")) if dyn is not None
            and dyn.get("damping") else None,
        }

    # WHICH LINKS ARE MASSLESS FRAMES. A fixed joint whose child has no
    # <inertial> gets NO joint prim in the stage, and that is correct rather
    # than missing: PhysX constrains rigid bodies, and a pure frame is not one,
    # so the importer bakes the transform into the Xform hierarchy instead.
    # base_footprint and camera_optical_frame are both of these, and reporting
    # them as absent joints was two false failures out of three.
    massless = {l.get("name") for l in root.findall("link")
                if l.find("inertial") is None}
    # EITHER SIDE, not just the child. base_joint runs base_footprint ->
    # base_link, so the massless frame is its PARENT; a child-only rule left it
    # reported as a missing joint. The rule is "one end is not a rigid body",
    # which is what makes the constraint meaningless to PhysX.
    frame_only = set()
    for j in root.findall("joint"):
        if j.get("type") != "fixed":
            continue
        ends = {j.find("parent").get("link"), j.find("child").get("link")}
        if ends & massless:
            frame_only.add(j.get("name"))

    phys_blocks = _blocks(PHYS.read_text())
    physx_blocks = _blocks((STAGE_DIR / "payloads/Physics/physx.usda").read_text())
    isaac_j = {}
    for name, want in urdf_j.items():
        if name in frame_only:
            # Represented as a transform, not a joint prim. Do NOT just copy
            # the URDF values across -- that passes without checking anything.
            # Assert the child Xform actually EXISTS in the geometry layer, so
            # a frame the importer genuinely dropped still fails.
            jel = next(j for j in root.findall("joint")
                       if j.get("name") == name)
            ends = [jel.find("parent").get("link"),
                    jel.find("child").get("link")]
            child = next(e for e in ends if e in massless)
            present = any(b[3] == child and b[2] == "Xform"
                          for b in _blocks(GEOM.read_text()))
            isaac_j[name] = dict(
                want if present else dict.fromkeys(want),
                velocity_raw_deg=None,
                note=(f"frame only: {child} carries the transform, no joint "
                      f"prim expected") if present
                else f"frame only, but {child} Xform is MISSING from the stage")
            continue
        b = next((x for x in phys_blocks if x[3] == name), None)
        px = next((x for x in physx_blocks if x[3] == name), None)
        rec = {"type": None, "effort": None, "velocity": None,
               "velocity_raw_deg": None, "damping": None}
        if b:
            body, ind = b[5], b[0]
            kind = b[2]
            rec["type"] = {"PhysicsRevoluteJoint": "continuous",
                           "PhysicsFixedJoint": "fixed"}.get(kind, kind)
            rec["effort"] = _own_attr1(body, ind,
                                       "float drive:angular:physics:maxForce")
            rec["damping"] = _own_attr1(body, ind,
                                        "float drive:angular:physics:damping")
        if px:
            raw = _own_attr1(px[5], px[0], "float physxJoint:maxJointVelocity")
            rec["velocity_raw_deg"] = raw
            rec["velocity"] = round(raw / DEG, 6) if raw is not None else None
        isaac_j[name] = rec
    return urdf_j, isaac_j


# ------------------------------------------------------------------ the diff
def close(a, b):
    if a is None and b is None:
        return True
    if a is None or b is None:
        return False
    if isinstance(a, bool) or isinstance(b, bool):
        return a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b) <= max(ABS_TOL, REL_TOL * max(abs(a), abs(b)))
    if isinstance(a, list) and isinstance(b, list):
        return len(a) == len(b) and all(close(x, y) for x, y in zip(a, b))
    if isinstance(a, dict) and isinstance(b, dict):
        return set(a) == set(b) and all(close(a[k], b[k]) for k in a)
    return a == b


def main():
    if not URDF.exists():
        sys.exit(f"no expanded URDF at {URDF}; run scripts/import_rover.py")
    if not GEOM.exists():
        sys.exit(f"no imported stage at {GEOM}; run scripts/import_rover.py")

    u = urdf_side()
    i = usd_side(u)

    rows, diffs = [], []
    for name in u:
        for f in FIELDS:
            a, b = u[name][f], i[name][f]
            ok = close(a, b)
            rows.append({"link": name, "field": f, "urdf": a, "isaac": b,
                         "same": ok})
            if not ok:
                diffs.append((name, f, a, b))

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps({"rows": rows, "n_compared": len(rows),
                               "n_diff": len(diffs)}, indent=2))

    print(f"READBACK: {len(u)} links x {len(FIELDS)} fields "
          f"= {len(rows)} values compared")
    print(f"tolerances: rel {REL_TOL}, abs {ABS_TOL}, types exact\n")
    w = max(len(n) for n in u)
    for name in u:
        bad = [f for f in FIELDS if not close(u[name][f], i[name][f])]
        print(f"  {name:<{w}}  {'DIFF' if bad else 'ok':<4} "
              f"{('(' + ', '.join(bad) + ')') if bad else ''}")
    # ---- joints, compared in matched units
    uj, ij = joints_side()
    jfields = ["type", "effort", "velocity", "damping"]
    jrows, jdiffs = [], []
    for name in uj:
        for f in jfields:
            a, b = uj[name][f], ij[name][f]
            # A URDF `continuous` joint and a `revolute` one are both a
            # PhysicsRevoluteJoint in USD -- the difference is the absence of
            # limits, not a different prim type. Treat both as matching.
            if f == "type" and a in ("continuous", "revolute") \
                    and b == "continuous":
                ok = True
            else:
                ok = close(a, b)
            jrows.append({"joint": name, "field": f, "urdf": a, "isaac": b,
                          "same": ok})
            if not ok:
                jdiffs.append((name, f, a, b))

    print(f"JOINTS: {len(uj)} joints x {len(jfields)} fields "
          f"= {len(jrows)} values compared")
    jw = max(len(n) for n in uj)
    for name in uj:
        bad = [r["field"] for r in jrows
               if r["joint"] == name and not r["same"]]
        print(f"  {name:<{jw}}  {'DIFF' if bad else 'ok':<4} "
              f"{('(' + ', '.join(bad) + ')') if bad else ''}")
    # The unit story, printed whether or not it differs: this is the number
    # that looks wrong and is not.
    for name in uj:
        raw = ij[name]["velocity_raw_deg"]
        if raw is not None:
            print(f"    {name}: urdf velocity {uj[name]['velocity']} rad/s "
                  f"-> stage physxJoint:maxJointVelocity {raw} deg/s "
                  f"= {ij[name]['velocity']} rad/s")
    print()

    OUT.write_text(json.dumps(
        {"rows": rows, "n_compared": len(rows), "n_diff": len(diffs),
         "joint_rows": jrows, "n_joint_compared": len(jrows),
         "n_joint_diff": len(jdiffs)}, indent=2))

    total, total_diff = len(rows) + len(jrows), len(diffs) + len(jdiffs)
    if not total_diff:
        print(f"NO DIFFERENCES: all {total} values agree.")
        return 0
    print(f"{total_diff} of {total} values differ:\n")
    for name, f, a, b in diffs:
        print(f"  {name}.{f}\n      urdf  = {a}\n      isaac = {b}")
    for name, f, a, b in jdiffs:
        print(f"  {name}.{f}\n      urdf  = {a}\n      isaac = {b}")
    return 1


def selftest():
    """Prove the comparison catches what it claims to, BOTH directions.

    Built from synthetic values rather than pinned to a real notes's file:
    a fixture pinned to a real artefact goes stale the moment that artefact is
    fixed, and then prints BROKEN forever while the gate is fine.

    Each case asserts a verdict, so a change that makes the checker blind fails
    here instead of quietly passing everything.
    """
    cases = [
        # (a, b, expect_same, what it guards)
        (6.0, 6.0, True, "identical scalars"),
        (6.0, 6.5, False, "a mass that moved"),
        (6.0, 6.0 + 1e-9, True, "float32 storage noise is not a defect"),
        ([1.0, 2.0, 3.0], [1.0, 2.0, 3.0], True, "identical vectors"),
        # THE ONE THAT MATTERS: a permutation. Same three values, wrong axes.
        ([8.5e-4, 8.5e-4, 6e-5], [8.5e-4, 6e-5, 8.5e-4], False,
         "a permuted inertia diagonal (the mast defect)"),
        ("sphere", "mesh", False, "a substituted collision type"),
        ("sphere", "sphere", True, "an unchanged collision type"),
        ({"radius": 0.03}, {"radius": 0.03}, True, "identical dims"),
        ({"radius": 0.03}, {"radius": 0.031}, False, "a resized shape"),
        (None, 6.0, False, "a value appearing where the URDF had none"),
        (None, None, True, "absent on both sides"),
    ]
    bad = []
    for a, b, expect, what in cases:
        got = close(a, b)
        if got != expect:
            bad.append(f"  close({a!r}, {b!r}) = {got}, want {expect}  [{what}]")
    # And the frame maths: a 90 deg rotation about X must swap the y/z entries.
    got = _rpy_to_link_frame([1.0, 2.0, 3.0], [3.14159265358979 / 2, 0, 0])
    if not (abs(got[0] - 1.0) < 1e-9 and abs(got[1] - 3.0) < 1e-9
            and abs(got[2] - 2.0) < 1e-9):
        bad.append(f"  _rpy_to_link_frame 90deg about x gave {got}, "
                   f"want [1, 3, 2]")
    # A permutation must NOT be reported as equal once rotated into one frame.
    if close([8.5e-4, 8.5e-4, 6e-5], sorted([8.5e-4, 6e-5, 8.5e-4])):
        bad.append("  sorted-magnitude comparison passes a permutation")
    if bad:
        print("SELFTEST FAILED:")
        print("\n".join(bad))
        return 1
    print(f"selftest ok: {len(cases)} comparison cases + frame maths")
    return 0


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    sys.exit(main())
