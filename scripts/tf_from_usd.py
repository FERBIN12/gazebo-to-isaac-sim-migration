"""Derive the frame tree the USD stage implies, and diff it against the TF tree
Gazebo actually publishes.

Why this is not a like-for-like comparison, and why saying so is the notes:
Gazebo's TF tree comes from `robot_state_publisher`, which reads the URDF and
publishes a transform per joint every tick. Isaac has no robot_state_publisher.
The stage carries the same information as an Xform HIERARCHY plus joint prims,
and getting it onto /tf requires an ActionGraph you build. So the question is
not "do the two trees match" but "does the stage CONTAIN the tree".

Two different structures in the stage encode it, and they do not agree in count:
  * the Xform nesting     -- parent/child by containment
  * the joint prims       -- body0/body1 pairs in the physics layer
Notes 4.1 measured that 2 of the 8 URDF joints get NO prim (a fixed joint to a
massless frame is baked into the hierarchy instead), so the joint-prim count is
expected to be short. The hierarchy is the authoritative one.

Usage:
    scripts/tf_from_usd.py
    python3 scripts/tf_from_usd.py --selftest
"""
import json
import re
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover"
OUT = BASE / "data/tf_from_usd.json"


def hierarchy_edges(text, xform_only=True):
    """parent->child edges from prim nesting, by tracking brace depth.

    XFORM ONLY, BY DEFAULT, and that distinction is load-bearing. Including
    `Scope` prims produced 17 edges against Gazebo's 8, and the 9 extras were
    not frames at all: 5 were the imported SDF friction tags
    (`surface -> friction -> ode -> mu/mu2`, all Scope), `Geometry` is a Scope
    container, and the rest were visual meshes. A Scope carries no transform, so
    it can never be a TF frame. Verified by grepping the `def` type of each
    extra rather than assuming.

    `lidar_housing` IS an Xform and is a real child of `lidar_link` -- a visual
    mesh with its own transform. It is legitimately in the hierarchy and
    legitimately NOT in /tf, because robot_state_publisher publishes one frame
    per URDF LINK and a visual is not a link.
    """
    edges = []
    stack = []          # (name, depth_at_open)
    depth = 0
    pending = None
    pat = r'def Xform "([^"]+)"' if xform_only else r'def (?:Xform|Scope) "([^"]+)"'
    for line in text.splitlines():
        m = re.search(pat, line)
        if m:
            pending = m.group(1)
        opens = line.count("{")
        closes = line.count("}")
        for _ in range(opens):
            depth += 1
            if pending:
                if stack:
                    edges.append((stack[-1][0], pending))
                stack.append((pending, depth))
                pending = None
        for _ in range(closes):
            while stack and stack[-1][1] >= depth:
                stack.pop()
            depth -= 1
    return edges


def joint_edges(text):
    """body0->body1 pairs from joint prims in the physics layer."""
    edges = []
    for blk in re.split(r'def Physics(?:Fixed|Revolute|Prismatic)Joint ', text)[1:]:
        b0 = re.search(r'body0 = \[?<([^>]+)>', blk)
        b1 = re.search(r'body1 = \[?<([^>]+)>', blk)
        if b0 and b1:
            edges.append((b0.group(1).rsplit("/", 1)[-1],
                          b1.group(1).rsplit("/", 1)[-1]))
    return edges


def selftest():
    ok = True
    # hierarchy: a nested pair must produce exactly one edge, in the right order
    t = '''def Xform "a"
{
    def Xform "b"
    {
    }
}
'''
    e = hierarchy_edges(t)
    if e != [("a", "b")]:
        print(f"  FAIL simple nesting gave {e}"); ok = False
    # two children of one parent = two edges, and NOT an edge between siblings
    t2 = '''def Xform "a"
{
    def Xform "b"
    {
    }
    def Xform "c"
    {
    }
}
'''
    e2 = sorted(hierarchy_edges(t2))
    if e2 != [("a", "b"), ("a", "c")]:
        print(f"  FAIL two children gave {e2}"); ok = False
    # a flat file with no nesting must produce NO edges
    t3 = 'def Xform "a"\n{\n}\ndef Xform "b"\n{\n}\n'
    if hierarchy_edges(t3):
        print(f"  FAIL flat file gave {hierarchy_edges(t3)}"); ok = False
    # A SCOPE IS NOT A FRAME. The assertion the real bug needed: with Scopes
    # included the stage read 17 edges against Gazebo's 8, and 5 of the extras
    # were the imported SDF friction tags. Both directions.
    ts = ('def Xform "link"\n{\n'
          '    def Scope "surface"\n    {\n'
          '        def Scope "friction"\n        {\n        }\n    }\n}\n')
    if hierarchy_edges(ts) != []:
        print(f"  FAIL Scope leaked into frames: {hierarchy_edges(ts)}"); ok = False
    if len(hierarchy_edges(ts, xform_only=False)) != 2:
        print("  FAIL xform_only=False lost the Scopes: "
              f"{hierarchy_edges(ts, xform_only=False)}"); ok = False
    # joints: reads body0/body1 and strips the path
    tj = '''def PhysicsFixedJoint "j"
{
    rel physics:body0 = </rover/Geometry/mast>
    rel physics:body1 = </rover/Geometry/mast/lidar_link>
}
'''
    ej = joint_edges(tj)
    if ej != [("mast", "lidar_link")]:
        print(f"  FAIL joint parse gave {ej}"); ok = False
    # and a file with no joints gives none
    if joint_edges('def Xform "a"\n{\n}\n'):
        print("  FAIL invented a joint edge"); ok = False
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()

    base = (STAGE / "payloads/base.usda")
    phys = (STAGE / "payloads/Physics/physics.usda")
    if not base.exists():
        print(f"no stage at {base}")
        return 1

    h = hierarchy_edges(base.read_text())
    j = joint_edges(phys.read_text()) if phys.exists() else []

    # what Gazebo actually publishes, from the 7.1 audit
    gz = []
    aud = BASE / "data/interface_audit.json"
    if aud.exists():
        gz = [tuple(e) for e in json.loads(aud.read_text())["tf_edges"]]

    # compare only the ROBOT links: the stage has no odom frame, because odom is
    # produced by the controller at runtime, not by the description.
    hset = set(h)
    gzset = set(gz)
    gz_robot = {(p, c) for p, c in gzset if p != "odom"}
    common = hset & gz_robot
    only_gz = sorted(gz_robot - hset)
    only_usd = sorted(hset - gz_robot)

    res = {"usd_hierarchy_edges": sorted(map(list, hset)),
           "usd_joint_edges": sorted(map(list, set(j))),
           "gazebo_tf_edges": sorted(map(list, gzset)),
           "gazebo_robot_edges": sorted(map(list, gz_robot)),
           "in_both": sorted(map(list, common)),
           "only_in_gazebo": sorted(map(list, only_gz)),
           "only_in_usd": sorted(map(list, only_usd))}
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))

    print(f"USD Xform hierarchy edges   {len(hset)}")
    for p, c in sorted(hset):
        print(f"    {p} -> {c}")
    print(f"USD joint prims             {len(set(j))}")
    for p, c in sorted(set(j)):
        print(f"    {p} -> {c}")
    print(f"Gazebo /tf edges            {len(gzset)}  ({len(gz_robot)} robot-internal)")
    print(f"in BOTH                     {len(common)}")
    print(f"only in Gazebo /tf          {only_gz or 'none'}")
    print(f"only in the USD hierarchy   {only_usd or 'none'}")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
