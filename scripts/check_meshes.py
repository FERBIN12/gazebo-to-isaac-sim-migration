#!/usr/bin/env python3
"""a later check: every mesh the URDF references must exist in the imported stage.

WHY THIS IS NOT COVERED BY THE READBACK. `readback_isaac.py` compares
collision geometry, because collision is what the physics acts on. A visual
mesh that fails to resolve changes nothing it looks at. Measured: importing
this robot with an EMPTY ros_package_paths drops the lidar housing entirely,
and the readback still reports the same "1 of 77 values differ" as a correct
import. Both runs exit 0. Nothing anywhere says the mesh is gone.

WHAT AN UNRESOLVED package:// ACTUALLY DOES, measured on Isaac Sim 6.0.1:

    with ros_package_paths      9 layers, geometries.usd present (2422 bytes,
                                holds lidar_housing), instances.usda present
    without                     7 layers, geometries.usd ABSENT,
                                instances.usda ABSENT, import still succeeds

So the failure is structural and silent: two whole layers do not get written.
That is what this checks -- not "did the file parse" but "is the geometry the
URDF asked for actually in the stage".

WHERE MESHES LIVE, which is not where you would look first. The converter does
not inline a mesh into base.usda beside the primitives. It writes a separate
geometry LIBRARY (`payloads/geometries.usd`, binary) and references it through
`payloads/instances.usda`. Grepping base.usda for `def Mesh` finds nothing even
on a correct import, which cost a wrong conclusion before this was written.

Run:
    python3 scripts/check_meshes.py
    python3 scripts/check_meshes.py --selftest

Exit 1 if any referenced mesh is missing from the stage.
"""
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
URDF = BASE / "data/rover.urdf"
STAGE_DIR = BASE / "sim-workspace/isaac/rover.usd/rover"


def urdf_meshes(urdf_path):
    """Every mesh filename the URDF references, with the link and the tag."""
    root = ET.parse(urdf_path).getroot()
    out = []
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for el in link.findall(tag):
                g = el.find("geometry")
                if g is None:
                    continue
                m = g.find("mesh")
                if m is not None and m.get("filename"):
                    out.append({"link": link.get("name"), "tag": tag,
                                "uri": m.get("filename")})
    return out


def stage_meshes(stage_dir):
    """Mesh names present in the stage's geometry library.

    The library is BINARY .usd, so this reads it with `strings` rather than
    parsing USD -- the same reason the rest of these checks avoid the Isaac
    runtime. A name appearing in that binary is the evidence the mesh landed.
    """
    geo = stage_dir / "payloads/geometries.usd"
    inst = stage_dir / "payloads/instances.usda"
    if not geo.exists():
        return None, inst.exists()
    try:
        blob = subprocess.run(["strings", str(geo)], capture_output=True,
                              text=True).stdout
    except FileNotFoundError:
        blob = geo.read_bytes().decode("latin-1")
    return blob, inst.exists()


def mesh_stem(uri):
    """`package://pkg/meshes/lidar_housing.stl` -> `lidar_housing`."""
    return Path(uri.split("://")[-1]).stem


def check(urdf_path, stage_dir):
    refs = urdf_meshes(urdf_path)
    if not refs:
        return refs, []
    blob, has_instances = stage_meshes(stage_dir)
    fails = []
    if blob is None:
        fails.append(
            f"{len(refs)} mesh(es) referenced but payloads/geometries.usd does "
            f"not exist -- the import dropped them silently, and it still "
            f"exited 0")
        return refs, fails
    if not has_instances:
        fails.append("payloads/instances.usda is missing, so nothing "
                     "references the geometry library")
    # WHOLE-TOKEN MATCH, not a substring. `stem in blob` reports a mesh named
    # `thing` as present in a library that only contains `somethingelse` --
    # my own selftest fixture hit exactly that and the failure was real.
    tokens = set(re.findall(r"[A-Za-z0-9_]+", blob))
    for r in refs:
        if mesh_stem(r["uri"]) not in tokens:
            fails.append(f"{r['link']}.{r['tag']}: {r['uri']} -> "
                         f"'{mesh_stem(r['uri'])}' is not in the geometry library")
    return refs, fails


def main():
    if not URDF.exists():
        sys.exit(f"no expanded URDF at {URDF}; run scripts/import_rover.py")
    if not STAGE_DIR.exists():
        sys.exit(f"no imported stage at {STAGE_DIR}; run scripts/import_rover.py")
    refs, fails = check(URDF, STAGE_DIR)
    print(f"MESHES: {len(refs)} referenced by the URDF")
    for r in refs:
        ok = not any(mesh_stem(r["uri"]) in f for f in fails)
        print(f"  {r['link']}.{r['tag']:<9} {'ok  ' if ok else 'MISSING'}  "
              f"{r['uri']}")
    print()
    if not refs:
        print("no meshes referenced; nothing to check")
        return 0
    if fails:
        print(f"{len(fails)} problem(s):")
        for f in fails:
            print("  -", f)
        return 1
    print(f"all {len(refs)} referenced mesh(es) are present in the stage")
    return 0


def selftest():
    """Both directions, on synthetic inputs.

    The real stage is currently correct, so a test that only ran against it
    could not prove the check can fail.
    """
    import tempfile
    bad = []
    urdf = """<robot name="t">
      <link name="a">
        <visual><geometry>
          <mesh filename="package://p/meshes/thing.stl"/>
        </geometry></visual>
      </link>
      <link name="b"><visual><geometry><box size="1 1 1"/></geometry></visual></link>
    </robot>"""
    with tempfile.TemporaryDirectory() as td:
        u = Path(td) / "r.urdf"
        u.write_text(urdf)
        if len(urdf_meshes(u)) != 1:
            bad.append("  a box visual was counted as a mesh, or the mesh was missed")

        # present
        st = Path(td) / "ok/payloads"
        st.mkdir(parents=True)
        (st / "geometries.usd").write_bytes(b"\x00\x00thing\x00\x00Mesh\x00")
        (st / "instances.usda").write_text("#usda 1.0\n")
        _, fails = check(u, st.parent)
        if fails:
            bad.append(f"  a present mesh reported {len(fails)} failure(s)")

        # library exists but does not contain it
        st2 = Path(td) / "wrongname/payloads"
        st2.mkdir(parents=True)
        (st2 / "geometries.usd").write_bytes(b"\x00somethingelse\x00")
        (st2 / "instances.usda").write_text("#usda 1.0\n")
        _, fails = check(u, st2.parent)
        if not fails:
            bad.append("  a geometry library WITHOUT the mesh passed")

        # library absent entirely -- the real failure mode
        st3 = Path(td) / "nolib/payloads"
        st3.mkdir(parents=True)
        _, fails = check(u, st3.parent)
        if not fails:
            bad.append("  a stage with NO geometry library passed")

        # instances missing
        st4 = Path(td) / "noinst/payloads"
        st4.mkdir(parents=True)
        (st4 / "geometries.usd").write_bytes(b"\x00thing\x00")
        _, fails = check(u, st4.parent)
        if not fails:
            bad.append("  a stage with no instances.usda passed")

        # a URDF with no meshes must NOT fail
        u2 = Path(td) / "nomesh.urdf"
        u2.write_text('<robot name="t"><link name="a"><visual><geometry>'
                      '<box size="1 1 1"/></geometry></visual></link></robot>')
        _, fails = check(u2, st3.parent)
        if fails:
            bad.append("  a URDF with no meshes was reported as failing")
    if bad:
        print("SELFTEST FAILED:")
        print("\n".join(bad))
        return 1
    print("selftest ok: 5 stage cases and 1 no-mesh case asserted, both directions")
    return 0


if __name__ == "__main__":
    sys.exit(selftest() if "--selftest" in sys.argv else main())
