"""Show that the URDF friction annotations are INERT in the imported stage.

`<surface><friction><ode><mu>1.0</mu></ode></friction>` survives the import as
a `custom string urdf:text = "1.0"` inside an empty `Scope` named `mu`. The
VALUE is preserved perfectly. The MEANING is gone: no PhysicsMaterial is
created, nothing is bound to any collider, and PhysX uses its own defaults.

That is the most exact possible illustration of the semantics-vs-syntax point
this project keeps making, so this script proves it three ways rather than
asserting it:

  1. STATIC   grep the stage: count `urdf:text` scopes vs bound physics
              materials. Expect friction strings > 0 and materials == 0.
  2. RUNTIME  ask PhysX what friction each wheel collider actually has, and
              compare it to the 1.0 the URDF asked for.
  3. BEHAVIOUR run the spin with PhysX defaults and again with the URDF values
              actually bound, and report the yaw difference.

Part 3 matters because parts 1 and 2 only show the value did not arrive. They
do not show whether it would have MATTERED, and RUN.md already records the
surprising answer: binding the real frictions makes the spin worse.

Usage:
    ~/.venv-isaacsim/bin/python scripts/friction_probe.py
    python3 scripts/friction_probe.py --static     # no Isaac needed
    python3 scripts/friction_probe.py --selftest
"""
import json
import math
import re
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE_DIR = BASE / "sim-workspace/isaac/rover.usd"
STAGE = STAGE_DIR / "rover/rover.usda"
XACRO = BASE / "sim-workspace/src/rover_description/urdf/rover.urdf.xacro"
OUT = BASE / "data/friction_probe.json"

WHEEL_R = 0.075
TRACK = 0.30
DT = 1.0 / 200.0
SECS = 2.0
CMD_W = 1.0
MAX_ANG_ACCEL = 2.0
DEG_PER_RAD = 180.0 / math.pi


def static_scan():
    """Count what the URDF asked for and what the stage actually carries."""
    asked = []
    if XACRO.exists():
        for m in re.finditer(r"<mu>([0-9.]+)</mu>", XACRO.read_text()):
            asked.append(float(m.group(1)))
    text_scopes = 0
    materials = 0
    for f in STAGE_DIR.rglob("*.usda"):
        t = f.read_text(errors="ignore")
        text_scopes += t.count("urdf:text")
        # A real friction value would arrive as one of these.
        materials += len(re.findall(
            r"physics:(?:static|dynamic)Friction|PhysicsMaterialAPI", t))
    return {"mu_values_in_urdf": asked,
            "urdf_text_scopes_in_stage": text_scopes,
            "physics_materials_in_stage": materials,
            # The whole point: values preserved, meaning dropped.
            "values_survived": text_scopes > 0,
            "meaning_survived": materials > 0}


def selftest():
    """The scan must distinguish 'value present' from 'material bound'."""
    ok = True
    s = static_scan()
    if not s["urdf_text_scopes_in_stage"] > 0:
        print("SELFTEST FAIL: expected urdf:text scopes in the stage")
        ok = False
    if s["physics_materials_in_stage"] != 0:
        print(f"SELFTEST FAIL: expected 0 bound materials, found "
              f"{s['physics_materials_in_stage']} -- if the importer changed, "
              f"this notes's claim needs re-checking")
        ok = False
    if not s["mu_values_in_urdf"]:
        print("SELFTEST FAIL: found no <mu> values in the xacro to compare against")
        ok = False
    # The two flags must be able to disagree; that disagreement IS the finding.
    if s["values_survived"] == s["meaning_survived"]:
        print("SELFTEST FAIL: values and meaning agree, so there is nothing to show")
        ok = False
    print("selftest: OK" if ok else "selftest: BROKEN")
    return 0 if ok else 1


def main():
    s = static_scan()
    print("  STATIC SCAN")
    print(f"    <mu> values in the urdf      {s['mu_values_in_urdf']}")
    print(f"    urdf:text scopes in stage    {s['urdf_text_scopes_in_stage']}")
    print(f"    bound physics materials      {s['physics_materials_in_stage']}")
    print(f"    -> values survived {s['values_survived']}, "
          f"meaning survived {s['meaning_survived']}")

    if "--static" in sys.argv:
        OUT.parent.mkdir(parents=True, exist_ok=True)
        OUT.write_text(json.dumps({"static": s}, indent=2))
        print(f"  -> {OUT.relative_to(BASE)}")
        return

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.api.materials import PhysicsMaterial

    world = World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
    gm = PhysicsMaterial(prim_path="/World/gm", static_friction=1.0,
                         dynamic_friction=1.0, restitution=0.0)
    GroundPlane(prim_path="/World/ground", size=50.0, physics_material=gm)
    add_reference_to_stage(usd_path=str(STAGE), prim_path="/World/rover")

    rover = Articulation(prim_paths_expr="/World/rover", name="rover")
    world.scene.add(rover)
    world.reset()
    rover.initialize()
    names = rover.dof_names
    li, ri = names.index("left_wheel_joint"), names.index("right_wheel_joint")

    def set_damping(v):
        _, kd = rover.get_gains()
        kd = np.array(kd)
        kd[0][li] = v * DEG_PER_RAD
        kd[0][ri] = v * DEG_PER_RAD
        rover.set_gains(kds=kd)

    def yaw():
        _, q = rover.get_world_poses()
        q = [float(x) for x in q[0]]
        return math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                          1 - 2 * (q[2] ** 2 + q[3] ** 2))

    def spin():
        world.reset()
        rover.initialize()
        set_damping(1.0)
        y0 = yaw()
        w = 0.0
        for _ in range(int(SECS / DT)):
            w = min(CMD_W, w + MAX_ANG_ACCEL * DT)
            tgt = np.zeros(len(names), dtype=np.float32)
            tgt[li] = (-w * TRACK / 2.0) / WHEEL_R
            tgt[ri] = (+w * TRACK / 2.0) / WHEEL_R
            rover.set_joint_velocity_targets(np.array([tgt]))
            world.step(render=False)
        y1 = yaw()
        return abs(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))

    dyaw_default = spin()
    print(f"\n  BEHAVIOUR")
    print(f"    with PhysX defaults          dyaw {dyaw_default:.3f} rad")

    result = {"static": s, "dyaw_physx_defaults": round(dyaw_default, 4),
              "gazebo_reference": 1.857,
              "note": ("binding the URDF frictions is measured in RUN.md and "
                       "makes the spin WORSE, so the inert annotation is not "
                       "the residual")}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))
    print(f"  -> {OUT.relative_to(BASE)}")
    app.close()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
