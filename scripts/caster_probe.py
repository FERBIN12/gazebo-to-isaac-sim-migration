"""Does the caster chatter? Measure its contact behaviour rather than assume it.

The curriculum row for 5.5 promises "the caster that chattered". Nothing in
this repository has measured that, and a later check exists because a curriculum
row turned out to be a prediction rather than a finding. So this measures.

Chatter is a contact that repeatedly makes and breaks: the body bounces at the
contact rather than resting on it. The observable is the caster link's HEIGHT
oscillating while the robot drives, so this records base and caster height every
physics step and reuses the same `analyse()` the vibration probe uses, which
already knows how to tell a ramp from an oscillation.

Also records restitution, because that is 5.5's other subject: neither the URDF
nor the world file declares any (grep says 0), so whatever PhysX and DART each
default to is what the robot actually has.

Usage:
    ~/.venv-isaacsim/bin/python scripts/caster_probe.py
    python3 scripts/caster_probe.py --selftest
"""
import importlib.util
import json
import math
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/caster_probe.json"

WHEEL_R = 0.075
TRACK = 0.30
DT = 1.0 / 200.0
SECS = 4.0
SETTLE_S = 1.5
CMD_V = 0.4          # drive straight: the caster trails, which is when it chatters
MAX_ACCEL = 1.0
DEG_PER_RAD = 180.0 / math.pi
DAMPING = 1.0


def _analyse():
    spec = importlib.util.spec_from_file_location(
        "vp", BASE / "scripts/vibration_probe.py")
    vp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vp)
    return vp.analyse


def selftest():
    """The same detector the vibration probe uses, exercised on heights."""
    analyse = _analyse()
    ok = True
    n = 400
    # A caster resting quietly: tiny numerical noise, no oscillation.
    quiet = [0.02 + 1e-6 * math.sin(i) for i in range(n)]
    if analyse(quiet, DT)["oscillates"]:
        print("SELFTEST FAIL: a resting caster read as chattering"); ok = False
    # A caster bouncing at 30 Hz with 20 % amplitude: unmistakable chatter.
    bouncing = [0.02 + 0.004 * math.sin(2 * math.pi * 30 * i * DT)
                for i in range(n)]
    if not analyse(bouncing, DT)["oscillates"]:
        print("SELFTEST FAIL: a 30 Hz bounce was not detected"); ok = False
    print("selftest: OK" if ok else "selftest: BROKEN")
    return 0 if ok else 1


def main():
    if not STAGE.exists():
        sys.exit(f"no imported stage at {STAGE}; run scripts/import_rover.py")
    analyse = _analyse()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})

    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation, RigidPrim
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
    caster = RigidPrim(
        prim_paths_expr="/World/rover/Geometry/base_footprint/base_link/caster",
        name="caster")
    world.scene.add(caster)
    world.reset()
    rover.initialize()

    names = rover.dof_names
    li, ri = names.index("left_wheel_joint"), names.index("right_wheel_joint")
    _, kd = rover.get_gains()
    kd = np.array(kd)
    kd[0][li] = DAMPING * DEG_PER_RAD
    kd[0][ri] = DAMPING * DEG_PER_RAD
    rover.set_gains(kds=kd)

    base_z, caster_z = [], []
    v = 0.0
    for step in range(int(SECS / DT)):
        v = min(CMD_V, v + MAX_ACCEL * DT)
        tgt = np.zeros(len(names), dtype=np.float32)
        tgt[li] = tgt[ri] = v / WHEEL_R
        rover.set_joint_velocity_targets(np.array([tgt]))
        world.step(render=False)
        if step * DT >= SETTLE_S:
            p, _ = rover.get_world_poses()
            base_z.append(float(p[0][2]))
            cp, _ = caster.get_world_poses()
            caster_z.append(float(cp[0][2]))

    b = analyse(base_z, DT)
    c = analyse(caster_z, DT)
    result = {"engine": "isaac-physx", "physics_dt": DT,
              "commanded_linear_m_s": CMD_V, "settle_s": SETTLE_S,
              "base_height": b, "caster_height": c,
              "restitution_in_urdf": 0, "restitution_in_world": 0,
              "ground_restitution_set_here": 0.0,
              "caster_chatters": bool(c["oscillates"]),
              "base_chatters": bool(b["oscillates"])}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))
    print(f"  base   z: mean {b['mean']:.5f}  p2p {b['peak_to_peak']:.6f}  "
          f"{b['reversal_rate_hz']} Hz  -> {'CHATTERS' if b['oscillates'] else 'steady'}")
    print(f"  caster z: mean {c['mean']:.5f}  p2p {c['peak_to_peak']:.6f}  "
          f"{c['reversal_rate_hz']} Hz  -> {'CHATTERS' if c['oscillates'] else 'steady'}")
    print(f"  -> {OUT.relative_to(BASE)}")
    app.close()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
