"""Measure wheel slip directly, and test the a later check prediction.

THE PREDICTION (notes 5.1): the robot's wheels turn the correct amount and
the robot rotates 16.4 % less than it should, so the difference went into
SLIDING rather than into turning the robot. That is slip, and it is the only
mechanism I could name that converts correct wheel motion into insufficient
robot motion.

This measures it rather than arguing it. Slip is defined here as the difference
between the distance the wheel rim TRAVELLED (its angular displacement times the
radius) and the distance the contact patch actually MOVED over the ground:

    rim_travel   = |wheel angle swept| * WHEEL_R
    ground_travel = arc the wheel's contact point traced in the world
    slip_fraction = 1 - ground_travel / rim_travel

A perfectly gripping wheel has slip 0. A wheel spinning on ice has slip 1.

WHY BOTH ENGINES. a later check exists because I once reported a divergence with
one side measured. The Gazebo half runs through `slip_probe_gz.py`, which reads
`/joint_states` and `/odom` and computes the SAME quantity with the same
formula, so a difference between the two numbers is a difference in the
simulators rather than in my arithmetic.

Usage:
    ~/.venv-isaacsim/bin/python scripts/slip_probe.py
    python3 scripts/slip_probe.py --selftest
"""
import json
import math
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/slip_probe.json"

WHEEL_R = 0.075
TRACK = 0.30
DT = 1.0 / 200.0
SECS = 2.0
CMD_W = 1.0
MAX_ANG_ACCEL = 2.0
DEG_PER_RAD = 180.0 / math.pi
DAMPING = 1.0          # the gain notes 4.4 chose


def slip_fraction(rim_travel, ground_travel):
    """1 - ground/rim, clamped to [0, 1]. Zero rim travel means no answer."""
    if rim_travel <= 1e-9:
        return None
    return max(0.0, min(1.0, 1.0 - ground_travel / rim_travel))


def selftest():
    ok = True
    # A perfectly gripping wheel: everything the rim travelled reached the ground.
    if abs(slip_fraction(1.0, 1.0) - 0.0) > 1e-9:
        print("SELFTEST FAIL: a gripping wheel should read slip 0"); ok = False
    # A wheel spinning on ice: the rim moves, the robot does not.
    if abs(slip_fraction(1.0, 0.0) - 1.0) > 1e-9:
        print("SELFTEST FAIL: a spinning-on-ice wheel should read slip 1"); ok = False
    # Half the rim travel reaching the ground is 50 % slip.
    if abs(slip_fraction(2.0, 1.0) - 0.5) > 1e-9:
        print("SELFTEST FAIL: half-transferred travel should read slip 0.5"); ok = False
    # A wheel that somehow moved FURTHER than it rolled is not negative slip.
    if slip_fraction(1.0, 1.5) != 0.0:
        print("SELFTEST FAIL: over-travel should clamp to 0, not go negative"); ok = False
    # No rotation means the question is unanswerable, not "zero slip".
    if slip_fraction(0.0, 0.0) is not None:
        print("SELFTEST FAIL: zero rim travel should return None, not 0"); ok = False
    print("selftest: OK" if ok else "selftest: BROKEN")
    return 0 if ok else 1


def main():
    if not STAGE.exists():
        sys.exit(f"no imported stage at {STAGE}; run scripts/import_rover.py")

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

    _, kd = rover.get_gains()
    kd = np.array(kd)
    kd[0][li] = DAMPING * DEG_PER_RAD
    kd[0][ri] = DAMPING * DEG_PER_RAD
    rover.set_gains(kds=kd)

    def yaw():
        _, q = rover.get_world_poses()
        q = [float(x) for x in q[0]]
        return math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                          1 - 2 * (q[2] ** 2 + q[3] ** 2))

    y0 = yaw()
    # Integrate the wheel angle from measured velocity: joint POSITION wraps,
    # and unwrapping it is one more place to introduce a bug.
    swept_r = 0.0
    w = 0.0
    for _ in range(int(SECS / DT)):
        w = min(CMD_W, w + MAX_ANG_ACCEL * DT)
        tgt = np.zeros(len(names), dtype=np.float32)
        tgt[li] = (-w * TRACK / 2.0) / WHEEL_R
        tgt[ri] = (+w * TRACK / 2.0) / WHEEL_R
        rover.set_joint_velocity_targets(np.array([tgt]))
        world.step(render=False)
        swept_r += abs(float(rover.get_joint_velocities()[0][ri])) * DT
    y1 = yaw()
    dyaw = abs(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))

    # During a pure spin each wheel's contact point traces an arc of radius
    # TRACK/2 about the robot centre, so the ground distance it covered is
    # dyaw * TRACK/2.
    rim_travel = swept_r * WHEEL_R
    ground_travel = dyaw * (TRACK / 2.0)
    slip = slip_fraction(rim_travel, ground_travel)

    result = {
        "engine": "isaac-physx", "damping": DAMPING, "physics_dt": DT,
        "wheel_angle_swept_rad": round(swept_r, 4),
        "rim_travel_m": round(rim_travel, 5),
        "dyaw_rad": round(dyaw, 4),
        "ground_travel_m": round(ground_travel, 5),
        "slip_fraction": None if slip is None else round(slip, 4),
        "gazebo_reference_dyaw": 1.857,
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))
    print(f"  wheel swept    {swept_r:.4f} rad")
    print(f"  rim travel     {rim_travel:.5f} m")
    print(f"  dyaw           {dyaw:.4f} rad")
    print(f"  ground travel  {ground_travel:.5f} m")
    print(f"  SLIP FRACTION  {slip:.4f}" if slip is not None else "  SLIP  n/a")
    print(f"  -> {OUT.relative_to(BASE)}")
    app.close()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
