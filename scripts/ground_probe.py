"""Sweep the GROUND material and see what the robot's turn actually depends on.

a later check showed the URDF's per-link frictions are inert, so every contact on
this robot runs on whatever the ground material says. That makes the ground the
single place friction is actually decided, and this sweeps it.

TWO QUESTIONS, because they have different answers:

  1. How much does ground friction change the turn at all? If the wheels were
     gripping perfectly, raising mu would do nothing and lowering it would ruin
     the turn. The shape of that curve says how close to slipping we are.

  2. Does the wheel-to-caster RATIO matter more than the absolute values?
     Notes 5.2 argued it does, from the fact that binding the URDF frictions
     (wheels 1.0, caster 0.05) made the spin WORSE than uniform defaults. That
     was an explanation offered for someone else's measurement; this tests it.

Both are run at drive damping 1.0, the value 4.4 chose, so the drive is not the
variable.

Usage:
    ~/.venv-isaacsim/bin/python scripts/ground_probe.py
    python3 scripts/ground_probe.py --selftest
"""
import json
import math
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/ground_probe.json"

WHEEL_R = 0.075
TRACK = 0.30
DT = 1.0 / 200.0
SECS = 2.0
CMD_W = 1.0
MAX_ANG_ACCEL = 2.0
DEG_PER_RAD = 180.0 / math.pi
DAMPING = 1.0
GAZEBO_DYAW = 1.857

# Question 1: uniform ground friction, spanning ice to grippy.
MUS = [0.05, 0.2, 0.5, 1.0, 2.0]

# Question 2: the URDF's own asymmetry, applied per-collider.
#   (wheel mu, caster mu) -- uniform 1.0 is the control.
RATIOS = [(1.0, 1.0), (1.0, 0.05), (0.5, 0.05), (1.0, 0.5)]


def monotone(vals, tol=1e-9):
    return all(b >= a - tol for a, b in zip(vals, vals[1:]))


def selftest():
    ok = True
    if not monotone([0.5, 1.0, 1.4, 1.5]):
        print("SELFTEST FAIL: a rising series should read monotone"); ok = False
    if monotone([0.5, 1.4, 1.0, 1.5]):
        print("SELFTEST FAIL: a dipping series should NOT read monotone"); ok = False
    # A saturating curve is monotone: raising mu past the grip point does
    # nothing, and "does nothing" must not read as a failure.
    if not monotone([0.9, 1.4, 1.55, 1.55, 1.55]):
        print("SELFTEST FAIL: a saturating curve is still monotone"); ok = False
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
    from pxr import UsdShade
    import omni.usd

    world = World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
    gmat = PhysicsMaterial(prim_path="/World/gm", static_friction=1.0,
                           dynamic_friction=1.0, restitution=0.0)
    GroundPlane(prim_path="/World/ground", size=50.0, physics_material=gmat)
    add_reference_to_stage(usd_path=str(STAGE), prim_path="/World/rover")

    rover = Articulation(prim_paths_expr="/World/rover", name="rover")
    world.scene.add(rover)
    world.reset()
    rover.initialize()
    names = rover.dof_names
    li, ri = names.index("left_wheel_joint"), names.index("right_wheel_joint")
    stage = omni.usd.get_context().get_stage()

    R = "/World/rover/Geometry/base_footprint/base_link"
    COLLIDERS = {"left": f"{R}/left_wheel/cylinder_1",
                 "right": f"{R}/right_wheel/cylinder_1",
                 "caster": f"{R}/caster/sphere_1"}

    def set_ground(mu):
        gmat.set_static_friction(mu)
        gmat.set_dynamic_friction(mu)

    def bind(path, mu):
        """Bind a per-shape material override, creating it once per value."""
        mp = f"/World/mat_{str(mu).replace('.', '_')}"
        if not stage.GetPrimAtPath(mp):
            PhysicsMaterial(prim_path=mp, static_friction=mu,
                            dynamic_friction=mu, restitution=0.0)
        prim = stage.GetPrimAtPath(path)
        if not prim:
            return False
        UsdShade.MaterialBindingAPI.Apply(prim)
        UsdShade.MaterialBindingAPI(prim).Bind(
            UsdShade.Material(stage.GetPrimAtPath(mp)),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants,
            materialPurpose="physics")
        return True

    def spin():
        world.reset()
        rover.initialize()
        _, kd = rover.get_gains()
        kd = np.array(kd)
        kd[0][li] = kd[0][ri] = DAMPING * DEG_PER_RAD
        rover.set_gains(kds=kd)
        _, q = rover.get_world_poses()
        q = [float(x) for x in q[0]]
        y0 = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                        1 - 2 * (q[2] ** 2 + q[3] ** 2))
        w = 0.0
        for _ in range(int(SECS / DT)):
            w = min(CMD_W, w + MAX_ANG_ACCEL * DT)
            tgt = np.zeros(len(names), dtype=np.float32)
            tgt[li] = (-w * TRACK / 2.0) / WHEEL_R
            tgt[ri] = (+w * TRACK / 2.0) / WHEEL_R
            rover.set_joint_velocity_targets(np.array([tgt]))
            world.step(render=False)
        _, q = rover.get_world_poses()
        q = [float(x) for x in q[0]]
        y1 = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                        1 - 2 * (q[2] ** 2 + q[3] ** 2))
        return abs(math.atan2(math.sin(y1 - y0), math.cos(y1 - y0)))

    print("  UNIFORM GROUND FRICTION")
    uniform = []
    for mu in MUS:
        set_ground(mu)
        d = spin()
        uniform.append({"mu": mu, "dyaw_rad": round(d, 4),
                        "vs_gazebo_pct": round(100 * abs(d - GAZEBO_DYAW) / GAZEBO_DYAW, 1)})
        print(f"    mu {mu:>5}   dyaw {d:.4f} rad   {uniform[-1]['vs_gazebo_pct']:.1f} % off")

    set_ground(1.0)
    print("\n  PER-SHAPE OVERRIDES  (wheel, caster)")
    ratios = []
    for wmu, cmu in RATIOS:
        ok = all([bind(COLLIDERS["left"], wmu), bind(COLLIDERS["right"], wmu),
                  bind(COLLIDERS["caster"], cmu)])
        d = spin()
        ratios.append({"wheel_mu": wmu, "caster_mu": cmu,
                       "ratio": round(wmu / cmu, 2), "bound": ok,
                       "dyaw_rad": round(d, 4),
                       "vs_gazebo_pct": round(100 * abs(d - GAZEBO_DYAW) / GAZEBO_DYAW, 1)})
        print(f"    wheel {wmu:<5} caster {cmu:<5} ratio {wmu/cmu:>6.1f}   "
              f"dyaw {d:.4f} rad   {ratios[-1]['vs_gazebo_pct']:.1f} % off")

    dy = [r["dyaw_rad"] for r in uniform]
    result = {"engine": "isaac-physx", "damping": DAMPING, "physics_dt": DT,
              "gazebo_reference_dyaw": GAZEBO_DYAW,
              "uniform_ground": uniform, "per_shape_overrides": ratios,
              "uniform_monotone_in_mu": monotone(dy),
              "uniform_range_rad": [min(dy), max(dy)],
              "best_uniform_vs_gazebo_pct": min(r["vs_gazebo_pct"] for r in uniform),
              "best_override_vs_gazebo_pct": min(r["vs_gazebo_pct"] for r in ratios)}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))
    print(f"\n  monotone in mu: {result['uniform_monotone_in_mu']}")
    print(f"  best uniform:   {result['best_uniform_vs_gazebo_pct']} % off gazebo")
    print(f"  best override:  {result['best_override_vs_gazebo_pct']} % off gazebo")
    print(f"  -> {OUT.relative_to(BASE)}")
    app.close()


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        sys.exit(selftest())
    main()
