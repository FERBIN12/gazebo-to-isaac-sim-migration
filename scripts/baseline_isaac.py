#!/usr/bin/env python3
"""a later check: the SAME measurement as baseline_gz.py, run in Isaac Sim.

This is the other half of the A/B rig. It deliberately mirrors
`scripts/baseline_gz.py` manoeuvre for manoeuvre and formula for formula,
because a comparison is only worth anything if both sides were measured the
same way. Same commands, same duration, same body-frame projection, same
closed-form prediction.

WHAT IS DELIBERATELY DIFFERENT, AND WHY IT IS STILL A FAIR COMPARISON.
The Gazebo side drives through ROS 2: publish `cmd_vel`, let
`diff_drive_base_controller` convert that to wheel velocities, read
`/odom`. Here there is no controller and no bridge -- this drives the PhysX
articulation's two wheel joints directly and integrates the base link's pose
from the simulator itself.

That is on purpose. Putting the ROS 2 bridge in the loop for the FIRST diff
would mean a divergence could come from the bridge, the controller, the
importer or the physics, with no way to tell which. So the first comparison
removes the two shared software layers and asks only: given the same wheel
velocities, do the two physics engines move the robot the same way? The bridge
and the controller come back in this repo, once geometry, joints and contact
have been signed off.

To keep it honest, the wheel velocities are computed with the SAME diff-drive
kinematics the Gazebo controller uses, from the same cmd_vel, including the
same acceleration ramp -- so the input really is the same command stream.

Run:
    ~/.venv-isaacsim/bin/python scripts/baseline_isaac.py

Writes data/baseline_isaac.json. `scripts/diff_ab.py` reads that and
data/baseline_gz.json and prints the diff against declared tolerances.
"""
import json
import math
import os
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
OUT = BASE / "data/baseline_isaac.json"

# THE ROBOT'S GEOMETRY, TAKEN FROM THE URDF, NOT GUESSED.
WHEEL_R = 0.075     # m
TRACK = 0.30        # m, wheel separation
# The controller's limits, matching gz.launch.py's diff_drive_base_controller.
MAX_ACCEL = 1.0     # m/s^2
MAX_ANG_ACCEL = 2.0  # rad/s^2

# The two manoeuvres, identical to baseline_gz.py.
STRAIGHT = {"lin": 0.4, "ang": 0.0, "secs": 2.0}
SPIN = {"lin": 0.0, "ang": 1.0, "secs": 2.0}

PHYSICS_DT = 1.0 / 200.0   # PhysX step. Stated, because it is a variable that
#                            changes the answer -- a later check measures it.


def predict_straight():
    """Trapezoid, not rectangle. Same reasoning as the Gazebo baseline.

    The controller ramps at MAX_ACCEL, so the robot does not travel v*t. At
    0.4 m/s with 1.0 m/s^2 the ramp takes 0.40 s and covers 0.080 m, then
    1.60 s of cruise covers 0.640 m: 0.720 m, not 0.800 m.
    """
    v, a, t = STRAIGHT["lin"], MAX_ACCEL, STRAIGHT["secs"]
    t_ramp = v / a
    return 0.5 * a * t_ramp ** 2 + v * (t - t_ramp)


def predict_spin():
    w, a, t = SPIN["ang"], MAX_ANG_ACCEL, SPIN["secs"]
    t_ramp = w / a
    return 0.5 * a * t_ramp ** 2 + w * (t - t_ramp)


def yaw_of(q):
    """Yaw from a (w, x, y, z) quaternion. Same formula as the Gazebo side."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


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

    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT,
                  stage_units_in_meters=1.0)
    # A GROUND PLANE IS NOT OPTIONAL AND IS NOT IN THE IMPORT. The URDF says
    # nothing about the world, so a stage imported from it has no floor and the
    # robot falls forever -- which reads as a wild divergence from Gazebo rather
    # than as a missing world. Gazebo's flat.sdf supplies one; so must this.
    # Friction is a property of a PhysicsMaterial, not a GroundPlane kwarg.
    # It has to be stated: the URDF's mu=1.0 on the wheels is a <gazebo>-side
    # surface parameter that the importer carries only as a custom
    # `urdf:...:mu` annotation, so PhysX would otherwise use its own default
    # and the two engines would be compared on different ground.
    ground_mat = PhysicsMaterial(
        prim_path="/World/ground_material", static_friction=1.0,
        dynamic_friction=1.0, restitution=0.0)
    GroundPlane(prim_path="/World/ground", size=50.0,
                physics_material=ground_mat)
    add_reference_to_stage(usd_path=str(STAGE), prim_path="/World/rover")

    # OPTIONAL: bind the URDF's per-link friction, to PROVE the mechanism.
    #
    # The import carries <surface><friction><ode><mu> across as an inert
    # `custom string urdf:text` inside an empty Scope. It is structurally
    # present and physically ignored, so every contact on this robot gets
    # PhysX's default friction: the wheels lose their mu=1.0 and, more
    # importantly, the caster loses its mu=0.05 and starts dragging.
    #
    # Run with FRICTION=urdf to apply them and see the spin come back.
    if os.environ.get("FRICTION") == "urdf":
        from pxr import UsdShade, UsdPhysics
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        def bind(prim_path, mu):
            mat_path = f"/World/mat_mu_{str(mu).replace('.', '_')}"
            if not stage.GetPrimAtPath(mat_path):
                m = PhysicsMaterial(prim_path=mat_path, static_friction=mu,
                                    dynamic_friction=mu, restitution=0.0)
            prim = stage.GetPrimAtPath(prim_path)
            if not prim:
                print(f"  friction: NO PRIM at {prim_path}")
                return
            UsdShade.MaterialBindingAPI.Apply(prim)
            UsdShade.MaterialBindingAPI(prim).Bind(
                UsdShade.Material(stage.GetPrimAtPath(mat_path)),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                materialPurpose="physics")
            print(f"  friction: bound mu={mu} to {prim_path}")
        R = "/World/rover/Geometry/base_footprint/base_link"
        bind(f"{R}/left_wheel/cylinder_1", 1.0)
        bind(f"{R}/right_wheel/cylinder_1", 1.0)
        bind(f"{R}/caster/sphere_1", 0.05)

    rover = Articulation(prim_paths_expr="/World/rover", name="rover")
    world.scene.add(rover)
    world.reset()
    rover.initialize()

    names = rover.dof_names
    try:
        li = names.index("left_wheel_joint")
        ri = names.index("right_wheel_joint")
    except ValueError:
        sys.exit(f"wheel joints not in the articulation. dof_names = {names}")

    def settle(seconds=1.0):
        for _ in range(int(seconds / PHYSICS_DT)):
            world.step(render=False)

    def pose():
        p, q = rover.get_world_poses()
        return (float(p[0][0]), float(p[0][1]),
                yaw_of([float(v) for v in q[0]]))

    def drive(lin, ang, secs):
        """Command a ramped cmd_vel as wheel velocities; return the motion.

        Returns (forward, lateral, dyaw, peak_wheel_speed_m_s), exactly the
        tuple baseline_gz.py's drive() returns, in the same units and the same
        body frame, so the two can be diffed field by field.
        """
        x0, y0, yaw0 = pose()
        peak = 0.0
        n = int(secs / PHYSICS_DT)
        v_cur, w_cur = 0.0, 0.0
        for _ in range(n):
            # The SAME ramp the Gazebo controller applies, so the command
            # stream really is identical rather than merely similar.
            v_cur = min(lin, v_cur + MAX_ACCEL * PHYSICS_DT) if lin >= 0 \
                else max(lin, v_cur - MAX_ACCEL * PHYSICS_DT)
            w_cur = min(ang, w_cur + MAX_ANG_ACCEL * PHYSICS_DT) if ang >= 0 \
                else max(ang, w_cur - MAX_ANG_ACCEL * PHYSICS_DT)
            # Standard diff-drive inverse kinematics.
            v_l = (v_cur - w_cur * TRACK / 2.0) / WHEEL_R
            v_r = (v_cur + w_cur * TRACK / 2.0) / WHEEL_R
            tgt = np.zeros(len(names), dtype=np.float32)
            tgt[li], tgt[ri] = v_l, v_r
            rover.set_joint_velocity_targets(np.array([tgt]))
            world.step(render=False)
            jv = rover.get_joint_velocities()[0]
            peak = max(peak, abs(float(jv[li])) * WHEEL_R,
                       abs(float(jv[ri])) * WHEEL_R)
        # Stop, and let it come to rest, matching the Gazebo side's zero-publish
        # plus 30 spins.
        rover.set_joint_velocity_targets(
            np.array([np.zeros(len(names), dtype=np.float32)]))
        settle(0.6)

        x1, y1, yaw1 = pose()
        dxw, dyw = x1 - x0, y1 - y0
        # Project into the body frame at the START of the move -- the same
        # correction the Gazebo baseline needed. Diffing world-frame x/y makes a
        # robot that started at a non-zero heading look like it slid sideways,
        # which a diff-drive robot cannot do.
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        fwd, lat = c * dxw - s * dyw, s * dxw + c * dyw
        dyaw = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
        return fwd, lat, dyaw, peak

    settle(1.0)   # let it rest on the plane before measuring anything

    res = {"engine": "isaac-physx", "physics_dt": PHYSICS_DT,
           "friction": os.environ.get("FRICTION", "physx-default"),
           "wheel_radius": WHEEL_R, "track": TRACK,
           "dof_names": list(names)}

    fwd, lat, dyaw, peak = drive(**STRAIGHT)
    res["straight"] = {"dx": fwd, "dy": lat, "dyaw": dyaw,
                       "peak_wheel_m_s": peak, "predict": predict_straight()}
    print(f"STRAIGHT  dx={fwd:+.3f} m (predict {predict_straight():.3f})  "
          f"dy={lat:+.3f}  dyaw={dyaw:+.3f}  peak={peak:.3f} m/s")

    settle(0.5)
    fwd, lat, dyaw, peak = drive(**SPIN)
    res["spin"] = {"dx": fwd, "dy": lat, "dyaw": dyaw,
                   "peak_wheel_m_s": peak, "predict": predict_spin()}
    print(f"SPIN      dyaw={dyaw:+.3f} rad (predict {predict_spin():.3f})  "
          f"dx={fwd:+.3f}  dy={lat:+.3f}  peak={peak:.3f} m/s")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))
    print(f"\nwrote {OUT.relative_to(BASE)}")
    sys.stdout.flush()
    # Everything above happens BEFORE close(): fastShutdown does not return.
    app.close()


if __name__ == "__main__":
    main()
