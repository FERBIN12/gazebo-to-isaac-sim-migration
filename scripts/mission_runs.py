"""The overall condition from write-up 1.7: the same mission, 20 runs per engine.

> "Run the same mission on both simulators, twenty times each, and the
>  completion rate must agree within ten percent, with no failure mode present
>  in Isaac that is absent in Gazebo."

Two halves, and 1.7 said the second matters more. So this records BOTH:
  * the completion rate, as a fraction of runs that reached the goal
  * the SET of failure modes observed, per engine, so "a new failure mode in
    Isaac" is answerable rather than inferred from a rate

WHAT COUNTS AS COMPLETION, defined before running, because a success criterion
invented after seeing the data is not a criterion. The mission is: drive
forward for 4 simulated seconds at 0.4 m/s. It COMPLETES if the robot travels
at least 75 % of the commanded distance along its own heading and drifts less
than 0.15 m laterally. Both thresholds are derived below.

WHY 75 %: 8.1 measured that the 8 rad/s joint limit caps the achievable speed
at 0.6 m/s against a commanded 0.8, i.e. 75 % -- so a threshold at 75 % of the
COMMANDED 0.4 m/s distance is comfortably above what the limit allows, and a
run that fails it has failed for some other reason.

WHY 0.15 m LATERAL: 1.7's contact condition allows 5 cm of lateral drift, and
8.6 measured a 7.5 mm reproducibility floor. 0.15 m is 3x the stated tolerance
and 20x the noise floor, so it flags a genuine veer rather than jitter.

Usage:
    scripts/mission_runs.py --engine gazebo [--runs 20]
    scripts/mission_runs.py --engine isaac  [--runs 20]
    python3 scripts/mission_runs.py --selftest
"""
import json
import math
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
SECS = 4.0
TARGET_MS = 0.4
COMPLETION_FRACTION = 0.75      # see docstring
LATERAL_LIMIT_M = 0.15          # see docstring


def classify(dx, dy, dyaw, secs=SECS, target=TARGET_MS):
    """Completed, or the named failure mode. Defined before any run."""
    expected = target * secs
    if abs(dx) < COMPLETION_FRACTION * expected:
        return "short_travel"
    if abs(dy) > LATERAL_LIMIT_M:
        return "lateral_drift"
    if abs(dyaw) > 0.35:            # ~20 degrees on a straight-line mission
        return "heading_error"
    return "completed"


def selftest():
    ok = True
    exp = TARGET_MS * SECS          # 1.6 m
    # a good run completes
    if classify(1.61, -0.05, -0.002) != "completed":
        print("  FAIL a good run did not complete"); ok = False
    # each failure mode must be detected, and named distinctly
    if classify(0.20, 0.0, 0.0) != "short_travel":
        print("  FAIL short travel undetected"); ok = False
    if classify(1.61, 0.9, 0.0) != "lateral_drift":
        print("  FAIL lateral drift undetected"); ok = False
    if classify(1.61, 0.0, 1.2) != "heading_error":
        print("  FAIL heading error undetected"); ok = False
    # THE ORDER MATTERS: a run that is both short AND drifting is reported as
    # short, deterministically, so the failure-mode SETS are comparable between
    # engines rather than depending on evaluation order.
    if classify(0.20, 0.9, 1.2) != "short_travel":
        print("  FAIL classification is not deterministic under multiple faults")
        ok = False
    # the threshold must sit above what the joint limit allows, not below
    if COMPLETION_FRACTION * exp >= exp:
        print("  FAIL completion threshold is not below the commanded distance")
        ok = False
    # and a robot that never moved must NOT complete
    if classify(0.0, 0.0, 0.0) == "completed":
        print("  FAIL a stationary robot completed"); ok = False
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def run_gazebo(runs):
    import subprocess, os, time
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    n = Node("mission_runs")
    pub = n.create_publisher(TwistStamped,
                             "/diff_drive_base_controller/cmd_vel", 10)
    st = {"sim": None, "odom": None}
    n.create_subscription(Clock, "/clock",
                          lambda m: st.update(sim=m.clock.sec + m.clock.nanosec * 1e-9), 10)

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        st["odom"] = (p.x, p.y, yaw)
    n.create_subscription(Odometry, "/diff_drive_base_controller/odom", on_odom, 10)

    def wait_ready(timeout=25):
        kick = TwistStamped()
        t0 = time.time()
        while st["sim"] is None or st["odom"] is None:
            kick.header.stamp = n.get_clock().now().to_msg()
            pub.publish(kick)
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.time() - t0 > timeout:
                raise SystemExit("no /clock or /odom; is the stack up?")

    def reset():
        subprocess.run(
            ["gz", "service", "-s", "/world/flat/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000",
             "--req", 'name: "rover", position: {x: 0, y: 0, z: 0.05}, '
                      'orientation: {x: 0, y: 0, z: 0, w: 1}'],
            capture_output=True, timeout=15, env=dict(os.environ))
        for _ in range(200):
            rclpy.spin_once(n, timeout_sec=0.02)

    wait_ready()
    out = []
    for i in range(runs):
        reset()
        start = st["odom"]
        m = TwistStamped(); m.twist.linear.x = TARGET_MS
        t0 = st["sim"]
        while st["sim"] - t0 < SECS:
            m.header.stamp = n.get_clock().now().to_msg()
            pub.publish(m); rclpy.spin_once(n, timeout_sec=0.0); time.sleep(0.01)
        m.twist.linear.x = 0.0
        for _ in range(20):
            pub.publish(m); rclpy.spin_once(n, timeout_sec=0.0); time.sleep(0.01)
        t1 = st["sim"]
        while st["sim"] - t1 < 0.5:
            rclpy.spin_once(n, timeout_sec=0.0); time.sleep(0.005)
        end = st["odom"]
        dx, dy = end[0] - start[0], end[1] - start[1]
        dyaw = end[2] - start[2]
        verdict = classify(dx, dy, dyaw)
        out.append({"run": i, "dx": dx, "dy": dy, "dyaw": dyaw,
                    "verdict": verdict})
        print(f"  run {i:2d}: dx={dx:+.4f} dy={dy:+.4f} dyaw={dyaw:+.4f}  {verdict}")
    n.destroy_node(); rclpy.shutdown()
    return out


def run_isaac(runs):
    import numpy as np
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    from isaacsim.core.api import World
    from isaacsim.core.prims import Articulation
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.core.api.objects import GroundPlane
    from isaacsim.core.api.materials import PhysicsMaterial

    STAGE = BASE / "sim-workspace/isaac/rover.usd/rover/rover.usda"
    DT = 0.005
    world = World(physics_dt=DT, rendering_dt=DT, stage_units_in_meters=1.0)
    gm = PhysicsMaterial(prim_path="/World/gm", static_friction=1.0,
                         dynamic_friction=1.0, restitution=0.0)
    GroundPlane(prim_path="/World/ground", size=50.0, physics_material=gm)
    add_reference_to_stage(usd_path=str(STAGE), prim_path="/World/rover")
    rover = Articulation(prim_paths_expr="/World/rover", name="rover")
    world.scene.add(rover); world.reset(); rover.initialize()
    names = rover.dof_names
    li, ri = names.index("left_wheel_joint"), names.index("right_wheel_joint")
    cmd = np.zeros((1, len(names)))
    cmd[0, li] = cmd[0, ri] = TARGET_MS / 0.10

    def pose():
        p, q = rover.get_world_poses()
        p, q = np.array(p)[0], np.array(q)[0]
        yaw = math.atan2(2 * (q[0]*q[3] + q[1]*q[2]), 1 - 2 * (q[2]**2 + q[3]**2))
        return float(p[0]), float(p[1]), float(yaw)

    out = []
    steps = int(SECS / DT)
    for i in range(runs):
        world.reset(); rover.initialize()
        for _ in range(int(0.5 / DT)):
            world.step(render=False)
        start = pose()
        for _ in range(steps):
            rover.set_joint_velocity_targets(cmd); world.step(render=False)
        zero = np.zeros_like(cmd)
        for _ in range(int(0.5 / DT)):
            rover.set_joint_velocity_targets(zero); world.step(render=False)
        end = pose()
        dx, dy = end[0] - start[0], end[1] - start[1]
        dyaw = end[2] - start[2]
        verdict = classify(dx, dy, dyaw)
        out.append({"run": i, "dx": dx, "dy": dy, "dyaw": dyaw,
                    "verdict": verdict})
        print(f"  run {i:2d}: dx={dx:+.4f} dy={dy:+.4f} dyaw={dyaw:+.4f}  {verdict}")
    # write BEFORE closing: 8.6 lost a whole result to app.close() ordering
    return out, app


def main():
    if "--selftest" in sys.argv:
        return selftest()
    engine = None
    runs = 20
    if "--engine" in sys.argv:
        engine = sys.argv[sys.argv.index("--engine") + 1]
    if "--runs" in sys.argv:
        runs = int(sys.argv[sys.argv.index("--runs") + 1])
    if engine not in ("gazebo", "isaac"):
        sys.exit("need --engine gazebo|isaac")

    app = None
    if engine == "gazebo":
        rows = run_gazebo(runs)
    else:
        rows, app = run_isaac(runs)

    # THE DEAD-PATH GUARD, ported from determinism_gz.py. 8.6 established that
    # a run of exact zeroes is a broken harness, not a result -- and I shipped
    # this script WITHOUT the guard and immediately got 20 runs of dx=0.0000
    # reported as "0/20 completed, failure mode: short_travel". That reads as a
    # measured finding about the robot. It was a dead controller.
    moved = [abs(r["dx"]) for r in rows]
    if rows and max(moved) < 1e-6:
        raise SystemExit(
            f"REFUSING: all {len(rows)} runs moved < 1e-6 m. The robot never "
            f"moved, so there is no mission to score. Check the controllers "
            f"are active and /odom is publishing.")

    completed = sum(1 for r in rows if r["verdict"] == "completed")
    modes = sorted({r["verdict"] for r in rows if r["verdict"] != "completed"})
    res = {"engine": engine, "runs": len(rows),
           "completed": completed,
           "completion_rate": completed / len(rows) if rows else 0.0,
           "failure_modes": modes,
           "thresholds": {"completion_fraction": COMPLETION_FRACTION,
                          "lateral_limit_m": LATERAL_LIMIT_M,
                          "secs": SECS, "target_ms": TARGET_MS},
           "results": rows}
    out = BASE / f"data/mission_{engine}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(res, indent=2))
    print(f"\n{engine}: {completed}/{len(rows)} completed "
          f"({res['completion_rate']:.0%})")
    print(f"failure modes: {modes or 'none'}")
    print(f"-> wrote {out.relative_to(BASE)}")
    if app is not None:
        app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
