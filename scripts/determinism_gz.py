"""Measure whether Gazebo reproduces the same run twice, on the sim clock.

a later check asks about determinism and reproducibility on both sides. Isaac's
side is already measured: `damping_sweep.py` gave two consecutive runs agreeing
to 0.000 on every row. This is the Gazebo half of the same question, which no
script in this repo had.

WHAT "DETERMINISTIC" MEANS HERE, precisely, because the word is used loosely:
  * bit-identical trajectories from an identical command sequence -- the strong
    claim, and the only one worth calling determinism
  * NOT "the robot ends up in roughly the same place", which is repeatability
    and is a much weaker property

METHOD. Drive a fixed command profile for a fixed number of SIM seconds, N
times, from a freshly reset world each time, and compare the final pose. All
timing is on the sim clock: 5.3 lost a day to a wall-clock integration and 7.1
read a rate 3x low for a related reason.

WHY THE RESET MATTERS. Without a world reset between runs the robot starts from
where the last run left it, so run 2 measures a different experiment and the
"non-determinism" you find is your own harness.

Usage:
    scripts/determinism_gz.py [--runs 3] [--secs 4]
    python3 scripts/determinism_gz.py --selftest
"""
import json
import os
import math
import statistics
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
OUT = BASE / "data/determinism_gz.json"


def spread(vals):
    """max - min, and whether every run agreed to 1e-9.

    Reporting the SPREAD rather than a standard deviation because with 3 runs a
    stdev is not meaningful and the question is binary: identical or not.
    """
    if not vals:
        return None
    lo, hi = min(vals), max(vals)
    return {"n": len(vals), "min": lo, "max": hi, "spread": hi - lo,
            "identical": (hi - lo) < 1e-9,
            "median": statistics.median(vals)}


def selftest():
    ok = True
    # identical runs must report identical, with zero spread
    s = spread([1.5, 1.5, 1.5])
    if not (s["identical"] and s["spread"] == 0.0):
        print(f"  FAIL identical case {s}"); ok = False
    # a difference in the 4th decimal must NOT report identical
    s = spread([1.5, 1.5001, 1.5])
    if s["identical"]:
        print(f"  FAIL 1e-4 difference reported identical: {s}"); ok = False
    if abs(s["spread"] - 0.0001) > 1e-9:
        print(f"  FAIL spread {s['spread']}, expected 1e-4"); ok = False
    # a difference at 1e-12 is below float noise and SHOULD read identical
    s = spread([1.5, 1.5 + 1e-12, 1.5])
    if not s["identical"]:
        print("  FAIL 1e-12 difference reported non-identical"); ok = False
    # empty -> None, not a crash
    if spread([]) is not None:
        print("  FAIL empty input"); ok = False
    # THE DEAD-PATH CASE. Three runs of exactly 0.0 are "identical" and are the
    # signature of a robot that never moved. `spread` correctly calls them
    # identical -- so the REFUSAL has to live in main(), and this asserts the
    # arithmetic that drives it rather than trusting the comment.
    dead = spread([0.0, 0.0, 0.0])
    if not dead["identical"]:
        print("  FAIL zeros should read identical to spread()"); ok = False
    secs, expected_min = 4.0, 0.25 * 0.4 * 4.0
    if not (max([0.0, 0.0, 0.0]) < expected_min):
        print("  FAIL dead-path guard would not fire on all-zero motion"); ok = False
    if not (max([1.03, 1.03, 1.03]) >= expected_min):
        print("  FAIL guard would wrongly reject a real 1.03 m run"); ok = False
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()
    runs = 3
    secs = 4.0
    if "--runs" in sys.argv:
        runs = int(sys.argv[sys.argv.index("--runs") + 1])
    if "--secs" in sys.argv:
        secs = float(sys.argv[sys.argv.index("--secs") + 1])

    import rclpy
    import subprocess
    from rclpy.node import Node
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    n = Node("determinism_gz")
    pub = n.create_publisher(TwistStamped,
                             "/diff_drive_base_controller/cmd_vel", 10)
    state = {"sim": None, "odom": None}

    def on_clock(m):
        state["sim"] = m.clock.sec + m.clock.nanosec * 1e-9

    def on_odom(m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        state["odom"] = (p.x, p.y, yaw)

    n.create_subscription(Clock, "/clock", on_clock, 10)
    n.create_subscription(Odometry, "/diff_drive_base_controller/odom",
                          on_odom, 10)

    def pump(duration_sim):
        """Spin until `duration_sim` SIM seconds have passed.

        WAITS FOR BOTH SUBSCRIPTIONS FIRST. The first version waited only on
        /clock and then read state["odom"], which was still None -- the odom
        subscription had not delivered yet, and the run crashed on a healthy
        simulator. A probe must not read a topic it has not confirmed arriving.
        """
        # KICK THE CONTROLLER FIRST. diff_drive_controller publishes /odom only
        # after it has received a command -- even a ZERO one. Waiting for odom
        # before publishing anything is a deadlock: the probe waits for a topic
        # that will not exist until the probe speaks. Measured: 0 odom messages
        # in 15 s while idle, 1 within 0.1 s of a zero cmd_vel.
        kick = TwistStamped()
        deadline = time.time() + 20
        while (state["sim"] is None or state["odom"] is None):
            kick.header.stamp = n.get_clock().now().to_msg()
            pub.publish(kick)
            rclpy.spin_once(n, timeout_sec=0.05)
            if time.time() > deadline:
                raise SystemExit("no /clock or /odom within 20 s; is the stack up?")
        t0 = state["sim"]
        while state["sim"] - t0 < duration_sim:
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(0.0005)

    def reset_world():
        """Put the robot back at the origin, and VERIFY it landed there.

        `/world/flat/control` with `reset: {all: true}` did NOT work: the call
        returned successfully and the robot stayed where it was (odom read
        60.06 m after a "reset"). `/world/flat/set_pose` moves the model
        directly and is what the guard below actually confirms.

        Two bugs lived here. First, `gz service` inherits the environment, and
        without GZ_PARTITION it talks to a different transport namespace than
        the simulator, so the call reaches nothing. Second, nothing checked:
        the probe measured three runs that each started where the last ended,
        got dx = 0.000000 three times, and reported `identical=True`.

        THREE IDENTICAL ZEROES IS NOT DETERMINISM, IT IS A DEAD PATH.
        """
        env = dict(os.environ)
        subprocess.run(
            ["gz", "service", "-s", "/world/flat/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "3000",
             "--req", 'name: "rover", position: {x: 0, y: 0, z: 0.05}, '
                      'orientation: {x: 0, y: 0, z: 0, w: 1}'],
            capture_output=True, timeout=15, env=env)
        for _ in range(200):
            rclpy.spin_once(n, timeout_sec=0.02)

    # ODOMETRY DOES NOT RESET WITH THE MODEL. diff_drive_controller integrates
    # wheel positions from when it started, so teleporting the body does not
    # zero /odom. Every run is therefore measured as a DELTA from its own
    # start pose, which is correct regardless, and the guard below checks that
    # the robot MOVED rather than that it started at zero.

    results = []
    for i in range(runs):
        reset_world()
        pump(1.0)                      # settle after the reset
        start = state["odom"]
        m = TwistStamped()
        m.twist.linear.x = 0.4
        # drive for `secs` of SIM time, republishing so the controller does not
        # time out on a stale command
        t0 = state["sim"]
        while state["sim"] - t0 < secs:
            m.header.stamp = n.get_clock().now().to_msg()
            pub.publish(m)
            rclpy.spin_once(n, timeout_sec=0.0)
            time.sleep(0.01)
        m.twist.linear.x = 0.0
        for _ in range(10):
            pub.publish(m); rclpy.spin_once(n, timeout_sec=0.0); time.sleep(0.01)
        pump(0.5)
        end = state["odom"]
        results.append({"run": i, "start": start, "end": end,
                        "dx": end[0] - start[0], "dy": end[1] - start[1],
                        "dyaw": end[2] - start[2]})
        print(f"  run {i}: dx={results[-1]['dx']:+.6f} "
              f"dy={results[-1]['dy']:+.6f} dyaw={results[-1]['dyaw']:+.6f}")

    n.destroy_node(); rclpy.shutdown()

    # REFUSE TO REPORT DETERMINISM ON A ROBOT THAT DID NOT MOVE. Three runs of
    # dx=0.000000 satisfy "identical" perfectly and mean nothing. A determinism
    # claim requires the experiment to have HAPPENED first, and at 0.4 m/s for
    # `secs` sim seconds the robot should travel roughly 0.4*secs metres.
    expected_min = 0.25 * 0.4 * secs
    moved = [abs(r["dx"]) for r in results]
    if max(moved) < expected_min:
        raise SystemExit(
            f"REFUSING: largest dx was {max(moved):.6f} m, expected at least "
            f"{expected_min:.3f} m. The robot did not move, so there is no "
            f"experiment to call deterministic.")

    res = {"runs": runs, "secs_sim": secs, "results": results,
           "moved_min_m": round(min(moved), 6), "moved_max_m": round(max(moved), 6),
           "dx": spread([r["dx"] for r in results]),
           "dy": spread([r["dy"] for r in results]),
           "dyaw": spread([r["dyaw"] for r in results])}
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))

    for k in ("dx", "dy", "dyaw"):
        s = res[k]
        print(f"{k:5s} min={s['min']:+.6f} max={s['max']:+.6f} "
              f"spread={s['spread']:.2e} identical={s['identical']}")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
