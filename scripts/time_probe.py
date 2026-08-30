"""Measure which clock each node on the stack is actually reading.

Notes 5.3 cost a day to integrating on the wall clock while the simulator ran
on its own, and 7.1 measured /scan at 2.96 Hz for a related reason. So this
measures rather than reading the launch file.

use_sim_time is a PER-NODE parameter. A node with it false reads the system
clock; a node with it true reads /clock. Two nodes in the same graph can
disagree, and when they do, a timestamp from one is meaningless to the other --
which surfaces as a TF extrapolation error, not as a clock error.

WHAT IT CHECKS:
  * every node's use_sim_time parameter, queried from the running node rather
    than read from the launch file, because a param file can set it too
  * the real-time factor, from /clock against the wall clock
  * message header stamps against both clocks, per topic: a topic stamped on the
    wrong clock is off by the entire unix epoch, which is unmistakable once you
    look and invisible if you do not

Usage:
    scripts/time_probe.py
    python3 scripts/time_probe.py --selftest
"""
import json
import re
import subprocess
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
OUT = BASE / "data/time_probe.json"


def classify_stamp(stamp_sec, sim_now, wall_now, tol=5.0):
    """Which clock does this header stamp belong to?

    A sim clock starts near 0 and a wall clock is ~1.79e9, so the two are never
    ambiguous in practice. Returns "sim", "wall", "zero" or "unknown".
    """
    if stamp_sec == 0:
        return "zero"
    if abs(stamp_sec - sim_now) <= tol:
        return "sim"
    if abs(stamp_sec - wall_now) <= tol:
        return "wall"
    return "unknown"


def node_use_sim_time(node):
    """Query the running node for its own use_sim_time. None if unavailable."""
    p = subprocess.run(["ros2", "param", "get", node, "use_sim_time"],
                       capture_output=True, text=True, timeout=20)
    m = re.search(r"Boolean value is: (True|False)", p.stdout)
    return (m.group(1) == "True") if m else None


def selftest():
    ok = True
    SIM, WALL = 12.5, 1787419000.0
    # a sim stamp must read as sim, a wall stamp as wall
    if classify_stamp(12.4, SIM, WALL) != "sim":
        print("  FAIL sim stamp misread"); ok = False
    if classify_stamp(1787418999.0, SIM, WALL) != "wall":
        print("  FAIL wall stamp misread"); ok = False
    # zero is its own case: an unstamped message, not a clock choice
    if classify_stamp(0, SIM, WALL) != "zero":
        print("  FAIL zero stamp misread"); ok = False
    # and something in neither band must NOT be silently bucketed
    if classify_stamp(500000.0, SIM, WALL) != "unknown":
        print("  FAIL out-of-band stamp was bucketed"); ok = False
    # the two clocks must not be confusable at these magnitudes
    if classify_stamp(WALL, SIM, WALL) == "sim":
        print("  FAIL wall stamp read as sim"); ok = False
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan, JointState
    from nav_msgs.msg import Odometry

    nodes = subprocess.run(["ros2", "node", "list"], capture_output=True,
                           text=True, timeout=30).stdout.split()
    params = {n: node_use_sim_time(n) for n in nodes}

    rclpy.init()
    n = Node("time_probe")
    sim = []
    stamps = {}

    def on_clock(m):
        sim.append(m.clock.sec + m.clock.nanosec * 1e-9)

    def grab(name):
        def cb(m):
            s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            stamps.setdefault(name, []).append(s)
        return cb

    n.create_subscription(Clock, "/clock", on_clock, 10)
    n.create_subscription(LaserScan, "/scan", grab("/scan"), qos_profile_sensor_data)
    n.create_subscription(JointState, "/joint_states", grab("/joint_states"), 10)
    n.create_subscription(Odometry, "/diff_drive_base_controller/odom",
                          grab("odom"), 10)

    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.0005)
    wall_elapsed = time.time() - t0
    n.destroy_node(); rclpy.shutdown()

    rtf = ((sim[-1] - sim[0]) / wall_elapsed) if len(sim) > 1 else None
    sim_now = sim[-1] if sim else 0.0
    wall_now = time.time()

    per_topic = {}
    for t, vals in stamps.items():
        per_topic[t] = {"n": len(vals), "last": vals[-1],
                        "clock": classify_stamp(vals[-1], sim_now, wall_now)}

    res = {"use_sim_time": params, "rtf": rtf, "sim_now": sim_now,
           "wall_now": wall_now, "stamps": per_topic}
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))

    print(f"sim clock now   {sim_now:.3f}")
    print(f"wall clock now  {wall_now:.3f}")
    print(f"RTF             {rtf:.4f}" if rtf else "RTF unavailable")
    print("use_sim_time, per node:")
    for k, v in sorted(params.items()):
        print(f"    {k:44s} {v}")
    print("header stamps, per topic:")
    for k, v in sorted(per_topic.items()):
        print(f"    {k:44s} {v['clock']:8s} last={v['last']:.3f} n={v['n']}")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
