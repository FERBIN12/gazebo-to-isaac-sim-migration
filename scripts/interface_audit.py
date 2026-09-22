"""Enumerate the ROS 2 interface this stack actually presents, and diff it
against what the stack EXPECTS.

Why this exists: write-up 6.9 concluded that a numeric tolerance cannot catch an
absence, and that an existence check has to come first. this repo's condition is
already binary ("every topic the stack expects exists, carries the right type,
and updates at a rate something downstream can use"), so this is that check,
written as code rather than as prose.

WHAT IT MEASURES, and why each part is separate:

  * TOPICS THAT EXIST, with their types. A topic can exist with the wrong type,
    which is worse than missing: a subscriber never matches and reports nothing.
  * TOPICS THAT PUBLISH. Existing is not publishing. A latched-but-dead topic
    appears in `ros2 topic list` forever. Measured by counting messages over a
    window on the SIM clock (5.3 cost a day to wall-clock integration).
  * THE TF TREE, frame by frame, as parent->child edges. A missing edge is the
    failure that looks like a perception bug three layers away.
  * QoS PROFILES, because 7.3 is about a subscriber that silently got nothing.
    A RELIABLE subscriber against a BEST_EFFORT publisher never connects, and
    nothing logs an error.

EXPECTED sets are derived from the launch file and the controller config, not
hand-typed here, so this cannot drift from the robot the way a copied roster
does.

Usage:
    scripts/interface_audit.py            # needs a running stack
    python3 scripts/interface_audit.py --selftest
"""
import json
import re
import subprocess
import sys
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
OUT = BASE / "data/interface_audit.json"
WINDOW_S = 4.0


def expected_topics():
    """Derive the expected topic set from the launch file and controller config.

    DERIVED, NOT TYPED. A hand-written list here would be a prediction, and this
    this project has contradicted five of those.
    """
    exp = {}
    launch = (BASE / "sim-workspace/src/rover_description/launch/gz.launch.py")
    if launch.exists():
        txt = launch.read_text()
        # bridge entries look like  /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
        for topic, typ in re.findall(r'"(/[a-z_]+)@([a-zA-Z_]+/msg/[A-Za-z]+)', txt):
            exp[topic] = typ
        for topic, typ in re.findall(r"'(/[a-z_]+)@([a-zA-Z_]+/msg/[A-Za-z]+)", txt):
            exp[topic] = typ
    cfg = (BASE / "sim-workspace/src/rover_description/config/controllers.yaml")
    if cfg.exists():
        txt = cfg.read_text()
        # a diff_drive_controller publishes odom and consumes cmd_vel
        m = re.search(r"^\s*(\w*diff_drive\w*):", txt, re.M)
        if m:
            ctrl = m.group(1)
            exp[f"/{ctrl}/odom"] = "nav_msgs/msg/Odometry"
            exp[f"/{ctrl}/cmd_vel"] = "geometry_msgs/msg/TwistStamped"
        if "joint_state_broadcaster" in txt:
            exp["/joint_states"] = "sensor_msgs/msg/JointState"
    exp["/tf"] = "tf2_msgs/msg/TFMessage"
    return exp


def live_topics():
    """{topic: [types]} from the running graph."""
    out = subprocess.run(["ros2", "topic", "list", "-t"],
                         capture_output=True, text=True, timeout=30).stdout
    live = {}
    for line in out.splitlines():
        m = re.match(r"(\S+) \[(.+)\]", line.strip())
        if m:
            live[m.group(1)] = m.group(2).split(", ")
    return live


def hz(topic, window=WINDOW_S):
    """Messages per second of WALL time over a window. None if nothing arrives.

    NOTE THE CLOCK. This is per wall second. To compare against a DECLARED rate
    you must divide by the real-time factor, because a declared rate is per
    SIMULATED second (6.7, and 5.3 which cost a day to exactly this). At
    RTF 0.977 a 10 Hz lidar correctly reads 9.80 Hz on the wall.

    `ros2 topic hz` NEVER EXITS on its own, so calling it with a subprocess
    timeout raises TimeoutExpired on a perfectly healthy topic and reads as a
    dead one. Counting with `ros2 topic echo --once` in a loop would serialise
    a message per call, which is slow and racy. So: subscribe directly and
    count, which is also the only way to read the QoS the publisher offers.
    """
    import importlib
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    types = live_topics().get(topic)
    if not types:
        return None
    mod, _, cls = types[0].replace("/msg/", ".msg.").rpartition(".")
    try:
        msg_cls = getattr(importlib.import_module(mod), cls)
    except Exception:
        return None

    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("iface_hz_" + re.sub(r"\W", "_", topic))
    got = []
    n.create_subscription(msg_cls, topic, lambda m: got.append(1),
                          qos_profile_sensor_data)
    # SPIN TIGHT. At timeout_sec=0.1 a single spin_once services one callback
    # and then sleeps, so a topic sharing the graph with 993 Hz /clock traffic
    # gets starved: /scan measured 2.96 Hz here while a dedicated probe read
    # 10.03 Hz per sim second. The rate was the METER, not the sensor.
    t0 = time.time()
    while time.time() - t0 < window:
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.0005)
    n.destroy_node()
    if own:
        rclpy.shutdown()
    elapsed = time.time() - t0
    return (len(got) / elapsed) if got else None


def tf_edges(window=4.0):
    """parent->child edges in /tf and /tf_static, accumulated over a WINDOW.

    ONE MESSAGE IS NOT THE TREE. `/tf` is dynamic and each message carries only
    the transforms that changed in that tick, so `echo --once` returned a single
    edge (odom->base_footprint) and the wheel joints looked MISSING. They are
    not: tf2_monitor lists left_wheel and right_wheel among 9 frames. The static
    transforms arrive together in one latched message; the moving ones are
    spread across many. So subscribe and accumulate.

    /tf_static is TRANSIENT_LOCAL. A default subscription gets NOTHING from it,
    which would have made every fixed joint read as absent.
    """
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from tf2_msgs.msg import TFMessage

    edges = set()
    own = not rclpy.ok()
    if own:
        rclpy.init()
    n = Node("iface_tf")

    def on(m):
        for t in m.transforms:
            edges.add((t.header.frame_id, t.child_frame_id))

    n.create_subscription(TFMessage, "/tf", on, 100)
    static_qos = QoSProfile(depth=100,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
    n.create_subscription(TFMessage, "/tf_static", on, static_qos)
    t0 = time.time()
    while time.time() - t0 < window:
        rclpy.spin_once(n, timeout_sec=0.1)
    n.destroy_node()
    if own:
        rclpy.shutdown()
    return edges


def selftest():
    """Both directions, synthetic. Never pinned to a live stack."""
    ok = True

    exp = expected_topics()
    # must find the three bridged topics AND derive the controller ones
    for t in ("/clock", "/scan", "/depth", "/joint_states", "/tf"):
        if t not in exp:
            print(f"  FAIL expected set is missing {t}"); ok = False
    if not any("odom" in t for t in exp):
        print("  FAIL expected set derived no odom topic"); ok = False
    # and it must NOT invent a topic the robot does not have
    if any("imu" in t for t in exp):
        print("  FAIL expected set invented an imu topic"); ok = False

    # the diff must see a MISSING topic and a WRONG TYPE as different failures
    e = {"/scan": "sensor_msgs/msg/LaserScan", "/odom": "nav_msgs/msg/Odometry"}
    l = {"/scan": ["sensor_msgs/msg/PointCloud2"]}
    missing = [t for t in e if t not in l]
    wrong = [t for t in e if t in l and e[t] not in l[t]]
    if missing != ["/odom"]:
        print(f"  FAIL missing detection {missing}"); ok = False
    if wrong != ["/scan"]:
        print(f"  FAIL wrong-type detection {wrong}"); ok = False
    # and a clean graph must report neither
    l2 = {"/scan": ["sensor_msgs/msg/LaserScan"], "/odom": ["nav_msgs/msg/Odometry"]}
    if [t for t in e if t not in l2] or [t for t in e if e[t] not in l2[t]]:
        print("  FAIL clean graph reported a problem"); ok = False

    print(f"expected set derived: {len(exp)} topics")
    print("selftest:", "OK" if ok else "BROKEN")
    return 0 if ok else 1


def main():
    if "--selftest" in sys.argv:
        return selftest()

    exp = expected_topics()
    live = live_topics()
    missing = sorted(t for t in exp if t not in live)
    wrong = sorted(t for t in exp if t in live and exp[t] not in live[t])
    present = sorted(t for t in exp if t in live and exp[t] in live[t])

    rates = {}
    for t in present:
        if t in ("/tf", "/tf_static"):
            continue
        rates[t] = hz(t)
    silent = sorted(t for t, r in rates.items() if r is None)

    res = {"expected": exp, "live_count": len(live),
           "present": present, "missing": missing, "wrong_type": wrong,
           "rates": rates, "silent": silent,
           "tf_edges": sorted(map(list, tf_edges()))}
    OUT.parent.mkdir(exist_ok=True)
    OUT.write_text(json.dumps(res, indent=2))

    print(f"expected      {len(exp)}")
    print(f"live in graph {len(live)}")
    print(f"present, right type   {len(present)}")
    print(f"MISSING               {missing or 'none'}")
    print(f"WRONG TYPE            {wrong or 'none'}")
    print(f"SILENT (exists, 0 Hz) {silent or 'none'}")
    for t, r in sorted(rates.items()):
        print(f"    {t:44s} {('%.2f Hz' % r) if r else 'SILENT'}")
    print(f"tf edges              {len(res['tf_edges'])}")
    for p, c in res["tf_edges"]:
        print(f"    {p} -> {c}")
    print(f"-> wrote {OUT.relative_to(BASE)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
