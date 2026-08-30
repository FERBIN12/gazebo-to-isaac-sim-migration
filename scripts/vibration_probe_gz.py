"""The Gazebo half of the vibration probe. Same manoeuvre, same statistics.

scripts/vibration_probe.py found a genuine 85-130 Hz oscillation in the Isaac
wheel velocity. That is only half a finding. a later check of this project exists
because I once reported a Gazebo-vs-Isaac divergence having measured only the
Isaac side, and the two engines turned out to agree. So this runs the same 4 s
constant-spin command against Gazebo, samples /joint_states, and feeds the
result through the SAME analyse() function.

Reads the joint velocity from ROS rather than from the engine directly, which
is a real difference from the Isaac side worth stating: /joint_states is
published at the controller_manager's update_rate (100 Hz here), while the
Isaac probe samples every physics step (200 Hz). A 130 Hz oscillation is above
the Nyquist frequency of a 100 Hz sampler, so THIS SCRIPT CANNOT SEE IT even
if it is there. That is exactly why the script says so out loud rather than
reporting "steady".

Usage:
    scripts/vibration_probe_gz.py            # needs a running gz sim
"""
import json
import math
import subprocess
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
OUT = BASE / "data/vibration_probe_gz.json"
sys.path.insert(0, str(BASE / "scripts"))

DT_PUBLISH = 1.0 / 100.0     # controller_manager update_rate from controllers.yaml
SECS = 4.0
SETTLE_S = 1.5


ISAAC_BAND_LO_HZ = 85.0    # the Isaac oscillation sits at 85-130 Hz


def can_see_isaac_band(nyquist_hz):
    """Can a sampler with this Nyquist frequency resolve the Isaac band at all?"""
    return nyquist_hz >= ISAAC_BAND_LO_HZ


def nyquist_verdict(nyquist_hz):
    """'comparable', or an explicit INCONCLUSIVE.

    The whole point of this probe: a 'steady' reading below Nyquist does NOT
    mean Gazebo is steady, it means the instrument cannot answer. So it must
    REFUSE rather than report. See [[feedback_actuator_lag_above_nyquist_is_a_noop]].
    """
    if can_see_isaac_band(nyquist_hz):
        return "comparable"
    return ("INCONCLUSIVE: sampled below Nyquist for the "
            f"{ISAAC_BAND_LO_HZ:.0f}-130 Hz band")


def _selftest():
    """Assert BOTH directions, and that the refusal is not silent."""
    ok = True
    cases = [("50 Hz nyquist (100 Hz sampler)",  50.0, False),
             ("84.9 Hz, just under",             84.9, False),
             ("85 Hz, exactly at the band",      85.0, True),
             ("500 Hz, plenty",                 500.0, True)]
    for name, nq, want_see in cases:
        got = can_see_isaac_band(nq)
        v = nyquist_verdict(nq)
        good = (got is want_see) and (
            v == "comparable" if want_see else v.startswith("INCONCLUSIVE"))
        ok &= good
        print(f"  selftest {name:32s} see={got!s:5s} verdict={v[:34]:34s} "
              f"{'ok' if good else '*** FAIL'}")
    return 0 if ok else 1


def main():
    # Import analyse() from the Isaac probe so both sides use identical
    # statistics. Diverging the analysis between the two halves of an A/B is
    # the same class of error as not running one half.
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "vp", BASE / "scripts/vibration_probe.py")
    vp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(vp)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import TwistStamped

    rclpy.init()
    node = Node("vibration_probe_gz")
    samples = []
    t_start = [None]

    pub = node.create_publisher(
        TwistStamped, "/diff_drive_base_controller/cmd_vel", 10)

    def on_js(msg):
        if "right_wheel_joint" not in msg.name:
            return
        i = msg.name.index("right_wheel_joint")
        if i < len(msg.velocity):
            if t_start[0] is None:
                t_start[0] = time.time()
            if time.time() - t_start[0] >= SETTLE_S:
                samples.append(abs(float(msg.velocity[i])))

    node.create_subscription(JointState, "/joint_states", on_js, 50)

    def tick():
        m = TwistStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.twist.angular.z = 1.0
        pub.publish(m)

    node.create_timer(0.02, tick)

    t0 = time.time()
    while time.time() - t0 < SECS + SETTLE_S + 1.0:
        rclpy.spin_once(node, timeout_sec=0.01)

    stop = TwistStamped()
    stop.header.stamp = node.get_clock().now().to_msg()
    pub.publish(stop)
    rclpy.spin_once(node, timeout_sec=0.1)

    if len(samples) < 20:
        print(f"REFUSING: only {len(samples)} joint_state samples. "
              f"Is the sim running with controllers active?", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(2)

    stats = vp.analyse(samples, DT_PUBLISH)
    nyquist = 0.5 / DT_PUBLISH
    result = {
        "engine": "gazebo-dart",
        "sample_rate_hz": 1.0 / DT_PUBLISH,
        "nyquist_hz": nyquist,
        "secs": SECS, "settle_s": SETTLE_S,
        "wheel_velocity": stats,
        # THE IMPORTANT FIELD. The Isaac oscillation sits at 85-130 Hz, above
        # this sampler's 50 Hz Nyquist. A "steady" result here does NOT mean
        # Gazebo is steady; it means this instrument cannot answer.
        "can_see_isaac_band": can_see_isaac_band(nyquist),
        "verdict": nyquist_verdict(nyquist),
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))
    print(f"  samples        {stats['n']} at {1.0/DT_PUBLISH:.0f} Hz")
    print(f"  wheel vel      mean {stats['mean']:.4f}  p2p {stats['peak_to_peak']:.5f}")
    print(f"  reversals      {stats['reversals']} ({stats['reversal_rate_hz']} Hz)")
    print(f"  oscillates     {stats['oscillates']}")
    print(f"  nyquist        {nyquist:.0f} Hz")
    print(f"  VERDICT        {result['verdict']}")
    print(f"  -> {OUT.relative_to(BASE)}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    main()
