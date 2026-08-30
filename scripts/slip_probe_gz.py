"""The Gazebo half of the slip measurement. Same formula, same definition.

`slip_probe.py` measured 11.3 % slip in Isaac. On its own that is half a
finding: 11.3 % is only meaningful next to whatever Gazebo does, and a later check
of this project exists because I once reported a divergence having measured one
side. This runs the same 2 s spin against Gazebo and computes slip with the
SAME function, imported from the Isaac probe so the arithmetic cannot drift.

Reads the swept wheel angle by integrating `/joint_states` velocity and the yaw
from `/diff_drive_base_controller/odom`, which is the closest available match to
what the Isaac probe reads out of PhysX directly.

Usage:
    scripts/slip_probe_gz.py        # needs a running gz sim with controllers
"""
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

BASE = Path(__file__).resolve().parent.parent
OUT = BASE / "data/slip_probe_gz.json"

WHEEL_R = 0.075
TRACK = 0.30
SECS = 2.0
CMD_W = 1.0


THEORETICAL_SWEEP = 3.501   # rad, closed form for the commanded turn
SWEEP_TOL = 0.05            # 5% -- see the note in main() on why this REFUSES


def judge_sweep(swept):
    """Return (error_fraction, trustworthy) for a measured wheel sweep.

    Extracted from main() so it can be selftested without a simulator. The
    instrument REFUSES rather than reports: two runs of the identical command
    gave slip 0.372 and 0.937, and a 0.57 spread cannot resolve an 0.11
    measurement. The closed-form sweep is what exposes that.
    """
    err = abs(swept - THEORETICAL_SWEEP) / THEORETICAL_SWEEP
    return err, err <= SWEEP_TOL


def _selftest():
    """Assert BOTH directions: a good sweep is trusted, a bad one is refused."""
    ok = True
    cases = [("exact",        THEORETICAL_SWEEP,        True),
             ("within 4%",    THEORETICAL_SWEEP * 1.04, True),
             ("6% high",      THEORETICAL_SWEEP * 1.06, False),
             ("30% low",      THEORETICAL_SWEEP * 0.70, False)]
    for name, swept, want in cases:
        err, got = judge_sweep(swept)
        good = got is want
        ok &= good
        print(f"  selftest {name:12s} swept={swept:6.3f} err={err:6.1%} "
              f"trustworthy={got!s:5s} want={want!s:5s} "
              f"{'ok' if good else '*** FAIL'}")
    return 0 if ok else 1


def main():
    spec = importlib.util.spec_from_file_location(
        "sp", BASE / "scripts/slip_probe.py")
    sp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(sp)

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped

    rclpy.init()
    n = Node("slip_probe_gz")
    state = {"swept": 0.0, "t_last": None, "yaw0": None, "yaw1": None}

    def yaw_of(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y ** 2 + q.z ** 2))

    def on_js(msg):
        if "right_wheel_joint" not in msg.name:
            return
        i = msg.name.index("right_wheel_joint")
        if i >= len(msg.velocity):
            return
        # INTEGRATE ON THE SIM CLOCK, NOT THE WALL CLOCK. The first version
        # used time.time(), which counts real seconds while the simulator
        # counts its own. At the RTF this machine achieves that over-counted
        # the swept angle by 2.6x (9.237 rad against a theoretical 3.501) and
        # produced an absurd 93.7 % slip. The message stamp is sim time
        # because everything here runs with use_sim_time.
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if state["t_last"] is not None and state["yaw0"] is not None:
            dt = now - state["t_last"]
            # Guard against a stale or duplicated stamp.
            if 0.0 < dt < 0.5:
                state["swept"] += abs(float(msg.velocity[i])) * dt
        state["t_last"] = now

    def on_odom(msg):
        y = yaw_of(msg.pose.pose.orientation)
        if state["yaw0"] is None:
            state["yaw0"] = y
        state["yaw1"] = y

    n.create_subscription(JointState, "/joint_states", on_js, 50)
    n.create_subscription(Odometry, "/diff_drive_base_controller/odom", on_odom, 50)
    pub = n.create_publisher(
        TwistStamped, "/diff_drive_base_controller/cmd_vel", 10)

    # Let the subscriptions connect and the odom baseline land before commanding.
    t0 = time.time()
    while time.time() - t0 < 1.5:
        rclpy.spin_once(n, timeout_sec=0.01)
    if state["yaw0"] is None:
        sys.exit("REFUSING: no odometry. Is the sim running with controllers?")
    state["swept"] = 0.0

    def tick():
        m = TwistStamped()
        m.header.stamp = n.get_clock().now().to_msg()
        m.twist.angular.z = CMD_W
        pub.publish(m)

    # Run the command for SECS of SIM time. Spinning for SECS of wall time
    # would command a different number of simulated seconds on every machine.
    timer = n.create_timer(0.02, tick)
    sim_t0 = state["t_last"]
    guard = time.time()
    while True:
        rclpy.spin_once(n, timeout_sec=0.005)
        if state["t_last"] is not None and sim_t0 is not None \
                and state["t_last"] - sim_t0 >= SECS:
            break
        if time.time() - guard > SECS * 20 + 30:
            print("  WARNING: wall-clock guard fired; sim clock may be stalled",
                  file=sys.stderr)
            break
    # STOP INTEGRATING THE MOMENT THE COMMAND ENDS. The first version kept
    # accumulating while the stop message propagated and the robot coasted,
    # which stretched a 2.00 s window into 2.90 s of measured sweep (5.07 rad
    # against a theoretical 3.50). Freeze the accumulator, THEN stop the robot.
    n.destroy_timer(timer)
    swept_at_stop = state["swept"]
    yaw_at_stop = state["yaw1"]
    stop = TwistStamped()
    stop.header.stamp = n.get_clock().now().to_msg()
    pub.publish(stop)
    for _ in range(50):
        rclpy.spin_once(n, timeout_sec=0.005)
    state["swept"] = swept_at_stop
    state["yaw1"] = yaw_at_stop

    swept = state["swept"]
    dyaw = abs(math.atan2(math.sin(state["yaw1"] - state["yaw0"]),
                          math.cos(state["yaw1"] - state["yaw0"])))
    rim = swept * WHEEL_R
    ground = dyaw * (TRACK / 2.0)
    slip = sp.slip_fraction(rim, ground)

    # SANITY-CHECK THE SWEEP AGAINST THEORY BEFORE REPORTING SLIP.
    #
    # A 2 s ramped spin at 1.0 rad/s sweeps 3.501 rad of wheel, which is pure
    # arithmetic and does not depend on the engine. The Isaac probe reads 3.498.
    # This ROS-topic instrument has read 9.237, 5.384, 5.072 and 6.941 on four
    # runs of the identical command, giving slip anywhere from 0.372 to 0.937.
    #
    # A 0.57 spread cannot resolve an 0.11 measurement, so reporting the number
    # would be worse than reporting nothing. The cause is that /joint_states
    # sampling, sim-time stamps and command-window edges all interact, and none
    # of them is under this script's control the way PhysX is on the Isaac side.
    err, trustworthy = judge_sweep(swept)
    result = {"engine": "gazebo-dart", "wheel_angle_swept_rad": round(swept, 4),
              "theoretical_sweep_rad": THEORETICAL_SWEEP,
              "sweep_error_fraction": round(err, 4),
              "rim_travel_m": round(rim, 5), "dyaw_rad": round(dyaw, 4),
              "ground_travel_m": round(ground, 5),
              "slip_fraction": (round(slip, 4)
                                if (slip is not None and trustworthy) else None),
              "instrument_trustworthy": trustworthy,
              "verdict": ("usable" if trustworthy else
                          "REJECTED: swept angle disagrees with theory by "
                          f"{err:.0%}; this instrument cannot resolve slip")}
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(result, indent=2))
    print(f"  wheel swept    {swept:.4f} rad")
    print(f"  rim travel     {rim:.5f} m")
    print(f"  dyaw           {dyaw:.4f} rad")
    print(f"  ground travel  {ground:.5f} m")
    if trustworthy and slip is not None:
        print(f"  SLIP FRACTION  {slip:.4f}")
    else:
        print(f"  SLIP           REJECTED -- swept {swept:.3f} rad vs a "
              f"theoretical {THEORETICAL_SWEEP:.3f} ({err:.0%} off)")
        print(f"                 the instrument, not the simulator, is at fault")
    print(f"  -> {OUT.relative_to(BASE)}")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    main()
