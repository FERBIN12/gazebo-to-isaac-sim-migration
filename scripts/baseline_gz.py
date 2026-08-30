#!/usr/bin/env python3
"""a later check's measurement: does the rover actually drive, in numbers.

This exists because a robot that LOOKS fine on camera can be stuck in a corner
with its wheels spinning. Eyeballing a simulator is how that ships. So:
command a known cmd_vel, integrate the odometry, and compare against the
closed-form diff-drive prediction.

Pass conditions (declared BEFORE measuring, per G24). The controller's
`max_acceleration` means the robot does NOT travel v*t: it ramps. So the
prediction is the trapezoid, not the rectangle, and the tolerance is stated
against that:

  straight 2.0 s at 0.4 m/s, accel 1.0 m/s^2
      ramp   0.40 s covering 0.080 m
      cruise 1.60 s covering 0.640 m
      predicted 0.720 m, tolerance 25 % (command latency at 50 Hz publish costs
      the first sample or two, which is real and must not be tuned away)
  straight: |y| < 0.05 m and |yaw| < 0.05 rad -- the wheels must match
  spin     2.0 s at 1.0 rad/s, accel 2.0 rad/s^2 -> predicted 1.750 rad, tol 15 %
           (this read 1.875 for a while, which is the trapezoid for accel 4.0;
            see the comment at the spin check for why the error hid)
  wheels must actually turn: peak |wheel velocity| > 0.1 rad/s while commanded

The straight-line y and yaw bounds are the ones that catch the failure this
check exists for: a robot that drifts or spins while commanded straight.
"""
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Bench(Node):
    def __init__(self):
        super().__init__("baseline_gz")
        self.pub = self.create_publisher(TwistStamped, "/diff_drive_base_controller/cmd_vel", 10)
        self.create_subscription(Odometry, "/diff_drive_base_controller/odom", self.on_odom, 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.odom = None
        self.wheel_v = 0.0

    def on_odom(self, m):
        self.odom = m

    def on_js(self, m):
        vs = [abs(v) for n, v in zip(m.name, m.velocity)
              if n in ("left_wheel_joint", "right_wheel_joint")]
        if vs:
            self.wheel_v = max(vs)

    def wait_odom(self, timeout=30.0):
        t0 = time.time()
        while self.odom is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def drive(self, lin, ang, secs):
        """Publish continuously, sampling odom, and return (dx, dy, dyaw, max_wheel)."""
        if not self.wait_odom():
            raise RuntimeError("no odometry ever arrived")
        s = self.odom.pose.pose
        x0, y0, yaw0 = s.position.x, s.position.y, yaw_of(s.orientation)
        peak = 0.0
        t0 = time.time()
        while time.time() - t0 < secs:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.linear.x = lin
            m.twist.angular.z = ang
            self.pub.publish(m)
            rclpy.spin_once(self, timeout_sec=0.02)
            peak = max(peak, self.wheel_v)
        self.pub.publish(TwistStamped())
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.02)
        e = self.odom.pose.pose
        dxw, dyw = e.position.x - x0, e.position.y - y0
        # Project into the body frame at the START of the move. Without this a
        # robot that begins at a non-zero heading looks like it is sliding
        # sideways, which a diff-drive robot physically cannot do -- the bug is
        # then in the measurement, not the robot.
        c, s = math.cos(-yaw0), math.sin(-yaw0)
        fwd, lat = c * dxw - s * dyw, s * dxw + c * dyw
        dyaw = math.atan2(math.sin(yaw_of(e.orientation) - yaw0),
                          math.cos(yaw_of(e.orientation) - yaw0))
        return (fwd, lat, dyaw, peak)


def main():
    rclpy.init()
    n = Bench()
    fails = []
    res = {"engine": "gazebo-harmonic", "runs": {}}
    try:
        dx, dy, dyaw, peak = n.drive(0.4, 0.0, 2.0)
        exp = 0.720          # trapezoid, not 0.4*2.0
        print(f"STRAIGHT  dx={dx:+.3f} m (predict {exp:.3f})  dy={dy:+.3f}  "
              f"dyaw={dyaw:+.3f}  peak_wheel={peak:.2f} rad/s "
              f"(= {peak*0.075:.3f} m/s)")
        if peak < 0.1:
            fails.append("wheels never turned (this is the corner-stuck failure)")
        if abs(dx - exp) > 0.25 * exp:
            fails.append(f"straight travel {dx:.3f} m off predicted {exp:.3f} m by >25%")
        if abs(dy) > 0.05:
            fails.append(f"drifted sideways {dy:+.3f} m (>0.05) -- wheels mismatched")
        if abs(dyaw) > 0.05:
            fails.append(f"yawed {dyaw:+.3f} rad while commanded straight (>0.05)")
        res["runs"]["straight"] = {"dx": dx, "dy": dy, "dyaw": dyaw,
                                   "peak_wheel_m_s": peak * 0.075,
                                   "predict": exp}

        dx, dy, dyaw, peak = n.drive(0.0, 1.0, 2.0)
        # THE TRAPEZOID AT 2.0 rad/s^2 IS 1.75, NOT 1.875.
        #
        # This constant read 1.875 and its comment claimed "trapezoid at
        # 2.0 rad/s^2". It is not: 1.0 rad/s with a 2.0 rad/s^2 ramp is 0.5 s
        # of ramp covering 0.25 rad plus 1.5 s of cruise covering 1.5 rad, so
        # 1.75 rad. 1.875 is the trapezoid for an accel of 4.0, which is not
        # what controllers.yaml says (angular.z.max_acceleration: 2.0).
        #
        # It went unnoticed because the measured runs (1.871-1.891) sit close
        # to the WRONG number, which is itself the finding: Gazebo rotates
        # further than the configured ramp allows, i.e. it is barely applying
        # the angular acceleration limit. Against the correct 1.75 that is a
        # real +7% overshoot rather than a clean pass. this repo measures it.
        exp = 1.75           # trapezoid: 0.5 s ramp @ 2.0 rad/s^2, then cruise
        print(f"SPIN      dyaw={dyaw:+.3f} rad (predict {exp:.3f})  "
              f"dx={dx:+.3f}  dy={dy:+.3f}  peak_wheel={peak:.2f} rad/s")
        if peak < 0.1:
            fails.append("wheels never turned during spin")
        if abs(dyaw - exp) > 0.15 * exp:
            fails.append(f"rotation {dyaw:.3f} rad off predicted {exp:.3f} rad by >15%")
        res["runs"]["spin"] = {"dx": dx, "dy": dy, "dyaw": dyaw,
                              "peak_wheel_m_s": peak * 0.075, "predict": exp}
    finally:
        n.destroy_node()
        rclpy.shutdown()

    # WRITE THE NUMBERS OUT, not just print them. This script only ever printed,
    # so the RUN.md table was transcribed by hand and no diff script could read
    # it -- which is how this repo ended up with a "diff script" notes and no
    # machine-readable baseline on either side.
    import json as _json
    from pathlib import Path as _Path
    out = _Path(__file__).resolve().parent.parent / "data/baseline_gz.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    res["straight"] = res["runs"].get("straight")
    res["spin"] = res["runs"].get("spin")
    out.write_text(_json.dumps(res, indent=2))
    print(f"\nwrote {out}")

    if fails:
        print("\nBASELINE FAIL:")
        for f in fails:
            print("  -", f)
        return 1
    print("\nBASELINE OK: the rover drives and turns as predicted.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
