#!/usr/bin/env python3
"""Power-on bring-up check for the rover arm, run BEFORE ros2_control.

Talks straight to the AVATAR bridge (arm.py): PWM out on /arm_target_angles,
encoder angles in on /enc_angles. Nothing else may be publishing PWM, so start
only arm.py (or, to practise, arm_real_sim.launch.py real_stack:=false).

Per motor it finds:
  * encoder data rate
  * deadband   -- slow PWM ramp in each direction until the encoder shows motion;
                  the sign of that motion shows whether encoder and motor agree
  * top speed  -- short run at a fixed PWM -> suggested ff_gain
and for the differential wrist it checks that both bevels together give pitch
and opposite bevels give roll.

Safety: PWM capped at --max-pwm, every move stops after --max-travel-deg and is
reversed, Ctrl-C sends 0 PWM, and interactive mode asks before each motor.
Lift the arm / clear its range and keep the kill switch in reach.

  ros2 run rover_arm_description arm_bringup.py                  # interactive, all motors
  ros2 run rover_arm_description arm_bringup.py --motors base,elbow
  ros2 run rover_arm_description arm_bringup.py --yes            # no questions (sim / repeat runs)
"""
import argparse
import datetime
import math
import os
import statistics
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, UInt32

MOTORS = ["base", "shoulder", "elbow", "bevel1", "bevel2", "gripper"]
NUM = len(MOTORS)
PWM_MAX = 255.0
PUBLISH_HZ = 50.0
MOVING_DEG_S = 1.0  # encoder speed that counts as "the motor moved"


class Abort(Exception):
    pass


class Bringup(Node):
    def __init__(self, args):
        super().__init__("arm_bringup")
        self.args = args
        self.lock = threading.Lock()
        self.angles = None          # degrees, latest /enc_angles
        self.angle_times = []       # arrival times (rate measurement)
        self.uptime = None
        self.pwm_pub = self.create_publisher(Float32MultiArray, "/arm_target_angles", 10)
        self.create_subscription(Float32MultiArray, "/enc_angles", self.on_angles, 10)
        self.create_subscription(UInt32, "/arm_esp_uptime", self.on_uptime, 10)
        self.report = {"motors": {}, "wrist": {}, "warnings": []}

    # -- I/O ---------------------------------------------------------------
    def on_angles(self, msg):
        if len(msg.data) < NUM:
            return
        with self.lock:
            self.angles = list(msg.data[:NUM])
            self.angle_times.append(time.monotonic())
            self.angle_times = self.angle_times[-200:]

    def on_uptime(self, msg):
        with self.lock:
            if self.uptime is not None and msg.data < self.uptime:
                self.report["warnings"].append("ESP32 reset during the run (uptime went backwards)")
                self.get_logger().error("ESP32 RESET detected during bring-up -- results after this are suspect.")
            self.uptime = msg.data

    def read(self):
        with self.lock:
            return None if self.angles is None else list(self.angles)

    def send(self, pwm):
        self.pwm_pub.publish(Float32MultiArray(data=[float(p) for p in pwm]))

    def stop(self, secs=0.3):
        self.drive([0.0] * NUM, secs)

    def drive(self, pwm, secs, watch=None, start=None):
        """Publish pwm for secs. If watch (motor index) is given, stop early once it has
        travelled max_travel_deg from start. Returns True if stopped by the travel limit."""
        end = time.monotonic() + secs
        while time.monotonic() < end:
            self.send(pwm)
            if watch is not None:
                a = self.read()
                if a is not None and abs(a[watch] - start) >= self.args.max_travel_deg:
                    return True
            time.sleep(1.0 / PUBLISH_HZ)
        return False

    # -- helpers ---------------------------------------------------------------
    def ask(self, question):
        if self.args.yes or not sys.stdin.isatty():
            return None
        while True:
            ans = input(f"    {question} [y/n/s=skip] ").strip().lower()
            if ans in ("y", "n", "s"):
                return ans

    def speed(self, m, secs=0.3):
        """Average encoder speed of motor m (deg/s) over the next secs while holding the current PWM."""
        a0, t0 = self.read()[m], time.monotonic()
        time.sleep(secs)
        return (self.read()[m] - a0) / (time.monotonic() - t0)

    # -- checks -----------------------------------------------------------------
    def check_prerequisites(self):
        others = [p for p in self.get_publishers_info_by_topic("/arm_target_angles")
                  if p.node_name != self.get_name()]
        if others:
            names = ", ".join(sorted({p.node_name for p in others}))
            raise Abort(f"Something else is publishing PWM on /arm_target_angles ({names}). "
                        "Stop arm_real.launch.py / RoverArmHardware first -- only arm.py may run.")
        deadline = time.monotonic() + 5.0
        while self.read() is None and time.monotonic() < deadline:
            self.send([0.0] * NUM)  # wakes firmware that only streams while commanded
            time.sleep(0.05)
        if self.read() is None:
            raise Abort("No /enc_angles within 5 s. Is arm.py running and the ESP32 connected?")
        self.stop(1.0)
        with self.lock:
            ts = list(self.angle_times)
        rate = (len(ts) - 1) / (ts[-1] - ts[0]) if len(ts) > 2 and ts[-1] > ts[0] else 0.0
        self.report["encoder_rate_hz"] = round(rate, 1)
        print(f"Encoders: /enc_angles at {rate:.0f} Hz. Start angles (deg): "
              + ", ".join(f"{n}={a:.1f}" for n, a in zip(MOTORS, self.read())))
        if rate < 50:
            self.report["warnings"].append(f"encoder rate only {rate:.0f} Hz (firmware streams 100 Hz)")
            print("  WARNING: below the firmware's 100 Hz -- check the serial link / firmware version.")

    def find_deadband(self, m, sign):
        """Ramp PWM in direction sign until motor m moves. Returns (pwm, encoder_sign, travel_hit)."""
        start = self.read()[m]
        pwm_val = 0.0
        while pwm_val < self.args.max_pwm:
            pwm_val = min(pwm_val + self.args.ramp_step, self.args.max_pwm)
            cmd = [0.0] * NUM
            cmd[m] = sign * pwm_val
            hit = self.drive(cmd, self.args.ramp_dwell, watch=m, start=start)
            v = self.speed(m, 0.15) if not hit else (self.read()[m] - start)
            if hit or abs(v) >= MOVING_DEG_S:
                moved = self.read()[m] - start
                self.stop()
                return pwm_val, (1 if moved > 0 else -1) * sign, hit
        self.stop()
        return None, 0, False

    def measure_speed(self, m, sign, pwm_val):
        """Run motor m at sign*pwm_val; returns |deg/s| or None. Stops at the travel limit
        and measures over whatever window it got (fast joints hit the limit sooner)."""
        start = self.read()[m]
        cmd = [0.0] * NUM
        cmd[m] = sign * pwm_val
        hit = self.drive(cmd, 0.2, watch=m, start=start)             # let it get up to speed
        a0, t0 = self.read()[m], time.monotonic()
        if not hit:
            self.drive(cmd, self.args.speed_secs, watch=m, start=start)
        a1, t1 = self.read()[m], time.monotonic()
        self.stop()
        if t1 - t0 < 0.12:
            return None
        v = abs(a1 - a0) / (t1 - t0)
        return v if v > MOVING_DEG_S else None

    def test_motor(self, m):
        name = MOTORS[m]
        print(f"\n=== {name} (motor {m}) ===")
        if self.ask(f"Ready to move {name}? Small moves (<= {self.args.max_travel_deg:.0f} deg) both ways.") in ("n", "s"):
            print("    skipped")
            return
        res = {}
        db_pos, enc_sign_pos, _ = self.find_deadband(m, +1)
        physical = self.ask(f"Did {name} move in its URDF-POSITIVE direction just now?") if db_pos else None
        db_neg, enc_sign_neg, _ = self.find_deadband(m, -1)

        if db_pos is None and db_neg is None:
            res["status"] = "NO MOTION"
            print(f"    NO MOTION detected up to PWM {self.args.max_pwm:.0f}. Motor not connected, deadband "
                  f"above --max-pwm, or encoder not counting (did the joint physically move?).")
            self.report["motors"][name] = res
            return
        res["deadband_pos"], res["deadband_neg"] = db_pos, db_neg
        print(f"    deadband: +{db_pos if db_pos else '>max'} / -{db_neg if db_neg else '>max'} PWM")

        enc_agrees = (enc_sign_pos or enc_sign_neg) > 0
        res["encoder_matches_pwm"] = enc_agrees
        if physical == "y":
            res["motor_direction_ok"] = True
        elif physical == "n":
            res["motor_direction_ok"] = False
        print(f"    encoder vs PWM: {'OK (positive PWM -> angle increases)' if enc_agrees else 'REVERSED (positive PWM -> angle DECREASES)'}")

        # Speed at two PWM levels, each both ways (so the joint returns roughly to start).
        # A line through (pwm, speed) gives the true deadband (where speed hits 0) and the
        # slope, independent of the ramp's step size.
        db_ramp = max(d for d in (db_pos, db_neg) if d)
        p1 = min(db_ramp + self.args.speed_margin / 2, self.args.max_pwm)
        p2 = min(db_ramp + self.args.speed_margin * 1.5, self.args.max_pwm)
        points = []
        for p in sorted({p1, p2}):
            vs = [v for v in (self.measure_speed(m, +1, p), self.measure_speed(m, -1, p)) if v]
            if vs:
                points.append((p, statistics.mean(vs)))
        db_est = db_ramp - self.args.ramp_step / 2  # fallback: middle of the last ramp step
        slope = None
        if len(points) == 2 and points[1][1] > points[0][1]:
            (q1, v1), (q2, v2) = points
            slope = (v2 - v1) / (q2 - q1)                     # deg/s per PWM
            db_est = min(max(q1 - v1 / slope, 0.0), db_ramp)
        elif points:
            q1, v1 = points[0]
            if q1 > db_est:
                slope = v1 / (q1 - db_est)
        if slope:
            v_max_rad = math.radians(slope * (PWM_MAX - db_est))
            res.update(speed_points=[[round(q), round(v, 2)] for q, v in points],
                       deadband_fit=round(db_est, 1), est_full_pwm_rad_s=round(v_max_rad, 3),
                       ff_gain=round(PWM_MAX / v_max_rad, 1))
            print("    speed: " + ", ".join(f"{v:.1f} deg/s at PWM {q:.0f}" for q, v in points)
                  + f" -> deadband ~{db_est:.0f}, ~{v_max_rad:.2f} rad/s at full PWM -> ff_gain ~ {PWM_MAX / v_max_rad:.0f}")
        else:
            print("    speed: not measurable (moved too little at the test PWMs)")
        self.report["motors"][name] = res

    def test_wrist(self):
        print("\n=== differential wrist (bevel1 + bevel2) ===")
        if self.ask("Ready to move the wrist? (pitch, then roll)") in ("n", "s"):
            print("    skipped")
            return
        mb = self.report["motors"]
        db = max([mb.get(b, {}).get(k) or 0 for b in ("bevel1", "bevel2") for k in ("deadband_pos", "deadband_neg")]
                 or [0])
        pwm_val = min(db + self.args.speed_margin, self.args.max_pwm)
        for label, s1, s2 in (("pitch", +1, +1), ("roll", +1, -1)):
            start = self.read()
            cmd = [0.0] * NUM
            cmd[3], cmd[4] = s1 * pwm_val, s2 * pwm_val
            self.drive(cmd, self.args.speed_secs, watch=3, start=start[3])
            mid = self.read()
            self.stop()
            d1, d2 = mid[3] - start[3], mid[4] - start[4]
            back = [-c for c in cmd]
            self.drive(back, self.args.speed_secs, watch=3, start=mid[3])
            self.stop()
            same_sign = d1 * d2 > 0
            ok = (same_sign if label == "pitch" else not same_sign) and abs(d1) > 0.5 and abs(d2) > 0.5
            balanced = min(abs(d1), abs(d2)) / max(abs(d1), abs(d2), 1e-6)
            physical = self.ask(f"Did the wrist {label.upper()} (and nothing else)?")
            self.report["wrist"][label] = {"bevel1_deg": round(d1, 2), "bevel2_deg": round(d2, 2),
                                           "encoders_consistent": ok, "balance": round(balanced, 2),
                                           "operator_ok": None if physical is None else physical == "y"}
            print(f"    {label}: bevel1 {d1:+.1f} deg, bevel2 {d2:+.1f} deg -> "
                  f"{'OK' if ok else 'UNEXPECTED (check bevel motor/encoder directions)'}"
                  f"{'' if balanced > 0.7 else f' -- unbalanced ({balanced:.0%}), check gearing/calibration'}")

    # -- summary ------------------------------------------------------------------
    def summarise(self):
        print("\n================ SUMMARY ================")
        enc_dir = []
        dir_notes = []
        for name in MOTORS:
            r = self.report["motors"].get(name)
            if not r or "encoder_matches_pwm" not in r:
                enc_dir.append("?")
                continue
            motor_ok = r.get("motor_direction_ok")
            enc_ok = r["encoder_matches_pwm"]
            if motor_ok is False:
                dir_notes.append(f"{name}: motor turns the WRONG way -> flip dir_dir[] for {name}; "
                                 "re-run (the encoder sign may change with it)")
            enc_dir.append("1" if enc_ok else "-1")
        if any(e == "-1" for e in enc_dir):
            print("arm.c enc_dir[] should be: {" + ", ".join(e if e != "?" else "1" for e in enc_dir) + "}"
                  "   (base, shoulder, elbow, bevel1, bevel2, gripper)")
            if "?" in enc_dir:
                print("  (untested: " + ", ".join(n for n, e in zip(MOTORS, enc_dir) if e == "?")
                      + " -- shown as 1, run them too)")
        else:
            print("Encoder directions: all agree with PWM" + (" (some motors untested)" if "?" in enc_dir else ""))
        for n in dir_notes:
            print("  " + n)
        dbs = [r["deadband_fit"] if "deadband_fit" in r else max(r.get("deadband_pos") or 0, r.get("deadband_neg") or 0)
               for r in self.report["motors"].values() if r.get("deadband_pos") or r.get("deadband_neg")]
        ffs = [r["ff_gain"] for r in self.report["motors"].values() if "ff_gain" in r]
        if dbs:
            print(f"pwm_deadband: per-motor {min(dbs):.0f}..{max(dbs):.0f} -> suggest pwm_deadband:={statistics.median(dbs):.0f}"
                  "  (one value for all motors; use the lowest if any joint overshoots at slow speed)")
        if ffs:
            print(f"ff_gain: per-motor {min(ffs):.0f}..{max(ffs):.0f} -> suggest ff_gain:={statistics.median(ffs):.0f}")
        if dbs or ffs:
            print("Try them:  ros2 launch rover_arm_description arm_real.launch.py "
                  + (f"pwm_deadband:={statistics.median(dbs):.0f} " if dbs else "")
                  + (f"ff_gain:={statistics.median(ffs):.0f}" if ffs else ""))
        for w in self.report["warnings"]:
            print("WARNING: " + w)
        self.report["suggested"] = {"enc_dir": enc_dir,
                                    "pwm_deadband": statistics.median(dbs) if dbs else None,
                                    "ff_gain": statistics.median(ffs) if ffs else None}
        self.save_report()

    def save_report(self):
        out_dir = os.path.expanduser(self.args.report_dir)
        os.makedirs(out_dir, exist_ok=True)
        path = os.path.join(out_dir, datetime.datetime.now().strftime("bringup_%Y%m%d_%H%M%S.yaml"))
        with open(path, "w") as f:
            f.write(to_yaml(self.report))
        print(f"Report saved: {path}")


def to_yaml(obj, indent=0):
    pad = "  " * indent
    if isinstance(obj, dict):
        lines = []
        for k, v in obj.items():
            if isinstance(v, (dict, list)) and v:
                lines.append(f"{pad}{k}:\n{to_yaml(v, indent + 1)}")
            else:
                lines.append(f"{pad}{k}: {to_yaml(v) if not isinstance(v, (dict, list)) else ('{}' if isinstance(v, dict) else '[]')}")
        return "\n".join(lines) + ("\n" if indent == 0 else "")
    if isinstance(obj, list):
        return "\n".join(f"{pad}- {to_yaml(v) if not isinstance(v, (dict, list)) else ''}" for v in obj)
    if obj is None:
        return "null"
    if isinstance(obj, bool):
        return "true" if obj else "false"
    return str(obj)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--motors", default=",".join(MOTORS), help="comma-separated subset of: " + ",".join(MOTORS))
    parser.add_argument("--max-pwm", type=float, default=120.0, help="never command more than this (default 120)")
    parser.add_argument("--max-travel-deg", type=float, default=10.0, help="stop any move after this travel (default 10)")
    parser.add_argument("--ramp-step", type=float, default=5.0, help="PWM step of the deadband ramp (default 5)")
    parser.add_argument("--ramp-dwell", type=float, default=0.25, help="seconds per ramp step (default 0.25)")
    parser.add_argument("--speed-margin", type=float, default=40.0,
                        help="speed tests at deadband + margin/2 and + 1.5*margin (default 40)")
    parser.add_argument("--speed-secs", type=float, default=0.5, help="speed test duration (default 0.5)")
    parser.add_argument("--no-wrist", action="store_true", help="skip the pitch/roll check")
    parser.add_argument("--yes", action="store_true", help="don't ask anything (practise runs in sim)")
    parser.add_argument("--report-dir", default="~/arm_bringup_reports")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    selected = [m.strip() for m in args.motors.split(",") if m.strip()]
    bad = [m for m in selected if m not in MOTORS]
    if bad:
        parser.error(f"unknown motor(s): {', '.join(bad)}")

    rclpy.init()
    node = Bringup(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    print("Rover arm bring-up. Lift the arm / clear its range, kill switch in reach. Ctrl-C stops everything.")
    code = 0
    try:
        time.sleep(1.0)  # discovery
        node.check_prerequisites()
        for name in selected:
            node.test_motor(MOTORS.index(name))
        if not args.no_wrist and "bevel1" in selected and "bevel2" in selected:
            node.test_wrist()
        node.summarise()
    except Abort as e:
        print(f"\nABORTED: {e}")
        code = 1
    except KeyboardInterrupt:
        print("\nInterrupted -- sending 0 PWM.")
        code = 130
    finally:
        for _ in range(10):
            node.send([0.0] * NUM)
            time.sleep(0.02)
        # Stop the spin thread before tearing down, or rclpy aborts on exit.
        executor.shutdown()
        spin.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
