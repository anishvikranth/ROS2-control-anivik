from AVATAR import AVATARlink
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, UInt32

# Serial port and baud are ROS parameters:
#   python3 arm.py --ros-args -p port:=/dev/ttyACM1 -p baud:=115200
# The link (re)connects in the background: if the ESP32 is unplugged or the
# port disappears, commands are dropped (the ESP32 watchdog stops the motors)
# and the node keeps retrying every RECONNECT_PERIOD_S.

RECONNECT_PERIOD_S = 1.0

LAYOUT = {
    "target1": 2,
    "target2": 3,
    "target3": 4,
    "target4": 5,
    "target5": 6,
    "target6": 7,
    "input_space": 8,
    "arm_state": 9,
    "echo1": 12,
    "echo2": 13,
    "echo3": 14,
    "echo4": 15,
    "echo5": 16,
    "echo6": 17,
    "enc1": 22,
    "enc2": 23,
    "enc3": 24,
    "enc4": 25,
    "enc5": 26,
    "enc6": 27,
    "angle1": 32,
    "angle2": 33,
    "angle3": 34,
    "angle4": 35,
    "angle5": 36,
    "angle6": 37,
    "uptime_ms": 40
}


class ArmComNode(Node):
    def __init__(self):
        super().__init__('arm_com_node')
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = self.declare_parameter('baud', 115200).value

        self.target_data = [0.0] * 6
        self.last_uptime_ms = None
        self.link = None
        self.link_lock = threading.Lock()
        self.running = True

        self.target_sub = self.create_subscription(Float32MultiArray, '/arm_target_angles', self.target_callback, 10)
        self.input_space_sub = self.create_subscription(Bool, '/input_space', self.input_space_callback, 10)
        self.arm_state_sub = self.create_subscription(Bool, '/arm_state', self.arm_state_callback, 10)
        self.echo_pub = self.create_publisher(Float32MultiArray, '/arm_target_echo', 10)
        self.enc_pub = self.create_publisher(Int32MultiArray, '/enc_counts', 10)
        self.angle_pub = self.create_publisher(Float32MultiArray, '/enc_angles', 10)
        # ESP32 millis(); RoverArmHardware treats it going backwards as an ESP reset
        # (encoder counts restart from 0, so positions are no longer valid).
        self.uptime_pub = self.create_publisher(UInt32, '/arm_esp_uptime', 10)

        self.connect_thread = threading.Thread(target=self.connect_loop, daemon=True)
        self.connect_thread.start()
        self.get_logger().info(f"AVATAR arm bridge started (port {self.port}, {self.baud} baud).")

    # -- serial link management ---------------------------------------------
    def connect_loop(self):
        failed_attempts = 0
        while self.running:
            with self.link_lock:
                link = self.link
            # The AVATAR read thread exits on a serial error (e.g. USB unplugged).
            if link is not None and not link._reader.is_alive():
                self.mark_link_broken("serial read failed")
                link = None
            if link is None:
                try:
                    new_link = AVATARlink(port=self.port, baud=self.baud, layout=LAYOUT)
                    new_link.packet_callback = self.on_packet
                    with self.link_lock:
                        self.link = new_link
                    # last_uptime_ms is kept across reconnects on purpose: an ESP32
                    # reset re-enumerates its USB port, so resets usually show up
                    # right after a reconnect.
                    self.get_logger().info(f"Serial link to ESP32 OPEN on {self.port}.")
                    failed_attempts = 0
                except Exception as e:  # SerialException, OSError, ...
                    failed_attempts += 1
                    if failed_attempts == 1 or failed_attempts % 10 == 0:
                        self.get_logger().error(
                            f"Cannot open {self.port} ({e}) -- retrying every "
                            f"{RECONNECT_PERIOD_S:.0f} s (attempt {failed_attempts}).")
            time.sleep(RECONNECT_PERIOD_S)

    def mark_link_broken(self, reason):
        with self.link_lock:
            link, self.link = self.link, None
        if link is None:
            return
        self.get_logger().error(
            f"Serial link to ESP32 LOST ({reason}) -- commands dropped (the ESP32 watchdog stops "
            f"the motors); reconnecting.")
        try:
            link.close()
        except Exception:
            pass

    def send(self, fn, *args):
        with self.link_lock:
            link = self.link
        if link is None:
            return
        try:
            fn(link, *args)
        except Exception as e:
            self.mark_link_broken(str(e))

    # -- ROS -> ESP32 ---------------------------------------------------------
    def target_callback(self, msg):
        n = min(len(msg.data), 6)
        payload = {}
        for i in range(n):
            self.target_data[i] = float(msg.data[i])
            payload[f"target{i+1}"] = self.target_data[i]
        self.send(AVATARlink.send_fields, payload)

    def input_space_callback(self, msg):
        self.send(AVATARlink.send_value, 'input_space', 1 if msg.data else 0)

    def arm_state_callback(self, msg):
        self.send(AVATARlink.send_value, 'arm_state', 1 if msg.data else 0)

    # -- ESP32 -> ROS (called from the AVATAR read thread) ---------------------
    def on_packet(self, fields):
        if "uptime_ms" in fields:
            uptime = int(fields["uptime_ms"])
            if self.last_uptime_ms is None:
                self.get_logger().info(f"ESP32 telemetry received (uptime {uptime / 1000.0:.1f} s).")
            elif uptime < self.last_uptime_ms:
                self.get_logger().error(
                    f"ESP32 RESET detected (uptime {self.last_uptime_ms} ms -> {uptime} ms): "
                    "encoder counts restarted from 0.")
            self.last_uptime_ms = uptime
            uptime_msg = UInt32()
            uptime_msg.data = uptime & 0xFFFFFFFF
            self.uptime_pub.publish(uptime_msg)
        if "echo1" in fields:
            echo_msg = Float32MultiArray()
            echo_msg.data = [float(fields.get(f"echo{i}", 0.0)) for i in range(1, 7)]
            self.echo_pub.publish(echo_msg)
        if "enc1" in fields:
            enc_msg = Int32MultiArray()
            enc_msg.data = [int(fields.get(f"enc{i}", 0)) for i in range(1, 7)]
            self.enc_pub.publish(enc_msg)
        if "angle1" in fields:
            angle_msg = Float32MultiArray()
            angle_msg.data = [float(fields.get(f"angle{i}", 0.0)) for i in range(1, 7)]
            self.angle_pub.publish(angle_msg)

    def shutdown(self):
        self.running = False
        with self.link_lock:
            link, self.link = self.link, None
        if link is not None:
            link.close()


def main():
    rclpy.init()
    node = ArmComNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C / launch shutdown: Jazzy raises ExternalShutdownException
    node.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
