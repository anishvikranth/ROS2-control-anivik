from AVATAR import AVATARlink
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray, Bool, UInt32

target_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
input_space = False
arm_state = False
node = None
last_uptime_ms = None

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

link = AVATARlink(port="/dev/ttyACM0", layout=LAYOUT)


class ArmComNode(Node):
    def __init__(self):
        super().__init__('arm_com_node')
        self.target_sub = self.create_subscription(Float32MultiArray, '/arm_target_angles', self.target_callback, 10)
        self.input_space_sub = self.create_subscription(Bool, '/input_space', self.input_space_callback, 10)
        self.arm_state_sub = self.create_subscription(Bool, '/arm_state', self.arm_state_callback, 10)
        self.echo_pub = self.create_publisher(Float32MultiArray, '/arm_target_echo', 10)
        self.enc_pub = self.create_publisher(Int32MultiArray, '/enc_counts', 10)
        self.angle_pub = self.create_publisher(Float32MultiArray, '/enc_angles', 10)
        # ESP32 millis(); RoverArmHardware treats it going backwards as an ESP reset
        # (encoder counts restart from 0, so positions are no longer valid).
        self.uptime_pub = self.create_publisher(UInt32, '/arm_esp_uptime', 10)
        self.get_logger().info("AVATAR arm bridge ready.")

    def target_callback(self, msg):
        global target_data
        n = min(len(msg.data), 6)
        payload = {}
        for i in range(n):
            target_data[i] = float(msg.data[i])
            payload[f"target{i+1}"] = target_data[i]
        link.send_fields(payload)

    def input_space_callback(self, msg):
        global input_space
        input_space = 1 if msg.data else 0
        link.send_value('input_space', input_space)

    def arm_state_callback(self, msg):
        global arm_state
        arm_state = 1 if msg.data else 0
        link.send_value('arm_state', arm_state)


def on_packet(fields):
    global last_uptime_ms
    if node is None:
        return
    if "uptime_ms" in fields:
        uptime = int(fields["uptime_ms"])
        if last_uptime_ms is None:
            node.get_logger().info(f"ESP32 telemetry received (uptime {uptime / 1000.0:.1f} s).")
        elif uptime < last_uptime_ms:
            node.get_logger().error(
                f"ESP32 RESET detected (uptime {last_uptime_ms} ms -> {uptime} ms): "
                "encoder counts restarted from 0.")
        last_uptime_ms = uptime
        uptime_msg = UInt32()
        uptime_msg.data = uptime & 0xFFFFFFFF
        node.uptime_pub.publish(uptime_msg)
    if "echo1" in fields:
        echo_msg = Float32MultiArray()
        echo_msg.data = [float(fields.get(f"echo{i}", 0.0)) for i in range(1, 7)]
        node.echo_pub.publish(echo_msg)
    if "enc1" in fields:
        enc_msg = Int32MultiArray()
        enc_msg.data = [int(fields.get(f"enc{i}", 0)) for i in range(1, 7)]
        node.enc_pub.publish(enc_msg)
    if "angle1" in fields:
        angle_msg = Float32MultiArray()
        angle_msg.data = [float(fields.get(f"angle{i}", 0.0)) for i in range(1, 7)]
        node.angle_pub.publish(angle_msg)


link.packet_callback = on_packet


def main():
    global node
    rclpy.init()
    node = ArmComNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    link.close()


if __name__ == '__main__':
    main()
