import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import json


class OrcaJointCommandRelay(Node):
    def __init__(self):
        super().__init__('orca_joint_command_relay')

        self.joint_names = [
            'left_thumb_mcp', 'left_thumb_abd', 'left_thumb_pip', 'left_thumb_dip',
            'left_index_abd', 'left_index_mcp', 'left_index_pip',
            'left_middle_abd', 'left_middle_mcp', 'left_middle_pip',
            'left_ring_abd', 'left_ring_mcp', 'left_ring_pip',
            'left_pinky_abd', 'left_pinky_mcp', 'left_pinky_pip',
            'left_wrist'
        ]

        self.latest_joint_state = None

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.command_pub = self.create_publisher(
            String,
            '/orca_hand/target_joint_position',
            10
        )

        self.timer = self.create_timer(0.1, self.publish_joint_command)  # 10 Hz

        self.get_logger().info("Initialized joint relay with timer-based publishing.")

    def joint_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def publish_joint_command(self):
        if self.latest_joint_state is None:
            return

        name_to_pos = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        command_dict = {}

        for name in self.joint_names:
            if name in name_to_pos:
                orca_joint = name.replace("left_", "")
                command_dict[orca_joint] = math.degrees(name_to_pos[name])

        json_msg = String()
        json_msg.data = json.dumps(command_dict)
        self.command_pub.publish(json_msg)
        self.get_logger().debug(f"Published: {json_msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = OrcaJointCommandRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
