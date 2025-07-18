import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStatePrinter(Node):
    def __init__(self):
        super().__init__('joint_state_printer')
        
        self.joint_names = [
            'left_thumb_mcp', 'left_thumb_abd', 'left_thumb_pip', 'left_thumb_dip',
            'left_index_abd', 'left_index_mcp', 'left_index_pip',
            'left_middle_abd', 'left_middle_mcp', 'left_middle_pip',
            'left_ring_abd', 'left_ring_mcp', 'left_ring_pip',
            'left_pinky_abd', 'left_pinky_mcp', 'left_pinky_pip',
            'left_wrist'
        ]

        # Joint biases in radians — customize as needed
        self.bias_map = {
            'left_thumb_abd': math.radians(5.0),
            'left_index_mcp': math.radians(90),
            'left_wrist': math.radians(10.0),
            'left_thumb_mcp':math.radians(0), 
            'left_thumb_abd':math.radians(0), 
            'left_thumb_pip':math.radians(0), 
            'left_thumb_dip':math.radians(0),
            'left_index_abd':math.radians(0), 
            'left_index_mcp':math.radians(180), 
            'left_index_pip':math.radians(180),
            'left_middle_abd':math.radians(0), 
            'left_middle_mcp':math.radians(0), 
            'left_middle_pip':math.radians(0),
            'left_ring_abd':math.radians(0), 
            'left_ring_mcp':math.radians(0), 
            'left_ring_pip':math.radians(0),
            'left_pinky_abd':math.radians(0), 
            'left_pinky_mcp':math.radians(0), 
            'left_pinky_pip':math.radians(0),
            'left_wrist':math.radians(0)
        }

        self.humanmsg = None

        self.subscription = self.create_subscription(
            JointState,
            '/human_hand_joint_states',
            self.joint_callback,
            10
        )
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        self.get_logger().info("Listening to /human_hand_joint_states...")
        self.timer = self.create_timer(0.1, self.publish_joint_states)

    def joint_callback(self, msg: JointState):
        self.get_logger().info("---- Received Joint States ----")
        self.humanmsg = msg
        for name, position in zip(msg.name, msg.position):
            deg = math.degrees(position)
            print(f"{name}: {deg:.1f}°")

    def publish_joint_states(self):
        if self.humanmsg is None:
            return

        name_to_pos = dict(zip(self.humanmsg.name, self.humanmsg.position))

        joint_out = JointState()
        joint_out.header.stamp = self.get_clock().now().to_msg()
        joint_out.name = self.joint_names

        joint_out.position = [
            name_to_pos.get(name, 0.0) + self.bias_map.get(name, 0.0)
            for name in self.joint_names
        ]

        self.publisher.publish(joint_out)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePrinter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
