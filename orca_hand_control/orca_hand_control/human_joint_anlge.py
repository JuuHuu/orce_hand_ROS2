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
            'left_thumb_abd': math.radians(-20),
            'left_thumb_mcp':math.radians(130), 
            'left_thumb_pip':math.radians(180), 
            'left_thumb_dip':math.radians(180),
            'left_index_abd':math.radians(0), 
            'left_index_mcp':math.radians(180), 
            'left_index_pip':math.radians(360),
            'left_middle_abd':math.radians(10), 
            'left_middle_mcp':math.radians(180), 
            'left_middle_pip':math.radians(360),
            'left_ring_abd':math.radians(0), 
            'left_ring_mcp':math.radians(180), 
            'left_ring_pip':math.radians(360),
            'left_pinky_abd':math.radians(0), 
            'left_pinky_mcp':math.radians(180), 
            'left_pinky_pip':math.radians(360),
            'left_wrist':math.radians(0)
        }
        self.scale_map = {
            'left_thumb_abd': 1,
            'left_thumb_mcp':-1, 
            'left_thumb_pip':-1, 
            'left_thumb_dip':-1,
            'left_index_abd':1, 
            'left_index_mcp':-1, 
            'left_index_pip':-1,
            'left_middle_abd':1, 
            'left_middle_mcp':-1, 
            'left_middle_pip':-1,
            'left_ring_abd':1, 
            'left_ring_mcp':-1, 
            'left_ring_pip':-1,
            'left_pinky_abd':1, 
            'left_pinky_mcp':-1, 
            'left_pinky_pip':-1,
            'left_wrist':1
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
        # Special summed joints
        combined_joints = {
            'left_index_pip': ('left_index_pip', 'left_index_dip'),
            'left_middle_pip': ('left_middle_pip', 'left_middle_dip'),
            'left_ring_pip': ('left_ring_pip', 'left_ring_dip'),
            'left_pinky_pip': ('left_pinky_pip', 'left_pinky_dip'),
        }

        joint_out.position = []
        for name in self.joint_names:
            if name in combined_joints:
                base, extra = combined_joints[name]
                base_val = name_to_pos.get(base, 0.0)
                extra_val = name_to_pos.get(extra, 0.0)
                value = base_val + extra_val*1.2
            else:
                value = name_to_pos.get(name, 0.0)

            value *= self.scale_map.get(name, 1.0)
            value += self.bias_map.get(name, 0.0)
            joint_out.position.append(value)

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
