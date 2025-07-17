import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from orca_core import OrcaHand
from ament_index_python.packages import get_package_share_directory
import os
import argparse  
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import math
import threading
import json
from std_msgs.msg import String
import time



class OrcaJointPublisher(Node):
    def __init__(self, model_path):
        super().__init__('orca_joint_publisher')

        self.hand = OrcaHand(model_path)
        status = self.hand.connect()
        if not status[0]:
            self.get_logger().error(f"Failed to connect to ORCA Hand: {status[1]}")
            rclpy.shutdown()
            return

        self.hand.enable_torque()
        # self.hand.disable_torque()
        self.get_logger().info("Connected and torque enabled.")

        # Define joint names — must match the names in the URDF
        # self.joint_names = [
        #     'right_thumb_mcp', 'right_thumb_abd', 'right_thumb_pip', 'right_thumb_dip',
        #     'right_index_abd', 'right_index_mcp', 'right_index_pip',
        #     'right_middle_abd', 'right_middle_mcp', 'right_middle_pip',
        #     'right_ring_abd', 'right_ring_mcp', 'right_ring_pip',
        #     'right_pinky_abd', 'right_pinky_mcp', 'right_pinky_pip',
        #     'right_wrist'
        # ]
        self.joint_names = [
            'left_thumb_mcp', 'left_thumb_abd', 'left_thumb_pip', 'left_thumb_dip',
            'left_index_abd', 'left_index_mcp', 'left_index_pip',
            'left_middle_abd', 'left_middle_mcp', 'left_middle_pip',
            'left_ring_abd', 'left_ring_mcp', 'left_ring_pip',
            'left_pinky_abd', 'left_pinky_mcp', 'left_pinky_pip',
            'left_wrist'
        ]

        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10Hz
        
        self.subscribtion = self.create_subscription(
            String,
            "/orca_hand/target_joint_position",
            self.joint_command_callback,
            10
        )
        
        self.subscribtion = self.create_subscription(
            String,
            "/orca_hand/enable_torque",
            self.torque_command_callback,
            10
        )
        

                
        
        
        self.lock = threading.Lock()
        self.command_thread = None
        
        self.torque_enable = 'enable'
        
        
        self.joint_dict = {
        "thumb_mcp": -30,
        }
        

    def publish_joint_states(self):
        with self.lock:
            positions = self.hand.get_joint_pos()   
        if positions is None or len(positions) != len(self.joint_names):
            self.get_logger().warn("Invalid joint data from hardware.")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = []

        # some joints was inverted for right hand, modify them
        for name, pos in zip(self.joint_names, positions):
            angle = math.radians(float(pos))
            # some joints was inverted for right hand, modify them here
            if name in ["right_index_abd",'right_middle_abd','right_ring_abd','right_pinky_abd','right_wrist',"left_index_abd",'left_middle_abd','left_ring_abd','left_pinky_abd']: 
                angle = -angle
            # some joints have error with physical set, modify them here
            if name in ["left_thumb_mcp"]: 
                angle = angle + math.radians(float(12.5))
            if name in ["left_index_pip"]: 
                angle = angle + math.radians(float(10))
            if name in ["left_middle_mcp"]: 
                angle = angle + math.radians(float(3)) 
            if name in ["left_ring_pip"]: 
                angle = angle + math.radians(float(5)) 
            if name in ["left_wrist"]: 
                angle = angle - math.radians(float(5)) 
                
                
                
                
            msg.position.append(angle)

        self.publisher_.publish(msg)
    
    def joint_command_callback(self, msg:String):
        try:
            joint_dict = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON: {e} ,{msg.data}")
            return
        
        
        if self.torque_enable == 'disable':
            self.hand.enable_torque()
        
        def send_command():
            with self.lock:
                # self.hand.set_joint_pos(joint_dict,num_steps=25,step_size=0.001)
                self.hand.set_joint_pos(joint_dict)
                
                self.get_logger().info(f"Sent command: {joint_dict}")
        
        self.command_thread = threading.Thread(target=send_command,daemon=True)
        self.command_thread.start()
        
        time.sleep(0.5)

        if self.torque_enable == 'disable':
            self.get_logger().info("Disabling torque.")
            self.hand.disable_torque()

    def destroy_node(self):
        self.get_logger().info("Disabling torque and disconnecting ORCA Hand.")
        self.hand.disable_torque()
        self.hand.disconnect()
        super().destroy_node()
        
    def torque_command_callback(self, msg:String):
        if msg.data == "enable":
            self.get_logger().info("Enabling torque.")
            self.hand.enable_torque()
            self.torque_enable = 'enable'
        elif msg.data == "disable":
            self.get_logger().info("Disabling torque.")
            self.hand.disable_torque()
            self.torque_enable = 'disable'
        
        




def main():
    rclpy.init()
    
    parser = argparse.ArgumentParser(description="Test the ORCA Hand.")  # Added parser
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    args = parser.parse_args()

    node = OrcaJointPublisher(args.model_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User interrupted.")
    finally:
        node.hand.disable_torque()
        node.hand.disconnect()
        node.destroy_node()
        rclpy.shutdown()
