#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import numpy as np
import math


class HumanJointMarkerPublisher(Node):
    def __init__(self):
        super().__init__('human_joint_marker_publisher')

        self.marker_pub = self.create_publisher(MarkerArray, '/human_hand_markers', 10)
        self.subscription = self.create_subscription(
            String,
            '/hand_joints',
            self.human_joint_callback,
            10
        )

        self.get_logger().info("Initialized joint marker publisher.")

    def human_joint_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            left_hand = data.get("left_hand", {})

            marker_array = MarkerArray()
            i = 0

            for name, bone in left_hand.items():
                if "pos" not in bone:
                    continue

                pos = bone["pos"]
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "left_hands"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = pos[0]
                marker.pose.position.y = pos[1]
                marker.pose.position.z = pos[2]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 0.1
                marker.color.g = 0.7
                marker.color.b = 1.0
                marker.text = name
                i += 1

                marker_array.markers.append(marker)
                
            right_hand = data.get("right_hand", {})

            i = 0    
                
            for name, bone in right_hand.items():
                if "pos" not in bone:
                    continue

                pos = bone["pos"]
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "right_hands"
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = pos[0]
                marker.pose.position.y = pos[1]
                marker.pose.position.z = pos[2]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.01
                marker.scale.y = 0.01
                marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.7
                marker.color.b = 0.1
                marker.text = name
                i += 1

                marker_array.markers.append(marker)
                
                

            self.marker_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f"Failed to process joint data: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = HumanJointMarkerPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
