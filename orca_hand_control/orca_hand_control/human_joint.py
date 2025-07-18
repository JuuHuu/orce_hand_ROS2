import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import JointState
import json
import numpy as np
import math


class HumanJointMarkerPublisher(Node):
    def __init__(self):
        super().__init__('human_joint_marker_publisher')

        self.marker_pub = self.create_publisher(MarkerArray, '/human_hand_markers', 10)
        self.joint_pub = self.create_publisher(JointState, '/human_hand_joint_states', 10)

        self.subscription = self.create_subscription(
            String,
            '/hand_joints',
            self.human_joint_callback,
            10
        )

        self.get_logger().info("Initialized joint marker + joint state publisher.")
        

    def normalize(self, v):
        v = np.array(v)
        norm = np.linalg.norm(v)
        return v / norm if norm > 0 else v

    def angle_between(self, p1, p2, p3):
        v1 = self.normalize(np.array(p1) - np.array(p2))
        v2 = self.normalize(np.array(p3) - np.array(p2))
        dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
        return math.degrees(math.acos(dot))

    def project_to_plane(self, v, normal):
        normal = self.normalize(normal)
        return v - np.dot(v, normal) * normal

    def compute_abduction_angle(self, base, tip, palm_reference, palm_normal):
        v_finger = np.array(tip) - np.array(base)
        v_proj = self.project_to_plane(v_finger, palm_normal)
        v_proj = self.normalize(v_proj)

        v_ref = np.array(palm_reference[1]) - np.array(palm_reference[0])
        v_ref = self.project_to_plane(v_ref, palm_normal)
        v_ref = self.normalize(v_ref)

        dot = np.clip(np.dot(v_ref, v_proj), -1.0, 1.0)
        cross = np.dot(np.cross(v_ref, v_proj), palm_normal)
        return math.degrees(math.atan2(cross, dot))

    def compute_finger_angles(self, hand_data, hand_label, joint_msg):
        def unity_to_ros(pos):
            x, y, z = pos
            return [z, -x, y]

        fingers = {
            "thumb": ["ThumbMetacarpal", "ThumbProximal", "ThumbDistal", "ThumbTip"],
            "index": ["IndexMetacarpal", "IndexProximal", "IndexIntermediate", "IndexDistal", "IndexTip"],
            "middle": ["MiddleMetacarpal", "MiddleProximal", "MiddleIntermediate", "MiddleDistal", "MiddleTip"],
            "ring": ["RingMetacarpal", "RingProximal", "RingIntermediate", "RingDistal", "RingTip"],
            "pinky": ["LittleMetacarpal", "LittleProximal", "LittleIntermediate", "LittleDistal", "LittleTip"]
        }

        try:
            palm = unity_to_ros(hand_data["Palm"]["pos"])
            palm_reference = [
                unity_to_ros(hand_data["MiddleMetacarpal"]["pos"]),
                unity_to_ros(hand_data["MiddleProximal"]["pos"])
            ]
        except KeyError:
            self.get_logger().warn("Missing palm or palm reference joints")
            return

        prox_keys = ["Palm", "IndexProximal", "RingProximal", "LittleProximal"]
        try:
            palm_pts = [np.array(unity_to_ros(hand_data[k]["pos"])) for k in prox_keys if k in hand_data]
        except KeyError:
            self.get_logger().warn("Palm proximal joint missing in Unity data.")
            return

        if len(palm_pts) < 3:
            self.get_logger().warn("Insufficient points to define palm plane")
            return

        v1 = palm_pts[1] - palm_pts[0]
        v2 = palm_pts[3] - palm_pts[0]
        palm_normal = self.normalize(np.cross(v2, v1))
        self.get_logger().info(f"Palm normal: {palm_normal}")

       

        for finger, joints in fingers.items():
            try:
                joint_pos = [unity_to_ros(hand_data[j]["pos"]) for j in joints if j in hand_data]
                if len(joint_pos) < len(joints):
                    self.get_logger().warn(f"Missing joints for {finger}")
                    continue

                mcp = self.angle_between(joint_pos[0], joint_pos[1], joint_pos[2]) if len(joint_pos) >= 3 else 0.0
                pip = dip = 0.0

                if finger == "thumb":
                    if len(joint_pos) >= 3:
                        pip = self.angle_between(joint_pos[0], joint_pos[1], joint_pos[2])
                    if len(joint_pos) >= 4:
                        dip = self.angle_between(joint_pos[1], joint_pos[2], joint_pos[3])
                    adb = self.compute_abduction_angle(
                        joint_pos[0], joint_pos[1], palm_reference, palm_normal
                    )
                else:
                    if len(joint_pos) >= 4:
                        pip = self.angle_between(joint_pos[1], joint_pos[2], joint_pos[3])
                    if len(joint_pos) >= 5:
                        dip = self.angle_between(joint_pos[2], joint_pos[3], joint_pos[4])
                    adb = self.compute_abduction_angle(
                        joint_pos[1], joint_pos[2], palm_reference, palm_normal
                    )

                  

                prefix = f"{hand_label}_{finger}"
                joint_msg.name.extend([
                    f"{prefix}_abd",
                    f"{prefix}_mcp",
                    f"{prefix}_pip",
                    f"{prefix}_dip"
                ])
                joint_msg.position.extend([
                    math.radians(adb),
                    math.radians(mcp),
                    math.radians(pip),
                    math.radians(dip)
                ])
            except Exception as e:
                self.get_logger().warn(f"Failed to compute {finger} angles: {e}")

                
    def make_point(self, xyz):
        from geometry_msgs.msg import Point
        return Point(x=xyz[0], y=xyz[1], z=xyz[2])

    def human_joint_callback(self, msg: String):
        try:
            data = json.loads(msg.data)

            def extract_hand(hand_data):
                transformed = {}
                for key, val in hand_data.items():
                    if "pos" in val:
                        transformed[key] = {"pos": val["pos"]}
                        if "rot" in val:
                            transformed[key]["rot"] = val["rot"]
                return transformed

            left_hand = extract_hand(data.get("left_hand", {}))
            right_hand = extract_hand(data.get("right_hand", {}))

            self.publish_marker_array(data)

            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = []
            joint_msg.position = []

            if left_hand:
                self.compute_finger_angles(left_hand, "left", joint_msg)
            if right_hand:
                self.compute_finger_angles(right_hand, "right", joint_msg)

            self.joint_pub.publish(joint_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process joint data: {e}")

    def publish_marker_array(self, data):
        marker_array = MarkerArray()

        def create_markers(hand_data, hand_ns, color, start_id):
            markers = []
            i = start_id
            for name, bone in hand_data.items():
                if "pos" not in bone:
                    continue
                pos = bone["pos"]
                marker = Marker()
                marker.header.frame_id = "world"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = hand_ns
                marker.id = i
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = pos[0]
                marker.pose.position.y = pos[1]
                marker.pose.position.z = pos[2]
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = 0.01
                marker.color.a = 1.0
                marker.color.r, marker.color.g, marker.color.b = color
                markers.append(marker)
                i += 1
            return markers

        left_hand = data.get("left_hand", {})
        right_hand = data.get("right_hand", {})

        marker_array.markers.extend(create_markers(left_hand, "left_hands", (0.1, 0.7, 1.0), 0))
        marker_array.markers.extend(create_markers(right_hand, "right_hands", (1.0, 0.7, 0.1), 100))

        self.marker_pub.publish(marker_array)


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
