from orca_hand_control import PyTac3D
import time
import threading
from collections import defaultdict
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, Vector3
from orca_msgs.msg import TactileFrame   # <-- now using your custom msg
import re
import numpy as np

sensor_data = defaultdict(dict)

def sanitize_topic_name(name: str) -> str:
    # Replace any invalid character with "_"
    return re.sub(r'[^a-zA-Z0-9_]', '_', name)

# Callback: update the latest frame for each sensor
def Tac3DRecvCallback(rec_frame, param):
    SN = rec_frame['SN']
    sensor_data[SN] = {
        'frameIndex': rec_frame['index'],
        'sendTimestamp': rec_frame['sendTimestamp'],
        'recvTimestamp': rec_frame['recvTimestamp'],
        'P': rec_frame.get('3D_Positions'),
        'N': rec_frame.get('3D_Normals'),
        'D': rec_frame.get('3D_Displacements'),
        'F': rec_frame.get('3D_Forces'),
        'Fr': rec_frame.get('3D_ResultantForce'),
        'Mr': rec_frame.get('3D_ResultantMoment')
    }

# Helpers for converting numpy arrays to ROS types
def to_ros_time(timestamp):
    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1e9)
    return RosTime(sec=sec, nanosec=nanosec)

def np_to_points(np_array):
    if np_array is None:
        return []
    arr = np.array(np_array)
    return [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in arr if len(p) >= 3]


def np_to_vectors(np_array):
    if np_array is None:
        return []
    arr = np.array(np_array)
    return [Vector3(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in arr if len(p) >= 3]


def np_to_vector(np_array):
    if np_array is None:
        return Vector3()
    arr = np.array(np_array).flatten()  # flatten to 1D
    if arr.size < 3:
        return Vector3()
    return Vector3(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

# ROS2 Node: creates a publisher per sensor SN dynamically
class MultiSensorTactilePublisher(Node):
    def __init__(self):
        super().__init__('multi_sensor_tactile_publisher')
        self._publishers_by_sn = {}  # {SN: publisher}
        self.timer = self.create_timer(0.05, self.publish_all_sensors)  # 20Hz

    def get_publisher_for_sn(self, SN):
        if SN not in self._publishers_by_sn:
            topic = f'/tactile_data/{sanitize_topic_name(SN)}'
            self.get_logger().info(f"Creating publisher for topic: {topic}")
            self._publishers_by_sn[SN] = self.create_publisher(TactileFrame, topic, 10)
        return self._publishers_by_sn[SN]

    def publish_all_sensors(self):
        for SN, data in sensor_data.items():
            pub = self.get_publisher_for_sn(SN)
            msg = TactileFrame()
            msg.sensor_sn = SN
            msg.frame_index = data['frameIndex']
            msg.send_timestamp = to_ros_time(data['sendTimestamp'])
            msg.recv_timestamp = to_ros_time(data['recvTimestamp'])
            msg.positions = np_to_points(data['P'])
            msg.normals = np_to_vectors(data['N'])
            msg.displacements = np_to_vectors(data['D'])
            msg.forces = np_to_vectors(data['F'])
            msg.resultant_force = np_to_vector(data['Fr'])
            msg.resultant_moment = np_to_vector(data['Mr'])
            pub.publish(msg)


# ---- Initialize sensor(s) ----
tactile_sensor = PyTac3D.Sensor(recvCallback=Tac3DRecvCallback, port=9988, maxQSize=5, callbackParam='')
tactile_sensor.waitForFrame()
tactile_sensor.calibrate("DM1-GWM0021")  # Calibrate initial sensor
tactile_sensor.calibrate("YDSt1-004")

# ---- Start ROS2 Node in a separate thread ----
def main():
    rclpy.init()
    node = MultiSensorTactilePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


