import PyTac3D
import time
import threading
from collections import defaultdict
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time as RosTime
from geometry_msgs.msg import Point, Vector3
from tactile_msgs.msg import TactileFrame  # <-- your custom message

sensor_data = defaultdict(dict)

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

def to_ros_time(timestamp):
    sec = int(timestamp)
    nanosec = int((timestamp - sec) * 1e9)
    return RosTime(sec=sec, nanosec=nanosec)

def np_to_points(np_array):
    return [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in np_array] if np_array is not None else []

def np_to_vectors(np_array):
    return [Vector3(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in np_array] if np_array is not None else []

def np_to_vector(np_array):
    return Vector3(x=float(np_array[0]), y=float(np_array[1]), z=float(np_array[2])) if np_array is not None else Vector3()

class TactilePublisher(Node):
    def __init__(self):
        super().__init__('tactile_sensor_publisher')
        self.publishers = {}  # topic per SN

    def get_publisher(self, SN):
        if SN not in self.publishers:
            topic = f'/tactile_data/{SN}'
            self.get_logger().info(f"Creating publisher for {topic}")
            self.publishers[SN] = self.create_publisher(TactileFrame, topic, 10)
        return self.publishers[SN]

    def publish_sensor_data(self):
        while rclpy.ok():
            for SN, data in sensor_data.items():
                pub = self.get_publisher(SN)
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
            time.sleep(0.05)  # 20 Hz

# ---- Create & start sensor ----
tactile_sensor = PyTac3D.Sensor(
    recvCallback=Tac3DRecvCallback, 
    port=9988, 
    maxQSize=5, 
    callbackParam=''
)
tactile_sensor.waitForFrame()
tactile_sensor.calibrate("DM1-GWM0021")

# ---- Run ROS2 in a separate thread ----
rclpy.init()
node = TactilePublisher()
pub_thread = threading.Thread(target=node.publish_sensor_data, daemon=True)
pub_thread.start()

try:
    print("Running. Press Ctrl+C to exit.")
    rclpy.spin(node)
except KeyboardInterrupt:
    print("\nStopped by user.")
finally:
    node.destroy_node()
    rclpy.shutdown()
