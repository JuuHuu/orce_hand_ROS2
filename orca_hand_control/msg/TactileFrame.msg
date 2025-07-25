# TactileFrame.msg
string sensor_sn
uint64 frame_index
builtin_interfaces/Time send_timestamp
builtin_interfaces/Time recv_timestamp
geometry_msgs/Point[] positions       # P
geometry_msgs/Vector3[] normals       # N
geometry_msgs/Vector3[] displacements # D
geometry_msgs/Vector3[] forces        # F
geometry_msgs/Vector3 resultant_force # Fr
geometry_msgs/Vector3 resultant_moment # Mr
