from orca_core import OrcaHand

import time  
import argparse  
import rclpy

from ament_index_python.packages import get_package_share_directory
import os

class OrcaHand_ROS(OrcaHand):
    def __init__(self, model_path):
        super().__init__(model_path)



def main():
    rclpy.init()
    
    # pkg_dir = get_package_share_directory("orca_hand_control")
    # model_path = os.path.join(pkg_dir, "models", "orcahand_v1_right", "calibration.yaml")

    
    parser = argparse.ArgumentParser(description="Test the ORCA Hand.")  # Added parser
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    args = parser.parse_args()
    hand = OrcaHand_ROS(args.model_path)
    status = hand.connect()

    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)
    
    
    hand.enable_torque()
    
    hand.set_zero_position()
    # hand.set_neutral_position()
    
    print(hand.get_joint_pos())
    

    
    hand.disable_torque()
    hand.disconnect()

    return 0
    
    
    
    
if __name__ == "__main__":
    main()