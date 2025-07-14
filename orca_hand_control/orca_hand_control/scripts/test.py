from orca_core import OrcaHand
import time  
import argparse  


def main(): 
    parser = argparse.ArgumentParser(description="Test the ORCA Hand.")  # Added parser
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    args = parser.parse_args()
    hand = OrcaHand(args.model_path)

    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.enable_torque()

    joint_dict = {
        "thumb_mcp": -30,


    }

    hand.set_joint_pos(joint_dict)
    

    time.sleep(2)
    hand.set_zero_position()

    
    print(hand.get_joint_pos())
    
    
    hand.disable_torque()

    hand.disconnect()

if __name__ == "__main__":  # Added main execution block
    main()