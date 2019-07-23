import sys
import urx
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

tcp_host_ip = "10.75.15.91"
if __name__ == '__main__':
    rob = urx.Robot(tcp_host_ip)
    # robotiqgrip = Robotiq_Two_Finger_Gripper(socket_host="/dev/ttyUSB0")
    robotiqgrip = Robotiq_Two_Finger_Gripper(rob)

    # if(len(sys.argv) != 2):
    # print("false")
    # sys.exit()

    if(sys.argv[1] == "close"):
        robotiqgrip.close_gripper()
    if(sys.argv[1] == "open"):
        robotiqgrip.open_gripper()
        print('hi')

    # try:
        # while True:
    bytes = robotiqgrip._get_gripper_status()
    print('bytes', bytes, '/n')
    # except:
    # # print('caught exception')
    # # print('closing robot')
    # rob.close()
    # # print('exiting')
    # sys.exit()
    # print('exiting again')
    # os._exit(1)
