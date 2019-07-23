import sys
import urx
import time
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper


import logging
logging.basicConfig()
log = logging.getLogger("urx")
log.setLevel(logging.DEBUG)


tcp_host_ip = "10.75.15.91"
if __name__ == '__main__':
    rob = urx.Robot(tcp_host_ip)

    rob.set_tcp((0, 0, 0, 0, 0, 0))
    rob.set_payload(0.5, (0, 0, 0))

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

    while True:
        # print('getting pos')
        # bytes = robotiqgrip.get_gripper_pos()
        # robotiqgrip.get_gripper_pos()
        # rob.send_message("hi")
        # rob.send_message("rq_get_position()")

        # bytes = None
        # bytes = 123
        # print('bytes', bytes, '/n')

        print("-------------------------\n")

        robotiqgrip.gripper_action(0)
        time.sleep(1)
        rob.set_analog_out_to_pos()
        time.sleep(0.5)
        data = rob.secmon.get_all_data(wait=False)
        print("\n  !----- analog state open: ",
              data['MasterBoardData']['analogOutput0'], '\n')
        time.sleep(1)

        robotiqgrip.gripper_action(255)
        time.sleep(1)
        rob.set_analog_out_to_pos()
        time.sleep(0.5)
        # pose = rob.getl()
        data = rob.secmon.get_all_data(wait=False)
        print("\n !-----  analog state close: ",
              data['MasterBoardData']['analogOutput0'], '\n')
        time.sleep(1)


# except:
# print('caught exception')
# print('closing robot')
# rob.close()
# print('exiting')
# sys.exit()
# print('exiting again')
# os._exit(1)
