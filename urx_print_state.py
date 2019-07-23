# https://github.com/SintefManufacturing/python-urx/blob/master/urx/urrobot.py
# Must be run with library from the repo

# 10 July 2019 -- this works even while inputting commands from pendant

import urx
import logging
import time
import numpy as np
import sys
import os


tcp_host_ip = "10.75.15.91"

if __name__ == "__main__":
    logging.basicConfig(level=logging.WARN)
    np.set_printoptions(precision=4)

    try:
        rob = urx.Robot(tcp_host_ip)
    except:
        sys.exit()
    #rob = urx.Robot("localhost")
    rob.set_tcp((0, 0, 0, 0, 0, 0))
    rob.set_payload(0.5, (0, 0, 0))
    try:
        while True:
            rob = urx.Robot(tcp_host_ip)
            pose = rob.getl()
            # pose = rob.secmon.get_cartesian_info(wait=True)
            # data = rob.secmon.get_all_data(wait=False)
            data = rob.getdata(wait=False)
            # print("robot tcp is at: ", np.array(pose), '\n')
            # print("robot state: ", data, '\n')
            # print("robot state analog: ",
            # data['ToolData'], '\n')
            # print("robot state masterboard: ",
                  # data['MasterBoardData'], '\n')
            time.sleep(0.05)
    except:
        # print('caught exception')
        # print('closing robot')
        # rob.close()
        # print('exiting')
        sys.exit()
        print('exiting again')
        os._exit(1)
