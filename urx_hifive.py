# https://github.com/SintefManufacturing/python-urx/blob/master/urx/urrobot.py
# Must be run with library from the repo
import urx
import logging
import time
import os
import sys

tcp_host_ip = "10.75.15.199"


def follow(thefile):
    thefile.seek(0, 2)
    while True:
        line = thefile.readline()
        if not line:
            time.sleep(0.1)
            continue
        # yield line


rob = urx.Robot(tcp_host_ip)
# rob = urx.Robot("localhost")

rob.set_tcp((0, 0, 0, 0, 0, 0))
rob.set_payload(0.5, (0, 0, 0))

if __name__ == "__main__":
    logfile = open("sensor_log.txt", "r")
    # delta = 0.05
    delta = 0.001
    v = 0.05
    a = 0.3

    while True:
        # try:
        pose = rob.getl()
        print("robot tcp is at: ", pose, '\n')

        # loglines = follow(logfile)
        logfile.seek(0, 0)
        line = logfile.readline()

        try:
            float(line)
            print('a line', float(line))

            if float(line) >= -50.5:
                rob.translate_tool((0, 0, delta), acc=a, vel=v)
                time.sleep(0.01)
            if float(line) < -50.5:
                rob.translate_tool((0, 0, 0), acc=a, vel=v)
                time.sleep(1)
        except:
            print('fail')
            pass

        # rob.translate_tool((0, 0, ), acc=a, vel=v)
        # except Exception as e:  # RobotError, ex:
        # print(
        # "Robot could not execute move (emergency stop for example), do something", e)
        # rob.close()
        # print('closing robot')
        # # print('exiting')
        # sys.exit()
        # print('exiting again')
        # os._exit(1)
        # finally:
        # rob.close()
