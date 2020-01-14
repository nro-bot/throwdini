# !/usr/bin/python3
# https://github.com/SintefManufacturing/python-urx/blob/master/urx/urrobot.py
# Must be run with library from the repo

# 10 July 2019 -- this works even while inputting commands from pendant

from tcpUR.pyUR import PyUR
import logging
import time
import numpy as np
import sys
import os
import signal


def keyboardInterruptHandler(signal, frame):
    print("KeyboardInterrupt (ID: {}) has been caught. Cleaning up...".format(signal))
    robot.close()
    # exit(0)
    print('exiting')
    sys.exit()
    print('exiting again')
    os._exit(1)


signal.signal(signal.SIGINT, keyboardInterruptHandler)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=4)

    try:
        robot = PyUR(send_ur5_progs=True)
        while True:
            pose = robot.get_state('cartesian_info')
            print("robot tcp is at: ", np.array(pose), "\n")
            width = robot.get_state('gripper_width')
            print("robot finger width", width)
            # width = rob.secmon.get_cartesian_info()
            # print(rob.secmon.get_all_data()["ToolData"]["analogInput2"])
            # print(rob.secmon.get_all_data()["ToolData"]["analogInput2"])
            # print(rob.secmon.get_all_data()["CartesianInfo"])
            # width = rob.secmon.get_tool_analog_in(2)
            time.sleep(0.05)
    except Exception as e:
        print("Oopsie. Except", e)
        # print('caught exception')
        # print('closing robot')
        print('exiting')
        sys.exit()
        print('exiting again')
        os._exit(1)
