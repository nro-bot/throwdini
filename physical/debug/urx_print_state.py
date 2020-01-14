# https://github.com/SintefManufacturing/python-urx/blob/master/urx/urrobot.py
# Must be run with library from the repo

# 10 July 2019 -- this works even while inputting commands from pendant

from pyUR.pyUR, import PyUR
# import logging
import time
import numpy as np
import sys
import os

if __name__ == "__main__":
    # logging.basicConfig(level=logging.DEBUG)
    np.set_printoptions(precision=4)

    try:
        robot = pyUR(send_ur5_progs=False)
    except:
        sys.exit()
    try:
        while True:
            pose = rob.getl()
            # # print(rob)
            print("robot tcp is at: ", np.array(pose), '\n')
            # width = rob.secmon.get_cartesian_info()
            # print(rob.secmon.get_all_data()["ToolData"]["analogInput2"])
            # print(rob.secmon.get_all_data()["ToolData"]["analogInput2"])
            # print(rob.secmon.get_all_data()["CartesianInfo"])
            # width = rob.secmon.get_tool_analog_in(2)
            # print("robot finger width", width)
            time.sleep(0.05)
    except Exception as e:
        print(e)
        # print('caught exception')
        # print('closing robot')
        # rob.close()
        # # print('exiting')
        # # sys.exit()
        # print('exiting again')
        # os._exit(1)
