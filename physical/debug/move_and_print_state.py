# https://github.com/SintefManufacturing/python-urx/blob/master/urx/urrobot.py
# Must be run with library from the repo
from tcpUR import pyUR
import logging
import time
import os
import sys


if __name__ == "__main__":
    try:
        robot = pyUR(send_ur5_progs=False)
    except:
        sys.exit()

    try:
        delta = 0.05
        v = 0.05
        a = 0.3
        pose = robot.get_state('joint_data')
        print("robot tcp is at: ", pose, '\n')

        print("absolute move in base coordinate ")
        pose[2] += delta

        robot.move_to(pose[:3], pose[3:], acc=a, vel=v)
        pose = robot.get_state('joint_data')
        print("robot tcp is at: ", pose, '\n')

        time.sleep(1)

        print("relative move in base coordinate ")
        # rob.translate((0, 0, -delta), acc=a, vel=v, relative=True)
        time.sleep(10)

        pose[2] -= delta

        robot.move_to(pose[:3], pose[3:], acc=a, vel=v)
        pose = robot.get_state('joint_data')
        print("robot tcp is at: ", pose, '\n')

        # print("relative move back and forth in tool coordinate")
        # rob.translate_tool((0, 0, -delta), acc=a, vel=v)
        # pose = rob.getl()
        # print("robot tcp is at: ", pose, '\n')

        # print("relative move back and forth in tool coordinate")
        # rob.translate_tool((0, 0, delta), acc=a, vel=v)
        # pose = rob.getl()
        # print("robot tcp is at: ", pose, '\n')
    except Exception as e:  # RobotError, ex:
        print("Robot could not execute move (emergency stop for example), do something", e)
        # print('exiting')
        sys.exit()
        print('exiting again')
        os._exit(1)
