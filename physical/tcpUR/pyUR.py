"""
Implements lower-level commands specific to the robot (ur5)
"""
from . import ursecmon  # NOTE: ursecmon in same folder
import logging
import math3d as m3d
import numpy as np
from . import constants
import signal
import sys
import os
import time

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2015, Sintef Raufoss Manufacturing"
__license__ = "LGPLv3"


class PyUR(object):
    def __init__(self, default_acc=None, default_vel=None, send_ur5_progs=True):

        self.csys = m3d.Transform()

        if default_acc is None:
            self.joint_acc = constants.DEFAULT_JOINT_ACC
        else:
            self.joint_acc = default_acc
        if default_vel is None:
            self.joint_vel = constants.DEFAULT_JOINT_VEL
        else:
            self.joint_vel = default_vel

        if constants.USE_GLOBAL_SEND_UR5_PROGS:
            self.send_ur5_progs = constants.GLOBAL_SEND_UR5_PROGS
        else:
            self.send_ur5_progs = send_ur5_progs

        # Does colored formatter exist, for bash?

        self.logger = logging.getLogger("urx")
        self.logger.debug("Opening secondary monitor socket")

        self.logger.propagate = False

        # Does colored formatter exist, for bash?
        try:
            file = open("logger.py")
            self.logger.debug('colored formatter available, using')
            from logger import ColoredFormatter
            ch = logging.StreamHandler()
            ch.setFormatter(ColoredFormatter())
            self.logger.addHandler(ch)
        except IOError:
            print("colored formatter not accessible")
        finally:
            file.close()

        if not self.send_ur5_progs:
            self.logger.warning('WARNING: We are *NOT* sending ur5 programs, ' + \
                'robot will NOT move')
        else:
            self.logger.warning('WARNING: sending ur5 progs enabled, EXPECT ROBOT MOVEMENT')
        self.secmon = ursecmon.SecondaryMonitor(
            constants.TCP_HOST_IP)  # host ip

        # self.activate_gripper()
        self.logger.debug('''
! -- NOTE: Make sure to deactivate gripper in Program robot>Installation tab!')
'Also, protip, for urscript, textmsg() shows up in progam robot > Log tab''')

        # Set limits on where robot can go
        self.workspace_limits = constants.WORKSPACE_LIMITS
        self.move_safety_limits = self.workspace_limits + 0.050

        # Tool pose tolerance for blocking calls (meters)
        self.pose_tolerance = [0.005, 0.005, 0.005, 0.020, 0.020, 0.020]

        self.max_float_length = 6  # according to python-urx lib, UR may have max float length

        # make sure we get data from robot before letting clients access our methods
        self.secmon.wait()

        # Allow Keyboard interrupts to be handled gracefully
        signal.signal(signal.SIGINT, self.keyboardInterruptHandler)


    def keyboardInterruptHandler(self, signal, frame):
        print(
            "KeyboardInterrupt (ID: {}) has been caught. Cleaning up...".format(signal))
        print('exiting')
        sys.exit()
        print('exiting again')
        os._exit(1)

    # -- Gripper commands
    def activate_gripper(self):
        self.logger.warning('Attempting to start RG2 gripper! Likely gripper open-close several times.' )
        self.logger.debug('Protip: Check Program>Installation>RG Configuration>Enable RG is **unchecked**')

        prog_sanity = \
            '''
        def sanityCheck():
            textmsg("sanity check")
            popup("sanity check")
            end
        '''

        prog =  \
            '''
        def start_rg2():
            mycount = 0
            timeout = 0
            set_tool_voltage(0)
            sleep(10.0)
            set_digital_out(8, False)
            sleep(10.0)
            set_digital_out(9, False)
            sleep(10.0)
            set_tool_voltage(24)

            while get_digital_in(9) == False:
                textmsg("digital in false")
                timeout = timeout+1
                # sleep(0.008)
                sleep(0.005)
                textmsg(timeout)
                if timeout > 800:
                    # wait at most 5 secs
                    textmsg("timeout")
                    textmsg(timeout)
                    popup("breaking")
                    break
                end
            end

            sleep(1.0)

            textmsg("beginning loop")
            set_digital_out(9, False)
            while True:
                mycount = mycount+1
                sleep(0.005)
                textmsg(mycount)
                set_digital_out(8, True)
                sleep(10.0)
                set_digital_out(8, False)
                sleep(10.0)
            end
        end
        '''
        self.logger.debug("Activating gripper")
        self.send_program(prog)
        # self.send_program(prog_sanity)

    def open_gripper(self):
        self.send_program("set_digital_out(8,False)\n")
        self.logger.debug("opening gripper")

    def close_gripper(self):
        self.send_program("set_digital_out(8,True)\n")
        self.logger.debug("Closing gripper")
        gripper_fully_closed = self.check_grasp()
        return gripper_fully_closed

    def check_grasp(self):
        # If tool_analog_input2 > 0.26, Gripper did not close all the way
        return self.get_state('gripper_width') > constants.GRIPPER_OPEN_THRESH

    # -- Data commands
    def get_state(self, subpackage):
        LOG = constants.LOG_WAIT_FOR_MOVE
        def get_joint_data():
            jts = self.secmon.get_joint_data()
            joint_positions = [jts["q_actual0"], jts["q_actual1"],
                               jts["q_actual2"], jts["q_actual3"],
                               jts["q_actual4"], jts["q_actual5"]]
            if LOG:
                self.logger.debug("Received joint data from robot: %s",
                                  str(joint_positions))
            return joint_positions

        def get_cartesian_info():
            pose = self.secmon.get_cartesian_info()
            if pose:
                pose = [pose["X"], pose["Y"], pose["Z"],
                        pose["Rx"], pose["Ry"], pose["Rz"]]
            if LOG:
                self.logger.debug(
                    "Received pose data from robot: %s", str(pose))

            pose = self.csys.inverse * m3d.Transform(pose)
            pose = pose.pose_vector.tolist()
            return pose

        def get_gripper_width():
            width = self.secmon.get_tool_analog_in(2)
            if LOG:
                self.logger.debug(
                    "Received gripper width from robot: % s" %
                    str(width))
            return width

        parse_functions = {'joint_data': get_joint_data,
                           'cartesian_info': get_cartesian_info, 'gripper_width':
                           get_gripper_width}
        return parse_functions[subpackage]()


    # self.logger.info("NOT Safe. NOT moving to: %s, due to LIMITS: %s",
    # position, self.moveto_limits)

    # repetitive due to being convenience wrapper (backwards compatibility)
    # position in meters, orientation in axis-angle degrees
    def move_to(self, position, orientation, vel=None, acc=None, radius=None):
        # todo; this seems dumb
        if acc is None:
            acc = self.joint_acc
        if vel is None:
            vel = self.joint_vel
        if radius is None:
            radius = 0.001
        pose = [{'type': 'p',
                 'pose': np.append(position, orientation),
                 'vel': vel, 'acc': acc, 'radius': radius}]
        self.combo_move(pose, wait_last_move=True)

    # repetitive due to being convenience wrapper (backwards compatibility)
    # position and orientation in radians
    def move_joints(self, position, orientation, vel=None, acc=None, radius=None):
        if acc is None:
            acc = self.joint_acc
        if vel is None:
            vel = self.joint_vel
        if radius is None:
            radius = 0.001
        pose = [{'type': 'j',
                 'pose': np.append(position, orientation),
                 'vel': vel, 'acc': acc, 'radius': radius}]
        self.combo_move(pose, wait_last_move=True)

    # -- This function is needed to batch all the moves, so the robot does not
    # stop between moves
    def combo_move(self, moves_list, wait_last_move=False):
        """
        Example use:
        pose_list = [ {type:p, vel:0.1, acc:0.1, radius:0.2},
                    {type: open}]
        """
        prog = "def combo_move():\n"

        for idx, move in enumerate(moves_list):
            # -- gripper commands
            if move["type"] == 'open':
                prog += "\tset_digital_out(8, False)"
            elif move["type"] == 'close':
                prog += "\tset_digital_out(8, True)"
            # -- UR5 commands
            else:
                # -- Sensible defaults used
                if 'radius' not in move:
                    move['radius'] = 0.01
                if 'acc' not in move:
                    move['acc'] = self.joint_acc
                if 'vel' not in move:
                    move['vel'] = self.joint_vel
                if idx == (len(moves_list) - 1):
                    radius = 0.001
                acc, vel, radius = move["acc"], move["vel"], move["radius"]
                # -- move specified in joint coordinates
                if move["type"] == 'j':
                    # TODO!! how to convert joint config into cartesian for safety
                    # check?
                    prog += self._format_move(
                        "movel", move['pose'], acc, vel, radius, prefix="") + "\n"
                # -- move specified in cartesian coordinates
                elif move["type"] == 'p':
                    prog += self._format_move(
                        'movel', move['pose'], acc, vel, radius, prefix="p") + "\n"
        prog += "end\n"

        if wait_last_move:  # wait for last move?
            # -- determine type of last move
            is_joint_config = None
            if move["type"] == 'j':
                is_joint_config = True
            elif move["type"] == 'p':
                is_joint_config = False
            else:
                # TODO: write more useful error message (include last command)
                # TODO: Perhaps this should throw an exception and halt program?
                self.logger.error('Could not wait for move, last specified \
                                   not a position (probably gripper command)! Continuing...')
            self._wait_for_move(target=move['pose'],
                                threshold=self.pose_tolerance,
                                joints=is_joint_config)
        self.send_program(prog)
        return self.get_state('cartesian_info')

    # Quick dumb way to make sure x,y,z limits are respected
    def _is_safe(self, position):
        x, y, z = position
        xlims, ylims, zlims = self.move_safety_limits
        safe = (xlims[0] <= x <= xlims[1] and ylims[0] <= y <= ylims[1] and
                zlims[0] <= z <= zlims[1])  # could do a loop but eh
        return safe

    def _format_move(self, command, tpose, acc, vel, radius=0, time=0, prefix=""):
        # prefix= p for position, none for joints
        tpose = [round(i, self.max_float_length) for i in tpose]
        # can i specifiy time?
        tpose.append(acc)
        tpose.append(vel)
        tpose.append(radius)
        tpose.append(time)
        return "\t{}({}[{}, {}, {}, {}, {}, {}], a={}, v={}, r={}, t={})\n".format(command, prefix, *tpose)

    # def get_gripper_width(self):
        # return self.get_state('tool_data')
        # if self._is_safe(position, self.moveto_limits) or override_safety:

    def is_running(self):
        """
        Return True if robot is running (not
        necessary running a program, it might be idle)
        """
        return self.secmon.running

    def is_program_running(self):
        """
        check if program is running on the robot
        Warning!!!!!:  After sending a program it might take several 10th of
        a second before the robot enters the running state
        """
        return self.secmon.is_program_running()

    def _wait_for_move(self, target, threshold=None, joints=False):
        """
        Wait for a move to complete. Unfortunately there is no good way to know
        when a move has finished so for every received data from robot we
        compute a dist equivalent and when it is lower than 'threshold' we
        return.
        if threshold is not reached within timeout, an exception is raised
        """
        self.logger.debug(
            "Waiting for move completion using threshold %s and target %s", threshold, target)

        # start_dist = self._get_dist(target, joints)
        while True:
            if not self.is_running():
                # raise RobotException("Robot stopped")
                self.logger.exception("ROBOT STOPPED!")
                time.sleep(0.1)
            elif not self.secmon.is_program_running():
                # Once program has sent / been executed...
                if joints:
                    actual_pose = self.get_state('joint_data')
                else:
                    actual_pose = self.get_state('cartesian_info')

                if all([np.abs(actual_pose[j] - target[j]) < self.pose_tolerance[j] for j in range(3)]):
                    self.logger.debug(
                        "We are threshold(%s) close to target, move has ended" % str(threshold))
                return

    '''

    def _get_dist(self, target, joints=False):
        if joints:
            return self._get_joints_dist(target)
        else:
            return self._get_lin_dist(target)

    def _get_lin_dist(self, target):
        self.logger.debug(
            "Getting linear distance between target and current pose")
        # FIXME: we have an issue here, it seems sometimes the state in
        # axis angle received from robot changes drastically
        pose = self.get_state('cartesian_info')
        dist = 0
        for i in range(3):
            dist += (target[i] - pose[i]) ** 2
        for i in range(3, 6):
            dist += ((target[i] - pose[i]) / 5) ** 2
        return dist ** 0.5

    def _get_joints_dist(self, target):
        self.logger.debug(
            "Getting joints distance between target and current pose")
        pose = self.get_state('joint_data')
        dist = 0
        for i in range(6):
            dist += (target[i] - pose[i]) ** 2
    '''

    def close(self):
        self.logger.debug("Closing robot.")
        self.secmon.close()
        self.logger.debug("Robot closed.")

    def send_program(self, prog):
        # mostly adding a printout for ease of debugging
        if self.send_ur5_progs:
            self.logger.debug("Sending program: " + prog)
            self.secmon.send_program(prog)
        else:
            self.logger.debug("SIM. Would have sent program: " + prog)
