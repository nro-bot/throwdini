"""
Implements lower-level commands specific to the robot (ur5)
"""
import ursecmon  # NOTE: ursecmon in same folder
import logging
import numpy as np
import constants

__author__ = "Olivier Roulet-Dubonnet"
__copyright__ = "Copyright 2011-2015, Sintef Raufoss Manufacturing"
__license__ = "LGPLv3"


class PyUR(object):
    def __init__(self, send_ur5_progs=True):
        self.send_ur5_progs = True
        self.joint_acc = constants.DEFAULT_JOINT_ACC
        self.joint_vel = constants.DEFAULT_JOINT_VEL

        self.logger = logging.getLogger("urx")
        self.logger.debug("Opening secondary monitor socket")

        self.secmon = ursecmon.SecondaryMonitor(tcp_host_ip)  # host ip

        self.home_joint_config = constants.GRASP_HOME
        self.logger.debug("Home config: " + str(self.home_joint_config))

        # Set limits on where robot can go
        self.workspace_limits = constants.WORKSPACE_LIMITS
        self.move_safety_limits = self.workspace_limits + 0.050

        # Tool pose tolerance for blocking calls (meters)
        self.pose_tolerance = [0.005, 0.005, 0.005, 0.020, 0.020, 0.020]

        self.max_float_length = 6  # according to python-urx lib, UR may have max float length

    # -- Gripper commands
    def activate_gripper(self):
        prog =  \
            '''
        def start_rg2():
            set_tool_voltage(0)
            sleep(1.0)
            set_digital_out(8, False)
            set_digital_out(9, False)
            set_tool_voltage(24)
            timeout = 0
            while get_digital_in(9) == False:
                timeout = timeout+1
                # sleep(0.008)
                sleep(0.005)
                if timeout > 800:
                    # wait at most 5 secs
                    textmsg("breaking")
                    break
                end
            end

            count = 0
            textmsg("beginning loop")
            set_digital_out(9, False)
            while True:
                textmsg(count)
                set_digital_out(8, True)
                sleep(1)
                set_digital_out(8, False)
                sleep(1)
                count = count + 1
            end
        end
        '''
        self.logger.debug("Activating gripper")
        self.send_program(prog)

    def open_gripper(self, gripper_async=False):
        self.send_program("set_digital_out(8,False)\n")
        self.logger.debug("opening gripper")

    def close_gripper(self, gripper_async=False):
        self.send_program("set_digital_out(8,True)\n")
        self.logger.debug("Closing gripper")
        gripper_fully_closed = self.check_grasp()
        return gripper_fully_closed

    def check_grasp(self):
        # If tool_analog_input2 > 0.26
        # Gripper did not close all the way
        return self.get_state('gripper_width') > 0.26

    # -- Data commands
    def get_state(self, subpackage):
        def get_joint_data(_log=True):
            jts = self.secmon.get_joint_data()
            joint_positions = [jts["q_actual0"], jts["q_actual1"],
                               jts["q_actual2"], jts["q_actual3"],
                               jts["q_actual4"], jts["q_actual5"]]
            if _log:
                self.logger.debug("Received joint data from robot: %s",
                                  str(joint_positions))
            return joint_positions

        def get_cartesian_info(_log=True):
            pose = self.secmon.get_cartesian_info()
            if pose:
                pose = [pose["X"], pose["Y"], pose["Z"],
                        pose["Rx"], pose["Ry"], pose["Rz"]]
            if _log:
                self.logger.debug(
                    "Received pose data from robot: %s", str(pose))
            return pose

        def get_gripper_width(_log=True):
            width = self.secmon.get_tool_analog_in(2)
            if _log:
                self.logger.debug(
                    "Received gripper width from robot: % s" %
                    str(width))
            return width

        parse_functions = {'joint_data': get_joint_data,
                           'cartesian_info': get_cartesian_info, 'gripper_width':
                           get_gripper_width}
        return parse_functions[subpackage]()

    def send_program(self, prog, send_ur5_progs=True):
        # mostly adding a printout for ease of debugging
        if not is_sim:
            self.logger.info("Sending program: " + prog)
            self.secmon.send_program(prog)
        else:
            self.logger.info("SIM. Would have sent program: " + prog)

    # self.logger.info("NOT Safe. NOT moving to: %s, due to LIMITS: %s",
    # position, self.moveto_limits)

    # TODO: convenience wrapper (back support)
    def move_to(positon, orientation):
        pass

    # TODO: convenience wrapper (back support)
    def move_joints(positon, orientation):
        pass

    # -- This funciton is needed to batch all the moves, so the robot does not
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
                if 'wait' not in move:
                    wait = False
                if 'radius' not in move:
                    move['radius'] = 0.01
                if 'acc' not in move:
                    move['acc'] = self.joint_acc
                if 'vel' not in move:
                    move['vel'] = self.joint_vel
                if idx == (len(moves_list) - 1):
                    radius = 0.001
                acc, vel, radius = move["acc"], move["vel"], move["radius"]

                # WARNING: this does not have safety checks!
                # -- move specified in joint coordinates
                if move["type"] == 'j':
                    prog += self._format_move(
                        "movel", move['pose'], acc, vel, radius, prefix="") + "\n"
                # -- move specified in cartesian coordinates
                elif move["type"] == 'p':
                    prog += self._format_move(
                        'movel', move['pose'], acc, vel, radius, prefix="p") + "\n"
        prog += "end\n"

        if wait_last_move:  # wait for last move
            if move["type"] == 'j':
                is_joint_config = True
            elif move["type"] == 'p':
                is_joint_config = False
            else:
                break
            if moves_list[-1]['type'] == 'j':
                joint_flag = True
            self._wait_for_move(target=moves_list[-1]['pose'],
                                threshold=self.pose_tolerance, joints=joint_flag)
        self.send_program(prog)
        return self.get_state('cartesian_info')

    # quick dumb way to make sure x,y,z limits are respected
    def _is_safe(self, position):
        x, y, z = position 
        xlims, ylims, zlims = self.move_safety_limits

        safe = xlims[0] <= x <= xlims[1] and ylims[0] <= y <= ylims[1] and \
                zlims[0] <= z <= zlims[1]
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

        start_dist = self._get_dist(target, joints)
        if threshold is None:
            # threshold = [0.001] * 6
            threshold = start_dist * 0.8
            threshold = self.pose_tolerance  # NOTE
            if threshold < 0.001:  # roboten precision is limited
                threshold = 0.001
            self.logger.debug("No threshold set, setting it to %s", threshold)
        count = 0
        while True:
            if not self.is_running():
                # raise RobotException("Robot stopped")
                self.logger.exception("ROBOT STOPPED!")
            # if joints:
                # actual_pose = self.get_state('joint_data')
            # else:
                # actual_pose = self.get_state('cartesian_info')

            # dist = [np.abs(actual_pose[j] - target[j]) for j in range(6)]
            dist = self._get_dist(target, joints)
            self.logger.debug(
                "distance to target is: %s, target dist is %s", dist, threshold)
            # if all([np.abs(actual_pose[j] - target[j]) < self.pose_tolerance[j] for j in range(6)]):
            #  TODO: we are having some issue where the rx ry rz over ethernet !=
            # pendant rx ry rz
            if all([np.abs(actual_pose[j] - target[j]) < self.pose_tolerance[j] for j in range(3)]):
                self.logger.debug(
                    "We are threshold(%s) close to target, move has ended", threshold)
                return

    def _get_dist(self, target, joints=False):
        if joints:
            return self._get_joints_dist(target)
        else:
            return self._get_lin_dist(target)

    def _get_lin_dist(self, target):
        # FIXME: we have an issue here, it seems sometimes the axis angle received from robot
        pose = URRobot.getl(self, wait=True)
        dist = 0
        for i in range(3):
            dist += (target[i] - pose[i]) ** 2
        for i in range(3, 6):
            dist += ((target[i] - pose[i]) / 5) ** 2  # arbitraty length like
        return dist ** 0.5

    def _get_joints_dist(self, target):
        joints = self.getj(wait=True)
        dist = 0
        for i in range(6):
            dist += (target[i] - joints[i]) ** 2
