'''
 Implements higher-level primitives specific to our task
'''
import socket
import select
import struct
import time
import os
import numpy as np
import utils
import serial
import binascii

import constants
from physical.tcpUR.pyUR import PyUR
from real.camera import Camera

class Robot(object):
    def __init__(self, send_ur5_progs=True):
        self.orig = False
        #self.is_bullet_sim = False

        self.r = PyUR(send_ur5_progs=send_ur5_progs)
        # Move robot to home pose
        self.r.move_to(constants.GRASP_START_HOME)
        # self.r.close_gripper()
        # self.r.open_gripper()

        # Fetch RGB-D data from RealSense camera
        self.camera = Camera()
        # Load camera pose (from running calibrate.py), intrinsics and depth scale
        self.cam_intrinsics = self.camera.intrinsics
        self.cam_pose = np.loadtxt("real/camera_pose.txt", delimiter=" ")
        self.cam_depth_scale = np.loadtxt(
            "real/camera_depth_scale.txt", delimiter=" ")

    def get_camera_data(self):
        # Get color and depth image from ROS service
        color_img, depth_img = self.camera.get_data()

        return color_img, depth_img

    def grasp(self, position, heightmap_rotation_angle, workspace_limits):
        self.logger.info('Executing: grasp at (%f, %f, %f)' %
              (position[0], position[1], position[2]))

        grasp_orientation = [1.0, 0.0]
        # Compute tool orientation from heightmap rotation angle
        if heightmap_rotation_angle > np.pi:
            heightmap_rotation_angle = heightmap_rotation_angle - 2*np.pi
        tool_rotation_angle = heightmap_rotation_angle/2
        # NOTE: this is a weird way of saying [np.cos, np.sin]
        tool_orientation = np.asarray([
            grasp_orientation[0]*np.cos(tool_rotation_angle) -
            grasp_orientation[1]*np.sin(tool_rotation_angle),
            grasp_orientation[0]*np.sin(tool_rotation_angle) +
            grasp_orientation[1]*np.cos(tool_rotation_angle),
            0.0]) * np.pi

        position = np.asarray(position).copy()
        # Ensure z safety
        position[2] = max(position[2] - 0.05, workspace_limits[2][0])

        # -----
        # Attempt grasp 
        # -----

        # Go to grasp xy location, but stay high
        position[2] += 0.05
        above = {'type': 'p', 'pose': np.append(position, tool_orientation),
                 'acc': self.joint_acc * 0.5, 'vel': self.joint_vel * 0.5, 'rad': 0.09}

        # Descend
        position[2] -= 0.05
        down = {'type': 'p', 'pose': np.append(position, tool_orientation),
                'acc': self.joint_acc * 0.1, 'vel': self.joint_vel * 0.1, 'rad': 0.00}

        # Block until robot reaches target tool position
        self.r.combo_move([{'type': 'open'}, above, down, {'type': 'close'},
                           down],
                          wait=True)
        self.logger.info('combo move got to object down position')
        self.logger.info(down)


        is_gripper_open = self.r.check_grasp()

        if is_gripper_open:
            # We've grasped something. Move to pretoss position
            # blend in nice arc b/tw current pos and pretoss pos
            # blend_radius = min(0.02,
            #                    abs(home_position[1] - position[1])/2 - 0.01)
            # position[2] = home_position[2]  # pretoss_position[2]

            self.logger.info(" grasped object, going home now")


            start_gripper_width = self.r.get_state('gripper_width')
            # NOTE: May decide to have separate THROW HOME in future
            self.r.move_to(constants.GRASP_HOME, tool_orientation)
            self.logger.info("reached home")

            # Check if grip has slipped (width has changed). If not, success
            if np.abs(self.r.get_state('gripper_width') - start_gripper_width) < 0.1:
                grasp_primitive_success = True
            else:
                # failed to keep object in grasp, move up (z+0.1)& to home to try again
                self.logger.info("!----- failed to keep object grasped ---")
                grasp_primitive_success = False
                position[2] += 0.1
                self.r.move_to(position, tool_orientation, radius=0.09)
                self.r.move_to(constants.GRASP_HOME, tool_orientation)
        return grasp_primitive_success

    def throw(self, position):
        self.logger.info('Executing: throw at (%f, %f, %f)' %
              (position[0], position[1], position[2]))
        pretoss_jconf = np.array([0.000000, -0.7854, 1.571, -1.726, -1.570796,
                                  3.14159])
        posttoss_jconf = np.array([0.000000, -1.0088, 0.613, -2.522, -1.570796,
                                   3.14159])

        pretoss = {'type': 'j', 'pose': pretoss_jconf,
                   'acc': 8.00, 'vel': 3.00, 'rad': 0.09}
        posttoss = {'type': 'j', 'pose': posttoss_jconf,
                    'acc': 28.00, 'vel': 4.20, 'rad': 0.4}
        back_to_pretoss = {'type': 'j', 'pose': pretoss_jconf,
                           'acc': 3.00, 'vel': 3.00, 'rad': 0.0}

        # Block until robot reaches target tool position and gripper fingers have stopped moving
        self.r.combo_move([{'type': 'close'}, pretoss, posttoss, {'type': 'open'},
                           back_to_pretoss], wait=True)

    # TODO fill in this function
    def restart_real(self):
        self.logger.info('DEBUG: restarting real')
