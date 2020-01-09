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

from pyUR import PyUR

from real.camera import Camera
import constants


class Robot(object):
    def __init__(self, is_bullet_sim, send_ur5_progs):

        self.orig = False
        self.is_bullet_sim = False

        self.r = PyUR(send_ur5_progs)
        # Move robot to home pose
        self.r.go_home()
        # self.r.activate_gripper()
        # self.r.close_gripper()
        # self.r.open_gripper()

        # Fetch RGB-D data from RealSense camera
        self.camera = Camera()
        self.cam_intrinsics = self.camera.intrinsics

        # Load camera pose (from running calibrate.py), intrinsics and depth scale
        self.cam_pose = np.loadtxt("real/camera_pose.txt", delimiter=" ")
        self.cam_depth_scale = np.loadtxt(
            "real/camera_depth_scale.txt", delimiter=" ")

    def get_camera_data(self):

        if self.is_bullet_sim:
            pass

        else:
            # Get color and depth image from ROS service
            color_img, depth_img = self.camera.get_data()

        return color_img, depth_img

    def grasp(self, position, heightmap_rotation_angle, workspace_limits):
        print('Executing: grasp at (%f, %f, %f)' %
              (position[0], position[1], position[2]))

        if self.is_bullet_sim:
            pass

        else:
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
            tool_orientation_angle = np.linalg.norm(tool_orientation)
            tool_orientation_axis = tool_orientation / tool_orientation_angle
            tool_orientation_rotm = utils.angle2rotm(
                tool_orientation_angle, tool_orientation_axis, point=None)[:3, :3]

            # Compute tilted tool orientation during dropping into bin
            tilt_rotm = utils.euler2rotm(np.asarray([-np.pi/4, 0, 0]))
            tilted_tool_orientation_rotm = np.dot(
                tilt_rotm, tool_orientation_rotm)
            tilted_tool_orientation_axis_angle = utils.rotm2angle(
                tilted_tool_orientation_rotm)
            tilted_tool_orientation = tilted_tool_orientation_axis_angle[0]*np.asarray(
                tilted_tool_orientation_axis_angle[1:4])

            if tool_orientation_angle > np.pi:
                print('SAFETYING tool orientation angle (> pi)- subtract 2 pi')
                tool_orientation_angle = tool_orientation_angle - 2*np.pi

            position = np.asarray(position).copy()
            # Ensure z safety
            position[2] = max(position[2] - 0.05, workspace_limits[2][0])

            # Attempt grasp
            position[2] += 0.05
            above = {'type': 'p', 'pose': np.append(position, tool_orientation),
                     'acc': self.joint_acc * 0.5, 'vel': self.joint_vel * 0.5, 'rad': 0.09}
            position[2] -= 0.05
            down = {'type': 'p', 'pose': np.append(position, tool_orientation),
                    'acc': self.joint_acc * 0.1, 'vel': self.joint_vel * 0.1, 'rad': 0.00}

            # Block until robot reaches target tool position and gripper fingers have stopped moving
            self.r.combo_move([{'type': 'open'}, above, down, {'type': 'close'},
                               down],
                              wait=True)
            print('combo move got to object down position', down)

            # NOTE: HARDCODED

            # prepick_position = [0, 0, 0]
            # pretoss_position = [0, 0, 0]

            gripper_width = self.r.get_state('gripper_width')
            print('!---gripper_width', gripper_width)
            # NOTE: HARDCODED
            gripper_open = gripper_width > 0.26

            grasp_success = False
            if gripper_open:
                # We've grasped something. Move to pretoss position
                # blend in nice arc b/tw current pos and pretoss pos
                # blend_radius = min(0.02,
                                   # abs(home_position[1] - position[1])/2 - 0.01)

                # position[2] = home_position[2]  # pretoss_position[2]
                # position[2] = home_position[2]  # pretoss_position[2]

                print("!----- grasped object, going home now ---")
                # home_position = [-0.440, -0.170, -0.200]
                home_position = [-0.445, -0.238, -0.150]

                home = {'type': 'p', 'pose': np.append(home_position, tool_orientation),
                        'acc': self.joint_acc, 'vel': self.joint_vel, 'rad': 0.00}

                # self.r.combo_move([home], wait=True) #TODO: why does this
                # fail?
                self.r.move_to(home_position, tool_orientation,
                               override_safety=True)
                print("!--reached home--")
                # self.r.move_to(home_position, tool_orientation, radius=blend_radius,
                # wait=True)

                # Check if grip has slipped (width has changed). If not, success
                # TODO: this relies on gripper continuing to try to close
                print('measured gripper width',
                      self.r.get_state('gripper_width'))
                if np.abs(self.r.get_state('gripper_width') - gripper_width) < 0.1:
                    grasp_success = True
            else:
                print("!----- failed to grasp object ---")
                # failed to keep object in grasp, move up & to home to
                # prepare to try again
                position[2] += 0.1
                self.r.move_to(position, tool_orientation,
                               acc=self.joint_acc,
                               vel=self.joint_vel,
                               radius=0.09)
                self.r.combo_move([home], wait=True)
                # self.r.move_to(home_position, tool_orientation,
            return grasp_success

    def throw(self, position, heightmap_rotation_angle, workspace_limits):
        print('Executing: throw at (%f, %f, %f)' %
              (position[0], position[1], position[2]))
        pretoss_jconf = np.array([0.000000, -0.7854, 1.571, -1.726, -1.570796,
                                  3.14159])
        posttoss_jconf = np.array([0.000000, -1.0088, 0.613, -2.522, -1.570796,
                                   3.14159])

        pretoss = {'type': j, 'pose': pretoss_jconf,
                   'acc': 8.00, 'vel': 3.00, 'rad': 0.09}
        posttoss = {'type': j, 'pose': posttoss_jconf,
                    'acc': 28.00, 'vel': 4.20, 'rad': 0.4}
        back_to_pretoss = {'type': j, 'pose': pretoss_jconf,
                           'acc': 3.00, 'vel': 3.00, 'rad': 0.0}

        # Block until robot reaches target tool position and gripper fingers have stopped moving
        self.r.combo_move([{'type': 'close'}, pretoss, posttoss, {'type': 'open'},
                           back_to_pretoss], wait=True)

    def restart_real(self):

        print('DEBUG: restarting real')
