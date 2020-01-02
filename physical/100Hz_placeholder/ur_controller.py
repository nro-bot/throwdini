#! /usr/bin/env python
# -*- coding: utf-8
import math
import numpy as np
import socket
import time
from math import pi, sin, cos
import urx
import math3d as m3d
from . import util
# from util import rotationMatrixToEulerAngles, eulerAnglesToRotationMatrix
import cv2
from threading import Thread
import random

def rotate_theta(v, theta):
    return np.array([
        cos(theta)*v[0] - sin(theta)*v[1],
        sin(theta)*v[0] + cos(theta)*v[1]])

class UR_Controller(Thread):
    def __init__(self, HOST="10.42.0.128", PORT=30003):
        Thread.__init__(self)

        self.rob = urx.Robot(HOST, use_rt=True)

        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((HOST, PORT))

        # self.pose0 = np.array([0.05, -0.40, 0.1, 0.72901107, -1.7599884, 1.7599884])

        self.pose0 = np.array([-0.27, -0.39, 0.17, 0.72901107, -1.7599884, 1.7599884])
        # self.pose1 = np.array([-0.22, -0.39, 0.17, 0.72901107, -1.7599884, 1.7599884])

        self.theta = pi
        self.theta_goal = pi

        self.dt = 0.02

        self.distance = 0.22
        self.relative_vector = np.array([0., -self.distance, 0.]) #gelsight tip relative to robot tip

        self.sign = 1.

        self.gelsight_theta = 0.
        # self.limitx = 0.2
        self.limitx = 0.25
        self.limity = 0.08
        self.ready_to_grasp = False 

        self.robot_on = False
        self.running = False # UR5 Only move when running = False
        self.go_forward = False
        self.finished = True # Main controller can read whether current command is finished

        self.v_out = np.array([0. ,0.])
        self.dp = None
        self.flag_contact = False
        self.cable_center = np.array([0., 0.])
        self.open_curve = False
        self.offset = 1.
        self.target_v = np.array([0., 0.])
        # print(util.rotationMatrixToEulerAngles)

    def __del__(self):
        self.rob.close()
        self.s.close()
        

    def getl_rt(self):
        # get current pose with urx urrtmon with 125 Hz
        return self.rob.rtmon.getTCF(True)

    def send(self, cmd):
        cmd = str.encode(cmd)
        self.s.send(cmd)

    def speedl(self, v, a=0.5, t=0.05):
        # send speedl command in socket
        cmd =  "speedl({}, a={}, t={})\n".format(str(list(v)), a, t)
        self.send(cmd)

        return time.time()

    def movel_home(self, a=1, v= 0.1):


        self.flag_contact = False
        self.go_forward = False
        time.sleep(0.5)

        # noise_intensity = 0.02
        noise_intensity = 0.
        pose = self.pose0 + [0., random.random()*noise_intensity - noise_intensity/2, 0, 0, 0, 0]

        # pose = self.pose0 + [0., -self.cable_center[1]/5000., 0, 0, 0, 0]
        pose = self.pose0.copy()

        r = np.array([0., -self.distance, 0.])
        self.xyz_target_gelsight = pose[:3] + r

        self.movel_wait(pose, a, v)


        self.go_forward = True

        time.sleep(1)

        if np.abs(self.cable_center[1]/5000.) > 2e-3:
            # Regrasp
            self.open_curve = True
            time.sleep(0.5)
            pose = self.pose0 + [0., +self.cable_center[1]/5000.*self.offset, 0, 0, 0, 0]

            r = np.array([0., -self.distance, 0.])
            self.xyz_target_gelsight = pose[:3] + r

            self.offset += 0.1
            if self.offset > 1.2:
                self.offset = 1.2

            self.movel_wait(pose, a, v)

            self.open_curve = False
            time.sleep(1)



        #########

        self.tm_last_contact = time.time()
        self.last_dp = None

    def movel_wait(self, pose, a=1, v=0.04):
        # linear move in tool space and wait
        s = self.s

        cmd = "movel(p{}, a={}, v={})\n".format(str(list(pose)), a, v)
        cmd = str.encode(cmd)
        s.send(cmd)

        while True:
            p = self.getl_rt()
            diff = np.sum((p - pose)**2)
            if diff < 1e-5:
                break
            time.sleep(0.02)

    def get_euler(self, pose):
        trans = self.rob.csys.inverse * m3d.Transform(pose)
        rotation_matrix = trans.get_orient().array
        e = util.rotationMatrixToEulerAngles(rotation_matrix)
        return e

    def get_euler_z(self, pose):
        e = self.get_euler(pose)
        theta = e[2]
        if theta < 0.:
            theta += 2*pi
        return theta

    def get_target_xyz(self, theta0, theta, xyz_target_gelsight):
        R = util.eulerAnglesToRotationMatrix([0., 0., theta-theta0])
        # print('R', R)
        r = np.array([0., -self.distance, 0.])
        # xyz_target_gelsight = self.pose0[:3] + r
        xyz_robot_tcp = xyz_target_gelsight - R.dot(r) 
        return xyz_robot_tcp


    def get_target_xyz_right(self, theta0, theta, xyz_target_gelsight):
        R = util.eulerAnglesToRotationMatrix([0., 0., theta-theta0])
        # print('R', R)
        r = np.array([0., -self.distance, 0.])
        # xyz_target_gelsight = self.pose0[:3] + r
        xyz_robot_tcp = xyz_target_gelsight - R.dot(r) 
        return xyz_robot_tcp


    def check_safe_zoom(self,pose):
        if pose[0]+ self.relative_vector[0] < self.pose0[0]-1e1:
            self.sign = 1
            self.movel_home(v=0.1)
            print('out of safe range')

        if pose[0] + self.relative_vector[0] > self.pose0[0] + self.limitx:
            # self.sign = -1
            # print('out of safe range')
            r = np.array([0., -self.distance, 0.])
            self.xyz_target_gelsight = self.pose0[:3] + r

            # self.movel_wait(self.pose0, v=0.1)
            self.movel_home(v=0.1)

        if pose[1] + self.relative_vector[1]  < self.pose0[1] -self.distance - self.limity or pose[1] + self.relative_vector[1] > self.pose0[1] -self.distance + self.limity:
            # self.v_norm = 0.01

            r = np.array([0., -self.distance, 0.])
            self.xyz_target_gelsight = self.pose0[:3] + r
            time.sleep(0.2)
            # self.movel_wait(self.pose0, v=0.1)
            self.movel_home(v=0.1)

            print('out of safe range')

        if self.cable_center[1]/5000 > 6e-3 or self.cable_center[1]/5000 < -6e-3:
            self.movel_home(v=0.1)
            print('out of safe range')



    def rotate(self):
        rob = self.rob
        self.start_rotate = False

        a = 2.0
        dt = 0.01
        # dt = 0.03

        theta0 = pi 
        theta = pi
        w = 0.01
        xyz_robot = self.pose0[:3] 

        theta_goal = pi
        xyz_goal = self.pose0[:3] 

        # theta = self.get_euler_z(self.pose0)
        # xyz_goal_test = self.get_target_xyz(theta0, theta+pi/6)
        # print ('xyz_goal', xyz_goal, 'xyz_goal_test', xyz_goal_test)
        r = np.array([0., -self.distance, 0.])
        self.xyz_target_gelsight = self.pose0[:3] + r


        kp = 36./pi / 2.0
        kd = 36./pi * 1

        kp_xyz = -20.
        # kp_xyz = 0. 
        kd_xyz = 0.
        dv = np.array([0.,0.,0.]) 
        self.v_norm = 0.


        theta_error_p = 0.
        xyz_error_p = 0.
        v_vector = np.array([0.,0.,0.]) 
        v_vector_p = np.array([0.,0.,0.]) 

        time_start = time.time()

        while not self.start_rotate:
            time.sleep(0.02)

        time.sleep(3)

        self.go_forward = True

        self.flag_contact = False

        self.tm_last_contact = time.time()
        self.last_dp = None

        while True:

            theta_goal = self.theta_goal


            # PD controller for xyz 
     

            # PD control for theta
            # theta_error = (theta_goal - theta) 
            theta_target = theta - theta0 + self.gelsight_theta
            # theta_target = theta0


            # v_vector = np.array([self.v_norm*self.sign*cos(theta_target), self.v_norm*self.sign*sin(theta_target), 0.])
            v_vector = np.array([0.01, 0., 0.])
            # print('v_vector',v_vector*100, 'theta_target', theta_target/pi*180)

            theta_error = self.gelsight_theta
            # theta_error = 0
            w = kp * theta_error + kd * (theta_error - theta_error_p)
            w /= 20
            # w = 0.
            theta_error_p = theta_error

            w_vector = np.array([0., 0., w])

            self.pose = self.getl_rt()
            pose = self.pose

            xyz_robot = pose[:3].copy()
            theta = self.get_euler_z(pose)
            self.theta = theta

            time_end = time.time()
            time_diff = time_end - time_start 

            if self.gelsight_theta != 0:
                self.flag_contact = True
                v_out = rotate_theta(self.v_out, (theta - theta0))

                # if xyz_robot[0] - self.pose0[0] < 0.0:
                #     self.xyz_target_gelsight += (np.array([1., 0., 0.])) * time_diff * 0.02
                # else:
                    # *np.abs(v_out[1])*2
                self.xyz_target_gelsight += (np.array([v_out[0], v_out[1] *np.abs(v_out[1]) * 2, 0.])) * time_diff * 0.02
                # self.xyz_target_gelsight += (np.array([v_out[0], v_out[1] * 2, 0.])) * time_diff * 0.02
                # self.xyz_target_gelsight += (np.array([0.016, 0., 0.])) * time_diff
                # self.xyz_target_gelsight += (np.array(v_vector_p)) * time_diff
                self.tm_last_contact = time.time()

            else:
                if self.flag_contact:
                    self.movel_home()
                if time.time() - self.tm_last_contact > 2:
                    self.movel_home()


            self.relative_vector = np.array([sin(theta-theta0)*self.distance, cos(theta-theta0)*-self.distance, 0.])


            xyz_goal = self.get_target_xyz(theta0, theta, self.xyz_target_gelsight)

            xyz_error = xyz_robot - xyz_goal 
            # print('xyz_error', xyz_error, 'time_diff', time_diff)
            # print('xyz_robot', xyz_robot, 'xyz_goal', xyz_goal)
            dv = kp_xyz * xyz_error + kd_xyz * (xyz_error - xyz_error_p)
            xyz_error_p = xyz_error.copy()

            v_tcp_robot = v_vector - np.cross(w_vector, self.relative_vector) + dv


            # dp = [v_tcp_robot[0], v_tcp_robot[1], v_tcp_robot[2] , 0, 0, w]
            # dp = np.array([v_tcp_robot[0], v_tcp_robot[1], v_tcp_robot[2] , 0, 0, 0])
            dp = np.array([self.target_v[0], self.target_v[1], 0, 0, 0, 0])

            # if xyz_robot[0] - self.pose0[0] > 0.:
            #     noise_v = 60 / 180. * pi
            #     noise_theta = random.random() * noise_v - noise_v / 2
            #     dp[:2] = rotate_theta(dp[:2], noise_theta)

            # alpha = 0.7
            # if self.last_dp is not None:
            #     dp = alpha * self.last_dp + (1 - alpha) * dp

            # self.last_dp = dp.copy()

            self.dp = dp

            if (theta > pi+pi * 0.25 and w > 0) or (theta < pi - pi * 0.25  and w < 0):
                dp = [0.] * 6

            if xyz_robot[0] - self.pose0[0] > 0.03:
                self.offset = 1.

            print('dp', dp)
            print("cable offset", self.cable_center)
            self.check_safe_zoom(pose)
            time_start = self.speedl(dp, a, dt*4)
            v_vector_p = v_vector.copy()
            
            time.sleep(dt)

    def follow(self):
        while True:
            if self.robot_on == False or self.running == True:
                time.sleep(0.02)
                continue

            time.sleep(2)
            print("UR5",self.go_forward)
            if self.go_forward:
                self.running = True
                self.movel_wait(self.pose1)
            else:
                self.running = True
                self.movel_wait(self.pose0)
            time.sleep(0.5)
            self.finished = True



    def run(self):
        print("run")
        self.movel_wait(self.pose0)
        # self.movel_wait(self.pose1)
        ##add go to initial position 

        self.ready_to_grasp = True 
        # self.follow()
        self.rotate()

        pass