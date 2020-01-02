import socket
import time
import numpy as np
from math import pi, sin, cos
import urx

rob = urx.Robot("10.42.0.128")

HOST = "10.42.0.128"    # The remote host
PORT = 30003              # The same port as used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
time.sleep(0.5)


pose0 = np.array([0.089, -0.40, 0.1, 1.6, 0, 0])


s.send("movel(p{}, a=1, v=0.1)\n".format(str(list(pose0))))
# time.sleep(3)
while True:
    pose = rob.getl()
    diff = np.sum((pose - pose0)**2)
    print "diff to initial pose", diff
    if diff < 1e-4:
        break
time.sleep(0.5)

R = 1
center = pose0 - [R, 0, 0, 0, 0, 0]

a = 2.0
v = 0.01

w = v / R
dt = 0.008
# dt = 0.05


tm = 0
theta = 0

T = 2 * pi * R / v

print(w)
try:

    for t in range(int(T)):
        pose = rob.getl()
        print("robot tcp is at: ", pose)
        # print("absolute move in base coordinate ")

        ori = cos(theta)
        dp = np.zeros(6)
        dp[0] = 1 / (1+ori**2)**0.5
        dp[1] = ori / (1+ori**2)**0.5
        dp = np.array(dp)
        dp[:3] *= v
        # dp[:3] *= 1.2

        # cmd =  "movel(p{}, a={}, v={})".format(str(list(pose)), a, v) + "\n"
        cmd =  "speedl({}, a={}, t={})".format(str(list(dp)), a, dt) + "\n"

        print(cmd)
        s.send(cmd)
        time.sleep(dt)

        theta = theta + 2*pi/T
        print((theta/pi*180)%360)

finally:
    rob.close()

s.close()