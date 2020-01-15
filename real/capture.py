#!/usr/bin/env python

import time
import matplotlib.pyplot as plt
from camera import Camera
import cv2

camera = Camera()
time.sleep(1) # Give camera some time to load data

timestr = time.strftime("%Y%m%d-%H%M%S")


def sanity_check_plt():
    while True:
        color_img, depth_img = camera.get_data()
        plt.subplot(211)
        plt.imshow(color_img)
        plt.subplot(212)
        plt.imshow(depth_img)
        plt.show()

def save_img():
    color_img, depth_img = camera.get_data()
    color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
    cv2.imwrite('tmp_%s.color.png' % timestr, color_img)


save_img()

