'''
author: nouyang
date: Jan 2020
'''

import constants
import sys
import utils
import logging
import numpy as np
from logger import ColoredFormatter
from tcpUR.pyUR import PyUR
from real.camera import Camera
import signal 


# ---------------------------------------------
# Pretty print -- rather unnecessary, can replace with print() if desired
logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
logger = logging.getLogger('Debug heightmap size')
logger.propagate = False

# add color formatting
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(ColoredFormatter())
logger.addHandler(ch)
logger.warning("Debugging heightmap size.")

# ---------------------------------------------
# Handle keyboard interrupts more gracefully
def keyboardInterruptHandler(signal, frame):
    logger.error('KeyboardInterrupt, closing camera and exiting')
    MyCam.close()
    sys.exit()

signal.signal(signal.SIGINT, keyboardInterruptHandler)

# ---------------------------------------------
# hardcoded for debugging
lims = constants.WORKSPACE_LIMITS
lims = np.asarray(
    [[-0.650, -0.500], [-0.150, -0], [-0.240, -0.090]])

rez = 0.002 #constants.HEIGHTMAP_RESOLUTION

diff = [ax[1] - ax[0] for ax in lims]
logger.debug('DIFF: %s' % str(diff))

#robot = PyUR(send_ur5_progs=True)
#middle = constants.WORKSPACE_AVG

cam_pose = np.loadtxt("real/camera_pose.txt", delimiter=" ")

# ---------------------------------------------
MyCam = Camera()
color_img, depth_img = MyCam.get_data()
logger.info('color_img.shape: %s' % str(color_img.shape))


'''
    heightmap_size = np.round(
        (
            (workspace_limits[1][1] - workspace_limits[1][0])/heightmap_resolution, 
            (workspace_limits[0][1] - workspace_limits[0][0]) /heightmap_resolution
        )
    )
    .astype(int)

# NOTE THAT X AND Y ARE FLIPPED
'''

color_heightmap, depth_heightmap = \
    utils.get_heightmap(color_img, depth_img, MyCam.intrinsics, cam_pose, lims, rez)
logger.info('color_heightmap.shape: %s' % str(color_heightmap.shape))


# ---------------------------------------------
MyCam.close()
