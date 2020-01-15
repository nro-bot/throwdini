"""
These should mostly be constant from run-to-run
"""
import numpy as np  # TODO yes I should calc actual constants and remove depend

GRASP_PREDICTIONS = "grasp_predictions"
VALID_DEPTH_HEIGHTMAP = "valid_depth_heightmap"
COLOR_HEIGHTMAP = "color_heightmap"
NO_CHANGE_COUNT = "no_change_count"
EXECUTING_ACTION = "executing_action"
PRIMITE_ACTION = "primitive_action"
BEST_PIX_IND = "best_pix_ind"
PUSH_SUCCESS = "push_success"
GRASP_SUCCESS = "grasp_success"

# ---- Physical robot setup ----

# -- Networking
# help='IP address , port to robot arm as TCP client (UR5)')
TCP_HOST_IP = "10.75.15.199"
TCP_PORT = 30002
# help='IP address , port to robot arm as real-time client (UR5)')
RTC_HOST_IP = "10.75.15.199"
RTC_PORT = 30003

USE_GLOBAL_SEND_UR5_PROGS = True 
# otherwise defined by instance init args, or if not specified on init default to TruE
GLOBAL_SEND_UR5_PROGS = False
# Mostly use this as a flag to DEFINITELY not send ur5 progs

# -- Movement

JOINT_POSE_TOLERANCE = [0.005, 0.005, 0.005, 0.020, 0.020, 0.020]
DEFAULT_JOINT_VEL = 1.0
DEFAULT_JOINT_ACC = 0.7

# [min max] for axes: x y z (in robot coordinates, in meters)
# ([[0.3, 0.748], [-0.224, 0.224], [-0.255, -0.1]])  -- orig = 0.448 x 0.448 x 0.155
# w
WORKSPACE_LIMITS = np.asarray(
    [[-0.600, -0.550], [-0.150, 0.100], [-0.280, -0.150]])
    #[[-0.700, -0.550], [-0.250, 0.150], [-0.240, -0.090]])


# -- Camera
HEIGHTMAP_RESOLUTION = 0.00115
# parser.add_argument('--heightmap_resolution', dest='heightmap_resolution',


# -- Gripper
GRIPPER_OPEN_THRESH = 0.26

# -- Calibrate.py
CALIBRATE_HOME = np.deg2rad(np.array([0, -16., 90., -253, -86.5, -181.]))
CALIBRATE_TOOL_ORIENTATION = [1.23, -1.19, -1.19]
# When grasping (vs calibrating), where robot should go when turn on
_delta = np.array([ lim[1] - lim[0] for lim in WORKSPACE_LIMITS])
_delta = np.average(_delta)
# GRID_STEP = 0.15
GRID_STEP = _delta / 2

# -- Touch.py
TOUCH_DEBUG_TOOL_ORIENTATION = [2.22, -2.22, 0]

# -- Main.py
TOUCH_DEBUG_TOOL_ORIENTATION = [2.22, -2.22, 0]
GRASP_START_HOME = np.deg2rad([-10.5, -50.5, 125., -71.9, -277.5, -1.4])
GRASP_HOME = [-0.445, -0.238, -0.150]
# NOTE: May decide to have separate THROW HOME


# -- Ursecmon.py
PACKET_TIMEOUT = 1.0
FIND_FIRST_PACKET_ATTEMPTS = 101

#SEND_UR7_PROGS = True
