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
# help='IP address to robot arm as TCP client (UR5)')
TCP_HOST_IP = "10.75.15.199"
# help='port to robot arm as TCP client (UR5)')
TCP_PORT = 30002
# help='IP address to robot arm as real-time client (UR5)')
RTC_HOST_IP = "10.75.15.199"
# help='port to robot arm as real-time client (UR5)')
RTC_PORT = 30003

# parser.add_argument('--heightmap_resolution', dest='heightmap_resolution',

CALIBRATE_HOME = np.deg2rad(np.array([0, -16., 90., -253, -86.5, -181.]))
CALIBRATE_TOOL_ORIENTATION = [1.23, -1.19, -1.19]
# When grasping (vs calibrating), where robot should go when turn on

GRASP_START_HOME = np.deg2rad([-10.5, -50.5, 125., -71.9, -277.5, -1.4])
GRASP_HOME = [-0.445, -0.238, -0.150]
# NOTE: May decide to have separate THROW HOME  

# [min max] for axes: x y z (define workspace limits in robot coordinates)
WORKSPACE_LIMITS = np.asarray(
    [[-0.700, -0.550], [-0.250, 0.150], [-0.240, -0.090]])

JOINT_POSE_TOLERANCE = [0.005, 0.005, 0.005, 0.020, 0.020, 0.020]
DEFAULT_JOINT_VEL = 1.0
DEFAULT_JOINT_ACC = 0.7

GRIPPER_OPEN_THRESH = 0.26
