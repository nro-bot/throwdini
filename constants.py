import numpy as np

GRASP_PREDICTIONS = "grasp_predictions"
VALID_DEPTH_HEIGHTMAP = "valid_depth_heightmap"
COLOR_HEIGHTMAP = "color_heightmap"
NO_CHANGE_COUNT = "no_change_count"
EXECUTING_ACTION = "executing_action"
PRIMITE_ACTION = "primitive_action"
BEST_PIX_IND = "best_pix_ind"
PUSH_SUCCESS = "push_success"
GRASP_SUCCESS = "grasp_success"

TCP_HOST_IP = "10.75.15.199"
TCP_PORT = 30002
# IP and port to robot arm as real-time client (UR5)
RTC_HOST_IP = "10.75.15.199"
RTC_PORT = 30003

CALIBRATE_HOME = np.deg2rad(np.array([0, -16., 90., -253, -86.5, -181.]))
CALIBRATE_TOOL_ORIENTATION = [1.23, -1.19, -1.19]

# Cols: min max, Rows: x y z (define workspace limits in robot coordinates)

WORKSPACE_LIMITS = np.asarray(
    [[-0.700, -0.550], [-0.250, 0.150], [-0.240, -0.090]])
