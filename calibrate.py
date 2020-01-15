#!/usr/bin/env python

import sys
import time
import logging
import cv2
import numpy as np

from scipy import optimize
# from robot import Robot

import constants
from logger import ColoredFormatter
from physical.tcpUR.pyUR import PyUR
from real.camera import Camera

# Pretty print -- rather unnecessary, can replace with print() if desired
logging.basicConfig(stream=sys.stdout, level=logging.INFO)
logger = logging.getLogger('Calibration Logger')
logger.propagate = False

# add color formatting
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(ColoredFormatter())
logger.addHandler(ch)
logger.warning("Now running calibration script, prepare for oobot moving")

# ---------------------------------------------
workspace_limits = constants.WORKSPACE_LIMITS

calib_grid_step = constants.GRID_STEP

checkerboard_offset_from_tool = [-0.0572, 0.000, 0.0185]
tool_orientation = constants.CALIBRATE_TOOL_ORIENTATION

# --- (Original)
'''
# checkerboard_offset_from_tool = [0, -0.13, 0.02]
# tool_orientation = [-np.pi/2, 0, 0]
# Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
# self.move_joints(home_joint_config) = 
# [-(180.0/360.0)*2*np.pi, -(84.2/360.0)*2*np.pi, (112.8/360.0)*2*np.pi, -(119.7/360.0)*2*np.pi, -(90.0/360.0)*2*np.pi, 0.0]
##workspace_limits = np.asarray([[0.3, 0.748], [0.05, 0.4], [-0.2, -0.1]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
##calib_grid_step = 0.05
## checkerboard_offset_from_tool = [0,-0.13,0.02]
## tool_orientation = [-np.pi/2,0,0] # [0,-2.22,2.22] # [2.22,2.22,0]
# Make robot gripper point upwards
# NOTE: this results in straight out robot, with gripper up, and facing me
## robot.move_joints([-np.pi, -np.pi/2, np.pi/2, 0, np.pi/2, np.pi]) 
# NOTE: this tool orientation = all the way opposite around!  Use +1.75, 0, 0 instead?
## robot.move_to(gridpoint_xyz, tool_orientation)
# NOTE: above doesn't seem to lead to sensible calibration steps with my workspace limits :(
'''

# ---------------------------------------------

# Construct 3D calibration grid across workspace
gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], 1 + (
    workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step)
gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], 1 + (
    workspace_limits[1][1] - workspace_limits[1][0])/calib_grid_step)
gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], 1 + (
    workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_step)
calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(
    gridspace_x, gridspace_y, gridspace_z)

num_calib_grid_pts = calib_grid_x.shape[0] * \
    calib_grid_x.shape[1]*calib_grid_x.shape[2]

logger.debug('Connecting to robot...')
logger.info("Calibrating using # grid points: %s" %
            str(num_calib_grid_pts))

calib_grid_x.shape = (num_calib_grid_pts, 1)
calib_grid_y.shape = (num_calib_grid_pts, 1)
calib_grid_z.shape = (num_calib_grid_pts, 1)
calib_grid_pts = np.concatenate(
    (calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

measured_pts = []
observed_pts = []
observed_pix = []

# ---------------------------------------------
# Move robot to home pose
robot = PyUR(send_ur5_progs=True)
robot.open_gripper()
MyCam = Camera()

# Slow down robot (USE physical PENDANT to do so for now)

# Make robot gripper point upwards
#robot.move_joints([-np.pi, -np.pi/2, np.pi/2, 0, np.pi/2, np.pi])
robot.move_joints(constants.CALIBRATE_HOME[:3], constants.CALIBRATE_HOME[3:])

# Move robot to each calibration point in workspace


logger.debug('Collecting data...')

start = time.time()
for calib_pt_idx in range(num_calib_grid_pts):
    gridpoint_xyz = calib_grid_pts[calib_pt_idx, :]

    dt = time.time() - start
    logger.info('# %d/%d . Moving to: %s. Elapsed: %.1f secs' % (calib_pt_idx,
                                                                 num_calib_grid_pts,
                                                                 gridpoint_xyz,
                                                                 dt))

    robot.move_to(gridpoint_xyz, tool_orientation)
    time.sleep(2.5)

    # Find checkerboard center
    checkerboard_size = (3, 3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS +
                       cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    camera_color_img, camera_depth_img = MyCam.get_data()
    bgr_color_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
    gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_RGB2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(
        gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        logger.debug("Found checkerboard.")
        corners_refined = cv2.cornerSubPix(
            gray_data, corners, (3, 3), (-1, -1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4, 0, :]).astype(int)
        checkerboard_z = camera_depth_img[checkerboard_pix[1]
                                          ][checkerboard_pix[0]]
        checkerboard_x = np.multiply(
            checkerboard_pix[0] - MyCam.intrinsics[0][2], checkerboard_z / MyCam.intrinsics[0][0])
        checkerboard_y = np.multiply(
            checkerboard_pix[1] - MyCam.intrinsics[1][2], checkerboard_z / MyCam.intrinsics[1][1])
        if checkerboard_z == 0:
            logger.debug('no depth info found')
            continue

        # Save calibration point and observed checkerboard center
        observed_pts.append(
            [checkerboard_x, checkerboard_y, checkerboard_z])
        # gridpoint_xyz[2] += checkerboard_offset_from_tool
        checker_position = gridpoint_xyz + checkerboard_offset_from_tool

        logger.debug('I measured (calculated)' + str(checker_position))
        logger.debug('I observed (realsense) %.2f %.2f %.2f' %
                     (checkerboard_x, checkerboard_y, checkerboard_z))

        measured_pts.append(gridpoint_xyz)
        observed_pix.append(checkerboard_pix)

        # Draw and display the corners
        # vis = cv2.drawChessboardCorners(robot.camera.color_data, checkerboard_size, corners_refined, checkerboard_found)
        vis = cv2.drawChessboardCorners(
            bgr_color_data, (1, 1), corners_refined[4, :, :], checkerboard_found)
        cv2.imwrite('%06d.png' % len(measured_pts), vis)
        cv2.imshow('Calibration', vis)
        cv2.waitKey(10)


# Move robot back to home pose
logger.info('Going home now!')
robot.move_joints(constants.CALIBRATE_HOME[:3], constants.CALIBRATE_HOME[3:])


measured_pts = np.asarray(measured_pts)
observed_pts = np.asarray(observed_pts)
observed_pix = np.asarray(observed_pix)
world2camera = np.eye(4)


# -----------
# Calculate matrix from camera to robot coords
# -----------

# Estimate rigid transform with SVD (from Nghia Ho)

def get_rigid_transform(A, B):
    assert len(A) == len(B)
    N = A.shape[0]  # Total points
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - np.tile(centroid_A, (N, 1))  # Centre the points
    BB = B - np.tile(centroid_B, (N, 1))
    H = np.dot(np.transpose(AA), BB)  # Dot is matrix multiplication for array
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:  # Special reflection case
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)
    t = np.dot(-R, centroid_A.T) + centroid_B.T
    return R, t


def get_rigid_transform_error(z_scale):
    global measured_pts, observed_pts, observed_pix, world2camera, camera

    # Apply z offset and compute new observed points using camera intrinsics
    observed_z = observed_pts[:, 2:] * z_scale
    observed_x = np.multiply(observed_pix[:, [
        0]] - MyCam.intrinsics[0][2], observed_z / MyCam.intrinsics[0][0])
    observed_y = np.multiply(observed_pix[:, [
        1]] - MyCam.intrinsics[1][2], observed_z / MyCam.intrinsics[1][1])
    new_observed_pts = np.concatenate(
        (observed_x, observed_y, observed_z), axis=1)

    # Estimate rigid transform between measured points and new observed points
    R, t = get_rigid_transform(np.asarray(
        measured_pts), np.asarray(new_observed_pts))
    t.shape = (3, 1)
    world2camera = np.concatenate(
        (np.concatenate((R, t), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

    # Compute rigid transform error
    registered_pts = np.dot(R, np.transpose(measured_pts)) + \
        np.tile(t, (1, measured_pts.shape[0]))
    error = np.transpose(registered_pts) - new_observed_pts
    error = np.sum(np.multiply(error, error))
    rmse = np.sqrt(error/measured_pts.shape[0])
    return rmse


# Optimize z scale w.r.t. rigid transform error
logger.debug('Calibrating...')

z_scale_init = 1
optim_result = optimize.minimize(
    get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
camera_depth_offset = optim_result.x

# Save camera optimized offset and camera pose
logger.debug('Saving...')
logger.info('observed_pix \n%s' % str(observed_pix))
logger.info('measured_ptx \n%s' % str(measured_pts))
logger.info('observed_pix \n%s' % str(observed_pts))
logger.info('len observed_pix \n%s' % str(len(observed_pts)))

np.savetxt('real/measured_pts.txt', measured_pts, delimiter=' ')
np.savetxt('real/observed_pts.txt', observed_pts, delimiter=' ')
np.savetxt('real/observed_pix.txt', observed_pix, delimiter=' ')
np.savetxt('real/camera_depth_scale.txt', camera_depth_offset, delimiter=' ')

get_rigid_transform_error(camera_depth_offset)
camera_pose = np.linalg.inv(world2camera)
logger.warning('camera pose\n%s' % str(camera_pose))
np.savetxt('real/camera_pose.txt', camera_pose, delimiter=' ')

logger.info('Done.')
