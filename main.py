'''
Train the ML models 
'''
#!/usr/bin/env pyth

import time
import os
import random
import threading
import argparse
import matplotlib.pyplot as plt
import numpy as np
import scipy as sc
import cv2
from collections import namedtuple
import torch
from torch.autograd import Variable
from trainer import Trainer
from logger import Logger
import logging
import utils
import constants
import runner
import mock_robot


def main(args):
    logging.basicConfig(level=logging.WARNING)

    # --------------- Setup options ---------------
    workspace_limits = constants.WORKSPACE_LIMITS

    #is_bullet_sim = args.is_bullet_sim  # Run in simulation?

    #heightmap_resolution = args.heightmap_resolution  # Meters per pixel of heightmap
    heightmap_resolution = constants.HEIGHTMAP_RESOLUTION 

    random_seed = args.random_seed
    force_cpu = args.force_cpu

    # ------------- Algorithm options -------------
    # 'reactive' (supervised learning) or 'reinforcement' (reinforcement learning ie Q-learning)
    method = args.method
    # Use immediate rewards (from change detection) for pushing?
    push_rewards = args.push_rewards if method == 'reinforcement' else None

    future_reward_discount = args.future_reward_discount
    experience_replay = args.experience_replay  # Use prioritized experience replay?

    # Use handcrafted grasping algorithm when grasping fails too many times in a row?
    heuristic_bootstrap = args.heuristic_bootstrap
    explore_rate_decay = args.explore_rate_decay
    grasp_only = args.grasp_only

    # -------------- Testing options --------------
    dont_train = args.dont_train

    # Maximum number of test runs per case/scenario
    max_test_trials = args.max_test_trials

    # ------ Pre-loading and logging options ------
    load_snapshot = args.load_snapshot  # Load pre-trained snapshot of model?
    snapshot_file = os.path.abspath(
        args.snapshot_file) if load_snapshot else None
    # Continue logging from previous session
    continue_logging = args.continue_logging
    logging_directory = os.path.abspath(
        args.logging_directory) if continue_logging else os.path.abspath('logs')
    # Save visualizations of FCN predictions? Takes 0.6s per training step if set to True
    save_visualizations = args.save_visualizations

    # Set random seed
    np.random.seed(random_seed)

    # Initialize pick-and-place system (camera and robot)
    if use_mock_robot:
        robot = mock_robot.MockRobot()
    else:
        from robot import Robot
        #robot = Robot(is_bullet_sim=args.is_bullet_sim, send_ur5_progs=args.send_ur5_progs)
        robot = Robot(send_ur5_progs=args.send_ur5_progs)

    # Initialize trainer
    trainer = Trainer(method, push_rewards, future_reward_discount,
                      dont_train, load_snapshot, snapshot_file, force_cpu)

    # Initialize data logger
    logger = Logger(continue_logging, logging_directory)
    # Save camera intrinsics and pose
    if not use_mock_robot:
        logger.save_camera_info(robot.cam_intrinsics,
                                robot.cam_pose, robot.cam_depth_scale)
    # Save heightmap parameters
    logger.save_heightmap_info(workspace_limits, heightmap_resolution)

    # Find last executed iteration of pre-loaded log, and load execution info and RL variables
    if continue_logging:
        trainer.preload(logger.transitions_directory)

    # Initialize variables for heuristic bootstrapping and exploration probability
    explore_prob = 0.5 if not dont_train else 0.0

    # Quick hack for nonlocal memory between threads in Python 2
    nonlocal_variables = {constants.EXECUTING_ACTION: False,
                          constants.PRIMITE_ACTION: None,
                          constants.BEST_PIX_IND: None,
                          constants.PUSH_SUCCESS: False,
                          constants.GRASP_SUCCESS: False,
                          constants.NO_CHANGE_COUNT: 2 if not dont_train else 0}

    # parallel thread to process network output and execute actions
    action_thread = threading.Thread(target=runner.process_actions, args=[
        nonlocal_variables, trainer, logger, robot, workspace_limits, heightmap_resolution,
        heuristic_bootstrap, save_visualizations
    ])
    action_thread.daemon = True
    action_thread.start()
    exit_called = False
    # -------------------------------------------------------------

    # Start main training/testing loop
    while True:
        self.logger.warning('\n%s iteration: %d' %
              ('Testing' if dont_train else 'Training', trainer.iteration))
        iteration_time_0 = time.time()

        # Make sure simulation is still stable (if not, reset simulation)
        # if is_bullet_sim:
            # robot.check_sim()

        # Get latest RGB-D image
        color_img, depth_img = robot.get_camera_data()
        # Apply depth scale from calibration
        depth_img = depth_img * robot.cam_depth_scale

        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        if use_mock_robot:
            color_heightmap = color_img
            depth_heightmap = depth_img
        else:
            color_heightmap, depth_heightmap = utils.get_heightmap(
                color_img, depth_img, robot.cam_intrinsics, robot.cam_pose, workspace_limits, heightmap_resolution)

        valid_depth_heightmap = depth_heightmap.copy()
        valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0

        # Save RGB-D images and RGB-D heightmaps
        logger.save_images(trainer.iteration, color_img, depth_img, '0')
        logger.save_heightmaps(
            trainer.iteration, color_heightmap, valid_depth_heightmap, '0')

        # Reset simulation or pause real-world training if table is empty
        stuff_count = np.zeros(valid_depth_heightmap.shape)
        stuff_count[valid_depth_heightmap > 0.001] = 1
        self.logger.debug('DEBUG: depthmap avg %0.2f' % np.average(valid_depth_heightmap))
        # stuff_count[valid_depth_heightmap > 0.02] = 1
        empty_threshold = 300

        if np.sum(stuff_count) < empty_threshold: #or (is_bullet_sim and nonlocal_variables[constants.NO_CHANGE_COUNT] > 10):
            nonlocal_variables[constants.NO_CHANGE_COUNT] = 0
            self.logger.debug('Not enough stuff on the table (value: %d)! Flipping over bin of objects...' % (
                np.sum(stuff_count)))
            time.sleep(1)
            robot.restart_real()

            trainer.clearance_log.append([trainer.iteration])
            logger.write_to_log('clearance', trainer.clearance_log)
            if dont_train and len(trainer.clearance_log) >= max_test_trials:
                # Exit after training thread (backprop and saving labels)
                exit_called = True
            continue

        if not exit_called:
            self.logger.debug("Let's get some grasp predictions")

            # Run forward pass with network to get affordances
            grasp_predictions = trainer.forward(
                color_heightmap, valid_depth_heightmap, is_volatile=True)

            # talk to the thread: process grasp predictions, maybe save some visualizations and execute robot actions
            self.logger.debug("executing action--parent")
            nonlocal_variables[constants.GRASP_PREDICTIONS] = grasp_predictions
            nonlocal_variables[constants.VALID_DEPTH_HEIGHTMAP] = valid_depth_heightmap
            nonlocal_variables[constants.COLOR_HEIGHTMAP] = color_heightmap
            nonlocal_variables[constants.EXECUTING_ACTION] = True

        # Run training iteration in current thread (aka training thread)
        if 'prev_color_img' in locals():

            # Detect changes
            change_detected, change_value = utils.detect_changes(
                depth_heightmap, prev_depth_heightmap)
            change_detected = change_detected or prev_grasp_success
            self.logger.debug('Change detected: %r (value: %d)' %
                  (change_detected, change_value))

            if change_detected:
                if prev_primitive_action == 'grasp':
                    nonlocal_variables[constants.NO_CHANGE_COUNT] = 0
            else:
                if prev_primitive_action == 'grasp':
                    nonlocal_variables[constants.NO_CHANGE_COUNT] += 1

            # Compute training labels
            label_value, prev_reward_value = trainer.get_label_value(
                prev_primitive_action, prev_push_success, prev_grasp_success, change_detected, None,
                prev_grasp_predictions, color_heightmap, valid_depth_heightmap
            )
            trainer.label_value_log.append([label_value])
            logger.write_to_log('label-value', trainer.label_value_log)
            trainer.reward_value_log.append([prev_reward_value])
            logger.write_to_log('reward-value', trainer.reward_value_log)

            # Backpropagate
            trainer.backprop(prev_color_heightmap, prev_valid_depth_heightmap,
                             prev_primitive_action, prev_best_pix_ind, label_value)

            # Adjust exploration probability
            if not dont_train:
                explore_prob = max(
                    0.5 * np.power(0.9998, trainer.iteration), 0.1) if explore_rate_decay else 0.5

            # Do sampling for experience replay
            if experience_replay and not dont_train:
                runner.training_step(prev_primitive_action,
                                     prev_reward_value, trainer, logger)

            # Save model snapshot
            if not dont_train:
                logger.save_backup_model(trainer.model, method)
                if trainer.iteration % 50 == 0:
                    logger.save_model(trainer.iteration, trainer.model, method)
                    if trainer.use_cuda:
                        trainer.model = trainer.model.cuda()

        # Sync both action thread and training thread
        while nonlocal_variables[constants.EXECUTING_ACTION]:
            time.sleep(0.01)

        if exit_called:
            break

        # Save information for next training step
        prev_color_img = color_img.copy()
        prev_depth_img = depth_img.copy()
        prev_color_heightmap = color_heightmap.copy()
        prev_depth_heightmap = depth_heightmap.copy()
        prev_valid_depth_heightmap = valid_depth_heightmap.copy()
        prev_push_success = nonlocal_variables[constants.PUSH_SUCCESS]
        prev_grasp_success = nonlocal_variables[constants.GRASP_SUCCESS]
        prev_primitive_action = nonlocal_variables[constants.PRIMITE_ACTION]
        prev_grasp_predictions = grasp_predictions.copy()
        prev_best_pix_ind = nonlocal_variables[constants.BEST_PIX_IND]

        trainer.iteration += 1
        iteration_time_1 = time.time()
        self.logger.debug('Time elapsed: %f' % (iteration_time_1-iteration_time_0))


if __name__ == '__main__':

    # Parse arguments
    parser = argparse.ArgumentParser(
        description='Train robotic agents to learn how to plan complementary \
        pushing and grasping actions for manipulation with deep reinforcement \
        learning in PyTorch.')

    # --------------- Setup options ---------------
    # Bullet sim is run in different 
    # parser.add_argument('--is_bullet_sim', dest='is_bullet_sim',
            # action='store_true', default=False,     help='run in simulation?')
    parser.add_argument('--random_seed', dest='random_seed', type=int, action='store', default=1234,
                                                    help='random seed for simulation and neural net initialization')
    parser.add_argument('--cpu', dest='force_cpu', action='store_true', default=False,
                                                    help='force code to run in CPU mode')

    # ------------- Algorithm options -------------
    # We only use reinforcement (immediate online rewards) for now
    # parser.add_argument('--method', dest='method', action='store', default='reinforcement',
                        # help='set to \'reactive\' (supervised learning) or \'reinforcement\' (reinforcement learning ie Q-learning)')
    parser.add_argument('--future_reward_discount', dest='future_reward_discount', type=float, action='store', default=0.5)
    parser.add_argument('--experience_replay', dest='experience_replay', action='store_true',
                        default=False,              help='use prioritized experience replay?')
    parser.add_argument('--heuristic_bootstrap', dest='heuristic_bootstrap', action='store_true', default=False,
                                                    help='use handcrafted grasping algorithm when grasping fails too many times in a row during training?')
    parser.add_argument('--explore_rate_decay', dest='explore_rate_decay', action='store_true', default=False)

    # ------------- Robot options -------------
    parser.add_argument('--use_mock_robot', dest='use_mock_robot', action='store_true', default=False)
    parser.add_argument('--grasp_only', dest='grasp_only', action='store_true', default=False)
    parser.add_argument('--send_ur5_progs', dest='--send_ur5_progs',
            action='store_true', default=True,
            help ='set False to connect to ur5 but not send any programs to it')

    # -------------- Testing options --------------
    # -- Renaming due to word overload (testing = ML test vs train, OR robot on vs off)
    parser.add_argument('--dont_train', dest='dont_train',
                        action='store_true', default=False)
    parser.add_argument('--max_test_trials', dest='max_test_trials', type=int, action='store',
                        default=30,                 help='maximum number of test runs per case/scenario')

    # ------ Pre-loading and logging options ------
    parser.add_argument('--load_snapshot', dest='load_snapshot', action='store_true',
                        default=False,              help='load pre-trained snapshot of model?')
    parser.add_argument('--snapshot_file',
                        dest='snapshot_file', action='store')
    parser.add_argument'--continue_logging', dest='continue_logging', action='store_true',
                        default=False,              help='continue logging from previous session?')
    parser.add_argument('--logging_directory',
                        dest='logging_directory', action='store')
    parser.add_argument('--save_visualizations', dest='save_visualizations', action='store_true',
                        default=False,              help='save visualizations of FCN predictions?')

    # Run main program with specified arguments
    args = parser.parse_args()
    main(args)
