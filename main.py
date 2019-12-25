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
    is_sim = args.is_sim  # Run in simulation?
    is_mock = args.is_mock
    # Directory containing 3D mesh files (.obj) of objects to be added to simulation
    obj_mesh_dir = os.path.abspath(args.obj_mesh_dir) if is_sim else None
    # Number of objects to add to simulation
    num_obj = args.num_obj if is_sim else None
    # IP and port to robot arm as TCP client (UR5)
    tcp_host_ip = args.tcp_host_ip if not is_sim else None
    tcp_port = args.tcp_port if not is_sim else None
    # IP and port to robot arm as real-time client (UR5)
    rtc_host_ip = args.rtc_host_ip if not is_sim else None
    rtc_port = args.rtc_port if not is_sim else None
    if is_sim:
        pass
        # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
        # workspace_limits = np.asarray(
        # [[-0.724, -0.276], [-0.224, 0.224], [-0.0001, 0.4]])
    else:
        # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
        # NOTE: orig
        # workspace_limits = np.asarray(
            # [[0.3, 0.748], [-0.224, 0.224], [-0.255, -0.1]])
        # NOTE: 5 Oct 2019
        workspace_limits = np.asarray(
            # [[-0.600, -0.400], [-0.190, 0.120], [-0.460, -0.200]])  # gripper is fat
            # [[-0.600, -0.400], [-0.190, 0.120], [-0.335, -0.090]])
            # [[-0.700, -0.550], [-0.250, 0.150], [-0.300, -0.090]])
            [[-0.700, -0.550], [-0.250, 0.150], [-0.300, -0.090]])

    heightmap_resolution = args.heightmap_resolution  # Meters per pixel of heightmap
    heightmap_resolution = 0.00115

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
    is_testing = args.is_testing
    # Maximum number of test runs per case/scenario
    max_test_trials = args.max_test_trials
    test_preset_cases = args.test_preset_cases
    test_preset_file = os.path.abspath(
        args.test_preset_file) if test_preset_cases else None

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

    # home_rad = np.deg2rad([20.2, -26.6, 116.8, -183.3, 268.8, 20.2])
    home_rad = np.deg2rad(
        [20.91, -31.33, 114., -171., -90., 15.9])  # out of the way
    # home_rad = np.deg2rad([0, -13.7, 90, -165., -90., -5.])  # 2 NOV 2-190

    # Initialize pick-and-place system (camera and robot)
    if is_mock:
        robot = mock_robot.MockRobot()
    else:
        from robot import Robot
        robot = Robot(is_sim, obj_mesh_dir, num_obj, workspace_limits,
                      tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
                      is_testing, test_preset_cases, test_preset_file,
                      home_joint_config=home_rad)

    # Initialize trainer
    trainer = Trainer(method, push_rewards, future_reward_discount,
                      is_testing, load_snapshot, snapshot_file, force_cpu)

    # Initialize data logger
    logger = Logger(continue_logging, logging_directory)
    # Save camera intrinsics and pose
    if not is_mock:
        logger.save_camera_info(robot.cam_intrinsics,
                                robot.cam_pose, robot.cam_depth_scale)
    # Save heightmap parameters
    logger.save_heightmap_info(workspace_limits, heightmap_resolution)

    # Find last executed iteration of pre-loaded log, and load execution info and RL variables
    if continue_logging:
        trainer.preload(logger.transitions_directory)

    # Initialize variables for heuristic bootstrapping and exploration probability
    explore_prob = 0.5 if not is_testing else 0.0

    # Quick hack for nonlocal memory between threads in Python 2
    nonlocal_variables = {constants.EXECUTING_ACTION: False,
                          constants.PRIMITE_ACTION: None,
                          constants.BEST_PIX_IND: None,
                          constants.PUSH_SUCCESS: False,
                          constants.GRASP_SUCCESS: False,
                          constants.NO_CHANGE_COUNT: 2 if not is_testing else 0}

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
        print('\n%s iteration: %d' %
              ('Testing' if is_testing else 'Training', trainer.iteration))
        iteration_time_0 = time.time()

        # Make sure simulation is still stable (if not, reset simulation)
        if is_sim:
            robot.check_sim()

        # Get latest RGB-D image
        color_img, depth_img = robot.get_camera_data()
        # Apply depth scale from calibration
        depth_img = depth_img * robot.cam_depth_scale

        # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
        print("This is the heightmap resolution", heightmap_resolution)
        heightmap_resolution = 0.0011

        print("This is the intrinsics", robot.cam_intrinsics)
        print("This is the depth scale", robot.cam_depth_scale)

        if is_mock:
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
        print('DEBUG: depthmap avg', np.average(valid_depth_heightmap))
        # stuff_count[valid_depth_heightmap > 0.02] = 1
        # empty_threshold = 300  # ORIG
        empty_threshold = 300
        if is_sim and is_testing:
            empty_threshold = 10
        print('DEBUG: stuff count', np.sum(stuff_count))
        if np.sum(stuff_count) < empty_threshold or (is_sim and nonlocal_variables[constants.NO_CHANGE_COUNT] > 10):
            nonlocal_variables[constants.NO_CHANGE_COUNT] = 0
            if is_sim:
                print('Not enough objects in view (value: %d)! Repositioning objects.' % (
                    np.sum(stuff_count)))
                robot.restart_sim()
                robot.add_objects()
                # If at end of test run, re-load original weights (before test run)
                if is_testing:
                    trainer.model.load_state_dict(torch.load(snapshot_file))
            else:
                # print('Not enough stuff on the table (value: %d)! Pausing for 30 seconds.' % (np.sum(stuff_count)))
                # time.sleep(30)

                # TODO: edit
                print('Not enough stuff on the table (value: %d)! Flipping over bin of objects...' % (
                    np.sum(stuff_count))
                )
                time.sleep(1)
                robot.restart_real()

            trainer.clearance_log.append([trainer.iteration])
            logger.write_to_log('clearance', trainer.clearance_log)
            if is_testing and len(trainer.clearance_log) >= max_test_trials:
                # Exit after training thread (backprop and saving labels)
                exit_called = True
            continue

        if not exit_called:
            print("Let's get some grasp predictions")

            # Run forward pass with network to get affordances
            grasp_predictions = trainer.forward(
                color_heightmap, valid_depth_heightmap, is_volatile=True)

            # talk to the thread: process grasp predictions, maybe save some visualizations and execute robot actions
            print("executing action--parent")
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
            print('Change detected: %r (value: %d)' %
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
            if not is_testing:
                explore_prob = max(
                    0.5 * np.power(0.9998, trainer.iteration), 0.1) if explore_rate_decay else 0.5

            # Do sampling for experience replay
            if experience_replay and not is_testing:
                runner.training_step(prev_primitive_action,
                                     prev_reward_value, trainer, logger)

            # Save model snapshot
            if not is_testing:
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
        print('Time elapsed: %f' % (iteration_time_1-iteration_time_0))


if __name__ == '__main__':

    # Parse arguments
    parser = argparse.ArgumentParser(
        description='Train robotic agents to learn how to plan complementary pushing and grasping actions for manipulation with deep reinforcement learning in PyTorch.')

    # --------------- Setup options ---------------
    parser.add_argument('--is_sim', dest='is_sim', action='store_true',
                        default=False,                                    help='run in simulation?')
    parser.add_argument('--is_mock', dest='is_mock',
                        action='store_true', default=False)
    parser.add_argument('--obj_mesh_dir', dest='obj_mesh_dir', action='store', default='objects/blocks',
                        help='directory containing 3D mesh files (.obj) of objects to be added to simulation')
    parser.add_argument('--num_obj', dest='num_obj', type=int, action='store', default=5,
                        help='number of objects to add to simulation')
    parser.add_argument('--tcp_host_ip', dest='tcp_host_ip', action='store', default='100.127.7.223',
                        help='IP address to robot arm as TCP client (UR5)')
    parser.add_argument('--tcp_port', dest='tcp_port', type=int, action='store',
                        default=30002,                           help='port to robot arm as TCP client (UR5)')
    parser.add_argument('--rtc_host_ip', dest='rtc_host_ip', action='store', default='100.127.7.223',
                        help='IP address to robot arm as real-time client (UR5)')
    parser.add_argument('--rtc_port', dest='rtc_port', type=int, action='store', default=30003,
                        help='port to robot arm as real-time client (UR5)')
    parser.add_argument('--heightmap_resolution', dest='heightmap_resolution',
                        type=float, action='store', default=0.0015, help='meters per pixel of heightmap')
    parser.add_argument('--random_seed', dest='random_seed', type=int, action='store', default=1234,
                        help='random seed for simulation and neural net initialization')
    parser.add_argument('--cpu', dest='force_cpu', action='store_true', default=False,
                        help='force code to run in CPU mode')

    # ------------- Algorithm options -------------
    parser.add_argument('--method', dest='method', action='store', default='reinforcement',
                        help='set to \'reactive\' (supervised learning) or \'reinforcement\' (reinforcement learning ie Q-learning)')
    parser.add_argument('--push_rewards', dest='push_rewards', action='store_true', default=False,
                        help='use immediate rewards (from change detection) for pushing?')
    parser.add_argument('--future_reward_discount',
                        dest='future_reward_discount', type=float, action='store', default=0.5)
    parser.add_argument('--experience_replay', dest='experience_replay', action='store_true',
                        default=False,              help='use prioritized experience replay?')
    parser.add_argument('--heuristic_bootstrap', dest='heuristic_bootstrap', action='store_true', default=False,
                        help='use handcrafted grasping algorithm when grasping fails too many times in a row during training?')
    parser.add_argument('--explore_rate_decay',
                        dest='explore_rate_decay', action='store_true', default=False)
    parser.add_argument('--grasp_only', dest='grasp_only',
                        action='store_true', default=False)

    # -------------- Testing options --------------
    parser.add_argument('--is_testing', dest='is_testing',
                        action='store_true', default=False)
    parser.add_argument('--max_test_trials', dest='max_test_trials', type=int, action='store',
                        default=30,                help='maximum number of test runs per case/scenario')
    parser.add_argument('--test_preset_cases',
                        dest='test_preset_cases', action='store_true', default=False)
    parser.add_argument('--test_preset_file', dest='test_preset_file',
                        action='store', default='test-10-obj-01.txt')

    # ------ Pre-loading and logging options ------
    parser.add_argument('--load_snapshot', dest='load_snapshot', action='store_true',
                        default=False,                      help='load pre-trained snapshot of model?')
    parser.add_argument('--snapshot_file',
                        dest='snapshot_file', action='store')
    parser.add_argument('--continue_logging', dest='continue_logging', action='store_true',
                        default=False,                help='continue logging from previous session?')
    parser.add_argument('--logging_directory',
                        dest='logging_directory', action='store')
    parser.add_argument('--save_visualizations', dest='save_visualizations', action='store_true',
                        default=False,          help='save visualizations of FCN predictions?')

    # Run main program with specified arguments
    args = parser.parse_args()
    main(args)
