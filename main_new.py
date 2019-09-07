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
from robot import Robot
from trainer import Trainer
from logger import Logger
import utils

def main(args):

    tcp_host_ip = "10.75.15.91"  # IP and port to robot arm as TCP client (UR5)
    tcp_port = 30002

    workspace_limits = np.asarray(
        [[-0.700, -0.350], [-0.125, 0.225], [-0.300, -0.000]])  # grasp pos

    # NOTE
    heightmap_resolution = 0.0011 #  TODO: recalibrate

    random_seed = args.random_seed

    # ------------- Algorithm options -------------
    # 'reactive' (supervised learning) or 'reinforcement' (reinforcement learning ie Q-learning)
    # method = 'reinforcement'

    future_reward_discount = args.future_reward_discount
    experience_replay = args.experience_replay  # Use prioritized experience replay?

    explore_rate_decay = args.explore_rate_decay
    grasp_only = args.grasp_only

    # -------------- Testing options --------------
    is_testing = args.is_testing
    # Maximum number of test runs per case/scenario
    max_test_trials = args.max_test_trials
    # test_preset_cases = args.test_preset_cases
    # test_preset_file = os.path.abspath(
        # args.test_preset_file) if test_preset_cases else None

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
    home_rad = np.deg2rad([16.6, -26.5, 116.8, -184.6, -90.4, 198.4])

    # Initialize pick-and-place system (camera and robot)
    # TODO
    robot = Robot(False, False, None, workspace_limits,
                  tcp_host_ip, tcp_port, None, None,
                  is_testing, None, None,
                  home_joint_config=home_rad)

    # robot = Robot(is_sim, obj_mesh_dir, num_obj, workspace_limits,
                  # tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
                  # is_testing, test_preset_cases, test_preset_file,
                  # home_joint_config=home_rad)

    # Initialize trainer
    #def __init__(self, 
                 #is_testing, load_snapshot, snapshot_file):

    trainer = Trainer(is_testing, load_snapshot, snapshot_file)

    # Initialize data logger
    logger = Logger(continue_logging, logging_directory)
    # Save camera intrinsics and pose
    logger.save_camera_info(robot.cam_intrinsics,
                            robot.cam_pose, robot.cam_depth_scale)

    # Save heightmap parameters
    logger.save_heightmap_info(workspace_limits, heightmap_resolution)

    # Find last executed iteration of pre-loaded log, and load execution info and RL variables
    if continue_logging:
        trainer.preload(logger.transitions_directory)

    # no_change_count = [2, 2] if not is_testing else [0, 0] # heuristic vars
    explore_prob = 0.5 if not is_testing else 0.0

    # Quick hack for nonlocal memory between threads in Python 2
    nonlocal_variables = {'executing_action': False,
                          'primitive_action': None,
                          'best_pix_ind': None,
                          'throw_success': False,
                          'grasp_success': False}

    # Parallel thread to process network output and execute actions
    # -------------------------------------------------------------

    def process_actions():
        while True:
            if nonlocal_variables['executing_action']:

                # Determine whether grasping or pushing should be executed based on network predictions
                best_grasp_conf = np.max(grasp_predictions)
                best_throw_conf = np.max(throw_predictions)
                print('Primitive confidence scores: %f (grasp), %f (throw)' %
                      (best_grasp_conf, best_throw_conf))
                nonlocal_variables['primitive_action'] = 'grasp'
                explore_actions = False
                if not grasp_only:
                    if is_testing and method == 'reactive':
                        if best_throw_conf > 2*best_grasp_conf:
                            nonlocal_variables['primitive_action'] = 'throw'
                    else:
                        if best_throw_conf > best_grasp_conf:
                            nonlocal_variables['primitive_action'] = 'throw'
                    explore_actions = np.random.uniform() < explore_prob
                    # Exploitation (do best action) vs exploration (do other action)
                    if explore_actions:
                        print('Strategy: explore (exploration probability: %f)' % (
                            explore_prob))
                        nonlocal_variables['primitive_action'] = 'throw' if np.random.randint(
                            0, 2) == 0 else 'grasp'
                    else:
                        print('Strategy: exploit (exploration probability: %f)' % (
                            explore_prob))
                trainer.is_exploit_log.append([0 if explore_actions else 1])
                logger.write_to_log('is-exploit', trainer.is_exploit_log)

                '''
                if heuristic_bootstrap and nonlocal_variables['primitive_action'] == 'throw' and no_change_count[0] >= 2:
                '''
                use_heuristic = False

                        # Get pixel location and rotation with highest affordance prediction from heuristic algorithms (rotation, y, x)
                if nonlocal_variables['primitive_action'] == 'grasp':
                    nonlocal_variables['best_pix_ind'] = np.unravel_index(
                        np.argmax(grasp_predictions), grasp_predictions.shape)
                    predicted_value = np.max(grasp_predictions)
                elif nonlocal_variables['primitive_action'] == 'throw':
                    nonlocal_variables['best_pix_ind'] = np.unravel_index(
                        np.argmax(throw_predictions), throw_predictions.shape)
                    predicted_value = np.max(throw_predictions)

                trainer.use_heuristic_log.append([1 if use_heuristic else 0])
                logger.write_to_log('use-heuristic', trainer.use_heuristic_log)

                # Save predicted confidence value
                trainer.predicted_value_log.append([predicted_value])
                logger.write_to_log('predicted-value',
                                    trainer.predicted_value_log)


                # Compute 3D position of pixel
                print('!------------------ Action: %s at (%d, %d, %d)' % (nonlocal_variables['primitive_action'], nonlocal_variables[
                      'best_pix_ind'][0], nonlocal_variables['best_pix_ind'][1], nonlocal_variables['best_pix_ind'][2]))
                best_rotation_angle = np.deg2rad(
                    nonlocal_variables['best_pix_ind'][0]*(360.0/trainer.model.num_rotations))
                best_pix_x = nonlocal_variables['best_pix_ind'][2]
                best_pix_y = nonlocal_variables['best_pix_ind'][1]

                if best_pix_x == valid_depth_heightmap.shape[1]:
                    best_pix_x -= 1
                if best_pix_y == valid_depth_heightmap.shape[0]:
                    best_pix_y -= 1

                primitive_position = [best_pix_x * heightmap_resolution +
                                      workspace_limits[0][0], best_pix_y *
                                      heightmap_resolution +
                                      workspace_limits[1][0],
                                      valid_depth_heightmap[best_pix_y][best_pix_x]
                                      + workspace_limits[2][0]]


                  # TODO: MIGRATE THIS CODE!
                  '''
                  # If pushing, adjust start position, and make sure z value is safe and not too low
                if nonlocal_variables['primitive_action'] == 'push': 
                    finger_width = 0.02
                    safe_kernel_width = int(np.round((finger_width/2)/heightmap_resolution))
                    local_region = valid_depth_heightmap[max(best_pix_y - safe_kernel_width, 0):min(best_pix_y + safe_kernel_width + 1, valid_depth_heightmap.shape[0]), max(best_pix_x - safe_kernel_width, 0):min(best_pix_x + safe_kernel_width + 1, valid_depth_heightmap.shape[1])]
                    if local_region.size == 0:
                        safe_z_position = workspace_limits[2][0]
                    else:
                        safe_z_position = np.max(local_region) + workspace_limits[2][0]
                    primitive_position[2] = safe_z_position
                '''

                # Save executed primitive
                if nonlocal_variables['primitive_action'] == 'throw':
                    trainer.executed_action_log.append([0, nonlocal_variables['best_pix_ind'][0], nonlocal_variables['best_pix_ind'][1], nonlocal_variables['best_pix_ind'][2]]) # 0 - throw
                elif nonlocal_variables['primitive_action'] == 'grasp':
                    trainer.executed_action_log.append([1, nonlocal_variables['best_pix_ind'][0], nonlocal_variables['best_pix_ind'][1], nonlocal_variables['best_pix_ind'][2]]) # 1 - grasp
                logger.write_to_log('executed-action', trainer.executed_action_log)

                # Visualize executed primitive, and affordances
                if save_visualizations:
                    throw_pred_vis = trainer.get_prediction_vis(throw_predictions, color_heightmap, nonlocal_variables['best_pix_ind'])
                    logger.save_visualizations(trainer.iteration, throw_pred_vis, 'throw')
                    cv2.imwrite('visualization.throw.png', throw_pred_vis)
                    grasp_pred_vis = trainer.get_prediction_vis(grasp_predictions, color_heightmap, nonlocal_variables['best_pix_ind'])
                    logger.save_visualizations(trainer.iteration, grasp_pred_vis, 'grasp')
                    cv2.imwrite('visualization.grasp.png', grasp_pred_vis)

                # Initialize variables that influence reward
                nonlocal_variables['throw_success'] = False
                nonlocal_variables['grasp_success'] = False
                change_detected = False

                # Execute primitive
                if nonlocal_variables['primitive_action'] == 'throw':
                    nonlocal_variables['throw_success'] = robot.throw(primitive_position, best_rotation_angle, workspace_limits)
                    print('Throw successful: %r' % (nonlocal_variables['throw_success']))
                elif nonlocal_variables['primitive_action'] == 'grasp':
                    nonlocal_variables['grasp_success'] = robot.grasp(primitive_position, best_rotation_angle, workspace_limits)
                    print('Grasp successful: %r' % (nonlocal_variables['grasp_success']))

                nonlocal_variables['executing_action'] = False

            time.sleep(0.01)
    action_thread = threading.Thread(target=process_actions)
    action_thread.daemon = True
    action_thread.start()
    exit_called = False
    # -------------------------------------------------------------
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
        #TODO : is this not specified earlier
        heightmap_resolution = 0.0011
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
        stuff_count[valid_depth_heightmap > 0.02] = 1
        print('DEBUG: depthmap avg', np.average(valid_depth_heightmap))
        # empty_threshold = 300  # ORIG
        empty_threshold = 300
        print('DEBUG: stuff count', np.sum(stuff_count))

        if np.sum(stuff_count) < empty_threshold:
            no_change_count = [0, 0]
                # print('Not enough stuff on the table (value: %d)! Pausing for 30 seconds.' % (np.sum(stuff_count)))
                # time.sleep(30)
            print('Not enough stuff on the table (value: %d)! Flipping over bin of objects...' % (
                np.sum(stuff_count)))
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

            # TODO: edit
            # Run forward pass with network to get affordances
            throw_predictions, grasp_predictions, state_feat = trainer.forward(
                color_heightmap, valid_depth_heightmap, is_volatile=True)
            # print("DEBUG: Grasp predictions", grasp_predictions)

            # TODO: edit
            # Execute best primitive action on robot in another thread
            nonlocal_variables['executing_action'] = True

        # Run training iteration in current thread (aka training thread)
        if 'prev_color_img' in locals():

            # Detect changes
            depth_diff = abs(depth_heightmap - prev_depth_heightmap)
            depth_diff[np.isnan(depth_diff)] = 0
            depth_diff[depth_diff > 0.3] = 0
            depth_diff[depth_diff < 0.01] = 0
            depth_diff[depth_diff > 0] = 1
            # TODO: what is this about?
            change_threshold = 300
            change_value = np.sum(depth_diff)
            change_detected = change_value > change_threshold or prev_grasp_success
            print('Change detected: %r (value: %d)' %
                  (change_detected, change_value))

            if change_detected:
                if prev_primitive_action == 'throw':
                    no_change_count[0] = 0
                elif prev_primitive_action == 'grasp':
                    no_change_count[1] = 0
            else:
                if prev_primitive_action == 'throw':
                    no_change_count[0] += 1
                elif prev_primitive_action == 'grasp':
                    no_change_count[1] += 1

            # Compute training labels
            label_value, prev_reward_value = trainer.get_label_value(
                prev_primitive_action, prev_throw_success, prev_grasp_success, change_detected, prev_throw_predictions, prev_grasp_predictions, color_heightmap, valid_depth_heightmap)
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
                    0.5 * np.power(0.9998, trainer.iteration),0.1) if explore_rate_decay else 0.5

            # Do sampling for experience replay
            if experience_replay and not is_testing:
                sample_primitive_action = prev_primitive_action
                if sample_primitive_action == 'throw':
                    sample_primitive_action_id = 0
                    if method == 'reactive':
                        # random.randint(1, 2) # 2
                        sample_reward_value = 0 if prev_reward_value == 1 else 1
                    elif method == 'reinforcement':
                        sample_reward_value = 0 if prev_reward_value == 0.5 else 0.5
                elif sample_primitive_action == 'grasp':
                    sample_primitive_action_id = 1
                    if method == 'reactive':
                        sample_reward_value = 0 if prev_reward_value == 1 else 1
                    elif method == 'reinforcement':
                        sample_reward_value = 0 if prev_reward_value == 1 else 1

                # Get samples of the same primitive but with different results
                sample_ind = np.argwhere(np.logical_and(np.asarray(trainer.reward_value_log)[1:trainer.iteration,0] == sample_reward_value, np.asarray(
                    trainer.executed_action_log)[1:trainer.iteration,0] == sample_primitive_action_id))

                if sample_ind.size > 0:

                    # Find sample with highest surprise value
                    if method == 'reactive':
                        sample_surprise_values = np.abs(np.asarray(trainer.predicted_value_log)[
                                                        sample_ind[:,0]] - (1 - sample_reward_value))
                    elif method == 'reinforcement':
                        sample_surprise_values = np.abs(np.asarray(trainer.predicted_value_log)[
                                                        sample_ind[:,0]] - np.asarray(trainer.label_value_log)[sample_ind[:,0]])
                    sorted_surprise_ind = np.argsort(
                        sample_surprise_values[:,0])
                    sorted_sample_ind = sample_ind[sorted_surprise_ind,0]
                    pow_law_exp = 2
                    rand_sample_ind = int(
                        np.round(np.random.power(pow_law_exp, 1)*(sample_ind.size-1)))
                    sample_iteration = sorted_sample_ind[rand_sample_ind]
                    print('Experience replay: iteration %d (surprise value: %f)' % (
                        sample_iteration, sample_surprise_values[sorted_surprise_ind[rand_sample_ind]]))

                    # Load sample RGB-D heightmap
                    sample_color_heightmap = cv2.imread(os.path.join(
                        logger.color_heightmaps_directory, '%06d.0.color.png' % (sample_iteration)))
                    sample_color_heightmap = cv2.cvtColor(
                        sample_color_heightmap, cv2.COLOR_BGR2RGB)
                    sample_depth_heightmap = cv2.imread(os.path.join(
                        logger.depth_heightmaps_directory, '%06d.0.depth.png' % (sample_iteration)), -1)
                    sample_depth_heightmap = sample_depth_heightmap.astype(
                        np.float32)/100000

                    # Compute forward pass with sample
                    sample_throw_predictions, sample_grasp_predictions, sample_state_feat = trainer.forward(
                        sample_color_heightmap, sample_depth_heightmap, is_volatile=True)

                    # Load next sample RGB-D heightmap
                    next_sample_color_heightmap = cv2.imread(os.path.join(
                        logger.color_heightmaps_directory, '%06d.0.color.png' % (sample_iteration+1)))
                    next_sample_color_heightmap = cv2.cvtColor(
                        next_sample_color_heightmap, cv2.COLOR_BGR2RGB)
                    next_sample_depth_heightmap = cv2.imread(os.path.join(
                        logger.depth_heightmaps_directory, '%06d.0.depth.png' % (sample_iteration+1)), -1)
                    next_sample_depth_heightmap = next_sample_depth_heightmap.astype(
                        np.float32)/100000

                    sample_throw_success = sample_reward_value == 0.5
                    sample_grasp_success = sample_reward_value == 1
                    sample_change_detected = sample_throw_success
                    new_sample_label_value, _ = trainer.get_label_value(sample_primitive_action, sample_throw_success, sample_grasp_success, sample_change_detected,
                                                                        sample_throw_predictions, sample_grasp_predictions, next_sample_color_heightmap, next_sample_depth_heightmap)

                    # Get labels for sample and backpropagate
                    sample_best_pix_ind = (np.asarray(trainer.executed_action_log)[
                                           sample_iteration,1:4]).astype(int)
                    trainer.backprop(sample_color_heightmap, sample_depth_heightmap, sample_primitive_action,
                                     sample_best_pix_ind, trainer.label_value_log[sample_iteration])

                    # Recompute prediction value and label for replay buffer
                    if sample_primitive_action == 'throw':
                        trainer.predicted_value_log[sample_iteration] = [
                            np.max(sample_throw_predictions)]
                        # trainer.label_value_log[sample_iteration] = [new_sample_label_value]
                    elif sample_primitive_action == 'grasp':
                        trainer.predicted_value_log[sample_iteration] = [
                            np.max(sample_grasp_predictions)]
                        # trainer.label_value_log[sample_iteration] = [new_sample_label_value]

                else:
                    print(
                        'Not enough prior training samples. Skipping experience replay.')

            # Save model snapshot
            if not is_testing:
                logger.save_backup_model(trainer.model, method)
                if trainer.iteration % 50 == 0:
                    logger.save_model(trainer.iteration, trainer.model, method)
                    if trainer.use_cuda:
                        trainer.model = trainer.model.cuda()

        # Sync both action thread and training thread
        while nonlocal_variables['executing_action']:
            time.sleep(0.01)

        if exit_called:
            break

        # Save information for next training step
        prev_color_img = color_img.copy()
        prev_depth_img = depth_img.copy()
        prev_color_heightmap = color_heightmap.copy()
        prev_depth_heightmap = depth_heightmap.copy()
        prev_valid_depth_heightmap = valid_depth_heightmap.copy()
        prev_throw_success = nonlocal_variables['throw_success']
        prev_grasp_success = nonlocal_variables['grasp_success']
        prev_primitive_action = nonlocal_variables['primitive_action']
        prev_throw_predictions = throw_predictions.copy()
        prev_grasp_predictions = grasp_predictions.copy()
        prev_best_pix_ind = nonlocal_variables['best_pix_ind']

        trainer.iteration += 1
        iteration_time_1 = time.time()
        print('Time elapsed: %f' % (iteration_time_1-iteration_time_0))


