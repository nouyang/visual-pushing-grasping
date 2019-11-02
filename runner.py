import os
import time
import numpy as np
import cv2
import constants


def process_actions(nonlocal_variables, trainer, logger, robot, workspace_limits, heightmap_resolution,
                    heuristic_bootstrap, save_visualizations):
    """
    The main function of a thread that will process grasp predictions and execute actions.
    :param nonlocal_variables:      Main connection between this thread and the main thread.
                                    A (hopefully thread-safe dictionary).
    :param trainer:                 Trainer object.
    :param logger:                  Logger object.
    :param robot:                   Robot object.
    :param workspace_limits:        Workspace limits.
    :param heightmap_resolution:    Height map resolution.
    :param heuristic_bootstrap:     Should we use a heuristic grasping algorithm after a certain number of actions
                                    that have no effects.
    :param save_visualizations:     Save visualizations.
    :return:                        None.
    """

    while True:
        # print('!-- Running process actions loop')
        if nonlocal_variables[constants.EXECUTING_ACTION]:
            print("executing action--child")

            grasp_predictions = nonlocal_variables[constants.GRASP_PREDICTIONS]
            valid_depth_heightmap = nonlocal_variables[constants.VALID_DEPTH_HEIGHTMAP]
            color_heightmap = nonlocal_variables[constants.COLOR_HEIGHTMAP]

            # Determine whether grasping or pushing should be executed based on network predictions
            # best_push_conf = np.max(push_predictions)
            best_grasp_conf = np.max(grasp_predictions)
            print('Primitive confidence scores: %f (grasp)' % best_grasp_conf)
            nonlocal_variables[constants.PRIMITE_ACTION] = 'grasp'
            explore_actions = False

            trainer.is_exploit_log.append([0 if explore_actions else 1])
            logger.write_to_log('is-exploit', trainer.is_exploit_log)

            if nonlocal_variables[constants.PRIMITE_ACTION] != 'grasp':
                raise ValueError("Pushing not used in this project.")

            print("valid depth heightmap size:",
                  valid_depth_heightmap.shape)

            # If heuristic bootstrapping is enabled: if change has not been detected more than 2 times, execute heuristic algorithm to detect grasps/pushes
            # NOTE: typically not necessary and can reduce final performance.
            if heuristic_bootstrap and nonlocal_variables[constants.PRIMITE_ACTION] == 'grasp' and nonlocal_variables[constants.NO_CHANGE_COUNT] >= 2:
                print('Change not detected for more than two grasps. Running heuristic grasping.')
                nonlocal_variables[constants.BEST_PIX_IND] = trainer.grasp_heuristic(valid_depth_heightmap)
                nonlocal_variables[constants.NO_CHANGE_COUNT] = 0
                predicted_value = grasp_predictions[nonlocal_variables[constants.BEST_PIX_IND]]
                use_heuristic = True
            else:
                use_heuristic = False

                # Get pixel location and rotation with highest affordance prediction from heuristic algorithms (rotation, y, x)
                nonlocal_variables[constants.BEST_PIX_IND] = np.unravel_index(
                    np.argmax(grasp_predictions), grasp_predictions.shape)
                predicted_value = np.max(grasp_predictions)

            trainer.use_heuristic_log.append([1 if use_heuristic else 0])
            logger.write_to_log('use-heuristic', trainer.use_heuristic_log)

            # Save predicted confidence value
            trainer.predicted_value_log.append([predicted_value])
            logger.write_to_log('predicted-value',
                                trainer.predicted_value_log)

            # Compute 3D position of pixel
            print('!------------------ Action: %s at (%d, %d, %d)' % (
                nonlocal_variables[constants.PRIMITE_ACTION], nonlocal_variables[constants.BEST_PIX_IND][0],
                nonlocal_variables[constants.BEST_PIX_IND][1], nonlocal_variables[constants.BEST_PIX_IND][2])
            )
            best_rotation_angle = np.deg2rad(
                nonlocal_variables[constants.BEST_PIX_IND][0] * (360.0 / trainer.model.num_rotations)
            )
            best_pix_x = nonlocal_variables[constants.BEST_PIX_IND][2]
            best_pix_y = nonlocal_variables[constants.BEST_PIX_IND][1]

            # NOTE: original
            # TODO: why is it outof bound by one? indexing error
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

            # Visualize executed primitive, and affordances
            if save_visualizations:

                # TODO: ValueError: operands could not be broadcast together with shapes (364,273,3) (364,364,3)
                grasp_pred_vis = trainer.get_prediction_vis(
                    grasp_predictions, color_heightmap, nonlocal_variables[constants.BEST_PIX_IND]
                )
                logger.save_visualizations(trainer.iteration, grasp_pred_vis, 'grasp')
                #cv2.imwrite('visualization.grasp.png', grasp_pred_vis)

            # Initialize variables that influence reward
            nonlocal_variables[constants.GRASP_SUCCESS] = False

            # Execute primitive
            if nonlocal_variables[constants.PRIMITE_ACTION] == 'grasp':
                # ! TODO
                nonlocal_variables[constants.GRASP_SUCCESS] = robot.grasp(
                    primitive_position, best_rotation_angle, workspace_limits)
                print('Grasp successful: %r' % (nonlocal_variables[constants.GRASP_SUCCESS]))

            nonlocal_variables[constants.EXECUTING_ACTION] = False
            print('!-- no longer executing action')
        time.sleep(0.01)


def training_step(prev_primitive_action, prev_reward_value, trainer, logger):
    """
    Run a single experience replay training step.
    :param prev_primitive_action:       Previous primitive action.
    :param prev_reward_value:           Previous reward.
    :param trainer:                     Trainer object.
    :param logger:                      Logger object.
    :return:                            None.
    """

    sample_primitive_action = prev_primitive_action
    if sample_primitive_action == 'grasp':
        sample_primitive_action_id = 1
        sample_reward_value = 0 if prev_reward_value == 1 else 1

    # Get samples of the same primitive but with different results
    sample_ind = np.argwhere(
        np.logical_and(np.asarray(trainer.reward_value_log)[1:trainer.iteration, 0] == sample_reward_value, np.asarray(
            trainer.executed_action_log)[1:trainer.iteration, 0] == sample_primitive_action_id))

    if sample_ind.size > 0:

        # Find sample with highest surprise value
        sample_surprise_values = np.abs(np.asarray(trainer.predicted_value_log)[
                                            sample_ind[:, 0]] - np.asarray(trainer.label_value_log)[
                                            sample_ind[:, 0]])
        sorted_surprise_ind = np.argsort(
            sample_surprise_values[:, 0])
        sorted_sample_ind = sample_ind[sorted_surprise_ind, 0]
        pow_law_exp = 2
        rand_sample_ind = int(
            np.round(np.random.power(pow_law_exp, 1) * (sample_ind.size - 1)))
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
            np.float32) / 100000

        # Compute forward pass with sample
        sample_push_predictions, sample_grasp_predictions, sample_state_feat = trainer.forward(
            sample_color_heightmap, sample_depth_heightmap, is_volatile=True)

        # Load next sample RGB-D heightmap
        next_sample_color_heightmap = cv2.imread(os.path.join(
            logger.color_heightmaps_directory, '%06d.0.color.png' % (sample_iteration + 1)))
        next_sample_color_heightmap = cv2.cvtColor(
            next_sample_color_heightmap, cv2.COLOR_BGR2RGB)
        next_sample_depth_heightmap = cv2.imread(os.path.join(
            logger.depth_heightmaps_directory, '%06d.0.depth.png' % (sample_iteration + 1)), -1)
        next_sample_depth_heightmap = next_sample_depth_heightmap.astype(
            np.float32) / 100000

        sample_push_success = sample_reward_value == 0.5
        sample_grasp_success = sample_reward_value == 1
        sample_change_detected = sample_push_success
        new_sample_label_value, _ = trainer.get_label_value(sample_primitive_action, sample_push_success,
                                                            sample_grasp_success, sample_change_detected,
                                                            sample_push_predictions, sample_grasp_predictions,
                                                            next_sample_color_heightmap, next_sample_depth_heightmap)

        # Get labels for sample and backpropagate
        sample_best_pix_ind = (np.asarray(trainer.executed_action_log)[
                               sample_iteration, 1:4]).astype(int)
        trainer.backprop(sample_color_heightmap, sample_depth_heightmap, sample_primitive_action,
                         sample_best_pix_ind, trainer.label_value_log[sample_iteration])

        # Recompute prediction value and label for replay buffer
        if sample_primitive_action == 'push':
            trainer.predicted_value_log[sample_iteration] = [
                np.max(sample_push_predictions)]
            # trainer.label_value_log[sample_iteration] = [new_sample_label_value]
        elif sample_primitive_action == 'grasp':
            trainer.predicted_value_log[sample_iteration] = [
                np.max(sample_grasp_predictions)]
            # trainer.label_value_log[sample_iteration] = [new_sample_label_value]

    else:
        print('Not enough prior training samples. Skipping experience replay.')
