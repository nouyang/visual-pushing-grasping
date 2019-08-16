#!/usr/bin/env python
# Checkerboard courtesy of https://calib.io/pages/camera-calibration-pattern-generator

import matplotlib.pyplot as plt
import numpy as np
import time
import cv2
from real.camera import Camera
from robot import Robot
from scipy import optimize
from mpl_toolkits.mplot3d import Axes3D

import logging
# logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.INFO)

# User options (change me)
# --------------- Setup options ---------------
# tcp_host_ip = '100.127.7.223' # IP and port to robot arm as TCP client (UR5)
tcp_host_ip = "10.75.15.91"
tcp_port = 30002
# rtc_host_ip = '100.127.7.223' # IP and port to robot arm as real-time client (UR5)
rtc_host_ip = "10.75.15.91"
rtc_port = 30003

# Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
# Original:
# workspace_limits = np.asarray([[0.3, 0.748], [0.05, 0.4], [-0.2, -0.1]])

# NOTE: Mine as measured on pendant (in meters) 0.4 to 0.75; -.25 to .15; -0.2 to -0.1

"""
# Magic constant = 0.4; my z axis has an offset of 0.4 from the pendant somehow
# Takes 10 minutes (at very safe 0.2 acc / 0.1 vel / 35% speeds):
workspace_limits = np.asarray(  # true-ish
    [[0.4, 0.75], [-0.1, 0.15], [-0.21 + 0.4, -0.1 + 0.4]])

workspace_limits = np.asarray(  # smaller true-ish (63 pts)
    [[0.400, 0.750], [0.000, 0.150], [-0.210 + 0.400, -0.100 + 0.400]])

# workspace_limits = np.asarray(  # quick test -- 12 pts should still be pretty
# accurate ! within 10 cm for sure. Good check that the offset is in the right
# direction..
"""
workspace_limits = np.asarray(
    [[-0.650, -0.400], [-0.100, 0.100], [-0.300, -0.150]])

# calib_grid_step = 0.05
#calib_grid_step = 0.15

calib_grid_step = 0.1


# checkerboard_offset_from_tool = [0, -0.13, 0.02] # ORIGINAL

# NOTE: measured
checkerboard_offset_from_tool = [-0.090, 0.000, 0.020]  # gripper is 2cm high

# Original
# tool_orientation = [-np.pi/2, 0, 0]

# NOTE: Mine is experimentally measured (from TCP pose status)
# NOTE: Can I provide this in not-axis angle?
tool_orientation = [1.19, -1.26, -1.22]
# from pendant, this is equivalent to 0, pi/2, pi
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
calib_grid_x.shape = (num_calib_grid_pts, 1)
calib_grid_y.shape = (num_calib_grid_pts, 1)
calib_grid_z.shape = (num_calib_grid_pts, 1)
calib_grid_pts = np.concatenate(
    (calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

print('Number grid points!', len(calib_grid_pts))

measured_pts = []
observed_pts = []
observed_pix = []

home_in_rad = np.deg2rad(
    # np.array([-61.25, -20.31, 113.11, -94.17, -335.09, -1.1]))
    np.array([-13.5, -25.5, 120.7, -90., 80., 0]))
# Move robot to home pose
print('Connecting to robot...')
robot = Robot(False, False, None, workspace_limits,
              tcp_host_ip, tcp_port, rtc_host_ip, rtc_port,
              False, None, None, home_joint_config=home_in_rad)
print('!------------ Initialized robot -------------------- \n\n')
# robot.close_gripper()
print('!------ Gripper Closed, moving gripper so checkerboard is facing up\n\n')

# Slow down robot
robot.joint_acc = 0.200
robot.joint_vel = 0.150
# robot.joint_acc = 1.4
# robot.joint_vel = 1.05

# Make robot gripper point upwards
# robot.r.move_joints(np.deg2rad(
# [-61.25, -20.31, 113.11, -94.17, -335.09, -1.1]))
#robot.r.move_joints([-np.pi, -np.pi/2, np.pi/2, 0, np.pi/2, np.pi])
print('!--------------- Moved gripper to point upward -------------------- \n\n')

# Move robot to each calibration point in workspace
print('Collecting data...')
start = time.time()
print('num calib pts', num_calib_grid_pts)

time.sleep(1)

for calib_pt_idx in range(num_calib_grid_pts):
    tool_position = calib_grid_pts[calib_pt_idx, :]

    print('!----- #: ' + str(calib_pt_idx) + ' / ' + str(num_calib_grid_pts),
          '. Moving to: ', tool_position, tool_orientation, ' ---------')
    dt = time.time() - start
    print('!- Elapsed Time: ' + str(dt) + ' secs  ----- \n\n')

    robot.r.move_to(tool_position, tool_orientation)
    time.sleep(1)

    # Find checkerboard center
    checkerboard_size = (3, 3)
    refine_criteria = (cv2.TERM_CRITERIA_EPS +
                       cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    camera_color_img, camera_depth_img = robot.get_camera_data()
    # print('camera data', robot.get_camera_data())
    bgr_color_data = cv2.cvtColor(camera_color_img, cv2.COLOR_RGB2BGR)
    gray_data = cv2.cvtColor(bgr_color_data, cv2.COLOR_RGB2GRAY)
    checkerboard_found, corners = cv2.findChessboardCorners(
        gray_data, checkerboard_size, None, cv2.CALIB_CB_ADAPTIVE_THRESH)
    if checkerboard_found:
        print("Found checkerboard.")
        corners_refined = cv2.cornerSubPix(
            gray_data, corners, (3, 3), (-1, -1), refine_criteria)

        # Get observed checkerboard center 3D point in camera space
        checkerboard_pix = np.round(corners_refined[4, 0, :]).astype(int)
        checkerboard_z = camera_depth_img[checkerboard_pix[1]
                                          ][checkerboard_pix[0]]
        # print('checkerboard', checkerboard_pix,  'depth', checkerboard_z)
        checkerboard_x = np.multiply(
            checkerboard_pix[0]-robot.cam_intrinsics[0][2], checkerboard_z/robot.cam_intrinsics[0][0])
        checkerboard_y = np.multiply(
            checkerboard_pix[1]-robot.cam_intrinsics[1][2], checkerboard_z/robot.cam_intrinsics[1][1])
        if checkerboard_z == 0:
            print('no depth info found')
            continue

        # Save calibration point and observed checkerboard center
        observed_pts.append([checkerboard_x, checkerboard_y, checkerboard_z])
        # tool_position[2] += checkerboard_offset_from_tool
        # TODO: Is this offset in the right direction?
        # TODO: Is this offset from the gripper jaws or from the tool of the
        # robot?
        checker_position = tool_position + checkerboard_offset_from_tool

        print('I measured (calculated)', checker_position)
        print('I observed (realsense)', checkerboard_x, checkerboard_y,
              checkerboard_z)
        measured_pts.append(checker_position)
        observed_pix.append(checkerboard_pix)

        # Draw and display the corners
        # vis = cv2.drawChessboardCorners(robot.camera.color_data, checkerboard_size, corners_refined, checkerboard_found)
        vis = cv2.drawChessboardCorners(
            bgr_color_data, (1, 1), corners_refined[4, :, :], checkerboard_found)
        print('saving file')
        cv2.imwrite('%06d.png' % len(measured_pts), vis)
        cv2.imshow('Calibration', vis)
        cv2.waitKey(10)

# Move robot back to home pose
print('!--------------------- Going home now -------------------- \n\n')
robot.r.go_home()
print('!--------------------- Homed -------------------- \n\n')

measured_pts = np.asarray(measured_pts)
observed_pts = np.asarray(observed_pts)
observed_pix = np.asarray(observed_pix)
world2camera = np.eye(4)

np.savetxt('real/measured_pts.txt', measured_pts, delimiter=' ')
np.savetxt('real/observed_pts.txt', observed_pts, delimiter=' ')
np.savetxt('real/observed_pix.txt', observed_pix, delimiter=' ')

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

    # NOTE: Camera intrinsics supplied by realsense camera over TCP
    # Apply z offset and compute new observed points using camera intrinsics
    print(observed_pts)
    observed_z = observed_pts[:, 2:] * z_scale
    print(observed_pts)
    observed_x = np.multiply(observed_pix[:, [
                             0]]-robot.cam_intrinsics[0][2], observed_z/robot.cam_intrinsics[0][0])
    observed_y = np.multiply(observed_pix[:, [
                             1]]-robot.cam_intrinsics[1][2], observed_z/robot.cam_intrinsics[1][1])
    new_observed_pts = np.concatenate(
        (observed_x, observed_y, observed_z), axis=1)

    # NOTE: ENH why not just use openCV for estimating where camera is in world
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
print('Calibrating...')
z_scale_init = 1
optim_result = optimize.minimize(
    get_rigid_transform_error, np.asarray(z_scale_init), method='Nelder-Mead')
camera_depth_offset = optim_result.x

# Save camera optimized offset and camera pose
print('Saving...')
np.savetxt('real/camera_depth_scale.txt', camera_depth_offset, delimiter=' ')
get_rigid_transform_error(camera_depth_offset)
camera_pose = np.linalg.inv(world2camera)
np.savetxt('real/camera_pose.txt', camera_pose, delimiter=' ')
print('Done.')

# DEBUG CODE -----------------------------------------------------------------------------------

# np.savetxt('measured_pts.txt', np.asarray(measured_pts), delimiter=' ')
# np.savetxt('observed_pts.txt', np.asarray(observed_pts), delimiter=' ')
# np.savetxt('observed_pix.txt', np.asarray(observed_pix), delimiter=' ')
# measured_pts = np.loadtxt('measured_pts.txt', delimiter=' ')
# observed_pts = np.loadtxt('observed_pts.txt', delimiter=' ')
# observed_pix = np.loadtxt('observed_pix.txt', delimiter=' ')

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(measured_pts[:,0],measured_pts[:,1],measured_pts[:,2], c='blue')

# print(camera_depth_offset)
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(observed_pts)) + np.tile(camera2robot[0:3,3:],(1,observed_pts.shape[0])))

# ax.scatter(t_observed_pts[:,0],t_observed_pts[:,1],t_observed_pts[:,2], c='red')

# new_observed_pts = observed_pts.copy()
# new_observed_pts[:,2] = new_observed_pts[:,2] * camera_depth_offset[0]
# R, t = get_rigid_transform(np.asarray(measured_pts), np.asarray(new_observed_pts))
# t.shape = (3,1)
# camera_pose = np.concatenate((np.concatenate((R, t), axis=1),np.array([[0, 0, 0, 1]])), axis=0)
# camera2robot = np.linalg.inv(camera_pose)
# t_new_observed_pts = np.transpose(np.dot(camera2robot[0:3,0:3],np.transpose(new_observed_pts)) + np.tile(camera2robot[0:3,3:],(1,new_observed_pts.shape[0])))

# ax.scatter(t_new_observed_pts[:,0],t_new_observed_pts[:,1],t_new_observed_pts[:,2], c='green')

# plt.show()
