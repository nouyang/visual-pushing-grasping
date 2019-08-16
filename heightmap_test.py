import numpy as np
import cv2
from logger import Logger
import robot
import utils
from real.camera import Camera
import os

debug_dir = './heightmap_debug'
# logger = Logger(False, debug_dir)

# color_img, depth_img = robot.get_camera_data()
# Apply depth scale from calibration
# depth_img = depth_img * robot.cam_depth_scale

# color_img, depth_img = robot.get_camera_data()
camera = Camera()
# cam_intrinsics = self.camera.intrinsics

cam_pose = np.loadtxt("real/camera_pose.txt", delimiter=" ")
cam_depth_scale = np.loadtxt(
    "real/camera_depth_scale.txt", delimiter=" ")

workspace_limits = np.asarray(
    [[-0.584, -0.380], [.100, 0.325], [-0.425, -0.340]])
# [[-0.584, -0.380], [.100, 0.325], [-0.250, -0.100]])

cam_intrinsics = np.array([[920.723,   0., 653.2795],
                           [0., 920.02124, 362.55386],
                           [0.,   0.,   1.]])

heightmap_resolution = 0.0015
# color_img = cv2.imread('color.png', 0)
# depth_img = cv2.imread('depth.png', 0)
depth_scale = np.array(0.99453125)
color_img, depth_img = camera.get_data()
depth_img = depth_img * depth_scale
# depth_img = depth_img.astype(float) * depth_scale

color_heightmap, depth_heightmap = utils.get_heightmap(
    color_img, depth_img, cam_intrinsics, cam_pose, workspace_limits, heightmap_resolution)

valid_depth_heightmap = depth_heightmap.copy()
valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0


# logger.save_heightmaps(1, color_heightmap, valid_depth_heightmap, '0')

color_image = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
cv2.imwrite(os.path.join(debug_dir, 'color.png'), color_image)
# Save depth in 1e-4 meters
depth_image = np.round(depth_img * 10000).astype(np.uint16)
cv2.imwrite(os.path.join(debug_dir, 'depth.png'), depth_image)

color_heightmap = cv2.cvtColor(color_heightmap, cv2.COLOR_RGB2BGR)
cv2.imwrite(os.path.join(debug_dir, 'color_heightmap.png'), color_heightmap)

# Save depth in 1e-5 meters
depth_heightmap = np.round(depth_heightmap * 100000).astype(np.uint16)
cv2.imwrite(os.path.join(debug_dir, 'depth_heightmap.png'), depth_heightmap)
