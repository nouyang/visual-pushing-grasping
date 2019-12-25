import cv2
import numpy as np
from real.camera import Camera
import utils
import matplotlib.pyplot as plt


def main():
    camera = Camera()
    cam_intrinsics = camera.intrinsics

    # Load camera pose (from running calibrate.py), intrinsics and depth scale
    # NOTE: Is this independent of where the camera is?
    cam_pose = np.loadtxt('real/camera_pose.txt', delimiter=' ')
    cam_depth_scale = np.loadtxt(
        'real/camera_depth_scale.txt', delimiter=' ')

    workspace_limits = np.asarray(
        [[0.350, 0.650], [-0.250, 0.180], [0.080, 0.350]])
    heightmap_resolution = 0.002  # DEFAULT  # help='meters per pixel of heightmap')

    color_img, depth_img = camera.get_data()
    # Apply depth scale from calibration
    depth_img = depth_img * cam_depth_scale

    # Get heightmap from RGB-D image (by re-projecting 3D point cloud)
    color_heightmap, depth_heightmap = utils.get_heightmap(
        color_img, depth_img, cam_intrinsics, cam_pose, workspace_limits, heightmap_resolution)

    valid_depth_heightmap = depth_heightmap.copy()
    valid_depth_heightmap[np.isnan(valid_depth_heightmap)] = 0

    # Save RGB-D images and RGB-D heightmaps
    # save_images(trainer.iteration, color_img, depth_img, '0')
    # save_heightmaps(
    # trainer.iteration, color_heightmap, valid_depth_heightmap, '0')
    plt.imsave('debug_cam_only.jpg', color_img)
    cv2.imshow('Calibration', color_img)
    cv2.waitKey(100)


if __name__ == '__main__':
    main()
