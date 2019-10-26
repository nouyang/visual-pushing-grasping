#!/usr/bin/env python

from collections import OrderedDict
import numpy as np
from scipy import ndimage
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.autograd import Variable
import torchvision
import matplotlib.pyplot as plt
import time
import andys_models
import resnet


class reinforcement_net(nn.Module):

    def __init__(self, use_cuda=False, device=None):  # , snapshot=None
        super(reinforcement_net, self).__init__()

        if device is not None:
            self.device = device
        else:
            if use_cuda:
                self.device = torch.device("cuda:0")
            else:
                self.device = torch.device("cpu")

        self.all_nets = andys_models.RegressionModel(num_input_channels=6)
        self.all_nets.to(device)

        # self.push_color_trunk = torchvision.models.densenet.densenet121(pretrained=True)
        # self.grasp_color_trunk = torchvision.models.densenet.densenet121(pretrained=True)

        self.num_rotations = 16

        #self.perception_net = resnet.ResNet7(resnet.BasicBlock, num_input_filters=4)
        #self.perception_net = self.perception_net.cuda()

        #torch.hub.load('pytorch/vision', 'resnet18', pretrained=True)

        # self.graspnet = nn.Sequential(OrderedDict([
        #    ('grasp-norm0', nn.BatchNorm2d(2048)),
        #    ('grasp-relu0', nn.ReLU(inplace=True)),
        #    ('grasp-conv0', nn.Conv2d(2048, 64, kernel_size=1, stride=1, bias=False)),
        #    ('grasp-norm1', nn.BatchNorm2d(64)),
        #    ('grasp-relu1', nn.ReLU(inplace=True)),
        #    ('grasp-conv1', nn.Conv2d(64, 1, kernel_size=1, stride=1, bias=False))
        #    # ('grasp-upsample2', nn.Upsample(scale_factor=4, mode='bilinear'))
        # ]))

        # Initialize network weights
        # for m in self.named_modules():
        #    if 'push-' in m[0] or 'grasp-' in m[0]:
        #        if isinstance(m[1], nn.Conv2d):
        #            nn.init.kaiming_normal(m[1].weight.data)
        #        elif isinstance(m[1], nn.BatchNorm2d):
        #            m[1].weight.data.fill_(1)
        #            m[1].bias.data.zero_()

        # Initialize output variable (for backprop)
        self.interm_feat = []
        self.output_prob = []

    def forward(self, input_color_data, input_depth_data, is_volatile=False, specific_rotation=-1):

        print("input color shape:", input_color_data.size())
        print("input depth shape:", input_depth_data.size())

        # if is_volatile:
        self.output_prob = []
        interm_feat = []
        physics_prediction = [1.0] * input_color_data.shape[0]

        # Apply rotations to images
        for rotate_idx in range(self.num_rotations):

            # do only one rotation
            if specific_rotation is not None and 0 <= specific_rotation < self.num_rotations:
                if rotate_idx != specific_rotation:
                    continue

            rotate_theta = np.radians(rotate_idx*(360/self.num_rotations))

            # Compute sample grid for rotation BEFORE neural network
            affine_mat_before = np.asarray([[np.cos(-rotate_theta),
                                             np.sin(-rotate_theta),
                                             0], [-np.sin(-rotate_theta),
                                                  np.cos(-rotate_theta), 0]])

            affine_mat_before = np.tile(affine_mat_before, reps=[
                                        len(physics_prediction), 1, 1])

            affine_mat_before = torch.from_numpy(
                affine_mat_before).float().to(self.device)

            flow_grid_before = F.affine_grid(Variable(affine_mat_before,
                                                      requires_grad=False).to(self.device),
                                             input_color_data.size())


            # Rotate images clockwise
            # if self.use_cuda:
            rotate_color = F.grid_sample(Variable(input_color_data,
                                                  volatile=True).to(self.device),
                                         flow_grid_before,
                                         mode='nearest')
            rotate_depth = F.grid_sample(Variable(input_depth_data,
                                                  volatile=True).to(self.device),
                                         flow_grid_before,
                                         mode='nearest')

            visual_input = torch.cat((rotate_color, rotate_depth), dim=1)  # NOTE: maybe 3?

            visual_features = self.all_nets.perception_net(visual_input.to(self.device))

            physics_prediction_image_shape = (
                visual_features.shape[0], 1, visual_features.shape[2], visual_features.shape[3]
            )

            # Fill physics 'image' with same value (prediction) & concat to
            # visual input
            one_images = np.ones(
                physics_prediction_image_shape, dtype=np.float32)
            physics_prediction = np.array(physics_prediction, dtype=np.float32)

            physics_images = one_images * \
                physics_prediction[:, np.newaxis, np.newaxis, np.newaxis]
            physics_images_t = torch.from_numpy(physics_images).to(self.device)

            visual_features_with_physics_channel = torch.cat(
                (visual_features, physics_images_t), dim=1)

            print("visual features shape:", visual_features.size())

            #with torch.no_grad():
            grasp_output = self.all_nets.grasp_net(visual_features_with_physics_channel)

            print("grasp output shape before rotation:", grasp_output.size())

            #print(torch.cuda.memory_allocated() / 1000000000, "GB")

            # interm_feat.append(self.visual_features)

            # Compute sample grid for rotation AFTER branches
            affine_mat_after = np.asarray([[np.cos(rotate_theta),
                                            np.sin(rotate_theta),
                                            0],
                                           [-np.sin(rotate_theta),
                                            np.cos(rotate_theta), 0]])
            affine_mat_after = np.tile(affine_mat_after, reps=[
                                       len(physics_prediction), 1, 1])
            affine_mat_after = torch.from_numpy(
                affine_mat_after).float()

            flow_grid_after = F.affine_grid(Variable(affine_mat_after,
                                                     requires_grad=False).to(self.device),
                                            grasp_output.data.size())
            grasp_output = F.grid_sample(
                grasp_output, flow_grid_after, mode='nearest')

            print("grasp output shape after rotation:", grasp_output.size())

            self.output_prob.append(grasp_output)

        return self.output_prob
        # return self.visual_features, self.visual_features_with_physics_channel
        # return output_prob#, interm_feat
