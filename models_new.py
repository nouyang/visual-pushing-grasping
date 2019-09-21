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
import resnet


class reinforcement_net(nn.Module):

    def __init__(self, use_cuda): # , snapshot=None
        super(reinforcement_net, self).__init__()
        self.use_cuda = use_cuda

        # self.push_color_trunk = torchvision.models.densenet.densenet121(pretrained=True)
        # self.grasp_color_trunk = torchvision.models.densenet.densenet121(pretrained=True)

        self.num_rotations = 16

        self.perception_net = resnet.ResNet7(resnet.BasicBlock, num_input_filters=4)
        self.perception_net = self.perception_net.cuda()

        #torch.hub.load('pytorch/vision', 'resnet18', pretrained=True)

        self.graspnet = nn.Sequential(OrderedDict([
            ('grasp-norm0', nn.BatchNorm2d(2048)),
            ('grasp-relu0', nn.ReLU(inplace=True)),
            ('grasp-conv0', nn.Conv2d(2048, 64, kernel_size=1, stride=1, bias=False)),
            ('grasp-norm1', nn.BatchNorm2d(64)),
            ('grasp-relu1', nn.ReLU(inplace=True)),
            ('grasp-conv1', nn.Conv2d(64, 1, kernel_size=1, stride=1, bias=False))
            # ('grasp-upsample2', nn.Upsample(scale_factor=4, mode='bilinear'))
        ]))

        # Initialize network weights
        for m in self.named_modules():
            if 'push-' in m[0] or 'grasp-' in m[0]:
                if isinstance(m[1], nn.Conv2d):
                    nn.init.kaiming_normal(m[1].weight.data)
                elif isinstance(m[1], nn.BatchNorm2d):
                    m[1].weight.data.fill_(1)
                    m[1].bias.data.zero_()

        # Initialize output variable (for backprop)
        self.interm_feat = []
        self.output_prob = []


    def forward(self, input_color_data, input_depth_data, physics_prediction, is_volatile=False, specific_rotation=-1):
    # if is_volatile:
        output_prob = []
        interm_feat = []

      # Apply rotations to images
        for rotate_idx in range(self.num_rotations):
            rotate_theta = np.radians(rotate_idx*(360/self.num_rotations))

            # Compute sample grid for rotation BEFORE neural network
            affine_mat_before = np.asarray([[np.cos(-rotate_theta),
                                             np.sin(-rotate_theta),
                                             0],[-np.sin(-rotate_theta),
                                                 np.cos(-rotate_theta), 0]])
            affine_mat_before = np.tile(affine_mat_before, reps=[len(physics_prediction), 1, 1])
            affine_mat_before.shape = (2, 3, len(physics_prediction))
            affine_mat_before = torch.from_numpy(affine_mat_before).permute(2,0,1).float().cuda()

            if self.use_cuda:
                flow_grid_before = F.affine_grid(Variable(affine_mat_before,
                                                          requires_grad=False).cuda(),
                                                 input_color_data.size())
            else:
                flow_grid_before = F.affine_grid(Variable(affine_mat_before,
                                                          requires_grad=False),
                                                 input_color_data.size())

            # Rotate images clockwise
            # if self.use_cuda:
            rotate_color = F.grid_sample(Variable(input_color_data,
                                                  volatile=True).cuda(),
                                         flow_grid_before,
                                         mode='nearest')
            rotate_depth = F.grid_sample(Variable(input_depth_data,
                                                  volatile=True).cuda(),
                                         flow_grid_before,
                                         mode='nearest')
            visual_input = torch.cat((rotate_color, rotate_depth), dim=1) #NOTE: maybe 3?

            # TODO: pull out .features() from resnet
            with torch.no_grad():
                self.visual_features = self.perception_net(visual_input.cuda())

            physics_prediction_image_shape = (
                self.visual_features.shape[0], 1, self.visual_features.shape[2], self.visual_features.shape[3]
            )
            # Fill physics 'image' with same value (prediction) & concat to
            # visual input
            one_images = np.ones(physics_prediction_image_shape, dtype=np.float32)
            physics_prediction = np.array(physics_prediction, dtype=np.float32)

            physics_images = one_images * physics_prediction[:, np.newaxis, np.newaxis, np.newaxis]
            physics_images_t = torch.from_numpy(physics_images).cuda()

            self.visual_features_with_physics_channel = torch.cat((self.visual_features, physics_images_t), dim=1)

            interm_feat.append(self.visual_features)

            # TODO: run code to understand shaping / unshaping
            '''
            # Compute sample grid for rotation AFTER branches
            affine_mat_after = np.asarray([[np.cos(rotate_theta),
                                            np.sin(rotate_theta),
                                            0],
                                           [-np.sin(rotate_theta),
                                                np.cos(rotate_theta), 0]])
            affine_mat_after.shape = (2,3,1)
            affine_mat_after = torch.from_numpy(affine_mat_after).permute(2,0,1).float()
            # if self.use_cuda:
            flow_grid_after = F.affine_grid(Variable(affine_mat_after,
                                                     requires_grad=False).cuda(),
                                            interm_push_feat.data.size())
        
            # Forward pass through branches, undo rotation on output predictions, upsample results
            output_prob.append([nn.Upsample(scale_factor=16, mode='bilinear').forward(F.grid_sample(self.pushnet(interm_push_feat), flow_grid_after, mode='nearest')),
                                nn.Upsample(scale_factor=16, mode='bilinear').forward(F.grid_sample(self.graspnet(interm_grasp_feat), flow_grid_after, mode='nearest'))])

            '''

        return interm_feat
        #return self.visual_features, self.visual_features_with_physics_channel
        #return output_prob#, interm_feat
