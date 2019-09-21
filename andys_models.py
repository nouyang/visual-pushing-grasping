import torch.nn as nn
import torch.nn.functional as F
from collections import OrderedDict
import resnet


class Interpolate(nn.Module):

    def __init__(self, scale_factor, mode, align_corners):
        super(Interpolate, self).__init__()
        self.scale_factor = scale_factor
        self.mode = mode
        self.align_corners = align_corners

    def forward(self, x):
        return F.interpolate(x, scale_factor=self.scale_factor, mode=self.mode, align_corners=self.align_corners)


class RegressionModel(nn.Module):

    def __init__(self, num_input_channels=4):

        super(RegressionModel,self).__init__()

        # Residual networks: https://arxiv.org/pdf/1512.03385.pdf
        # resnet.py contains an 8-stride implementation with dilation from: https://github.com/warmspringwinds/vision/blob/d6613c67dec12db3b57ec3c9b80e8d92abcaaa56/torchvision/models/resnet.py

        # Initialize shared perception network
        self.perception_net = nn.Sequential(OrderedDict([('perception-conv0',  nn.Conv2d(num_input_channels,64,kernel_size=3,stride=1,padding=1,bias=False)),
                                                          ('perception-norm0',  nn.BatchNorm2d(64)),
                                                          ('perception-relu0',  nn.ReLU(inplace=True)),
                                                          ('perception-pool1',  nn.MaxPool2d(kernel_size=3,stride=2,padding=1)),
                                                          ('perception-resn2',  resnet.BasicBlock(64,128,downsample=nn.Sequential(nn.Conv2d(64,128,kernel_size=1,bias=False),nn.BatchNorm2d(128)),dilation=1)),
                                                          ('perception-pool3',  nn.MaxPool2d(kernel_size=3,stride=2,padding=1)),
                                                          ('perception-resn4',  resnet.BasicBlock(128,256,downsample=nn.Sequential(nn.Conv2d(128,256,kernel_size=1,bias=False),nn.BatchNorm2d(256)),dilation=1)),
                                                          ('perception-resn6',  resnet.BasicBlock(256,512,downsample=nn.Sequential(nn.Conv2d(256,512,kernel_size=1,bias=False),nn.BatchNorm2d(512)),dilation=1))
                                                          ]))

        # Initialize grasping network
        self.grasp_net = nn.Sequential(OrderedDict([('grasp-resn0', resnet.BasicBlock(513,256,downsample=nn.Sequential(nn.Conv2d(513,256,kernel_size=1,bias=False),nn.BatchNorm2d(256)),dilation=1)),
                                                     ('grasp-resn2', resnet.BasicBlock(256,128,downsample=nn.Sequential(nn.Conv2d(256,128,kernel_size=1,bias=False),nn.BatchNorm2d(128)),dilation=1)),
                                                     ('grasp-upsm3', Interpolate(scale_factor=2,mode='bilinear',align_corners=True)),
                                                     ('grasp-resn4', resnet.BasicBlock(128,64,downsample=nn.Sequential(nn.Conv2d(128,64,kernel_size=1,bias=False),nn.BatchNorm2d(64)),dilation=1)),
                                                     ('grasp-upsm5', Interpolate(scale_factor=2,mode='bilinear',align_corners=True)),
                                                     ('grasp-conv6', nn.Conv2d(64,1,kernel_size=1,stride=1,bias=False))
                                                     ]))

        # Initialize tossing network
        self.toss_net = nn.Sequential(OrderedDict([('toss-resn0', resnet.BasicBlock(512,256,downsample=nn.Sequential(nn.Conv2d(512,256,kernel_size=1,bias=False)),dilation=1)),
                                                    ('toss-resn2', resnet.BasicBlock(256,128,downsample=nn.Sequential(nn.Conv2d(256,128,kernel_size=1,bias=False)),dilation=1)),
                                                    ('toss-upsm3', Interpolate(scale_factor=2,mode='bilinear',align_corners=True)),
                                                    ('toss-resn4', resnet.BasicBlock(128,64,downsample=nn.Sequential(nn.Conv2d(128,64,kernel_size=1,bias=False)),dilation=1)),
                                                    ('toss-upsm5', Interpolate(scale_factor=2,mode='bilinear',align_corners=True)),
                                                    ('toss-conv6', nn.Conv2d(64,1,kernel_size=1,stride=1,bias=False)),
                                                    ('toss-sgmd7', nn.Sigmoid())
                                                    ]))

        # Initialize random weights
        for m in self.named_modules():
            if isinstance(m[1], nn.Conv2d):
                nn.init.kaiming_normal_(m[1].weight.data)
            elif isinstance(m[1], nn.BatchNorm2d):
                m[1].weight.data.fill_(1)
                m[1].bias.data.zero_()
