import numpy as np
import torch
import models


#device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

net = models.reactive_net(True)

input_color_data = np.random.uniform(0, 1, size=(2, 3, 640, 480)).astype(np.float32)
input_depth_data = np.random.uniform(0, 1, size=(2, 1, 640, 480)).astype(np.float32)
physics_prediction = [1.0, .5]

input_color_data_t = torch.from_numpy(input_color_data).cuda()
input_depth_data_t = torch.from_numpy(input_depth_data).cuda()

#input_color_data_t.to(device)
#input_depth_data_t.to(device)

features = net.forward(input_color_data_t, input_depth_data_t, physics_prediction)

print(len(features))
