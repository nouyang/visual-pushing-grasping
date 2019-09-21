import numpy as np
import matplotlib.pyplot as plt
import torch
import models_new


#device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

net = models_new.reinforcement_net(True)

input_color_data = np.random.uniform(0, 1, size=(5, 3, 640, 480)).astype(np.float32)
input_depth_data = np.random.uniform(0, 1, size=(5, 1, 640, 480)).astype(np.float32)
physics_prediction = [1.0, 0.5, 1.0, 1.0, 1.0]

input_color_data_t = torch.from_numpy(input_color_data).cuda()
input_depth_data_t = torch.from_numpy(input_depth_data).cuda()

#input_color_data_t.to(device)
#input_depth_data_t.to(device)

features_t = net.forward(input_color_data_t, input_depth_data_t, physics_prediction)
features = [ft.cpu().numpy() for ft in features_t]

print(len(features))
print(features[0].shape)