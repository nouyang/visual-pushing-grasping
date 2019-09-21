import numpy as np
import matplotlib.pyplot as plt
import torch
import models_new


#device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

net = models_new.reinforcement_net(True)

input_color_data = np.random.uniform(0, 1, size=(1, 3, 600, 600)).astype(np.float32)
input_depth_data = np.random.uniform(0, 1, size=(1, 1, 600, 600)).astype(np.float32)
physics_prediction = [1.0]

input_color_data_t = torch.from_numpy(input_color_data).cuda()
input_depth_data_t = torch.from_numpy(input_depth_data).cuda()

#input_color_data_t.to(device)
#input_depth_data_t.to(device)

outputs_t = net.forward(input_color_data_t, input_depth_data_t, physics_prediction)
outputs = [ft.cpu().numpy() for ft in outputs_t]


for rot_idx in range(16):

    plt.subplot(4, 4, rot_idx + 1)

    img = outputs[rot_idx][0, 0, :, :]
    #img = np.transpose(img, axes=(1, 2, 0))

    plt.imshow(img)

plt.show()
