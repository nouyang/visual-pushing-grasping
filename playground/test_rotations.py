import numpy as np
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision
import torchvision.transforms as transforms
from torch.autograd import Variable


num_rotations = 16
batch_size = 6
device = torch.device("cpu")

transform = transforms.Compose(
    [transforms.ToTensor(),
     transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))]
)

trainset = torchvision.datasets.CIFAR10(root="./data", train=True, download=True, transform=transform)
trainloader = torch.utils.data.DataLoader(trainset, batch_size=batch_size, shuffle=True, num_workers=2)
dataiter = iter(trainloader)
images, labels = dataiter.next()
rotate_images = []
rotate_images_back = []

input_color_data_t = images

for rotate_idx in range(num_rotations):

    rotate_theta = np.radians(rotate_idx * (360 / num_rotations))

    # Compute sample grid for rotation BEFORE neural network
    affine_mat_before = np.asarray([[np.cos(-rotate_theta),
                                     np.sin(-rotate_theta),
                                     0], [-np.sin(-rotate_theta),
                                          np.cos(-rotate_theta), 0]])

    affine_mat_before = np.tile(affine_mat_before, reps=[batch_size, 1, 1])

    affine_mat_before = torch.from_numpy(affine_mat_before).float().to(device)

    flow_grid_before = F.affine_grid(
        Variable(affine_mat_before, requires_grad=False).to(device), images.size()
    )

    tmp_rotate_images = F.grid_sample(Variable(images, volatile=True).to(device), flow_grid_before, mode='nearest')
    rotate_images.append(tmp_rotate_images)

    affine_mat_after = np.asarray([[np.cos(rotate_theta),
                                    np.sin(rotate_theta),
                                    0],
                                   [-np.sin(rotate_theta),
                                    np.cos(rotate_theta), 0]])
    affine_mat_after = np.tile(affine_mat_after, reps=[
        batch_size, 1, 1])
    affine_mat_after = torch.from_numpy(
        affine_mat_after).float()

    flow_grid_after = F.affine_grid(Variable(affine_mat_after, requires_grad=False).to(device), tmp_rotate_images.size())

    tmp_rotate_images_back = F.grid_sample(tmp_rotate_images, flow_grid_after, mode='nearest')
    rotate_images_back.append(tmp_rotate_images_back)


plt.imshow(images[0, 0, :, :])
plt.show()

for rot_idx in range(num_rotations):


    plt.subplot(4, 4, rot_idx + 1)

    img = rotate_images[rot_idx][0, 0, :, :]
    # img = np.transpose(img, axes=(1, 2, 0))

    plt.imshow(img)

plt.show()

for rot_idx in range(num_rotations):


    plt.subplot(4, 4, rot_idx + 1)

    img = rotate_images_back[rot_idx][0, 0, :, :]
    # img = np.transpose(img, axes=(1, 2, 0))

    plt.imshow(img)

plt.show()
