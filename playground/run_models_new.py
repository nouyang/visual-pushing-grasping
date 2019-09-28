import argparse
import numpy as np
import matplotlib.pyplot as plt
import torch
import torchvision
import torchvision.transforms as transforms
import models


def main(args):

    device = torch.device(args.device)

    num_rotations = 16

    net = models.reinforcement_net(device=device)
    net.num_rotations = num_rotations

    if args.cifar:
        transform = transforms.Compose(
            [transforms.ToTensor(),
             transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5))]
        )

        trainset = torchvision.datasets.CIFAR10(root="./data", train=True, download=True, transform=transform)
        trainloader = torch.utils.data.DataLoader(trainset, batch_size=5, shuffle=True, num_workers=2)
        dataiter = iter(trainloader)
        images, labels = dataiter.next()

        input_color_data_t = images
        input_depth_data_t = images[:, 0:1, :, :]
    else:
        input_color_data = np.random.uniform(0, 1, size=(6, 3, 140, 180)).astype(np.float32)
        input_depth_data = np.random.uniform(0, 1, size=(6, 1, 140, 180)).astype(np.float32)
        input_color_data_t = torch.from_numpy(input_color_data).to(device)
        input_depth_data_t = torch.from_numpy(input_depth_data).to(device)

    physics_prediction = [1.0]

    if args.no_grad:
        with torch.no_grad():
            outputs_t = net.forward(
                input_color_data_t, input_depth_data_t, physics_prediction, specific_rotation=args.specific_rotation
            )
    else:
        outputs_t = net.forward(
            input_color_data_t, input_depth_data_t, physics_prediction, specific_rotation=args.specific_rotation
        )

    outputs = [ft.detach().cpu().numpy() for ft in outputs_t]

    for rot_idx in range(num_rotations):

        if args.specific_rotation is not None:
            # hacky way of showing only one image
            if rot_idx != args.specific_rotation:
                continue
            else:
                rot_idx = 0

        plt.subplot(4, 4, rot_idx + 1)

        img = outputs[rot_idx][0, 0, :, :]
        # img = np.transpose(img, axes=(1, 2, 0))

        plt.imshow(img)

    plt.show()


parser = argparse.ArgumentParser()
parser.add_argument("--device", default="cuda:0")
parser.add_argument("--no-grad", default=False, action="store_true")
parser.add_argument("--specific-rotation", type=int, default=None)
parser.add_argument("--cifar", default=False, action="store_true")
parsed = parser.parse_args()
main(parsed)
