#!/usr/bin/env python

# Generate the gaussian splatting images for the dataset from the saved transformations.

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import configGS

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
try:
    import cv2
except ImportError:
    raise RuntimeError('cannot import opencv, make sure cv2 package is installed')
import argparse
import importlib


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--model_GS',
        metavar='H',
        default='gof',
        help='Gaussian splatting model (gof, octree or hierarchical3DGS) (default: gof)')
    argparser.add_argument(
        '--scene',
        metavar='H',
        default='scene1',
        help='Gaussian splatting scene (scenes defined in configGS.py) (default: scene1)')
    argparser.add_argument(
        '--dataset_path',
        metavar='H',
        default='/dataset',
        help='Path to the dataset (default: /dataset)')
    args = argparser.parse_args()
    if os.path.exists(args.dataset_path):
        dataset_path = args.dataset_path
    else:
        print("Path does not exist")
        sys.exit(1)

    configGS.get_args(args, args.scene, args.model_GS)
    # Set the path to the Gaussian Splatting module
    if args.model_GS == "gof":
        renderer_module_path = "/workspace/submodules/gaussian-opacity-fields/"
    elif args.model_GS == "hierarchical3DGS":
        renderer_module_path = "/workspace/submodules/hierarchical-3d-gaussians/"
    elif args.model_GS == "octreeGS":
        renderer_module_path = "/workspace/submodules/Octree-GS/"

    sys.path.append(renderer_module_path)
    renderer_module = importlib.import_module("render_gaussian")
    Renderer = getattr(renderer_module, "Renderer")
    renderer = Renderer(args)

    train_folders = glob.glob(os.path.join(dataset_path+"/train", "*"))
    val_folders = glob.glob(os.path.join(dataset_path+"/val", "*"))
    filtered_folders = [folder for folder in [*train_folders, *val_folders] if args.scene in os.path.basename(folder)]
    for folder in filtered_folders:
        with open(os.path.join(folder, "transformations.csv"), 'r') as f:
            os.makedirs(os.path.join(folder, "rgb_images_{}".format(args.model_GS)),exist_ok=True)
            for i,line in enumerate(f):
                if i==0:continue
                line = line.split(',')
                image_path = os.path.join(folder, "rgb_images_{}/{}.png".format(args.model_GS,i))
                if os.path.exists(image_path): continue
                R = np.array([float(x) for x in line[0:9]]).reshape(3, 3)
                t = np.array([float(x) for x in line[9:12]])
                GS_image=renderer.render_view(R,t)
                GS_image = GS_image.swapaxes(0, 2).swapaxes(0, 1)
                GS_image = cv2.cvtColor(GS_image, cv2.COLOR_RGB2BGR)
                cv2.imwrite(image_path, GS_image)
                print("Saved image at: ", image_path)


if __name__ == '__main__':

    main()
