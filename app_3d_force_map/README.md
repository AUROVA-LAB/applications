# Zero-Shot Transfer of Force Map Estimation Across GelSight Mini Sensors

## Overview
This repository presents the code of our work [Zero-Shot Transfer of Force Map Estimation Across GelSight Mini Sensors]().

## Installation instructions

1. **Clone repository**:
	- Clone the aurova_tactile_sensing repository:
	- `git clone https://github.com/AUROVA-LAB/aurova_tactile_sensing`
	- Move to folder:
	- `cd aurova_tactile_sensing/3d_force_map_gsmini/`

	
3. **Set up conda environment**:
	```    
	conda create -n unit_force_env python==3.10
	conda activate unit_force_env
	pip install torch==2.6.0 torchvision==0.21.0 torchaudio==2.6.0 --index-url https://download.pytorch.org/whl/cu124
	pip install opencv-python
	pip install hydra-core
	pip install pytorch-lightning
	pip install wandb
	pip install einops
	pip install dill
	```
4. **Download weights from google drive**:
	https://drive.google.com/drive/folders/1ne4Psy9-H-rKTsbeVl46RH5aDO3z7tBN?usp=sharing

5. **Move the weights to the proper folder**:
	- Put the "checkpoint-epoch=34.ckpt" file in the ./unit/weights/vq_gan_mid folder.
	- Put the "unet_force_diff.pt" and/or "unet_force_rgb_nomark.pt" in the ./force/weights folder.

## Usage

1. **Connect your GelSight Mini sensor and run the following command**:

    `python force_estimation_live.py`


## Citation
If you find our code or papers useful, please cite:
`TODO`
