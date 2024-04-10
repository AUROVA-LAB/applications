# Grasping Force Estimation in Markerless  Visuo-Tactile Sensors

## Overview
This repository presents the code of our last work [], which has been implemented with ROS Noetic in Ubuntu 20.04.

## Installation instructions

1. **Create a catkin workspace**:
	```
    mkdir -p digit_ros/src
    cd digit_ros
	```

2. **Add ROS packages**:
	- Clone the aurova_tactile_sensing repository:
	- `git clone https://github.com/AUROVA-LAB/aurova_tactile_sensing`
	- Add the [digit_interface](https://github.com/AUROVA-LAB/aurova_tactile_sensing/tree/main/digit_interface) package to digit_ros/src.
	- Add the [force_estimation](https://github.com/AUROVA-LAB/aurova_tactile_sensing/tree/main/force_estimation) package to digit_ros/src.
	
3. **Set up conda environment**:
	```    
	conda create --name==digit_ros_torch python=3.8.10
	conda activate digit_ros_torch
	pip install rospkg empy
	pip install numpy
	pip install opencv-python
	pip install digit-interface
	conda install pytorch==1.12.0 torchvision==0.13.0 torchaudio==0.12.0 cudatoolkit=11.3 -c pytorch
	pip install tqdm
	pip install wandb
	```
4. **Compile catkin workspace with conda environment activated**:
	```
	cd aurova_tactile_sensing/digit_ros/
	conda activate digit_ros_torch
	catkin_make
	source devel/setup.bash
	```

## Usage

1. **Run roslaunch to get images from DIGIT sensors**:

    `roslaunch digit_interface digit.launch`

2. **Run python script to estimate forces from tactile images**:

	```
	cd digit_ros/src/scripts/
	python estimate_force.py
	```
3. **Possible error due to cv_bridge**:
	- In case you get the following error with cv_bridge:
	`"ImportError: /lib/libgdal.so.26: undefined symbol: TIFFReadRGBATileExt, version LIBTIFF_4.0"`
	- I managed to solve with:
	`export LD_PRELOAD=/usr/bin/x86_64-linux-gnu/libtiff.so.5`

## Citation
If you find our code or papers useful, please cite:
`TODO`
