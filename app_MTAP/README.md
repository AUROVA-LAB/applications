# Overview

This repository contains resources and instructions for setting up and utilizing CARLA Simulator for pedestrian avoidance with the framework presented in [Memory-based Trajectory and Action Prediction](). Currently, the repository has the code for generating the dataset and for evaluating approaches on the pedestrian avoidance problem. The code, weigths and instructions of MTAP will be provided after the acceptance of the paper.

![MTAP pedestrian avoidance](./media/MTAP_example1_compress.gif)

## Installation

System requirements: Ubuntu 20.04 and ROS Noetic.

1. Follow the instructions outlined in [CARLA's documentation](https://carla.readthedocs.io/en/latest/build_linux/) to install Unreal Engine and CARLA. IMPORTANT: when cloning the repository, clone our [fork](https://github.com/Aleoli2/carla):
    ```
    git clone https://github.com/Aleoli2/carla.git
    ```
2. Install the CARLA ROS bridge following [ROS Bridge Installation](https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros1/) by using the source repository, but for step 2 use our [fork](https://github.com/Aleoli2/ros-bridge):
    ```
    git clone --recurse-submodules https://github.com/Aleoli2/ros-bridge.git catkin_ws/src/
    ```
3. Install the python CARLA library: 
    ```
    cd <path/to/carla>/PythonAPI/carla/dist/
    pip install <carla_wheel>.whl
    ```
    Add the source path for the ROS bridge workspace. `source /path/to/catkin_ws/devel/setp.bash`. To add it permanently:
    ```
    echo "source /path/to/catkin_ws/devel/setp.bash" >> ~/.bashrc
    ```
4. Installation of AUROVA planning and localization.
    - External libraries: [ceres-solver-2.0.0](http://ceres-solver.org/installation.html) (IMPORTANT!! [download](https://drive.google.com/file/d/1acZtn_jaHfj2BVgwaDnQH2Lz-7022F1-/view?usp=share_link) version 2.0.0). Eigen and PCL are usually installed join with ROS.
    - Local libraries: [lib_localization](https://github.com/AUROVA-LAB/lib_localization) and [lib_planning](https://github.com/AUROVA-LAB/lib_planning).
    - Clone the ROS packages at `catkin_ws/src/` and build them with `catkin_make`.
        - External ROS packages: [iri_base_algorithm](https://gitlab.iri.upc.edu/labrobotica/ros/iri_core/iri_base_algorithm), "sudo apt-get install ros-noetic-ackermann-\*", "sudo apt-get install ros-noetic-robot-state-\*", "sudo apt-get install ros-noetic-hector-\*".
        - Local ROS packages: [aurova_preprocessed](https://github.com/AUROVA-LAB/aurova_preprocessed), [aurova_odom](https://github.com/AUROVA-LAB/aurova_odom),  [aurova_localization](https://github.com/AUROVA-LAB/aurova_localization), and [aurova_planning](https://github.com/AUROVA-LAB/aurova_planning).

## Usage

### CARLA Simulator Launch

1. Launch CARLA Simulator by running `CarlaUE4.sh`.
2. Maps with the suffix `Aurova` are optimized for pedestrian zone navigation for the robot BLUE. Navigate to `Content/Carla/Maps` to load one among these maps (this process may take several minutes).
3. Once loaded, click `Play` and wait for the simulation to start.

### Dataset generation and evaluation

The bash scripts [dataset_generation.sh](./scripts/dataset_generation.sh) and [evaluation_execution.sh](./scripts/evaluation_execution.sh) manage CARLA to spawn the vehicle, load the pedestrians and their routes, change to the specified weather of the route, and launching the ROS agent. 

For generating the dataset, follow the instructions of the bash script [dataset_generation.sh](./scripts/dataset_generation.sh). This script launch the expert agent (the dynamic version of our NVP algorithm) and save the 360 RGB-D images, the LiDAR data, the pedestrians' position (when they are close to the UGV), the desired Ackermann state and other measurements (goal coordinates, robot's pose, accelerometer, GPS...). Before executing it, load the respective town (with the suffix Aurova).
```
roscd app_MTAP/scripts
./dataset_generation.sh
```

In a similar way, follow the instructions of [evaluation_execution.sh](./scripts/evaluation_execution.sh) for evaluation. It is prepared to evaluate our static local planner NVP with ROS directly, and end-to-end methods through communication using ZeroMQ. In the last case, the agent should run in a separete terminal (Docker was used in our case). The metrics *Driving Score, Route Completion, Infraction Penalty, Robot Interaction Performance* and *Pedestrian Interaction Performance* are saved for each route. You could see `aurova_planning/MTAP_local_planning/agent.py` (when available) and edit [listener_agent.py](./scripts/listener_agent.py) for adding new architectures and communicate the method with the agent.


#### Utils

There are several Python scripts in [scripts](./scripts) folder:

- `python manual_control_joystick.py`: Spawns robot BLUE, controlled by a joystick.
- `python joystick_keymap.py`: Displays current joystick key mapping. Modify global variables in Python scripts to configure controls.
- `python pedestrians_fixed_routes.py`: Spawns the pedestrians. Configure it with command-line options or with the [`config.py`](./scripts/config.py) file.
- `python listener_agent.py --architecture X`: Spawns robot BLUE, publishing data for end-to-end architecture X, and awaits control messages.
- `python map2OpenStreet.py -m MAP_FILE`: Generates georeference location for locations in MAP_FILE. CARLA should be running the same map.
- `python metrics_average.py /path/to/folder/prefix`: Calculates the mean value of the metrics in `/path/to/folder/` that starts with `prefix`.

#### CARLA with ROS
- To spawn the robot and use AUROVA's localization and planning algorithms: `roslaunch app_MTAP nav_carla.launch`.
- To spawn the robot with pedestrian trajectory prediction: `roslaunch app_MTAP nav_carla_prediction.launch`.
