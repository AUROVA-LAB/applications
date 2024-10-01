# Robust Single Object Tracking and Following by Fusion Strategy

## Overview
This repository contains the code for the ICINCO 2023 paper [Robust Single Object Tracking and Following by Fusion Strategy](https://aurova-projects.github.io/tracker_fusion/). The code has been implemented with ROS Noetic in Ubuntu 20.04. 

## Installation instructions

1. **Create a catkin workspace**:
```
    mkdir tracker_fusion_ws
    cd tracker_fusion_ws
    catkin init
```

2. **Clone Repositories**:
    - Clone the [applications](https://github.com/AUROVA-LAB/applications) repository.

    - Clone the repository [aurova_detections](https://github.com/AUROVA-LAB/aurova_detections). To compile only the needed files, leave the dependecies:
      ```
      <exec_depend>tracker_filter</exec_depend>
      <exec_depend>detection_msgs</exec_depend>
      ```
      of the file `aurova_detections/aurova_detections/package.xml`, and comment the others dependecies.

    - Clone the repository [aurova_planning](https://github.com/AUROVA-LAB/aurova_planning). To compile only the needed files, leave the dependecies:
      ```
      <exec_depend>local_planning</exec_depend>
      ```
      of the file `aurova_planning/aurova_planning/package.xml`, and comment the others dependecies.

    - Clone the repository [aurova_preprocessed](https://github.com/AUROVA-LAB/aurova_preprocessed). To compile only the needed files, leave the dependecies:
      ```
      <exec_depend>lidar_obstacle_detection</exec_depend> 
      ```
      of the file `aurova_planning/aurova_planning/package.xml`, and comment the others dependecies.

3. **Install Libraries**:
    - Follow the instructions to install the [lib_planning](https://github.com/AUROVA-LAB/lib_planning) library.

4. **Build Docker Images**:
    - Follow the instuctions of [image_trackers_ouster](https://github.com/AUROVA-LAB/aurova_detections/tree/main/image_trackers_ouster) to build the Docker image with the images trackers.
    - Optionally, follow the instructions of [m2track_ros](https://github.com/AUROVA-LAB/aurova_detections/tree/main//m2track_ros) to build the Docker image with the point cloud tracker (consider that better results are reported without using it).

5. **Requirements for BLUE robot (Optional)**. 
    - Clone the repository [robot_blue](https://github.com/AUROVA-LAB/robot_blue).

6. **Install ROS packages**:
    ```
    cd tracker_fusion_ws
    catkin_make
    ```

## Usage

1. **Run Docker Images**:

    - For the image tracker:
      ```
      sudo docker run --rm --gpus all --ipc=host --ulimit memlock=-1 --ulimit stack=67108864 --net host -it aurova_image_tracker
      ```
    - Optionally for the point cloud tracker:
        ```
      sudo docker run --rm --gpus all --ipc=host --ulimit memlock=-1 --ulimit stack=67108864 --net host -it aurova_pc_tracker
      ```

2. **Launch ROS Nodes**:
    - Offline mode:
      ```
      roslaunch app_tracker_fusion nav_follow_offline.launch
      ```

    - Online mode:
      ```
      roslaunch app_tracker_fusion nav_follow_online.launch
	    ```

## Citation
If you find our code or papers useful, please cite:
```
@conference{olivas2023robust,
  author={Alejandro Olivas. and Miguel Muñoz{-}Bañón. and Edison Velasco. and Fernando Torres.},
  title={Robust Single Object Tracking and Following by Fusion Strategy},
  booktitle={Proceedings of the 20th International Conference on Informatics in Control, Automation and Robotics - Volume 1: ICINCO},
  year={2023},
  pages={624-631},
  publisher={SciTePress},
  organization={INSTICC},
  doi={10.5220/0012178900003543},
  isbn={978-989-758-670-5},
  issn={2184-2809},
}
```
