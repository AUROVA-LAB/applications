# Geo-Localization
This repository presents the code of the work [Geo-Localization Based on Dynamically Weighted Factor-Graph](https://aurova-projects.github.io/geo-localization_weighted/)

## Installation instructions (offline mode via rosbag)

### Requirements to use application:

- System requirements: Ubuntu 20.04 and ROS Noetic.
- External libraries: [ceres-solver-2.0.0](http://ceres-solver.org/installation.html) (IMPORTANT!! [download](https://drive.google.com/file/d/1acZtn_jaHfj2BVgwaDnQH2Lz-7022F1-/view?usp=share_link) version 2.0.0). Eigen and PCL are usually installed join with ROS.
- Local libraries: [lib_localization](https://github.com/AUROVA-LAB/lib_localization).
- External ROS packages: [iri_base_algorithm](https://gitlab.iri.upc.edu/labrobotica/ros/iri_core/iri_base_algorithm), "sudo apt-get install ros-noetic-ackermann-\*", "sudo apt-get install ros-noetic-robot-state-\*", "sudo apt-get install ros-noetic-hector-\*".
- Local ROS packages: [robot_blue](https://github.com/AUROVA-LAB/robot_blue), [aurova_preprocessed](https://github.com/AUROVA-LAB/aurova_preprocessed), [aurova_odom](https://github.com/AUROVA-LAB/aurova_odom), [aurova_detections](https://github.com/AUROVA-LAB/aurova_localization), and [aurova_localization](https://github.com/AUROVA-LAB/aurova_detections).

### Steps to use application:

- [Download](https://drive.google.com/file/d/1gHVI_dm_issXLnWmB-YW3GMwJ1N8QOri/view?usp=share_link) bag file for this example.
- Modify "launch/nav_GeoLoc_offline.launch" to provide correct link in "bag_file_1" argument.
- Modify "params/nav_GeoLoc_offline.yaml" to provide correct link in "url_to_map" variable.
- Run next command for localization:

```shell
roslaunch app_geo_localization nav_GeoLoc_offline.launch
```


