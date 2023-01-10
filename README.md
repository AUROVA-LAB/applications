# applications
The "applications" repository is a ROS package that contains files to combine different nodes with different robots to setup different applications. Each launch file in the "launch" directory is a concrette application. Next there are some examples of applications execution.

## Example 1: Autonomous navigation NVP + GN-fusion (offline mode via rosbag)

### Requirements to use application:

- System requirements: Ubuntu 20.04 and ROS Noetic.
- External libraries: [ceres-solver-2.0.0](http://ceres-solver.org/installation.html) (IMPORTANT!! [download](https://drive.google.com/file/d/1acZtn_jaHfj2BVgwaDnQH2Lz-7022F1-/view?usp=share_link) version 2.0.0). Eigen and PCL are usually installed join with ROS.
- Local libraries: [lib_planning](https://github.com/AUROVA-LAB/lib_planning).
- External ROS packages: [iri_base_algorithm](https://gitlab.iri.upc.edu/labrobotica/ros/iri_core/iri_base_algorithm), "sudo apt-get install ros-noetic-ackermann-\*", "sudo apt-get install ros-noetic-robot-state-\*", "sudo apt-get install ros-noetic-hector-\*".
- Local ROS packages: [robot_blue](https://github.com/AUROVA-LAB/robot_blue), [aurova_preprocessed](https://github.com/AUROVA-LAB/aurova_preprocessed), [aurova_odom](https://github.com/AUROVA-LAB/aurova_odom), [aurova_localization](https://github.com/AUROVA-LAB/aurova_localization), and [aurova_planning](https://github.com/AUROVA-LAB/aurova_planning).

### Steps to use application:

- [Download](https://drive.google.com/file/d/1gHVI_dm_issXLnWmB-YW3GMwJ1N8QOri/view?usp=share_link) bag file for this example.
- Modify "launch/nav_NVP_GN_offline.launch" to provide correct link in "bag_file_1" argument.
- Modify "params/nav_NVP_GN_offline.yaml" to provide correct link in "url_path" variable.
- Run next command for localization:

```shell
roslaunch applications nav_NVP_GN_offline.launch
```
- For autonomous navigation send "2D Nav Goal" from rviz.