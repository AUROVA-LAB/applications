# applications
The "applications" repository is a ROS package that contains files to combine different nodes with different robots to setup different applications. Each launch file in the "launch" directory is a concrette application.

## Autonomous navigation NVP + GPS/LiDAR (offline mode via rosbag)

### Requirements to use this package:

- System requirements: Ubuntu 20.04 and ROS Noetic.
- Libraries: [lib_planning](https://github.com/AUROVA-LAB/lib_planning), Eigen, and PCL.
- BLUE model for Gazebo: Follow instructions in [robot_blue_gazebo](https://github.com/AUROVA-LAB/robot_blue_gazebo).
- ROS packages: [aurova_preprocessed](https://github.com/AUROVA-LAB/aurova_preprocessed), [aurova_localization](https://github.com/AUROVA-LAB/aurova_localization), and [aurova_planning](https://github.com/AUROVA-LAB/aurova_planning). 
- ROS application packages: [application_navigation](https://github.com/AUROVA-LAB/application_navigation), and [application_localization](https://github.com/AUROVA-LAB/application_localization).
