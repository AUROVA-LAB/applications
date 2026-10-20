FROM ros:jazzy
ENV ROS_DISTRO=jazzy
RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-colcon-mixin python3-vcstool
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-xacro ros-${ROS_DISTRO}-joint-state-publisher-gui
RUN apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-rqt-tf-tree && \
    rm -rf /var/lib/apt/lists/*
RUN rosdep update --include-eol-distros
RUN rm /etc/apt/apt.conf.d/docker-clean

#Install libraries
#Ceres
RUN apt-get update && apt-get install -y cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev
RUN mkdir algorithms && cd algorithms && git clone https://ceres-solver.googlesource.com/ceres-solver && cd ceres-solver && git checkout 2.0.0
RUN mkdir algorithms/ceres-bin && cd algorithms/ceres-bin && cmake ../ceres-solver && make -j3 && make install

#lib_localization
RUN apt-get install -y ros-${ROS_DISTRO}-pcl-ros
RUN cd algorithms && git clone https://github.com/AUROVA-LAB/lib_localization && cd lib_localization && mkdir build && cd build && \
    cmake .. && make && make install
#lib_planning
RUN cd algorithms && git clone https://github.com/AUROVA-LAB/lib_planning && cd lib_planning && mkdir build && cd build && \
    cmake .. && make && make install
RUN apt-get install -y ros-${ROS_DISTRO}-ackermann-* ros-${ROS_DISTRO}-robot-state-* && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-pcl-conversions && \
    rm -rf /var/lib/apt/lists/*
RUN apt-get update && apt-get install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]