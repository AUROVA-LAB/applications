#!/bin/bash

# Generation of the dataset automatically. Launch the CARLA simulator and load the respective town. 
# Set your directories of the files CARLA_MAP and OSM_ROS (for each route).
# Set the outpur directory in the variable output_dir and check that folders "train" and "val" exist in that folder.
# For generating the data for other scene, edit the start_train, end_train, start_val, end_val, and scene variables.
# It is possible to resume the generation from a town setting the respective start route with start_train and start_val.

# Function to handle interruption
function handle_interrupt {
    echo "Interrupt received. Ending processes..."

    kill -s SIGINT $pid1 $pid2

    exit 1
}

# Use this if already saved some routes of the town.
start_train=23 #Scene 1 0-6, Scene 2 6-10, Scene 3 10-14, Scene 4 14-16, Scene 1 GOF 16-22, Scene 2 GOF 22-26
end_train=26
start_val=13 #Scene 1 0-4, Scene 2 4-6, Scene 3 6-8, Scene 4 8-9, Scene 1 GOF 9-13, Scene 2 GOF 13-15
end_val=15

wait_launch=6
wait=2
output_dir=/home/alolivas/aurova-lab/labrobotica/dataset/CARLA_GS_dataset

#ScientificPark
export CARLA_MAP="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/scripts/routes/ScientificPark.xml"
export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/scientific_park_simulation.osm"
export LAT_ZERO="38.38164422311365"
export LON_ZERO="-0.5275029920793286"

way_id_train=(1 2 3 4 5 6 12 13 14 15 18 19 20 21 22 24 24 31 32 33 34 35 36 41 42 43 44)
close_loop_train=("0 1 2 1" "2 3 4" "4 5 2" "1 0 4 0" "5 2 5 2 1" "3 4 5" "6 7 8 9 10 11" "11 10 9 8 7 6" "8 9 10" "8 9 8" "12 18 12" "18" "19 12" "14 18 12" "18 12 18" "20 21" "20 21" "32 33 34 33" "34 35 36" "36 37 34" "33 32 36 32" "37 34 37 34 33" "35 36 37" "38 39 40 41 42 43" "43 42 41 40 39 38" "40 41 42" "40 41 40")


way_id_val=(7 8 9 10 16 17 23 25 37 38 39 40 45 46)
close_loop_val=("3 4 3 4 0" "0 3 2" "1 5 4" "1 0 3 0" "8 7 6" "10 9 10 11" "16 18 12" "21 20" "35 36 35 36 32" "32 35 34" "33 37 36" "33 32 35 32" "40 39 38" "42 41 42 43")

scene="gof_scene2"
output_train=${output_dir}/train/ScientificPark_${scene}_session
output_val=${output_dir}/val/ScientificPark_${scene}_session

export PATH_DATASET="${output_train}"
for ((i=$start_train; i<${#way_id_train[@]} && i<$end_train; i++))
do
  export CARLA_WAY="${way_id_train[$i]}"
  export CLOSE_LOOP_ROS="${close_loop_train[$i]}"

  echo "Train way ${way_id_train[$i]}"
  #Spawn robot with ROS
  roslaunch app_MTAP nav_carla.launch &> /dev/null & 
  pid1=$!
  
  sleep $wait_launch
  rostopic pub --once /carla/base_link/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}"
  sleep 1

  #Record data TO DO check end of route in the recorder
  python record_ros_data.py --scene ${scene} &
  pid2=$!

  sleep $wait
  #Publish message for starting moving
rostopic pub --once /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 

  # Wait the end of the route
  wait $pid2

  sleep $wait

  kill -s SIGINT $pid1 

  wait
done

export PATH_DATASET="${output_val}"
for ((i=$start_val; i<${#way_id_val[@]} && i<$end_val; i++))
do
  export CARLA_WAY="${way_id_val[$i]}"
  export CLOSE_LOOP_ROS="${close_loop_val[$i]}"

  echo "Train way ${way_id_val[$i]}"
  #Spawn robot with ROS
  roslaunch app_MTAP nav_carla.launch &> /dev/null &
  pid1=$!

  sleep $wait_launch
  rostopic pub --once /carla/base_link/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 0.5, acceleration: 0.0, jerk: 0.0}"

  #Record data
  python record_ros_data.py --scene ${scene} &
  pid2=$!

  sleep $wait
  #Publish message for starting moving
rostopic pub /move_base_simple/goal --once geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 

  # Wait the end of the route
  wait $pid2

  sleep $wait

  kill -s SIGINT $pid1

  wait
  
done

wmctrl -a carla/PythonAPI/aurova




