#!/bin/bash

# Generation of the dataset automatically. Launch the CARLA simulator and load the respective town. 
# Set your directories of the files CARLA_MAP and OSM_ROS (for each route).
# Set the outpur directory in the variable output_dir and check that folders "train" and "val" exist in that folder.
# For generating the data for other town, firstly load it in CARLA and then uncomment the respective lines.
# It is possible to resume the generation from a town setting the respective start route with start_train and start_val.

# Function to handle interruption
function handle_interrupt {
    echo "Interrupt received. Ending processes..."

    kill -s SIGINT $pid1 $pid2

    exit 1
}

# Use this if already saved some routes of the town.
start_train=0
start_val=0

wait_launch=17
wait=5
output_dir=/home/alolivas/aurova-lab/labrobotica/dataset/CARLA_dataset

pedestrian_routes_prefix=routes/pedestrian_routes/

#Town01
export CARLA_MAP="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/routes/Town01.xml"
export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/Town01.osm"
export LAT_ZERO="38.38164422311365"
export LON_ZERO="-0.5255029920793286"

way_id_train=(1 -1 -1 2 -2 3 -3 -3 4 4 -4)
pedestrian_routes_train=(Town01_experiment1.xml Town01_experiment1.xml Town01_experiment1.xml Town01_experiment2.xml Town01_experiment2.xml Town01_experiment3.xml Town01_experiment3.xml Town01_experiment3.xml Town01_experiment4.xml Town01_experiment4.xml Town01_experiment4.xml)
weather_train=(4 4 0 4 2 5 5 4 6 1 0)
close_loop_train=("1 2 3 0" "3 2 1 0" "3 2 1 0" "1 0 3 0" "3 0 1 0" "2 3 0" "3 2 1" "3 2 1" "1 2 3 2" "1 2 3 2" "3 2 1 2")

way_id_val=(1 2 3 -4)
pedestrian_routes_val=(Town01_experiment1.xml Town01_experiment2.xml Town01_experiment3.xml Town01_experiment4.xml)
weather_val=(1 4 4 6)
close_loop_val=("1 2 3 0" "1 0 3 0" "2 3 0" "3 2 1 2")

output_train=${output_dir}/train/Town01_session
output_val=${output_dir}/val/Town01_session

#Town03
# export CARLA_MAP="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/routes/Town03.xml"
# export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/Town03.osm"
# export LAT_ZERO="38.3811"
# export LON_ZERO="0.001"

# way_id_train=(1 1 -1)
# pedestrian_routes_train=(Town03_experiment1.xml Town03_experiment1.xml Town03_experiment1.xml)
# weather_train=(2 4 2)
# close_loop_train=("1234560" "1234560" "5432106")

# way_id_val=(-1)
# pedestrian_routes_val=(Town03_experiment1.xml)
# weather_val=(1 4 4 6)
# close_loop_val=("5432106")

# output_train=${output_train}/train/Town03_session
# output_val=${output_train}/val/Town03_session

#Town04
# export CARLA_MAP="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/routes/Town04.xml"
# export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/Town04.osm"
# export LAT_ZERO="38.3839"
# export LON_ZERO="0.001"

# way_id_train=(1 -1 -1)
# pedestrian_routes_train=(Town04_experiment1.xml Town04_experiment1.xml Town04_experiment1.xml)
# weather_train=(4 4 2)
# close_loop_train=("1234567091080" "8109076543210" "8109076543210")

# way_id_val=(1)
# pedestrian_routes_val=(Town04_experiment1.xml)
# weather_val=(6)
# close_loop_val=("1234567091080")

# output_train=${output_train}/train/Town04_session
# output_val=${output_train}/val/Town04_session

#Town05
# export CARLA_MAP="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/routes/Town05.xml"
# export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/Town05.osm"
# export LAT_ZERO="38.381"
# export LON_ZERO="0.0"

# way_id_train=(1 1 -1)
# pedestrian_routes_train=(Town05_experiment1.xml Town05_experiment1.xml Town05_experiment1.xml)
# weather_train=(1 2 1)
# close_loop_train=("1234567891011120" "1234567891011120" "1211109876543210")

# way_id_val=(-1)
# pedestrian_routes_val=(Town05_experiment1.xml)
# weather_val=(6)
# close_loop_val=("1211109876543210")

# output_train=${output_train}/train/Town05_session
# output_val=${output_train}/val/Town05_session

export PATH_DATASET="${output_train}"
for ((i=$start_train; i<${#way_id_train[@]}; i++))
do
  export CARLA_WAY="${way_id_train[$i]}"
  export PEDESTRIAN_ROUTES="$pedestrian_routes_prefix${pedestrian_routes_train[$i]}"
  export CLOSE_LOOP_ROS="${close_loop_train[$i]}"

  echo "Train way ${way_id_train[$i]}"
  #Spawn robot with ROS
  roslaunch app_MTAP nav_carla_prediction.launch &> /dev/null & 
  pid1=$!

  sleep $wait_launch
  python change_weather.py --weather "${weather_train[$i]}"
  #Spawn pedestrians
  python pedestrians_fixed_routes.py &> /dev/null &
  pid2=$!

  sleep $wait
  #Record data
  python record_ros_data.py &
  pid3=$!

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

  kill -s SIGINT $pid1 $pid3

  wait
done

export PATH_DATASET="${output_val}"
for ((i=$start_val; i<${#way_id_val[@]}; i++))
do
  export CARLA_WAY="${way_id_val[$i]}"
  export PEDESTRIAN_ROUTES="$pedestrian_routes_prefix${pedestrian_routes_val[$i]}"
  export CLOSE_LOOP_ROS="${close_loop_val[$i]}"

  echo "Train way ${way_id_val[$i]}"
  #Spawn robot with ROS
  roslaunch app_MTAP nav_carla_prediction.launch &
  pid1=$!

  sleep $wait_launch
  #Spawn pedestrians
  python change_weather.py --weather "${weather_val[$i]}"
  python pedestrians_fixed_routes.py  &
  pid2=$!

  #Record data
  python record_ros_data.py &
  pid3=$!

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

  sleep $wait_launch

  kill -s SIGINT $pid1 $pid3

  wait
  
done

wmctrl -a carla/PythonAPI/aurova




