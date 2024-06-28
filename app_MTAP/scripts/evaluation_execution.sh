#!/bin/bash

# Execution of the test automatically. Launch the CARLA simulator and load the respective town. 
# The valid architectures are MTAP, TCP, transfuser and ROS. Except for ROS, the agent must be running 
# (it is possible to run it in a docker) and the programs are commnuicate with ZeroMQ library.
# Set your directories of the files CARLA_MAP and OSM_ROS (for each route).
# Set the outpur directory in the environment variable CARLA_RESULTS_PATH and check that the forlder exists.
# For generating the data for other town, firstly load it in CARLA and then uncomment the respective lines.
# It is possible to resume the generation from a town setting the respective start route with start_test.
# The variable output indicates the prefix of the output files.

# Function to handle interruption
function handle_interrupt {
    echo "Interrupt received. Ending processes..."

    kill -s SIGINT $pid1 $pid2

    exit 1
}

full_evaluation=true
start_test=0
wait_launch=14
wait=5
output=MTAP
architecture=MTAP
pedestrian_routes_prefix=routes/pedestrian_routes/
export CARLA_RESULTS_PATH="/media/alolivas/MSI_500/carla_limpio/carla/PythonAPI/aurova/results/"

#Town01
export CARLA_MAP="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/routes/Town01.xml"
export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/Town01.osm"
export LAT_ZERO="38.38164422311365"
export LON_ZERO="-0.5255029920793286"


if [ "$full_evaluation" = true ]; then
  way_id=(6 8 9 10 11 12 14)
  pedestrian_routes=(Town01_evaluation1_full.xml Town01_evaluation2_full.xml Town01_evaluation3_full.xml Town01_evaluation4_full.xml Town01_evaluation4_full.xml Town01_evaluation4_full.xml  Town01_evaluation5_full.xml)
  close_loop=("0 1" "0 3" "3 0" "0 5" "5 4" "4 5" "6 7")
  weather=(6 2 0 6 1 1 4)
else
  way_id=(6 8 10 11 12 14)
  pedestrian_routes=(Town01_evaluation1.xml Town01_evaluation2.xml Town01_evaluation4.xml Town01_evaluation4.xml Town01_evaluation4.xml  Town01_evaluation5.xml)
  close_loop=("0 1" "0 3" "0 5" "5 4" "4 5" "6 7")
  weather=(6 2 6 1 1 4)
fi

# Town02
# export CARLA_MAP="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/routes/Town02.xml"
# export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/Town02.osm"
# export LAT_ZERO="38.3799"
# export LON_ZERO="0.0"

# if [ "$full_evaluation" = true ]; then
#   way_id=(1 2 3 4 5 6 7 8 9 10 11)
#   pedestrian_routes=(Town02_evaluation1_full.xml Town02_evaluation1_full.xml Town02_evaluation1_full.xml Town02_evaluation1_full.xml Town02_evaluation1_full.xml Town02_evaluation1_full.xml Town02_evaluation1_full.xml Town02_evaluation1_full.xml Town02_evaluation2_full.xml Town02_evaluation2_full.xml Town02_evaluation2_full.xml)
#   close_loop=("1 2" "3" "1" "4 3" "6 5" "1 0" "4" "2" "3 4" "1 2" "5 6")
#   weather=(6 1 1 2 2 6 2 6 1 0 6)
# else
#   way_id=(1 2 4 6 7 8 9 11)
#   pedestrian_routes=(Town02_evaluation1.xml Town02_evaluation1.xml Town02_evaluation1.xml Town02_evaluation1.xml Town02_evaluation1.xml Town02_evaluation1.xml Town02_evaluation2.xml Town02_evaluation2.xml)
#   close_loop=("1 2" "3" "4 3" "1 0" "4" "2" "3 4" "5 6")
#   weather=(6 1 2 6 2 6 1 6)
# fi

#Town03
# export CARLA_MAP="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/routes/Town03.xml"
# export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/Town03.osm"
# export LAT_ZERO="38.3811"
# export LON_ZERO="0.001"

# if [ "$full_evaluation" = true ]; then
#   pedestrian_routes=(Town03_evaluation1_full.xml)
# else
#   pedestrian_routes=(Town03_evaluation1.xml)
# fi
# way_id=(-1)
# pedestrian_routes=(Town03_evaluation1.xml)
# close_loop=("5 4 3 2 1 0")
# weather=(6)

#Town04
# export CARLA_MAP="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/routes/Town04.xml"
# export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/Town04.osm"
# export LAT_ZERO="38.3839"
# export LON_ZERO="0.001"

# if [ "$full_evaluation" = true ]; then
#   way_id=(2 3 4 5)
#   pedestrian_routes=(Town04_evaluation1_full.xml Town04_evaluation1_full.xml Town04_evaluation1_full.xml Town04_evaluation1_full.xml)
#   close_loop=("11 7" "2 3 4 5 6 7" "0 7" "9 0")
#   weather=(6 2 0 4)
# else
#   way_id=(2 3 5)
#   pedestrian_routes=(Town04_evaluation1.xml Town04_evaluation1.xml Town04_evaluation1.xml)
#   close_loop=("11 7" "2 3 4 5 6 7" "9 0")
#   weather=(6 2 4)
# fi

#Town05
# export CARLA_MAP="/media/alolivas/MSI_500/aurova_carla/carla/PythonAPI/aurova/routes/Town05.xml"
# export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/Town05.osm"
# export LAT_ZERO="38.381"
# export LON_ZERO="0.0"

# if [ "$full_evaluation" = true ]; then
#   way_id=(2 3 4 5 6 7 8)
#   pedestrian_routes=(Town05_evaluation1_full.xml Town05_evaluation1_full.xml Town05_evaluation1_full.xml Town05_evaluation1_full.xml Town05_evaluation1_full.xml Town05_evaluation1_full.xml Town05_evaluation1_full.xml)
#   close_loop=("6" "7" "7" "9 10 11" "11" "12 0" "1 2 3 4 5")
#   weather=(6 1 1 6 6 2 2)
# else
#   way_id=(2 3 5 6 8)
#   pedestrian_routes=(Town05_evaluation1.xml Town05_evaluation1.xml Town05_evaluation1.xml Town05_evaluation1.xml Town05_evaluation1.xml)
#   close_loop=("6" "7" "9 10 11" "11" "1 2 3 4 5")
#   weather=(6 1 6 6 2)
# fi


for ((i=$start_test; i<${#way_id[@]}; i++))
do
    echo "Test way ${way_id[$i]}"
    export CARLA_WAY="${way_id[$i]}"
    export PEDESTRIAN_ROUTES="$pedestrian_routes_prefix${pedestrian_routes[$i]}"

    # Execute the python files
    if [ "$architecture" = "ROS" ]; then
        export CLOSE_LOOP_ROS="${close_loop[$i]}"

        roslaunch app_MTAP nav_carla.launch &
        pid1=$!

        sleep $wait_launch
        python change_weather.py --weather "${weather[$i]}"
        python pedestrians_fixed_routes.py --output $output -c &
        pid2=$!

        sleep $wait
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

    else
        python listener_agent.py --architecture $architecture &
        pid1=$!

        sleep $wait_launch

        python pedestrians_fixed_routes.py --output $output -s &
        pid2=$!
    fi

    # Wait the end of the route
    wait $pid2

    sleep $wait_launch

    kill -s SIGINT $pid1

    wait
  
done
wmctrl -a carla/PythonAPI/aurova

