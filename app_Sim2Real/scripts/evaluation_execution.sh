#!/bin/bash

# Execution of the test automatically. Launch the CARLA simulator and load the respective town. 
# The valid architectures are mtap and ROS. Except for ROS, the agent must be running 
# (it is possible to run it in a docker) and the programs are commnuicate with ZeroMQ library.
# Set your directories of the files CARLA_MAP and OSM_ROS (for each route).
# Set the output directory in the environment variable CARLA_RESULTS_PATH and check that the forlder exists.
# For generating the data for other town, firstly load it in CARLA and then uncomment the respective lines.
# It is possible to resume the generation from a town setting the respective start route with start_test.
# The variable output indicates the prefix of the output files.

# Function to handle interruption
function handle_interrupt {
    echo "Interrupt received. Ending processes..."

    kill -s SIGINT $pid1 $pid2

    exit 1
}


start_test=0
wait_launch=10
wait=1

architecture=mtap

#ScientifikPark (directories inside the docker)
export CARLA_MAP="/app_MTAP/scripts/routes/ScientificPark.xml"
export OSM_ROS="/app_MTAP/paths/scientific_park_simulation.osm"

#ScientifikPark for ROS (outside docker)
# export CARLA_MAP="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/scripts/routes/ScientificPark.xml"
# export OSM_ROS="/home/alolivas/aurova-lab/aurova_ws/src/applications/app_MTAP/paths/scientific_park_simulation.osm"
# export CARLA_RESULTS_PATH="/home/alolivas/aurova-lab/temporal_private_repositories/Sim2Real_local_planner/results/"

export LAT_ZERO="38.38164422311365"
export LON_ZERO="-0.5275029920793286"


model_GS="octreeGS"
# way_id=(7 8 9 10)
# close_loop=("3 4 3 4 0" "0 3 2" "1 5 4" "1 0 3 0")
# scene="scene1"

# way_id=(2)
# close_loop=("2 3 4")
# scene="scene1"

# way_id=(12)
# close_loop=("6 7 8 9 10 11")
# scene="scene2"

# way_id=(21)
# close_loop=("14 18 12")
# scene="scene3"

way_id=(24)
close_loop=("20 21")
scene="scene4"


# scene="test_route1"
# way_id=(26)
# close_loop=("22 23 24 25 26 31 27 28 29 30 22")
# output_dir="results/${scene}_${model_GS}_safety"

for ((i=$start_test; i<${#way_id[@]}; i++))
do
    echo "Test way ${way_id[$i]}"
    export CARLA_WAY="${way_id[$i]}"

    # Execute the python files
    if [ "$architecture" = "ros" ]; then
        export CLOSE_LOOP_ROS="${close_loop[$i]}"

        roslaunch app_MTAP nav_carla.launch &> /dev/null & 
        pid1=$!

        sleep $wait_launch
rostopic pub --once /carla/base_link/ackermann_cmd ackermann_msgs/AckermannDrive "{steering_angle: 0.0, steering_angle_velocity: 0.0, speed: 0.0, acceleration: 0.0, jerk: 0.0}"

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
        python listener_agent_GS.py --model_GS $model_GS --scene $scene --output_dir $output_dir &
        pid1=$!

        sleep $wait_launch
    fi

    # Wait the end of the route
    wait $pid1

    sleep $wait_launch

    kill -s SIGINT $pid1

    wait
  
done

