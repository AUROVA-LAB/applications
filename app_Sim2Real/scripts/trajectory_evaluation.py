import rosbag
import numpy as np
from nav_msgs.msg import Odometry
import argparse
import time
import matplotlib.pyplot as plt
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
import evo.common_ape_rpe as common
import xml.etree.ElementTree as ET
import sys
import tf.transformations
import os
from dtaidistance import dtw_ndim

EMERGENCY_STOP=0
REMOTE_CONTROL=1
ROS_CONTROL=2

def extract_trajectory(file_path, odom_topic, status_topic=None, min_distance=0.1):
    """Extract trajectory from rosbag or txt file filtering by minimum speed"""
    timestamps = []
    poses = {"position": [], "quaternion": []}
    emergency_count = 0
    local_minimum_count = 0
    previous_mode=-1
    start_index=None
    end_index=None
    
    if file_path.endswith('.txt'):
        with open(file_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                parts = line.strip().split(',')
                if len(parts) < 8:
                    continue
                t = float(parts[0])
                position = [float(parts[1]), float(parts[2]), float(parts[3])]
                rotation = [float(parts[4]), float(parts[5]), float(parts[6])]
                quaternion = tf.transformations.quaternion_from_euler(rotation[0], rotation[1], rotation[2])
                status = int(parts[7])
                if previous_mode==ROS_CONTROL and status == EMERGENCY_STOP:
                    emergency_count += 1
                elif previous_mode==ROS_CONTROL and status == REMOTE_CONTROL:
                    local_minimum_count += 1
                previous_mode = status
                timestamps.append(t)
                poses["position"].append(position)
                poses["quaternion"].append(quaternion)
                if len(poses["position"]) > 1:
                    distance = np.linalg.norm(np.array(poses["position"][-1]) - np.array(poses["position"][-2]))
                    if distance > min_distance:
                        if start_index is None:
                            start_index = len(poses["position"]) - 2
                        end_index = len(poses["position"]) - 1
    else:
        with rosbag.Bag(file_path) as bag:
            # First pass to get start and end times when robot is moving
            for topic, msg, t in bag.read_messages(topics=[odom_topic, status_topic]):
                if topic == status_topic:
                    if previous_mode==ROS_CONTROL and msg.data[0] == EMERGENCY_STOP:
                        emergency_count += 1
                    elif previous_mode==ROS_CONTROL and msg.data[0] == REMOTE_CONTROL:
                        local_minimum_count += 1
                    previous_mode = msg.data[0]
                else:
                    timestamps.append(t.to_sec())
                    position=[ msg.pose.pose.position.x,
                        msg.pose.pose.position.y,
                        msg.pose.pose.position.z]
                    quaternion=[msg.pose.pose.orientation.w, msg.pose.pose.orientation.x,
                                msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
                    poses["position"].append(position)
                    poses["quaternion"].append(quaternion)
                    if len(poses["position"]) > 1:
                        distance = np.linalg.norm(np.array(poses["position"][-1]) - np.array(poses["position"][-2]))
                        if distance > min_distance:
                            if start_index is None:
                                start_index = len(poses["position"]) - 2
                            end_index = len(poses["position"]) - 1

               
    if start_index is None or end_index is None:
        print("No valid trajectory found in" + file_path)
        exit()

    for key in poses:
        poses[key] = np.array(poses[key][start_index:end_index+1])      
    timestamps = np.array(timestamps[start_index:end_index+1])
    timestamps = (timestamps - timestamps[0]) 
    trajectory_time = timestamps[-1]
    return timestamps, poses, trajectory_time, emergency_count, local_minimum_count 

#Match the two trajectories using nearest neighbour heuristic.
#It assumes that the nearest neightbour index from one pose is in the range [i-window_size/2, i+window_size/2]
#where i is the index of the previous matched pose in the other trajectory.
def heuristic_nearest_neightbour_matcher(gt_traj:PoseTrajectory3D, test_traj:PoseTrajectory3D, window_size=20):
    gt_index = 0
    gt_positions = gt_traj.positions_xyz
    test_positions= test_traj.positions_xyz
    gt_indices, test_indices = [], []
    for test_index,test_position in enumerate(test_positions):
        min_distance = float("inf")
        min_gt_index = -1
        for gt_index in range(max(0, gt_index - window_size // 2), min(len(gt_positions), gt_index + window_size // 2)):
            gt_position = gt_positions[gt_index]
            distance = np.linalg.norm(test_position - gt_position)
            if distance < min_distance:
                min_distance = distance
                min_gt_index = gt_index
        if min_gt_index != -1:
            gt_indices.append(min_gt_index)
            gt_index = min_gt_index

    gt_traj.reduce_to_ids(gt_indices)
    return gt_traj, test_traj

def ReadRouteFromXML(filename, id_way=None):
    tree= ET.parse(filename)
    root = tree.getroot()
    map={}
    map["nodes"], map["way"]={},[]
    for node in root.findall("node"):

        map["nodes"][node.get("id")] = [float(node.get("pose_x")), float(node.get("pose_y"))]
    
    route=root.find("way")
    for route in root.iterfind("way"):
        if id_way is None or id_way==int(route.get("id")):
            for child in route:
                map["way"].append(child.get("ref"))
            return map
    raise Exception from None

def main():
    parser = argparse.ArgumentParser(description='Compare trajectories from rosbags')
    parser.add_argument('--gt_file', type=str, required=True, help='Ground truth rosbag or txt path')
    parser.add_argument('--test_file', type=str, required=True, help='Test rosbag ot txt path')
    parser.add_argument('--odom_topic', type=str, default='/localization', help='Odometry topic name')
    parser.add_argument('--status_topic', type=str, default='/CLEAR_status', help='Status topic name to read when operator takes the control.')
    parser.add_argument('--output_name', type=str, default='results/test_route2_dtw', help='Name of the output file')
    args = parser.parse_args()

    # Extract trajectories
    gt_times, gt_poses, gt_duration, _, _ = extract_trajectory(args.gt_file, args.odom_topic)
    test_times, test_poses, test_duration, emergency_count, local_minimum_count = extract_trajectory(
                    args.test_file, args.odom_topic, args.status_topic)

    print(f"Ground truth trajectory duration: {gt_duration:.2f} seconds")
    print(f"Test trajectory duration: {test_duration:.2f} seconds")
    print(f"Time factor: {(gt_duration/test_duration):.2f}")
    print(f"Emergency stops: {emergency_count}")
    print(f"Local minimums: {local_minimum_count-1}") #The same method is used to stop the robot at the end of the route.

    gt_traj = PoseTrajectory3D(positions_xyz=gt_poses["position"], orientations_quat_wxyz=gt_poses["quaternion"], timestamps=gt_times)
    test_traj = PoseTrajectory3D(positions_xyz=test_poses["position"], orientations_quat_wxyz=test_poses["quaternion"], timestamps=test_times)
    # Align trajectories
    # gt_traj, test_traj = heuristic_nearest_neightbour_matcher(gt_traj, test_traj, window_size=50)

    # ape_metric = metrics.APE(metrics.PoseRelation.translation_part)
    # ape_metric.process_data((gt_traj, test_traj))
    # print(f"Absolute Pose Error (APE): {ape_metric.error.mean()}")
    # distance = ape_metric.error.mean()
    dtw_distance = dtw_ndim.distance_fast(gt_traj.positions_xyz.astype(np.double), test_traj.positions_xyz.astype(np.double))
    distance = dtw_distance
    print(f"Dynamic Time Warping (DTW) distance: {dtw_distance:.2f}")
    with open(args.output_name+".csv","a") as f:
        name = args.test_file.split("/")[-1].split(".")[0]
        exp=name.replace("_compressed","").split("_")[-1]
        f.write("{},{},{},{},{}\n".format(exp, distance, gt_duration/test_duration, emergency_count, local_minimum_count-1))

if __name__ == "__main__":
    main()