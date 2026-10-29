#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time

# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
from ackermann_msgs.msg import AckermannDrive
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from sensor_msgs.msg import Image
import cv2

import argparse
import math
from utils_carla import *
from config import *
from fnmatch import fnmatch
import json
from cv_bridge import CvBridge
from copy import copy
import rospy
import numpy as np
import tf
from pathlib import Path

RAD2DEG = 180.0/math.pi

class Recorder():
    def __init__(self, args):
        self.rosbag = args.rosbag
        self.output_path = args.output_path
        self.start_time = args.start_time
        self.end_time = args.end_time
        self.bridge= CvBridge()
        self.init_record()
    
    def init_record(self):
        session=1
        while os.path.exists(self.output_path+str(session)):
            session+=1
        self.record_id=1
        self.record_path=self.output_path+str(session)+"/"
        os.makedirs(self.record_path)
        os.mkdir(self.record_path+"lidar")
        os.mkdir(self.record_path+"images")

        filename=self.record_path+"measurements.csv"
        file = open(filename,"w")
        file.write("target_pos_x,target_pos_y,speed,steering_angle,robot_pose_x,robot_pose_y,robot_pose_yaw")
        file.close()

        filename=self.record_path+"commands.csv"
        file = open(filename,"w")
        file.write("desired_speed,desired_steering_angle")
        file.close()

    def read_rosbag(self):
        typestore = get_typestore(Stores.ROS2_JAZZY)

        with AnyReader([Path(self.rosbag)], default_typestore=typestore) as bag:
            flags = [False, False, False, False, False, False]
            initial_timestamp = None
            for conn, timestamp, rawdata in bag.messages():
                if initial_timestamp is None:
                    initial_timestamp = timestamp/10**9
                    continue
                t = timestamp/10**9
                if t < self.start_time + initial_timestamp: continue
                if t > self.end_time + initial_timestamp: break
                if conn.topic == "/camera/color/image_raw":
                    flags[0] = True
                    image = typestore.deserialize_cdr(rawdata, conn.msgtype)
                elif conn.topic == "/ouster/range_image":
                    flags[1] = True
                    lidar = typestore.deserialize_cdr(rawdata, conn.msgtype)
                elif conn.topic == "/desired_ackermann_state":
                    flags[2] = True
                    desired_state = typestore.deserialize_cdr(rawdata, conn.msgtype)
                elif conn.topic == "/estimated_ackermann_state":
                    flags[3] = True
                    state = typestore.deserialize_cdr(rawdata, conn.msgtype)
                elif conn.topic == "/localization":
                    flags[4] = True
                    robot_pose = typestore.deserialize_cdr(rawdata, conn.msgtype).pose.pose
                elif conn.topic == "/semilocal_goal":
                    flags[5] = True
                    goal_pose = typestore.deserialize_cdr(rawdata, conn.msgtype).pose.pose.position

                if all(flags):
                    flags = [False, False, False, False, False, False]
                    # Process the data
                    image = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
                    cv2.imwrite(self.record_path+"images/"+str(self.record_id)+".png", image)
                    ranges = self.bridge.imgmsg_to_cv2(lidar, desired_encoding='mono16')
                    max_range = 50.0
                    np.place(ranges, ranges == 0, 65535) #Replace no detections with maximum value
                    ranges = ranges * 261.0 / pow(2,16) #Convert to meters
                    ranges = np.clip(ranges, 0, max_range)
                    ranges = max_range - ranges
                    ranges = (ranges / max_range * 65535).astype(np.uint16)
                    cv2.imwrite(self.record_path+"lidar/"+str(self.record_id)+".png", ranges)

                    filename=self.record_path+"measurements.csv"
                    file = open(filename,"a")
                    #Format of measurements:
                    # Target position (x,y) respect the robot position, Ackermann state (speed, steer), global robot position (x,y,theta)
                    # Get yaw from quaternion using tf
                    orientation = robot_pose.orientation
                    quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
                    _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
                    yaw = yaw * RAD2DEG
                    goal_pose.x, goal_pose.y=goal_pose.x-robot_pose.position.x, goal_pose.y-robot_pose.position.y
                    file.write("\n{},{},{},{},{},{},{}".format(goal_pose.x, goal_pose.y, abs(state.speed), state.steering_angle,
                            robot_pose.position.x, robot_pose.position.y, yaw))
                    file.close()

                    filename=self.record_path+"commands.csv"
                    file = open(filename,"a")
                    file.write("\n{},{}".format(desired_state.speed,desired_state.steering_angle))
                    file.close()
                    self.record_id+=1



# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '--rosbag',
        metavar='FILE',
        required=True,
        help='Path to the rosbag file to read')
    argparser.add_argument(
        '--output_path',
        metavar='DIR',
        required=True,
        help='Path to the output directory where the data will be saved')
    argparser.add_argument(
        '-s', '--start_time',
        metavar='P',
        default=0,
        type=float,
        help='Start time in seconds for the recording (default: 0)')
    argparser.add_argument(
        '-e', '--end_time',
        metavar='P',
        default=10000,
        type=float,
        help='End time in seconds for the recording (default: 10000)')


    args = argparser.parse_args()
    recorder = Recorder(args)
    recorder.read_rosbag()

if __name__ == '__main__':

    main()
