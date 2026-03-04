#!/usr/bin/env python

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass


# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode

from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import PointCloud2

import carla

from carla import ColorConverter as cc

import argparse
import math
from utils_carla import *
from config import *
from fnmatch import fnmatch
import json
import cv2
from copy import copy
import configGS
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2

RAD2DEG = 180.0/math.pi

class Recorder(CompatibleNode):
    def __init__(self, carla_world : carla.World, args):
        super(Recorder, self).__init__("recorder_ros_data")

        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        try:
            self.player = self.world.get_actors().filter(args.vehicle)[0]
        except:
            print("Error: Vehicle {} not found.".format(args.vehicle))
            sys.exit(1)
        #Get the sensors and create the needed ones.
        self.gnss_sensor = None
        for actor in self.world.get_actors():
            if actor.parent is not None and actor.parent.id == self.player.id:
                if fnmatch(actor.type_id,'*gnss'):
                    self.gnss_sensor = GnssSensorLink(actor)
        if self.gnss_sensor is None:
            print("Error: Sensors not found.")
            sys.exit(1)
        self.lidar_image = None
        self.channels= args.channels
        self.args = args

        #Subscribe to the topic of desired ackermann state.
        self._ackermann_control = carla.VehicleAckermannControl()
        self.control_subscriber = self.new_subscription(
            AckermannDrive,
            "/carla/" + args.name + "/ackermann_cmd",
            self.ackermann_command_updated,
            qos_profile=10
        )
        self.lidar_subscriber = self.new_subscription(
            PointCloud2,
            "/carla/base_link/lidar",
            self.lidar_updated,
            qos_profile=1
        )
        try:
            self.route,_ = ReadRouteFromXML(ROUTE_FILENAME,WAY_ID)
            self.way_id=0
        except:
            print("Error: Route filename {} or route id {} not found".format(ROUTE_FILENAME,WAY_ID))
            sys.exit(1)

        self.init_record()


    def get_simulation_time(self):
        return self.world.get_snapshot().timestamp.elapsed_seconds
    
    def init_record(self):
        session=1
        while os.path.exists(PATH_DATASET+str(session)):
            session+=1
        self.record_id=0
        self.record_path=PATH_DATASET+str(session)+"/"
        os.makedirs(self.record_path)
        os.mkdir(self.record_path+"lidar")

        filename=self.record_path+"measurements.csv"
        file = open(filename,"w")
        file.write("target_pos_x,target_pos_y,speed,steering_angle,robot_pose_x,robot_pose_y,robot_pose_yaw,latitude,longitude,accelerometer_x,accelerometer_y,accelerometer_z")
        file.close()

        filename=self.record_path+"commands.csv"
        file = open(filename,"w")
        file.write("desired_speed,desired_steering_angle")
        file.close()

        filename=self.record_path+"transformations.csv"
        file = open(filename,"w")
        file.write("Rxx,Rxy,Rxz,Ryx,Ryy,Ryz,Rzx,Rzy,Rzz,Tx,Ty,Tz")
        file.close()

        #Get next target position
        robot_pose=self.player.get_location()
        closest_id, closest_distance = 0, None
        for id, node in enumerate(self.route["way"]):
            distance = math.sqrt((robot_pose.x-self.route["nodes"][node][0])**2 + (robot_pose.y-self.route["nodes"][node][1])**2)
            if closest_distance is None or distance<closest_distance:
                closest_distance=distance
                closest_id=id
        self.way_id=closest_id #The target will be next position in the route


    def record(self):
        
        if self.way_id>=len(self.route["way"]):
            print(("End of the route. Finishing recording."))
            #I tried to use other methods to work with roscomp, but I couldn't shutdown the node even with sys.exit() or using exceptions.
            #In fact, without using roscomp it could be done using spin once. One option would be to add this function in roscomp.
            rospy.signal_shutdown("") 
        self.record_id+=1

        #Record image
        transform = np.array(self.player.get_transform().get_matrix())
        # Consider the offset
        transform[0,3]-=self.args.offset_GS[0]
        transform[1,3]-=self.args.offset_GS[1]
        yaw = self.player.get_transform().rotation.yaw
        rotation = np.array([[math.cos(yaw*DEG2RAD), -math.sin(yaw*DEG2RAD), 0],
                                [math.sin(yaw*DEG2RAD), math.cos(yaw*DEG2RAD), 0],
                                [0, 0, 1]])
        transform[:3,:3] = rotation
        # Apply transform from base to camera
        transform = BASE2CAMERA@transform
        # Unreal Engine coordinate system has  the Y axis inverted.
        aux = copy(transform[:,1])
        transform[:,1]=transform[:,2]
        transform[:,2] = -aux
        transform[1:3]*=-1
        transform[2,3]*=-1
            
        R = VIEW_ANGLE@transform[:3,:3]
        T=transform[:3,3]
        # Generate the images afterward with the script:
        filename=self.record_path+"transformations.csv"
        file = open(filename,"a")
        file.write("\n{},{},{},{},{},{},{},{},{},{},{},{}".format(R[0,0],R[0,1],R[0,2],
                     R[1,0],R[1,1],R[1,2],R[2,0],R[2,1],R[2,2],T[0],T[1],T[2]))
        file.close()

        #Record LIDAR
        filename=self.record_path+"lidar/"+str(self.record_id)+".png"
        cv2.imwrite(filename, self.lidar_image)

        #Record measurements
        filename=self.record_path+"measurements.csv"
        file = open(filename,"a")
        #Format of measurements:
        # Target position (x,y) respect the robot position, Ackermann state (speed, steer), global robot position (x,y,theta), gnss (lat,lon), accelerometer (x,y,z)
        robot_pose=self.player.get_location()
        yaw=self.player.get_transform().rotation.yaw
        goal_pose = carla.Location()
        node=self.route["way"][self.way_id]
        goal_pose.x, goal_pose.y=self.route["nodes"][node][0], self.route["nodes"][node][1]
        goal_pose.x, goal_pose.y=goal_pose.x-robot_pose.x, goal_pose.y-robot_pose.y
        #Check if the robot has arrived to next point
        if math.sqrt(goal_pose.x**2+goal_pose.y**2) < 1.75:
            print("Node", self.way_id)
            self.way_id+=1
        
        speed = self.player.get_velocity()
        speed = math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
        angle_right = self.player.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel) 
        angle_left = self.player.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        if angle_right>0:
            steer_angle=angle_right
        else:
            steer_angle=angle_left
        accel=self.player.get_acceleration()

        self.goal_polar = [math.sqrt(goal_pose.x**2+goal_pose.y**2),yaw-math.atan2(goal_pose.y,goal_pose.x)*180.0/math.pi]
        if self.goal_polar[1]>180: self.goal_polar[1]-=360
        elif self.goal_polar[1]<-180: self.goal_polar[1]+=360

        file.write("\n{},{},{},{},{},{},{},{},{},{},{},{}".format(goal_pose.x, goal_pose.y,
                   speed, steer_angle, robot_pose.x, robot_pose.y, yaw,
                   self.gnss_sensor.lat, self.gnss_sensor.lon,
                   accel.x,accel.y,accel.z))
        file.close()
        #Record command output - the Ackermann desired state (speed, steer)
        filename=self.record_path+"commands.csv"
        file = open(filename,"a")
        file.write("\n{},{}".format(self._ackermann_control.speed,-self._ackermann_control.steer))
    
    def ackermann_command_updated(self, ros_ackermann_drive):
        self.last_ackermann_msg_received_sec = self.get_time()
        # set target values (dataset in degrees)
        self._ackermann_control.steer=ros_ackermann_drive.steering_angle*RAD2DEG
        self._ackermann_control.speed=ros_ackermann_drive.speed

    def lidar_updated(self, msg):
        # Convert PointCloud2 message to numpy array
        points = [[] for i in range(self.channels)]
        for point in pc2.read_points(msg, field_names=("x", "y", "z", "intensity", "ring"), skip_nans=True):
            points[point[4]].append([point[0], -point[1], point[2]])
        points = [np.array(points[i], dtype=np.float32) for i in range(self.channels)]
        range_image= np.zeros((self.channels,2048),dtype=np.uint16)
        max_range = 50.0  # Set your max range
        # Project or process as needed, here just create a dummy image for example
        # (You should replace this with your actual projection/visualization logic)
        for channel in range(self.channels):
            if len(points[channel]) == 0:
                continue
            #Calculate the distances.
            ranges = np.linalg.norm(points[channel], axis=1)
            ranges = np.clip(ranges, 0, max_range)
            # The close objects are white, the far objects are black
            ranges = max_range - ranges
            ranges = (ranges / max_range * 65535).astype(np.uint16)
            azimuth = np.arctan2(points[channel][:,1], points[channel][:,0])
            pixels = ((azimuth + np.pi) / (2.0 * np.pi) * 2047).astype(np.int32)
            for offset in range(-2, 3):
                valid_pixels = (pixels + offset) % 2048
                range_image[channel, valid_pixels] = ranges
        self.lidar_image = range_image

    def run(self):
        """
        Control loop

        :return:
        """

        def loop(timer_event=None):
            self.record()

        #Wait for spawning the sensors.
        while self.lidar_image is None:
            time.sleep(1.0)
        self.new_timer(0.1, loop)
        self.spin()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--vehicle',
        metavar='NAME',
        default='*blue*',
        help='wildcard_pattern of the actor(default: "*blue*")')
    argparser.add_argument(
        '-t', '--timeout',
        metavar='P',
        default=500,
        help='timeout waiting response from neural network server')
    argparser.add_argument(
        '--channels',
        metavar='P',
        default=128,
        help='Number LiDAR channels (default: 128)')
    argparser.add_argument(
        '--name',
        metavar='NAME',
        default='base_link',
        help='Name of vehicle for reading the topic (default "base_link")')
    argparser.add_argument(
        '--scene',
        metavar='H',
        default='scene1',
        help='Gaussian splatting scene (scenes defined in configGS.py) (default: scene1)')


    args = argparser.parse_args()
    configGS.get_args(args, args.scene, "gof") #The model doen't matter here, only the GS offset is used.
    try:
        roscomp.init("recorder_ros_data", args=args)
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        sim_world.wait_for_tick()
        recorder = Recorder(sim_world, args)
        recorder.run()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

    finally:
        roscomp.shutdown()


if __name__ == '__main__':

    main()
