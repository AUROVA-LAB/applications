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

import carla

from carla import ColorConverter as cc

import argparse
import math
from utils import *
from config import *
from fnmatch import fnmatch
import json

import rospy

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
        self.lidar_sensor = None
        for actor in self.world.get_actors():
            if actor.parent is not None and actor.parent.id == self.player.id:
                if fnmatch(actor.type_id,'*gnss'):
                    self.gnss_sensor = GnssSensorLink(actor)
                elif fnmatch(actor.type_id,'*lidar.ray_cast'):
                    self.lidar_sensor = LidarSensorLink(actor)
        if self.gnss_sensor is None or self.lidar_sensor is None:
            print("Error: Sensors not found.")
            sys.exit(1)
        self.rgb_camera=RGBcamera(self.player)
        self.depth_camera=Depthcamera(self.player)
        self.sematic_camera=SemanticCamera(self.player)

        #Subscribe to the topic of desired ackermann state.
        self._ackermann_control = carla.VehicleAckermannControl()
        self.control_subscriber = self.new_subscription(
            AckermannDrive,
            "/carla/" + args.name + "/ackermann_cmd",
            self.ackermann_command_updated,
            qos_profile=10
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
        os.mkdir(self.record_path)
        for folder in ["rgb_images","depth_images","semantic_images","lidar", "pedestrians"]:
            os.mkdir(self.record_path+folder)

        filename=self.record_path+"measurements.csv"
        file = open(filename,"w")
        file.write("target_pos_x,target_pos_y,speed,steering_angle,robot_pose_x,robot_pose_y,robot_pose_yaw,latitude,longitude,accelerometer_x,accelerometer_y,accelerometer_z")
        file.close()

        filename=self.record_path+"commands.csv"
        file = open(filename,"w")
        file.write("desired_speed,desired_steering_angle")
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
        #Record images
        filename=self.record_path+"rgb_images/"+str(self.record_id)+".png"
        image=self.rgb_camera._image[:, :, [2, 1, 0]].swapaxes(0, 1)
        cv2.imwrite(filename,image) 
        filename=self.record_path+"depth_images/"+str(self.record_id)+".npy"
        np.save(filename,self.depth_camera._raw) 
        filename=self.record_path+"semantic_images/"+str(self.record_id)+".png"
        image=self.sematic_camera._image[:, :, 0].swapaxes(0, 1)
        cv2.imwrite(filename,image) 

        #Record LIDAR
        filename=self.record_path+"lidar/"+str(self.record_id)+".npy"
        np.save(filename, self.lidar_sensor._pointcloud)

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
        if math.sqrt(goal_pose.x**2+goal_pose.y**2) < 2.5:
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

        #Record pedestrians position/bounding box
        filename=self.record_path+"pedestrians/"+str(self.record_id)+".csv"
        file=open(filename,"w")
        file.write("pose_x,pose_y,pose_z,ori_x,ori_y,ori_z,box_x,box_y,speed_x,speed_y,speed_z")
        walkers = self.world.get_actors().filter("walker.*")
        for actor in walkers:
            loc = actor.get_location()
            if loc.distance(self.player.get_location()) > 10:
                continue
            _id = actor.id
            data = {}
            data["loc"] = [loc.x, loc.y, loc.z]
            ori = actor.get_transform().rotation.get_forward_vector()
            data["ori"] = [ori.x, ori.y, ori.z]
            box = actor.bounding_box.extent
            data["box"] = [box.x, box.y]
            speed = actor.get_velocity()
            data["speed"] = [speed.x, speed.y, speed.z]
            file.write("\n{},{},{},{},{},{},{},{},{},{},{}".format(*data["loc"],*data["ori"],*data["box"],*data["speed"]))
        file.close()
    
    def ackermann_command_updated(self, ros_ackermann_drive):
        self.last_ackermann_msg_received_sec = self.get_time()
        # set target values (dataset in degrees)
        self._ackermann_control.steer=ros_ackermann_drive.steering_angle*RAD2DEG
        self._ackermann_control.speed=ros_ackermann_drive.speed

    def run(self):
        """
        Control loop

        :return:
        """

        def loop(timer_event=None):
            self.record()

        #Wait for spawning the sensors.
        while not(all(self.rgb_camera.init) and all(self.depth_camera.init) and all(self.sematic_camera.init)):
            time.sleep(1.0)
        self.new_timer(0.1, loop)
        self.spin()

    def destroy(self):
        self.rgb_camera.destroy()
        self.depth_camera.destroy()
        self.sematic_camera.destroy()


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
        '--name',
        metavar='NAME',
        default='base_link',
        help='Name of vehicle for reading the topic (default "base_link")')

    args = argparser.parse_args()
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
        recorder.destroy()


if __name__ == '__main__':

    main()
