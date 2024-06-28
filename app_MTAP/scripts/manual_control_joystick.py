#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle with a gamepad. 
"""
Welcome to CARLA manual control.

Use a joystick/gamepad. You can configure the buttons in manual:control_joystick.py file.
Default countrols for PS4 gamepad

    Left axis    : Speed control (Up and down)
    Right axis   : Steering control (Left and right)
    Square       : Change view
    Triangle     : Change weather
    Circle       : Change vehicle
    Cross        : toggle autopilot
    R1/L1        : Change sensor
    L2           : Change 360 image (RGB or depth)
    R2           : Change visualization camera/LiDAR
    Start        : quit

"""

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys

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


import carla

from carla import ColorConverter as cc

import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
from utils import *
from config import *
import screeninfo

try:
    import pygame
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
try:
    import cv2
except ImportError:
    raise RuntimeError('cannot import opencv, make sure cv2 package is installed')


#Joystick key mapping
SPEED_AXIS = 1
STEERING_AXIS = 3
CHANGE_POSITION_BUTTON = 3
NEXT_SENSOR_BUTTON = 5
PREVIOUS_SENSOR_BUTTON = 4
CHANGE_WEATHER_BUTTON = 2
CHANGE_VEHICLE_BUTTON = 1
TOGGLE_AUTOPILOT_BUTTON = 0
TOGGLE_360IMAGE_BUTTON = 6
TOGGLE_LIDAR_BUTTON = 7
EXIT_BUTTON = 9
START_RECORD_BUTTON = 8

NEAR_COLLISION_DISTANCE = 1.25 # meters

WITHOUT_SENSORS = False  #Mode for fast simulation for exploring maps, it is also needed to increase FPS constant.

# ==============================================================================
# -- World ---------------------------------------------------------------------
# ==============================================================================


class World(object):
    def __init__(self, carla_world, hud, args):
        self.world = carla_world
        self.sync = args.sync
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        try:
            self.route, weather = ReadRouteFromXML(ROUTE_FILENAME,WAY_ID)
            self.way_id=0
            self.set_weather(weather)
        except:
            print("Error: Route filename {} or route id {} not found".format(ROUTE_FILENAME,WAY_ID))
            sys.exit(1)
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_id = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.show_depth_camera=False
        self.show_lidar=False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]
        

    def restart(self):
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = self.camera_manager.index if self.camera_manager is not None else 0
        cam_pos_index = self.camera_manager.transform_index if self.camera_manager is not None else 0
        # Get a random blueprint.
        blueprint = random.choice(get_actor_blueprints(self.world, self._actor_generation, self._actor_filter))
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('terramechanics'):
            blueprint.set_attribute('terramechanics', 'true')
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        else:
            node=self.route["way"][0]
            spawn_point=carla.Transform(carla.Location(x=self.route["nodes"][node][0], 
                                                       y=self.route["nodes"][node][1],
                                                       z=self.route["nodes"][node][2]+1),
                                                       carla.Rotation(yaw=self.route["nodes"][node][3],pitch=360))
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_points = self.map.get_spawn_points()
            spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        # Set up the sensors.
        # self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player)
        self.imu_sensor = IMUSensor(self.player)
        self.camera_manager = CameraManager(self.player, self.hud, self._gamma)
        self.camera_manager.transform_index = cam_pos_index
        self.camera_manager.set_sensor(cam_index, notify=False)
        if not WITHOUT_SENSORS:
            self.rgb_camera=RGBcamera(self.player, self.hud)
            self.depth_camera=Depthcamera(self.player, self.hud)
            self.sematic_camera=SemanticCamera(self.player)
            self.lidar_sensor=LidarSensor(self.player,self.hud)
        actor_type = get_actor_display_name(self.player)
        self.goal_polar=None
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

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
        self.hud.notification("Starting recording. Target node "+self.route["way"][self.way_id])

    def set_weather(self, id):
        if id < 0 or id >= len(self._weather_presets):
            print("Invalid weather id.")
            return
        self._weather_index = id
        preset = self._weather_presets[self._weather_index]
        self.world.set_weather(preset[0])
        
    def record(self):
        if self.way_id>=len(self.route["way"]):
            self.hud.notification("End of the route. Finishing recording.")
            self.recording_enabled=False
            return
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
            self.hud.notification("Robot arrived to node "+node)
            self.way_id+=1
        
        speed = self.player.get_velocity()
        speed = math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
        angle_right = self.player.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel) 
        angle_left = self.player.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        if angle_right>0:
            steer_angle=angle_right
        else:
            steer_angle=angle_left
        accel=self.imu_sensor.accelerometer

        self.goal_polar = [math.sqrt(goal_pose.x**2+goal_pose.y**2),yaw-math.atan2(goal_pose.y,goal_pose.x)*180.0/math.pi]
        if self.goal_polar[1]>180: self.goal_polar[1]-=360
        elif self.goal_polar[1]<-180: self.goal_polar[1]+=360

        file.write("\n{},{},{},{},{},{},{},{},{},{},{},{}".format(goal_pose.x, goal_pose.y,
                   speed, steer_angle, robot_pose.x, robot_pose.y, yaw,
                   self.gnss_sensor.lat, self.gnss_sensor.lon, *accel))
        file.close()
        #Record command output - the Ackermann desired state (speed, steer)
        filename=self.record_path+"commands.csv"
        file = open(filename,"a")
        desired_speed, desired_steer=self.hud._ackermann_control.speed, self.hud._ackermann_control.steer
        file.write("\n{},{}".format(desired_speed,desired_steer))

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


    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        if self.radar_sensor is None:
            self.radar_sensor = RadarSensor(self.player)
        elif self.radar_sensor.sensor is not None:
            self.radar_sensor.sensor.destroy()
            self.radar_sensor = None

    def toggle_360_camera(self):
        self.show_depth_camera=False if self.show_depth_camera else True
    
    def toggle_lidar(self):
        self.show_lidar=False if self.show_lidar else True

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)

    def render(self, display):
        if self.show_lidar:
            self.lidar_sensor.render(display)
        else:
            self.camera_manager.render(display)
        if not WITHOUT_SENSORS:
            if self.show_depth_camera:
                self.depth_camera.render(display)
            else:
                self.rgb_camera.render(display)
        self.hud.render(display)

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            # self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        
        if not WITHOUT_SENSORS:
            self.rgb_camera.destroy()
            self.depth_camera.destroy()
            self.sematic_camera.destroy()
            sensors.append(self.lidar_sensor.sensor)
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        
        if self.player is not None:
            self.player.destroy()


# ==============================================================================
# -- JoystickControl -----------------------------------------------------------
# ==============================================================================


class JoystickControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, start_in_autopilot, hud):
        self._autopilot_enabled = start_in_autopilot
        self._ackermann_reverse = 1
        self.node=0
        self.hud=hud
        self.previous_time=0.0
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._ackermann_control = carla.VehicleAckermannControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_autopilot(self._autopilot_enabled)
            world.player.set_light_state(self._lights)
            self.pid_controller=PID_Controller(KP,KI,KD,0.0,1.0)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._autopilot_enabled = False
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self.past_steering = 0.0
        pygame.joystick.init()
        if(pygame.joystick.get_count()>0):
            self.joystick=pygame.joystick.Joystick(0)
        else:
            print("The joystick is not detected.")
            return None
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)
        # self.collision_sensor=world.collision_sensor

    def check_near_collision(self, world : World):
        for col in range(800,1200):
            for row in range(32,128):
                if world.depth_camera._raw[col,row]*1000.0<NEAR_COLLISION_DISTANCE:
                    self.joystick.rumble(0.0, 0.5, 500)
                    return

    def parse_events(self, client, world:World, clock, sync_mode):
        if isinstance(self._control, carla.VehicleControl):
            current_lights = self._lights
        self.check_near_collision(world)
        # if self.collision_sensor.collision_rumble:
        #     self.joystick.rumble(0.7, 0.0, 500)
        #     self.collision_sensor.collision_rumble=False
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                self.joystick.quit()
                self.joystick.init()
            elif event.type == pygame.QUIT:
                return True
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == EXIT_BUTTON:
                    return True
                if event.button == CHANGE_VEHICLE_BUTTON:
                    if self._autopilot_enabled:
                        world.player.set_autopilot(False)
                        world.restart()
                        world.player.set_autopilot(True)
                    else:
                        world.restart()
                elif event.button == CHANGE_POSITION_BUTTON:
                    world.camera_manager.toggle_camera()
                elif event.button == CHANGE_WEATHER_BUTTON:
                    world.next_weather()
                elif event.button == NEXT_SENSOR_BUTTON:
                    world.camera_manager.next_sensor()
                elif event.button == PREVIOUS_SENSOR_BUTTON:
                    world.camera_manager.previous_sensor()
                elif event.button == TOGGLE_360IMAGE_BUTTON:
                    world.toggle_360_camera()
                elif event.button == TOGGLE_LIDAR_BUTTON:
                    world.toggle_lidar()
                elif event.button == TOGGLE_AUTOPILOT_BUTTON:
                    if not self._autopilot_enabled and not sync_mode:
                        print("WARNING: You are currently in asynchronous mode and could "
                                "experience some issues with the traffic simulation")
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.player.set_autopilot(self._autopilot_enabled)
                    world.hud.notification(
                        'Autopilot %s' % ('On' if self._autopilot_enabled else 'Off'))
                elif event.button == START_RECORD_BUTTON:
                    if world.recording_enabled:
                        world.recording_enabled=False
                        world.goal_polar=None
                        self.hud.notification("Record finished.")
                    else:
                        world.init_record()
                        world.recording_enabled=True
                        

        if not self._autopilot_enabled:
            if isinstance(self._control, carla.VehicleControl):
                self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time(), world)
                self.get_speed_action(world)
                self._control.reverse = self._control.gear < 0
                # Set automatic control-related vehicle lights
                if self._control.brake:
                    current_lights |= carla.VehicleLightState.Brake
                else: # Remove the Brake flag
                    current_lights &= ~carla.VehicleLightState.Brake
                if self._control.reverse:
                    current_lights |= carla.VehicleLightState.Reverse
                else: # Remove the Reverse flag
                    current_lights &= ~carla.VehicleLightState.Reverse
                if current_lights != self._lights: # Change the light state only if necessary
                    self._lights = current_lights
                    world.player.set_light_state(carla.VehicleLightState(self._lights))

                # Apply control
                world.player.apply_control(self._control)
                # Update hud and world with the newest ackermann control
                world.hud.update_ackermann_control(self._ackermann_control)
                # print("----------")
                # print(self._ackermann_control.speed, self._ackermann_control.steer)
                # speed = world.player.get_velocity()
                # speed = math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
                # angle_right = world.player.get_wheel_steer_angle(carla.VehicleWheelLocation.FR_Wheel) 
                # angle_left = world.player.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
                # if angle_right>0:
                #     steer_angle=angle_right
                # else:
                #     steer_angle=angle_left
                # print(speed,steer_angle) 



    def _parse_vehicle_keys(self, keys, milliseconds, world):
        speed_axis = -self.joystick.get_axis(SPEED_AXIS)
        speed_axis_abs=abs(speed_axis)
        steering_axis = self.joystick.get_axis(STEERING_AXIS)
        if speed_axis_abs>0.1:
            self._ackermann_control.speed = speed_axis*speed_axis_abs*MAX_SPEED
        else:
            self._ackermann_control.speed = 0

        if abs(steering_axis)>0.1:
            target_steer=steering_axis**3
            self._ackermann_control.steer = target_steer * MAX_STEERING_ANGLE
            
        else:
            self._ackermann_control.steer = 0.0
            target_steer=0.0

        # Steering regulation: changes cannot happen abruptly, can't steer too much.
        if target_steer > self.past_steering + 0.2:
            target_steer = self.past_steering + 0.2
        elif target_steer < self.past_steering - 0.2:
            target_steer = self.past_steering - 0.2
        
        self._control.steer=target_steer
        self.past_steering=target_steer
        

    def get_speed_action(self,world):
        speed = world.player.get_velocity()
        speed = math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
        diference=self._ackermann_control.speed-speed
        if isinstance(self._control, carla.VehicleControl):
            if self._ackermann_control.speed *self._control.gear>0 and diference*self._control.gear>-0.7:
                dt = world.hud.simulation_time - self.previous_time
                self._control.throttle = self.pid_controller.step(self._ackermann_control.speed*self._control.gear,speed, dt)
                self._control.brake = 0
            else:
                self._control.throttle = 0
                if  speed <= 0.1 and abs(self._ackermann_control.speed)>0.0:
                    self._control.gear = 1 if self._control.reverse else -1
                    self._control.brake = 0
                else:
                    self._control.brake = 0.1
            self.previous_time = world.hud.simulation_time 

        



# ==============================================================================
# -- HUD -----------------------------------------------------------------------
# ==============================================================================


class HUD(object):
    def __init__(self, width, height):
        self.dim = (width, height)
        font = pygame.font.Font(pygame.font.get_default_font(), 20)
        font_name = 'courier' if os.name == 'nt' else 'mono'
        fonts = [x for x in pygame.font.get_fonts() if font_name in x]
        default_font = 'ubuntumono'
        mono = default_font if default_font in fonts else fonts[0]
        mono = pygame.font.match_font(mono)
        self._font_mono = pygame.font.Font(mono, 12 if os.name == 'nt' else 14)
        self._notifications = FadingText(font, (width, 40), (0, height - 40))
        self.help = HelpText(pygame.font.Font(mono, 16),__doc__, width, height)
        self.server_fps = 0
        self.frame = 0
        self.simulation_time = 0
        self._show_info = True
        self._info_text = []
        self._server_clock = pygame.time.Clock()

        self._show_ackermann_info = False
        self._ackermann_control = carla.VehicleAckermannControl()

    def on_world_tick(self, timestamp):
        self._server_clock.tick()
        self.server_fps = self._server_clock.get_fps()
        self.frame = timestamp.frame
        self.simulation_time = timestamp.elapsed_seconds

    def tick(self, world, clock):
        self._notifications.tick(world, clock)
        if not self._show_info:
            return
        t = world.player.get_transform()
        v = world.player.get_velocity()
        c = world.player.get_control()
        compass = world.imu_sensor.compass
        heading = 'N' if compass > 270.5 or compass < 89.5 else ''
        heading += 'S' if 90.5 < compass < 269.5 else ''
        heading += 'E' if 0.5 < compass < 179.5 else ''
        heading += 'W' if 180.5 < compass < 359.5 else ''
        # colhist = world.collision_sensor.get_collision_history()
        # collision = [colhist[x + self.frame - 200] for x in range(0, 200)]
        # max_col = max(1.0, max(collision))
        # collision = [x / max_col for x in collision]
        vehicles = world.world.get_actors().filter('vehicle.*')
        goal_text= "" if world.goal_polar is None else 'Goal:{:.2f} m, {:.2f}º'.format(*world.goal_polar)
        self._info_text = [
            'Server:  % 16.0f FPS' % self.server_fps,
            'Client:  % 16.0f FPS' % clock.get_fps(),
            '',
            'Vehicle: % 20s' % get_actor_display_name(world.player, truncate=20),
            'Map:     % 20s' % world.map.name.split('/')[-1],
            'Simulation time: % 12s' % datetime.timedelta(seconds=int(self.simulation_time)),
            '',
            'Speed:   % 15.0f km/h' % (3.6 * math.sqrt(v.x**2 + v.y**2 + v.z**2)),
            u'Compass:% 17.0f\N{DEGREE SIGN} % 2s' % (compass, heading),
            'Accelero: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.accelerometer),
            'Gyroscop: (%5.1f,%5.1f,%5.1f)' % (world.imu_sensor.gyroscope),
            'Location:% 20s' % ('(% 5.1f, % 5.1f)' % (t.location.x, t.location.y)),
            'Yaw: % 5.1f' % (world.player.get_transform().rotation.yaw),
            goal_text,
            'GNSS:% 24s' % ('(% 2.6f, % 3.6f)' % (world.gnss_sensor.lat, world.gnss_sensor.lon)),
            'Height:  % 18.0f m' % t.location.z,
            '']
        if isinstance(c, carla.VehicleControl):
            self._info_text += [
                ('Throttle:', c.throttle, 0.0, 1.0),
                ('Steer:', c.steer, -1.0, 1.0),
                ('Brake:', c.brake, 0.0, 1.0),
                ('Reverse:', c.reverse),
                ('Hand brake:', c.hand_brake),
                ('Manual:', c.manual_gear_shift),
                'Gear:        %s' % {-1: 'R', 0: 'N'}.get(c.gear, c.gear)]
            if self._show_ackermann_info:
                self._info_text += [
                    '',
                    'Ackermann Controller:',
                    '  Target speed: % 8.0f km/h' % (3.6*self._ackermann_control.speed),
                ]
        elif isinstance(c, carla.WalkerControl):
            self._info_text += [
                ('Speed:', c.speed, 0.0, 5.556),
                ('Jump:', c.jump)]
        # self._info_text += [
        #     '',
        #     'Collision:',
        #     collision,
        #     '',
        #     'Number of vehicles: % 8d' % len(vehicles)]
        if len(vehicles) > 1:
            self._info_text += ['Nearby vehicles:']
            distance = lambda l: math.sqrt((l.x - t.location.x)**2 + (l.y - t.location.y)**2 + (l.z - t.location.z)**2)
            vehicles = [(distance(x.get_location()), x) for x in vehicles if x.id != world.player.id]
            for d, vehicle in sorted(vehicles, key=lambda vehicles: vehicles[0]):
                if d > 200.0:
                    break
                vehicle_type = get_actor_display_name(vehicle, truncate=22)
                self._info_text.append('% 4dm %s' % (d, vehicle_type))

    def show_ackermann_info(self, enabled):
        self._show_ackermann_info = enabled

    def update_ackermann_control(self, ackermann_control):
        self._ackermann_control = ackermann_control

    def toggle_info(self):
        self._show_info = not self._show_info

    def notification(self, text, seconds=2.0):
        self._notifications.set_text(text, seconds=seconds)

    def error(self, text):
        self._notifications.set_text('Error: %s' % text, (255, 0, 0))

    def render(self, display):
        if self._show_info:
            info_surface = pygame.Surface((220, self.dim[1]))
            info_surface.set_alpha(100)
            display.blit(info_surface, (0, 0))
            v_offset = 4
            bar_h_offset = 100
            bar_width = 106
            for item in self._info_text:
                if v_offset + 18 > self.dim[1]:
                    break
                if isinstance(item, list):
                    if len(item) > 1:
                        points = [(x + 8, v_offset + 8 + (1.0 - y) * 30) for x, y in enumerate(item)]
                        pygame.draw.lines(display, (255, 136, 0), False, points, 2)
                    item = None
                    v_offset += 18
                elif isinstance(item, tuple):
                    if isinstance(item[1], bool):
                        rect = pygame.Rect((bar_h_offset, v_offset + 8), (6, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect, 0 if item[1] else 1)
                    else:
                        rect_border = pygame.Rect((bar_h_offset, v_offset + 8), (bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect_border, 1)
                        f = (item[1] - item[2]) / (item[3] - item[2])
                        if item[2] < 0.0:
                            rect = pygame.Rect((bar_h_offset + f * (bar_width - 6), v_offset + 8), (6, 6))
                        else:
                            rect = pygame.Rect((bar_h_offset, v_offset + 8), (f * bar_width, 6))
                        pygame.draw.rect(display, (255, 255, 255), rect)
                    item = item[0]
                if item:  # At this point has to be a str.
                    surface = self._font_mono.render(item, True, (255, 255, 255))
                    display.blit(surface, (8, v_offset))
                v_offset += 18
        self._notifications.render(display)
        self.help.render(display)


# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    monitor = screeninfo.get_monitors()
    if len(monitor) > 1:
        monitor=monitor[1]
    pygame.init()
    pygame.font.init()
    x, y = monitor.x+monitor.width/2-args.width/2, monitor.y+monitor.height/2-(args.height/2+128)
    os.environ['SDL_VIDEO_WINDOW_POS'] = '%i,%i' % (x, y)
    os.environ['SDL_VIDEO_CENTERED'] = '0'
    world = None
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(2000.0)

        sim_world = client.get_world()
        if args.sync:
            original_settings = sim_world.get_settings()
            settings = sim_world.get_settings()
            if not settings.synchronous_mode:
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 1/FPS
            sim_world.apply_settings(settings)

            traffic_manager = client.get_trafficmanager()
            traffic_manager.set_synchronous_mode(True)

        if args.autopilot and not sim_world.get_settings().synchronous_mode:
            print("WARNING: You are currently in asynchronous mode and could "
                  "experience some issues with the traffic simulation")

        display = pygame.display.set_mode(
            (args.width, args.height+128),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        controller = JoystickControl(world, args.autopilot, hud)
        if controller is None: return

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            if controller.parse_events(client, world, clock, args.sync):
                return
            world.tick(clock)
            world.render(display)
            if world.recording_enabled: world.record()
            pygame.display.flip()

    finally:

        if original_settings:
            sim_world.apply_settings(original_settings)

        if (world and world.recording_enabled):
            client.stop_recorder()

        if world is not None:
            world.destroy()

        pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    argparser = argparse.ArgumentParser(
        description='CARLA Manual Control Client')
    argparser.add_argument(
        '-v', '--verbose',
        action='store_true',
        dest='debug',
        help='print debug information')
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
        '-a', '--autopilot',
        action='store_true',
        help='enable autopilot')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
    argparser.add_argument(
        '--filter',
        metavar='PATTERN',
        default='blue',
        help='actor filter (default: "blue")')
    argparser.add_argument(
        '--generation',
        metavar='G',
        default='2',
        help='restrict to certain actor generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--rolename',
        metavar='NAME',
        default='hero',
        help='actor role name (default: "hero")')
    argparser.add_argument(
        '--gamma',
        default=2.2,
        type=float,
        help='Gamma correction of the camera (default: 2.2)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Deactivate synchronous mode execution')
    args = argparser.parse_args()
    args.sync = not args.sync
    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()
