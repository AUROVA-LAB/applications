#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

# Allows controlling a vehicle listening a network (which ca be execute in a docker).
"""
CARLA agent using transformer network

The program will be closed when the route is complented.

""" 

from __future__ import print_function


# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import sys
import screeninfo
import itertools
import signal

import zmq.ssh

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
import datetime
import logging
import math
import random
from utils import *
from config import *
import json
import zmq

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
CHANGE_WEATHER_BUTTON=2
TOGGLE_360IMAGE_BUTTON = 6
EXIT_BUTTON = 9
RECORD_BUTTON = 8
EXPERT_AGENT_BUTTON = 7

NEAR_COLLISION_DISTANCE = 1.8 # meters
NEAR_COLLISION_DISTANCE2 = 0.8 # Stop if obstacle is nearest than this distance from the sensor

RUMBLE=False




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
        self.lidar_sensor = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self._architecture = args.architecture
        try:
            way_id = int(args.way_id) if args.way_id is not None else WAY_ID
            self.route, weather = ReadRouteFromXML(ROUTE_FILENAME,way_id)
            self.way_id=0
            self.set_weather(weather)
        except:
            print("Error: Route filename {} or route id {} not found".format(ROUTE_FILENAME,way_id))
            sys.exit(1)
        self.semantic=None
        self.info=None
        self.waypoints=None
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
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
        blueprint = random.choice(get_actor_blueprints(self.world, self._actor_generation))
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
            node=self.route["way"][self.way_id]
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
                sys.exit(0)
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
        self.rgb_camera=RGBcamera(self.player, self.hud)
        if self._architecture=="transfuser":
            self.lidar_sensor=LidarSensor(self.player,self.hud)
        if self._architecture=="MTAP":
            self.depth_camera=Depthcamera(self.player, self.hud)
        actor_type = get_actor_display_name(self.player)
        self.goal_polar=None
        self.hud.notification(actor_type)

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def record_video(self):
        camera=self.camera_manager._image.swapaxes(0, 1)
        camera=cv2.cvtColor(camera, cv2.COLOR_BGR2RGB)
        
        image360=cv2.resize(self.rgb_camera._image, dsize=(128,self.hud.dim[0])).swapaxes(0, 1)
        image360=cv2.cvtColor(np.float32(image360), cv2.COLOR_BGR2RGB)

        image = np.zeros((camera.shape[0]+256,camera.shape[1],3), np.uint8)
        image[0:-256]=camera
        image[-256:-128]=image360
        if self.semantic is not None:
            semantic=cv2.resize(self.semantic, dsize=(image.shape[1],128)) 
            semantic=cv2.cvtColor(semantic, cv2.COLOR_BGR2RGB)
            image[-128:]=semantic

        # Create a black image 
        size = (320,320,3)
        img = np.zeros(size, np.uint8)
        pixels_per_meter=150
        alpha=0.8
        cv2.circle(img,(150,150),6,(255,0,0),-1) 
        if self.waypoints is not None:
            for point in self.waypoints:
                center=(150-int(point[1]*pixels_per_meter*5), int(150-point[0]*pixels_per_meter))
                cv2.circle(img,center,4,(int(255*alpha),0,int(255*(1-alpha))),-1)
                alpha-=0.2
        if self.info is not None:
            cv2.putText(img, "Speed: {:.2f}, Steer: {:.2f}".format(self.info[0], self.info[1]), (2,300), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (255,255,255))
        
        image[:320,-320:]=img
        self.video.write(image)


    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def set_weather(self, id):
        if id < 0 or id >= len(self._weather_presets):
            print("Invalid weather id.")
            return
        self._weather_index = id
        preset = self._weather_presets[self._weather_index]
        self.world.set_weather(preset[0])

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

    def tick(self, clock, socket, control):
        self.hud.tick(self, clock)
        message={'end':False}
        speed = self.player.get_velocity()
        speed = math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
        if control.reverse: speed*=-1

        robot_pose=self.player.get_location()
        if self._architecture=="transfuser":
            image=self.rgb_camera._image.tobytes()
            message["image_shape"]=self.rgb_camera._image.shape
            message["speed"]=speed
            yaw=self.player.get_transform().rotation.yaw
            lidar=self.lidar_sensor._pointcloud.tobytes()
            message["lidar_shape"]=self.lidar_sensor._pointcloud.shape


            goal_pose = carla.Location()
            
            node=self.route["way"][self.way_id]
            goal_pose.x, goal_pose.y=self.route["nodes"][node][0], self.route["nodes"][node][1]
            goal_pose.x, goal_pose.y=goal_pose.x-robot_pose.x, goal_pose.y-robot_pose.y

            #Check if the robot has arrived to next point
            goal_distance=math.sqrt(goal_pose.x**2+goal_pose.y**2)
            if goal_distance < 2.5:
                self.hud.notification("Robot arrived to node "+node)
                self.way_id+=1
            self.goal_polar = [goal_distance,yaw-math.atan2(goal_pose.y,goal_pose.x)/DEG2RAD]
            if self.goal_polar[1]>180: self.goal_polar[1]-=360
            elif self.goal_polar[1]<-180: self.goal_polar[1]+=360
            theta = -yaw*DEG2RAD
            rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], 
                            [np.sin(theta),  np.cos(theta)]])
            #Add information of current yaw in the target_point
            message["target_point"]=np.dot(rotMatrix,[goal_pose.x,goal_pose.y]).tolist()
            if self.way_id>=len(self.route["way"]):
                message["end"]=True
            socket.send_string(json.dumps(message),zmq.SNDMORE)
            socket.send(image,zmq.SNDMORE)
            socket.send(lidar)
        elif self._architecture == "TCP":
            image=self.rgb_camera._image.tobytes()
            message["image_shape"]=self.rgb_camera._image.shape
            message["speed"]=speed
            message["steer"]=control.steer
            yaw=self.player.get_transform().rotation.yaw

            goal_pose = carla.Location()

            node=self.route["way"][self.way_id]
            goal_pose.x, goal_pose.y=self.route["nodes"][node][0], self.route["nodes"][node][1]
            goal_pose.x, goal_pose.y=goal_pose.x-robot_pose.x, goal_pose.y-robot_pose.y

            #Check if the robot has arrived to next point
            goal_distance=math.sqrt(goal_pose.x**2+goal_pose.y**2)
            if goal_distance < 2.5:
                self.hud.notification("Robot arrived to node "+node)
                self.way_id+=1
            self.goal_polar = [goal_distance,yaw-math.atan2(goal_pose.y,goal_pose.x)/DEG2RAD]
            if self.goal_polar[1]>180: self.goal_polar[1]-=360
            elif self.goal_polar[1]<-180: self.goal_polar[1]+=360
            theta = -yaw*DEG2RAD
            rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], 
                            [np.sin(theta),  np.cos(theta)]])
            #Add information of current yaw in the target_point
            message["target_point"]=np.dot(rotMatrix,[goal_pose.x,goal_pose.y]).tolist()
            if self.way_id>=len(self.route["way"]):
                message["end"]=True
            socket.send_string(json.dumps(message),zmq.SNDMORE)
            socket.send(image)
        elif self._architecture == "MTAP":
            image=self.rgb_camera._image.tobytes()
            # depth=self.depth_camera._image[:,:,0].tobytes()
            depth=self.depth_camera._raw.tobytes()
            message["image_shape"]=self.rgb_camera._image.shape
            message["speed"]=speed
            message["steer"]=control.steer
            message["clock"]=self.hud.simulation_time
            yaw=self.player.get_transform().rotation.yaw

            goal_pose = carla.Location()

            node=self.route["way"][self.way_id]
            goal_pose.x, goal_pose.y=self.route["nodes"][node][0], self.route["nodes"][node][1]
            goal_pose.x, goal_pose.y=goal_pose.x-robot_pose.x, goal_pose.y-robot_pose.y
            #Check if the robot has arrived to next point
            goal_distance=math.sqrt(goal_pose.x**2+goal_pose.y**2)
            if goal_distance < 2.5:
                self.hud.notification("Robot arrived to node "+node)
                self.way_id+=1
            self.goal_polar = [goal_distance,yaw-math.atan2(goal_pose.y,goal_pose.x)/DEG2RAD]
            if self.goal_polar[1]>180: self.goal_polar[1]-=360
            elif self.goal_polar[1]<-180: self.goal_polar[1]+=360
            theta = -yaw*DEG2RAD
            rotMatrix = np.array([[np.cos(theta), -np.sin(theta)], 
                            [np.sin(theta),  np.cos(theta)]])
            #Add information of current yaw in the target_point
            message["target_point"]=np.dot(rotMatrix,[goal_pose.x,goal_pose.y]).tolist()
            if self.way_id>=len(self.route["way"]):
                message["end"]=True
            socket.send_string(json.dumps(message),zmq.SNDMORE)
            socket.send(image,zmq.SNDMORE)
            socket.send(depth)
        return not message["end"]

    def render(self, display):
        if self.show_lidar:
            self.lidar_sensor.render(display)
        else:
            self.camera_manager.render(display)
        if self.show_depth_camera:
            self.depth_camera.render(display)
        else:
            self.rgb_camera.render(display)
        if self.recording_enabled: self.record_video()
        self.hud.render(display)

    def destroy_sensors(self):
        self.camera_manager.sensor.destroy()
        self.camera_manager.sensor = None
        self.camera_manager.index = None

    def destroy(self):
        if self.radar_sensor is not None:
            self.toggle_radar()
        sensors = [
            self.camera_manager.sensor,
            # self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        if self._architecture=="transfuser": sensors.append(self.lidar_sensor.sensor)
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        self.rgb_camera.destroy()
        if self._architecture=="MTAP": self.depth_camera.destroy()
        # self.sematic_camera.destroy()
        if self.player is not None:
            self.player.destroy()
        
# ==============================================================================
# -- JoystickControl -----------------------------------------------------------
# ==============================================================================


class JoystickControl(object):
    """Class that handles keyboard input."""
    def __init__(self, world, hud):
        self._ackermann_reverse = 1
        self.node=0
        self.hud=hud
        self.previous_time=0.0
        self.active=False
        if isinstance(world.player, carla.Vehicle):
            self._control = carla.VehicleControl()
            self._ackermann_control = carla.VehicleAckermannControl()
            self._lights = carla.VehicleLightState.NONE
            world.player.set_light_state(self._lights)
            self.pid_controller=PID_Controller(KP,KI,KD,0.0,1.0)
        elif isinstance(world.player, carla.Walker):
            self._control = carla.WalkerControl()
            self._rotation = world.player.get_transform().rotation
        else:
            raise NotImplementedError("Actor type not supported")
        self.past_steering = 0.0
        pygame.joystick.init()
        if(pygame.joystick.get_count()>0):
            self.joystick=pygame.joystick.Joystick(0)
        else:
            print("The joystick is not detected.")
        world.hud.notification("Press 'H' or '?' for help.", seconds=4.0)
        # self.collision_sensor=world.collision_sensor

    def check_near_collision(self, world : World, reverse=False): 
        if not RUMBLE: return     
        if reverse:
            for col in itertools.chain(range(0,200), range(1848,2048)):
                for row in range(64,128):
                    if world.depth_camera._raw[col,row]*1000.0<NEAR_COLLISION_DISTANCE:
                        self.joystick.rumble(0.0, 0.5, 500)
                        return
        else:
            for col in range(800,1200):
                for row in range(64,128):
                    if world.depth_camera._raw[col,row]*1000.0<NEAR_COLLISION_DISTANCE:
                        self.joystick.rumble(0.0, 0.5, 500)
                        return

    def parse_events(self, client, world:World, clock, sync_mode):
        # if RUMBLE and self.collision_sensor.collision_rumble:
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
                elif event.button == TOGGLE_360IMAGE_BUTTON:
                    world.toggle_360_camera()
                elif event.button == EXPERT_AGENT_BUTTON:
                    self.active = not self.active
                    self.hud.notification('%r' % "Expert" if self.active else "Agent")
                elif event.button == CHANGE_WEATHER_BUTTON:
                    world.next_weather()
                elif event.button == RECORD_BUTTON:
                    if world.recording_enabled:
                        print("End recording video")
                        world.video.release()
                        world.recording_enabled=False
                    else:
                        print("Starting recording video")
                        video=1
                        while os.path.exists("./videos/output"+str(video)+".avi"):
                            video+=1
                        fourcc = cv2.VideoWriter_fourcc(*'XVID')
                        world.video = cv2.VideoWriter("./videos/output"+str(video)+".avi", fourcc, 15, (self.hud.dim[0],self.hud.dim[1]+128))
                        world.recording_enabled=True
                        
        if self.active and isinstance(self._control, carla.VehicleControl):
            self._parse_vehicle_keys(pygame.key.get_pressed(), clock.get_time(), world)
            self.get_speed_action(world)
            self._control.reverse = self._control.gear < 0

            self.hud.update_ackermann_control(self._ackermann_control)
        self.previous_time = world.hud.simulation_time


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
                if dt==0:return
                self._control.throttle = self.pid_controller.step(self._ackermann_control.speed*self._control.gear,speed, dt)
                self._control.brake = 0
            else:
                self._control.throttle = 0
                if  speed <= 0.1 and abs(self._ackermann_control.speed)>0.0:
                    self._control.gear = 1 if self._control.reverse else -1
                    self._control.brake = 0
                else:
                    self._control.brake = 0.1
           



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

        self._show_ackermann_info = True
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
                    '  Target speed: % .2f m/s' % (self._ackermann_control.speed),
                    '  Target steering: % .2fº' % (self._ackermann_control.steer),
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



def render_waypoints(waypoints, display, width, info=None):
    # Create a black image
    size = (175,175,3) if info is not None else (150,150,3)
    img = np.zeros(size, np.uint8)
    pixels_per_meter=75
    alpha=0.8
    cv2.circle(img,(75,75),3,(255,0,0),-1) 
    if waypoints is not None:
        for point in waypoints:
            center=(75-int(point[1]*pixels_per_meter*5), int(75-point[0]*pixels_per_meter))
            cv2.circle(img,center,2,(int(255*alpha),0,int(255*(1-alpha))),-1)
            alpha-=0.2
    if info is not None:
        cv2.putText(img, "Speed: {:.2f}, Steer: {:.2f}".format(info[0], info[1]), (2,150), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255))
        if len(info)>2:
            cv2.putText(img, "Ctrl: {:.2f}, Ctrl: {:.2f}".format(info[2], info[3]), (2,160), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255))
            cv2.putText(img, "Traj: {:.2f}, Traj: {:.2f}".format(info[4], info[5]), (2,170), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255,255,255))
    surface=pygame.surfarray.make_surface(img.swapaxes(0, 1))
    display.blit(surface, (width-150, 0))

def render_semantic(semantic, display, width, height):
    if semantic is None: return
    # Resize image
    semantic=cv2.resize(semantic, dsize=(int(width/2),64))

    surface=pygame.surfarray.make_surface(semantic.swapaxes(0, 1))
    display.blit(surface, (int(width/2), height-64))

def get_speed_action(control, desired_speed, pid_controller,world, dt):
    speed = world.player.get_velocity()
    speed = math.sqrt(speed.x**2 + speed.y**2 + speed.z**2)
    diference=desired_speed-speed
    if isinstance(control, carla.VehicleControl):
            if desired_speed * control.gear>0 and diference*control.gear>-0.7:
                control.throttle = pid_controller.step(desired_speed*control.gear,speed, dt)
                control.brake = 0.1 if abs(desired_speed)<0.01 else 0
            else:
                control.throttle = 0
                if  speed <= 0.1 and abs(desired_speed)>0.0:
                    control.gear = -1 if control.gear>0  else 1
                    control.brake = 0
                else:
                    control.brake = 0.1

def check_near_collision(depth, reverse=False):
        min_distance=1
        if reverse:
            for col in itertools.chain(range(0,250), range(1798,2048)):
                for row in range(64,128):
                    if depth[col,row]<min_distance:
                        min_distance=depth[col,row]
        else:
            for col in range(750,1250):
                for row in range(64,128):
                    if depth[col,row]<min_distance:
                        min_distance=depth[col,row]
        return min_distance

# ==============================================================================
# -- game_loop() ---------------------------------------------------------------
# ==============================================================================


def game_loop(args):
    # Position of the new window.
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


        display = pygame.display.set_mode(
            (args.width, args.height+128),
            pygame.HWSURFACE | pygame.DOUBLEBUF)
        display.fill((0,0,0))
        pygame.display.flip()
        pid_controller=PID_Controller(KP,KI,KD,0.0,1.0)

        hud = HUD(args.width, args.height)
        world = World(sim_world, hud, args)
        controller = JoystickControl(world, hud)
        context = zmq.Context()
        
        waypoints = None
        semantic=None
        publisher = context.socket(zmq.PUB)
        publisher.setsockopt(zmq.LINGER, 500)
        socket_sub = context.socket(zmq.SUB)
        publisher.connect("tcp://127.0.0.1:5556")
        socket_sub.connect("tcp://127.0.0.1:5555")
        socket_sub.setsockopt_string(zmq.SUBSCRIBE, "")

        socket_evaluator = context.socket(zmq.PUB)
        socket_evaluator.setsockopt(zmq.LINGER, 500)
        socket_evaluator.bind("tcp://*:5554")

        poller = zmq.Poller()
        poller.register(socket_sub, zmq.POLLIN)

        if args.sync:
            sim_world.tick()
        else:
            sim_world.wait_for_tick()

        clock = pygame.time.Clock()
        control = carla.VehicleControl()
        control.steer=0.0
        previous_time = world.hud.simulation_time
        info=None
        wait_response=False
        safe_stop=False
        target_steer, target_speed = 0, 0

        if(args.record):
            if not os.path.exists("./videos"):
                os.mkdir("./videos")
            print("Starting recording video")
            video=1
            while os.path.exists("./videos/output"+str(video)+".avi"):
                video+=1
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            world.video = cv2.VideoWriter("./videos/output"+str(video)+".avi", fourcc, 15, (args.width, args.height+256))
            world.recording_enabled=True
        while True:
            if args.sync:
                sim_world.tick()
            clock.tick_busy_loop(60)
            
            if controller.parse_events(client, world, clock, args.sync):
                return
            
            if not wait_response:
                request_time=world.hud.simulation_time
                if not world.tick(clock,publisher,control):
                    return

            #Receive the control action from the network, non-blocking
            socks = dict(poller.poll(timeout=700))  # Timeout in milliseconds
            if socket_sub in socks and socks[socket_sub] == zmq.POLLIN:
                if(args.architecture=="MTAP"):
                    message = json.loads(socket_sub.recv())
                    semantic=socket_sub.recv()
                    semantic = np.frombuffer(semantic,'uint8')
                    semantic=np.reshape(semantic,[128,2048,3])
                    world.semantic=semantic
                else: 
                    message = json.loads(socket_sub.recv())
                
                target_steer = float(message["steer"])/MAX_STEERING_ANGLE
                target_steer = np.clip(target_steer,-1.0,1.0)
                target_speed = float(message["speed"])
                if (args.architecture=="MTAP" or args.architecture=="TCP"):
                    info=[float(message["speed"]),float(message["steer"]),float(message["speed_ctrl"]),
                        float(message["steer_ctrl"]),float(message["speed_traj"]),float(message["steer_traj"])]
                elif (args.architecture=="transfuser"):
                    info=[float(message["speed"]),float(message["steer"])]      
                waypoints = message["waypoints"]
                wait_response=False
            else:
                wait_response=True
                #"Emergency stop" if it does not received a meesage in timeout
                if (world.hud.simulation_time-request_time)*1000>args.timeout:
                    target_speed, target_steer=0.0, 0.0
                    info=[0.0,0.0]
                    #Sometimes throw an exception about it is not register some how.
                    poller.unregister(socket_sub)
                    socket_sub.close()
                    socket_sub = context.socket(zmq.SUB)
                    socket_sub.connect("tcp://127.0.0.1:5555")
                    socket_sub.setsockopt_string(zmq.SUBSCRIBE, '')
                    poller.register(socket_sub, zmq.POLLIN)
                    wait_response=False
                

            if args.architecture=="MTAP":
                closest_distance=check_near_collision(world.depth_camera._raw,target_speed<0)*1000.0
                if closest_distance<NEAR_COLLISION_DISTANCE2:
                    if not safe_stop and abs(target_speed)>0.01:
                        socket_evaluator.send_string("Safety stop",zmq.NOBLOCK)
                        safe_stop=True
                    target_speed=0.0
                elif closest_distance<NEAR_COLLISION_DISTANCE:
                    direction = 1 if target_speed>0 else -1
                    target_speed =  min(float(target_speed), 
                                        direction*MAX_SPEED * (closest_distance-NEAR_COLLISION_DISTANCE2)/(NEAR_COLLISION_DISTANCE-NEAR_COLLISION_DISTANCE2))
                else:
                    safe_stop=False
                if info is not None: info[0]=target_speed

            #Calculate control action
            if controller.active:
                controller.check_near_collision(world,control.reverse)
                control=controller._control
            else:
                dt = world.hud.simulation_time-previous_time
                previous_time = world.hud.simulation_time
                get_speed_action(control,target_speed,pid_controller,world,dt)
                control.reverse = control.gear < 0
                # Steering regulation: changes cannot happen abruptly, can't steer too much.
                if target_steer > control.steer + 0.2:
                    target_steer = control.steer + 0.2
                elif target_steer < control.steer - 0.2:
                    target_steer = control.steer - 0.2 
                control.steer=target_steer
            world.player.apply_control(control)
            world.info=info
            world.waypoints=waypoints
            world.render(display)
            render_waypoints(waypoints,display,args.width,info)
            render_semantic(semantic,display,args.width, args.height)
            pygame.display.flip()

    finally:
        if world.recording_enabled: 
            print("End recording video")
            world.video.release()
        if original_settings:
            sim_world.apply_settings(original_settings)

        if world is not None:
            world.destroy()
            poller.unregister(socket_sub)
            socket_sub.close()
            publisher.close()
            socket_evaluator.close()
            context.term()

        pygame.quit()

def handler(sig, frame):
    print('\nCancelled by user. Bye!')
    sys.exit(0)

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
        '--architecture',
        metavar='P',
        default="transfuser",
        help='Network arhitecture (transfuser, MTAP or TCP)')
    argparser.add_argument(
        '--res',
        metavar='WIDTHxHEIGHT',
        default='1280x720',
        help='window resolution (default: 1280x720)')
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
        '--way_id',
        default=None,
        help='Way ID of this execution. (Default defined in config.py)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        help='Deactivate synchronous mode execution')
    argparser.add_argument(
        '-t', '--timeout',
        metavar='P',
        default=500,
        help='timeout waiting response from neural network server')
    argparser.add_argument(
        '-r', '--record',
        action='store_true',
        help='Record a video, saved in ./videos')
    args = argparser.parse_args()
    if args.architecture != "transfuser" and args.architecture != "TCP" and args.architecture != "MTAP":
        print("ERROR: Neural network architecture should be transfuser, MTAP or TCP")
    args.sync = not args.sync
    args.width, args.height = [int(x) for x in args.res.split('x')]

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('listening to server %s:%s', args.host, args.port)

    signal.signal(signal.SIGINT, handler)
    print(__doc__)

    try:

        game_loop(args)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')


if __name__ == '__main__':

    main()

