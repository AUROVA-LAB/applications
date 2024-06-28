#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""

import glob
import os
import sys
import time
import signal

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

from carla import VehicleLightState as vls

import argparse
import logging
import math
import numpy as np
from numpy import random
from utils import get_actor_blueprints, ReadPedestrianRoutesFromXML, CollisionEvaluator
from evaluator import Evaluator
from config import *

#The pedestrian starts moving when is closer to the robot, depending of the start distantace in the main axis of its trajectory (X or Y)
START_DISTANCE=6 #Distance in the main axis of the trajectory. 
CLOSE_DISTANCE=15 #Distance in the other axis, to consider that the robot is close. 

APPARITION_DISTANCE=2.5 #Spawn distance of the apparition respect the robot
CLOSE_APPARTION_DISTANCE=2 #Distance respect the apparition node to spawn the apparition.

TEST_ROUTES=False #Test pedestrian routes without the robot

def main():
    argparser = argparse.ArgumentParser(
        description=__doc__)
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
        '--filterw',
        metavar='PATTERN',
        default='walker.pedestrian.*',
        help='Filter pedestrian type (default: "walker.pedestrian.*")')
    argparser.add_argument(
        '--generationw',
        metavar='G',
        default='2',
        help='restrict to certain pedestrian generation (values: "1","2","All" - default: "2")')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--asynch',
        action='store_true',
        help='Activate asynchronous mode execution')
    argparser.add_argument(
        '--output',
        default=None,
        help='Town_name+output+id.xml is the output evaluation file. (Default not evaluate)')
    argparser.add_argument(
        '--vehicle',
        metavar='NAME',
        default='*blue*',
        help='wildcard_pattern of the actor(default: "*blue*")')
    argparser.add_argument(
        '--margin',
        metavar='P',
        type=float,
        default=-0.2,
        help='Safety margin in meters to consider a collision.')
    argparser.add_argument(
        '--no_collision',"-c",
        action='store_true',
        help='Not check collision. Use when the robot is spawn by ROS, because the simulation crashes if a collision happens')
    argparser.add_argument(
        '--check_safety_stop',"-s",
        action='store_true',
        help='Penalize when a safety stop happens.')
    argparser.add_argument(
        '--pedestrian_routes',
        metavar='P',
        default=None,
        help='Pedestrian routes file. (Default defined in config.py, it can be defined also with environment variables)')
    argparser.add_argument(
        '--way_id',
        default=None,
        help='Way ID of this execution. (Default defined in config.py, it can be defined also with environment variables)')
    
    

    args = argparser.parse_args()
    

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    walkers_list = []
    all_id = []
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False
    
    try:
        world = client.get_world()
        

        settings = world.get_settings()
        if not args.asynch:
            if not settings.synchronous_mode:
                synchronous_master = True
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 1/FPS
            else:
                synchronous_master = False

        blueprintsWalkers = get_actor_blueprints(world, args.generationw, args.filterw)
        pedestrian_routes = args.pedestrian_routes if args.pedestrian_routes is not None else PEDESTRIANS_ROUTES_FILE
        seed, routes, apparitions_node = ReadPedestrianRoutesFromXML(pedestrian_routes)

        # @todo cannot import these directly.
        SpawnActor = carla.command.SpawnActor

        #Seet random seed for repeateability
        world.set_pedestrians_seed(seed)
        random.seed(seed)
        # percentagePedestriansRunning=0.25
        percentagePedestriansRunning=0.0

        #Wait until blue is spawned
        if not TEST_ROUTES:
            robot=world.get_actors().filter(args.vehicle)
            while len(robot)==0:
                time.sleep(1)
                robot=world.get_actors().filter(args.vehicle)
            robot=robot[0]
        evaluator = None if args.output is None else Evaluator(world,args)    
        # -------------
        # Spawn Walkers
        # -------------
        number_of_walkers = len(routes)
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(1.0)

        batch = []
        walker_speed = []
        for i in range(number_of_walkers):
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(min(3.0,float(walker_bp.get_attribute('speed').recommended_values[2])))
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(SpawnActor(walker_bp, routes[i][0]["pos"]))
        results = client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
                print(i,routes[i][0]["pos"])
                exit() #All the actors should be spawn
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id
        # we put together the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            all_id.append(walkers_list[i]["con"])
            all_id.append(walkers_list[i]["id"])
        all_actors = world.get_actors(all_id)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()

        # -------------
        # Define apparitions
        # -------------
        number_of_apparitions = len(apparitions_node)
       
        apparition_blueprints = []
        for i in range(number_of_apparitions):
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            apparition_blueprints.append(walker_bp)

        # wait for a tick to ensure client receives the last transform of the walkers we have just created
        if args.asynch or not synchronous_master:
            world.wait_for_tick()
        else:
            world.tick()


        # initialize 
        current_target = [1 for _ in range(number_of_walkers)]
        active_walkers = [False for _ in range(number_of_walkers)]
        active_apparitions = [False for _ in range(number_of_apparitions)]
        apparition_actors = []
        inactive_bones = []
        for i in range(1, len(all_id), 2):
            boneOut=all_actors[i].get_bones().bone_transforms
            boneIn=[]
            for bone in boneOut:
                boneIn.append([bone.name, bone.relative])
            inactive_bones.append(carla.WalkerBoneControlIn(boneIn))
            all_actors[i-1].start() 
            all_actors[i-1].set_max_speed(0)

        print('spawned  %d walkers, press Ctrl+C to exit.' % (len(walkers_list)))


        while True:
            if args.asynch or not synchronous_master:
                world.wait_for_tick()
            else:
                world.tick()

            #Manage walkers
            active_actors=[]
            for i in range(number_of_walkers):
                loc_walker = all_actors[i*2].get_location()
                target = routes[i][current_target[i]]["pos"].location
                if active_walkers[i]:
                    if(not all_actors[i*2+1].is_alive):
                        active_walkers[i]=False
                        continue
                    all_actors[i*2].go_to_location(target) #Avoid going to another target
                    active_actors.append(all_actors[i*2+1])
                    distance = math.sqrt((loc_walker.x-target.x)**2+(loc_walker.y-target.y)**2)
                    if distance<1.5:
                        current_target[i]+=1
                        if current_target[i]==len(routes[i]):current_target[i]=0
                        if routes[i][current_target[i]]["stop"]:
                            all_actors[i*2].set_max_speed(0)
                            all_actors[i*2+1].set_bones(inactive_bones[i])
                            all_actors[i*2+1].show_pose()
                            active_walkers[i]=False
                            if evaluator is not None: evaluator.pedestrian_traj_eval(all_actors[i*2+1].id,loc_walker)
                        else:
                            all_actors[i*2].go_to_location(routes[i][current_target[i]]["pos"].location)
                else:
                    if TEST_ROUTES:
                        all_actors[i*2+1].hide_pose()
                        all_actors[i*2].go_to_location(routes[i][current_target[i]]["pos"].location)
                        all_actors[i*2].set_max_speed(float(walker_speed[i]))
                        active_walkers[i]=True
                    else:
                        #Get main direction for the next target (X or Y)
                        if abs(loc_walker.x-target.x) > abs(loc_walker.y-target.y):
                            main_axis_walker, aux_axis_walker=loc_walker.y, loc_walker.x
                            main_axis_robot, aux_axis_robot=robot.get_location().y, robot.get_location().x
                        else:
                            main_axis_walker, aux_axis_walker=loc_walker.x, loc_walker.y
                            main_axis_robot, aux_axis_robot=robot.get_location().x, robot.get_location().y
                        distance = abs(main_axis_walker-main_axis_robot)
                        if abs(main_axis_walker-main_axis_robot)<=START_DISTANCE and abs(aux_axis_walker-aux_axis_robot)<=CLOSE_DISTANCE:
                            all_actors[i*2+1].hide_pose()
                            all_actors[i*2].go_to_location(routes[i][current_target[i]]["pos"].location)
                            all_actors[i*2].set_max_speed(float(walker_speed[i]))
                            active_walkers[i]=True
                            if evaluator is not None: 
                                evaluator.init_pedestrian_traj(all_actors[i*2+1].id,loc_walker,float(walker_speed[i]))
                                evaluator.init_robot_traj(all_actors[i*2+1].id,loc_walker,routes[i][current_target[i]]["pos"].location)
            
            #Manage apparitions
            if not TEST_ROUTES:
                for i in range(number_of_apparitions):
                    if not active_apparitions[i]:
                        distance = math.sqrt((apparitions_node[i].location.x-robot.get_location().x)**2+ 
                                            (apparitions_node[i].location.y-robot.get_location().y)**2)
                        if distance<=CLOSE_APPARTION_DISTANCE:
                            robot_np=np.array([robot.get_location().x,robot.get_location().y,robot.get_location().z])
                            apparition_location=apply_transform(robot_np,np.array([APPARITION_DISTANCE,0,0.5]),robot.get_transform().rotation.yaw)
                            try:

                                actor = world.spawn_actor(apparition_blueprints[i], carla.Transform(carla.Location(*apparition_location),
                                                            carla.Rotation(0,robot.get_transform().rotation.yaw+180,0)) ) 
                                apparition_actors.append(actor)
                                active_apparitions[i]=True
                                if evaluator is not None:  evaluator.init_robot_traj(actor.id,carla.Location(*apparition_location))
                            except Exception as e:
                                print(e)

            #Evaluation
            if evaluator is not None:
                if evaluator.check_events(): return
                if not args.no_collision and not (evaluator.check_collision_sensor()):
                    #Check the slight collision when the sensor fails.
                    evaluator.check_collision_iou(active_actors+apparition_actors)

    finally:
        if not args.asynch and synchronous_master:
            settings = world.get_settings()
            settings.synchronous_mode = False
            settings.no_rendering_mode = False
            settings.fixed_delta_seconds = None
            world.apply_settings(settings)
        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(all_id), 2):
            all_actors[i].stop()
        if evaluator is not None:
            if not args.no_collision:
                evaluator.collision_sensor.sensor.stop()
                evaluator.collision_sensor.sensor.destroy()
            evaluator.record_metrics()

        print('\ndestroying %d walkers and %d appartitions' % (len(walkers_list), len(apparition_actors)))
        client.apply_batch([carla.command.DestroyActor(x) for x in all_id])
        client.apply_batch([carla.command.DestroyActor(x) for x in apparition_actors])

        time.sleep(0.1)

#Applies a translation in yaw (in degrees) direction
def apply_transform(point, translation, yaw):
    yaw *= np.pi/180.0
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw), 0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]])
    translation=np.matmul(rotation_matrix,translation)
    point+=translation
    return point

def handler(sig, frame):
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
