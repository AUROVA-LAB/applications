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


import carla

from carla import ColorConverter as cc

import argparse
import math
from utils import *
from config import *
from fnmatch import fnmatch
import json
import zmq

class Evaluator(object):
    def __init__(self, carla_world : carla.World, args):
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
        #Get the collision sensor and lane invasor from the actor
        #The lane invasor only for offroad scenaries.
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        #The simulation crash when trying to use a collision sensor in a sensor spawn by ROS.
        if not args.no_collision:
            self.collision_sensor = CollisionEvaluator(self.player)
            if self.collision_sensor is None:
                print("Error: Sensors not found.")
                sys.exit(1)
        self.check_safety_stop=args.check_safety_stop
        if args.check_safety_stop:
            context = zmq.Context()
            self.socket_evaluator = context.socket(zmq.SUB)
            self.socket_evaluator.connect("tcp://127.0.0.1:5554")
            self.socket_evaluator.setsockopt_string(zmq.SUBSCRIBE, "")

        self.output=args.output
        self.robot_bounding_box = self.player.bounding_box.extent
        self.robot_bounding_box.x += args.margin
        self.robot_bounding_box.y += args.margin

        try:
            way_id = int(args.way_id) if args.way_id is not None else WAY_ID
            self.route, weather = ReadRouteFromXML(ROUTE_FILENAME,way_id)
            self.way_id=1

        except:
            print("Error: Route filename {} or way id {} not found".format(ROUTE_FILENAME,way_id))
            sys.exit(1)

        self.pedestrian_traj={}
        self.robot_traj={}
        self.start_metrics()


    def get_time(self):
        return self.world.get_snapshot().timestamp.elapsed_seconds
    
    def start_metrics(self):
        self.start_experiment_time=self.get_time()
        self.infraction_score=1.0
        self.statistics ={
            "pedestrian":0,
            "static":0,
            "off_road":0,
            "slight_collision":0,
            "trajectory_timeout":False,
            "local_minimum_timeout":False,
            "safety_stops":0,
        }
        self.start_trajectory_time=self.start_experiment_time
        node, previous=self.route["way"][self.way_id], self.route["way"][self.way_id-1]
        x, y=self.route["nodes"][node][0]-self.route["nodes"][previous][0], self.route["nodes"][node][1]-self.route["nodes"][previous][1]
        distance = math.sqrt(x**2+y**2)
        self.max_trajectory_time=distance*TIME_DISTANCE_COEFFICIENT
        self.local_minimum_time=self.start_experiment_time
        self.local_minimum_pose=self.player.get_location()

        #For detecting that a collision with static has ended and not count it several times
        self.on_collision, self.count_collision=False, 0
        #Only count one collision per object, as when it happens the pedestrian will be removed (and not count several times the same collision)
        self.collided_pedestrians=[]
        self.slight_collided_pedestrians=[]

        #Init the output file
        route_name = os.path.basename(ROUTE_FILENAME).split(".")[0]
        prefix_path = PATH_RESULTS+self.output+"_"+route_name+"_"
        self.id=1
        while os.path.exists(prefix_path+str(self.id)+".json"):
            self.id+=1


    def check_collision_sensor(self):
        #Collision_sensor sometimes fails, so there is also the function check_collision_iou
        if self.on_collision:
            
            if self.collision_sensor.collision_event==0:
                self.count_collision+=1
                if self.count_collision>5:
                    self.on_collision, self.count_collision=False, 0
            else: 
                self.collision_sensor.collision_event=0
                
            return False
        else:
            if self.collision_sensor.collision_event==2:
                if self.collision_sensor.other_actor_id not in self.collided_pedestrians:
                    self.infraction_score*=PENALTY_COLLISION_PEDESTRIAN
                    self.collision_sensor.collision_event=0
                    self.statistics["pedestrian"]+=1
                    self.collided_pedestrians.append(self.collision_sensor.other_actor_id)
                    if self.collision_sensor.other_actor_id in self.slight_collided_pedestrians:
                        self.statistics["slight_collision"]-=1
                        self.infraction_score/=PENALTY_COLLISION_SLIGHT
                    print("Collision pedestrian")
                return True
            elif self.collision_sensor.collision_event==1:
                self.on_collision=True
                self.infraction_score*=PENALTY_COLLISION_STATIC
                self.collision_sensor.collision_event=0
                self.statistics["static"]+=1
                print("Collision static")
                return True
        return False

    def check_collision_iou(self, actor_list):
        pedestrian_ids=[]
        for actor in actor_list:
            pedestrian_ids.append(actor.id)
            if actor.id not in self.slight_collided_pedestrians and self.is_intersection(actor)>0:
                print("Slight collision")
                self.statistics["slight_collision"]+=1
                self.infraction_score*=PENALTY_COLLISION_SLIGHT
                self.slight_collided_pedestrians.append(actor.id)
        #Remove from the collision list if the pedestrian is not in the actor list.
        #It may happen several slight collisions in differents moments.
        copy_slight=self.slight_collided_pedestrians
        for id in copy_slight:
            if id not in pedestrian_ids: 
                self.slight_collided_pedestrians.remove(id)
    
    def check_events(self):
        if self.offroad and self.lane_invasion_sensor.lane_invasion_event:
            self.infraction_score*=PENALTY_OFF_ROAD
            self.lane_invasion_sensor.lane_invasion_event=False
            self.statistics["off_road"]+=1

        #Check if exit from local minimum
        robot_pose=self.player.get_location()
        if math.sqrt((robot_pose.x-self.local_minimum_pose.x)**2+(robot_pose.y-self.local_minimum_pose.y)**2) > MINIMUM_DISTANCE:
            self.local_minimum_time=self.get_time()
            self.local_minimum_pose=robot_pose
            
        #Check shutdown events
        if self.get_time()-self.start_trajectory_time>self.max_trajectory_time:
            self.statistics["trajectory_timeout"]=True
            print("Trajectory timeout: saving evaluation")
            return True
        elif self.get_time()-self.local_minimum_time>MAX_TIME_LOCAL_MINIMUM:
            self.statistics["local_minimum_timeout"]=True
            print("Local minimum timeout: saving evaluation")
            return True
        
        #Check way points and robot trajectories
        self.robot_traj_eval()
        robot_pose=self.player.get_location()
        node =self.route["way"][self.way_id]
        x, y=self.route["nodes"][node][0]-robot_pose.x, self.route["nodes"][node][1]-robot_pose.y
        if(math.sqrt(x**2+y**2)<2.5):
            self.way_id+=1
            if self.way_id>=len(self.route["way"]): return True
            node =self.route["way"][self.way_id]
            x, y=self.route["nodes"][node][0]-robot_pose.x, self.route["nodes"][node][1]-robot_pose.y
            self.start_trajectory_time=self.get_time()
            distance = math.sqrt(x**2+y**2)
            self.max_trajectory_time=distance*TIME_DISTANCE_COEFFICIENT

        if self.check_safety_stop:
            try:
                self.socket_evaluator.recv(zmq.NOBLOCK)        
                print("Safety stop")
                self.statistics["safety_stops"]+=1
            except zmq.ZMQError:
                pass

        return False
    
    def record_metrics(self):
        route_name = os.path.basename(ROUTE_FILENAME).split(".")[0]
        path = PATH_RESULTS+self.output+"_"+route_name+"_"+str(self.id)+".json"

        route_length, completed_route_distance=0,0
        for id in range(1,len(self.route["way"])):
            node, previous=self.route["way"][id], self.route["way"][id-1]
            x, y=self.route["nodes"][node][0]-self.route["nodes"][previous][0], self.route["nodes"][node][1]-self.route["nodes"][previous][1]
            if id == self.way_id:
                robot_pose=self.player.get_location()
                x2, y2=self.route["nodes"][node][0]-robot_pose.x, self.route["nodes"][node][1]-robot_pose.y
                completed_route_distance = route_length+math.sqrt(x**2+y**2) - math.sqrt(x2**2+y2**2)
            route_length += math.sqrt(x**2+y**2)
        if self.way_id>=len(self.route["way"]):
            print("End of the route. Finishing experiment.")
            self.statistics['route_completion']=1.0
        else:
            self.statistics['route_completion']=max(completed_route_distance/route_length,0.0)
        self.statistics["infraction_penalty"]=self.infraction_score
        self.statistics["driving_score"]=self.statistics['route_completion']*self.statistics['infraction_penalty']
        self.statistics["driving_time"]=self.get_time()-self.start_experiment_time
        self.statistics["route_lenght"]=route_length
        self.get_pedestrian_traj_metric()
        self.get_robot_traj_metric()
        with open(path,"w") as fp:
            json.dump(self.statistics,fp, indent="\t")
        print("Metrics saved in "+path)

    def init_pedestrian_traj(self, id, pose, speed):
        if id in self.pedestrian_traj:
            self.pedestrian_traj[id]["pose"]=pose
            self.pedestrian_traj[id]["speed"]=speed
            self.pedestrian_traj[id]["time"]=self.get_time()
        else:
            self.pedestrian_traj[id]={}
            self.pedestrian_traj[id]["pose"]=pose
            self.pedestrian_traj[id]["speed"]=speed
            self.pedestrian_traj[id]["time"]=self.get_time()
            self.pedestrian_traj[id]["min_score"]=1.0

    def pedestrian_traj_eval(self,id, pose):
        distance = math.sqrt((pose.x-self.pedestrian_traj[id]["pose"].x)**2 + 
                             (pose.y-self.pedestrian_traj[id]["pose"].y)**2)
        optimal_time=distance/self.pedestrian_traj[id]["speed"]
        real_time=self.get_time()-self.pedestrian_traj[id]["time"]
        score=optimal_time/real_time
        if score<self.pedestrian_traj[id]["min_score"]:
            self.pedestrian_traj[id]["min_score"]=score
    
    #If collision, we consider that the pedestrian could not reach its objective and the minimum score was 0.
    def get_pedestrian_traj_metric(self):
        self.statistics["pedestrian_metric_mean"]=0.0
        count=0 #Check the real number, some trajectories can be active
        for id  in self.pedestrian_traj.keys():
            if self.pedestrian_traj[id]["min_score"]<1.0:
                self.statistics["pedestrian_metric_mean"]+=self.pedestrian_traj[id]["min_score"]
                count+=1
        if count>0: 
            self.statistics["pedestrian_metric_mean"]/=(count+self.statistics["pedestrian"])
        else: self.statistics["pedestrian_metric_mean"]=1.0 if self.statistics["pedestrian"]==0 else 0

    def init_robot_traj(self, id, pedestrian_pose, pedestrian_pose2=None):
        if pedestrian_pose2 is None: pedestrian_pose2=pedestrian_pose #For apparitions

        #Direction to target, only considering x and y axis.
        robot_pose=self.player.get_location()
        node, previous =self.route["way"][self.way_id], self.route["way"][self.way_id-1]
        x, y=self.route["nodes"][node][0], self.route["nodes"][node][1]
        x2, y2=self.route["nodes"][previous][0], self.route["nodes"][previous][1]
        if(abs(x-x2)>abs(y-y2)):
            sign = 1 if x-x2>0 else -1
            target = (np.max((pedestrian_pose.x*sign,pedestrian_pose2.x*sign))+DISTANCE2PEDESTRIAN)*sign
            minimum_time=(target-robot_pose.x)*sign/MAX_SPEED
            target = {"x":target}
        else:
            sign = 1 if y-y2>0 else -1
            target = (np.max((pedestrian_pose.y*sign,pedestrian_pose2.y*sign))+DISTANCE2PEDESTRIAN)*sign
            minimum_time=(target-robot_pose.y)*sign/MAX_SPEED
            target = {"y":target}
        if minimum_time>0:
            if id in self.robot_traj:
                if not self.robot_traj[id]["active"]:
                    self.robot_traj[id]["minimum_time"]=minimum_time
                    self.robot_traj[id]["time"]=self.get_time()
                    self.robot_traj[id]["target"]=target
                    self.robot_traj[id]["active"]=True
            else:
                self.robot_traj[id]={}
                self.robot_traj[id]["time"]=self.get_time()
                self.robot_traj[id]["minimum_time"]=minimum_time
                self.robot_traj[id]["target"]=target
                self.robot_traj[id]["active"]=True
                self.robot_traj[id]["count"]=0
                self.robot_traj[id]["score"]=0.0

    def robot_traj_eval(self):
        # The scores save are RI, RI², (RI-0.5)*2, RI*~collision=RI', RI'², (RI'-0.5)*2
        robot_pose=self.player.get_location()
        for id in self.robot_traj.keys():
            if self.robot_traj[id]["active"]:
                if (("x" in self.robot_traj[id]["target"] and abs(robot_pose.x-self.robot_traj[id]["target"]["x"])<0.1) or\
                    ("y" in self.robot_traj[id]["target"] and abs(robot_pose.y-self.robot_traj[id]["target"]["y"])<0.1)) and\
                    self.get_time()-self.robot_traj[id]["time"]>0:
                        self.robot_traj[id]["active"]=False
                        self.robot_traj[id]["count"]+=1
                        score=0.0 if id in self.collided_pedestrians else self.robot_traj[id]["minimum_time"]/(self.get_time()-self.robot_traj[id]["time"])
                        self.robot_traj[id]["score"]+=score

    
    def get_robot_traj_metric(self):
        self.statistics["robot_traj_metric"]=0
        count=0 #Check the real number, some trajectories can be active
        for id in self.robot_traj.keys():
            if self.robot_traj[id]["count"]>0:
                self.statistics["robot_traj_metric"]+=self.robot_traj[id]["score"]/self.robot_traj[id]["count"]
                count+=1
        if count>0: 
            self.statistics["robot_traj_metric"]/=count
        else: self.statistics["robot_traj_metric"]=0.0

    
    def is_intersection(self, actor):
        """
        Calculate the Intersection over Union (IoU) of two bounding boxes.
        """
        def get_corners(pose, box, yaw):
            # Calculate the corners of the bounding box
            corners_x = np.array([-box.x, box.x, box.x, -box.x])
            corners_y = np.array([-box.y, -box.y, box.y, box.y])

            # Rotate the corners based on the yaw angle
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            rotated_corners_x = pose.x + corners_x * cos_yaw - corners_y * sin_yaw
            rotated_corners_y = pose.y + corners_x * sin_yaw + corners_y * cos_yaw

            return rotated_corners_x, rotated_corners_y
        
        robot_pose, actor_pose = self.player.get_transform(), actor.get_transform()
        actor_box= actor.bounding_box.extent
        
        # Get corners of both bounding boxes
        box1_corners_x, box1_corners_y = get_corners(robot_pose.location, self.robot_bounding_box, robot_pose.rotation.yaw)
        box2_corners_x, box2_corners_y = get_corners(actor_pose.location, actor_box, actor_pose.rotation.yaw)
        # Calculate the intersection area
        intersection_x = np.max((0,np.min((np.max(box1_corners_x),np.max(box2_corners_x)))-np.max((np.min(box1_corners_x), np.min(box2_corners_x)))))
        intersection_y = np.max((0,np.min((np.max(box1_corners_y),np.max(box2_corners_y)))-np.max((np.min(box1_corners_y), np.min(box2_corners_y)))))

        intersection_area = intersection_x * intersection_y
        if(intersection_area>0): return True
        return False
