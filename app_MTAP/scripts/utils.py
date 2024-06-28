import xml.etree.ElementTree as ET
import numpy as np
import os
os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame
import weakref
import collections
import carla
from carla import ColorConverter as cc
import math
import cv2
import re
import open3d as o3d


def ReadRouteFromXML(filename, id_way=None):
    tree= ET.parse(filename)
    root = tree.getroot()
    map={}
    map["nodes"], map["way"]={},[]
    for node in root.findall("node"):

        map["nodes"][node.get("id")] = [float(node.get("pose_x")), float(node.get("pose_y")), float(node.get("pose_z",0)), float(node.get("yaw",0))]
    
    route=root.find("way")
    for route in root.iterfind("way"):
        if id_way is None or id_way==int(route.get("id")):
            weather = int(route.get("weather", default=6))
            for child in route:
                map["way"].append(child.get("ref"))
            return map, weather
    raise Exception from None

def ReadPedestrianRoutesFromXML(filename):
    print(filename)
    tree= ET.parse(filename)
    root = tree.getroot()
    routes=[]
    #Read the pedestrian routes
    seed = root.find("random")
    seed = int(seed.get("seed"))
    root = root.find("pedestrians")
    for pedestrian in root.findall("pedestrian"):

        list = []
        nodes = {}
        for node in pedestrian.findall("node"):
            nodes[node.get("id")] = carla.Transform(carla.Location(float(node.get("pose_x")), float(node.get("pose_y")), 
                                                    float(node.get("pose_z",0))), carla.Rotation(0,0,0))
        way = pedestrian.find("way")
        for child in way:
            s = "False" != child.get("stop")
            list.append({"pos":nodes[child.get("ref")], "stop":s})
        routes.append(list)

    #Read the apparition routes. The apparition is a special type of pedestrian that will be spawn in front of the
    #robot when it is close to its defined position.
    apparitions=[]
    for apparition in root.findall("apparition"):
        node = apparition.find("node")
        node = carla.Transform(carla.Location(float(node.get("pose_x")), float(node.get("pose_y")), 
                                                    float(node.get("pose_z",0))), carla.Rotation(0,0,0))
        apparitions.append(node)

    return seed, routes, apparitions

# ==============================================================================
# -- FadingText ----------------------------------------------------------------
# ==============================================================================


class FadingText(object):
    def __init__(self, font, dim, pos):
        self.font = font
        self.dim = dim
        self.pos = pos
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)

    def set_text(self, text, color=(255, 255, 255), seconds=2.0):
        text_texture = self.font.render(text, True, color)
        self.surface = pygame.Surface(self.dim)
        self.seconds_left = seconds
        self.surface.fill((0, 0, 0, 0))
        self.surface.blit(text_texture, (10, 11))

    def tick(self, _, clock):
        delta_seconds = 1e-3 * clock.get_time()
        self.seconds_left = max(0.0, self.seconds_left - delta_seconds)
        self.surface.set_alpha(500.0 * self.seconds_left)

    def render(self, display):
        display.blit(self.surface, self.pos)




# ==============================================================================
# -- HelpText ------------------------------------------------------------------
# ==============================================================================


class HelpText(object):
    """Helper class to handle text output using pygame"""
    def __init__(self, font,doc, width, height):
        lines = doc.split('\n')
        self.font = font
        self.line_space = 18
        self.dim = (780, len(lines) * self.line_space + 12)
        self.pos = (0.5 * width - 0.5 * self.dim[0], 0.5 * height - 0.5 * self.dim[1])
        self.seconds_left = 0
        self.surface = pygame.Surface(self.dim)
        self.surface.fill((0, 0, 0, 0))
        for n, line in enumerate(lines):
            text_texture = self.font.render(line, True, (255, 255, 255))
            self.surface.blit(text_texture, (22, n * self.line_space))
            self._render = False
        self.surface.set_alpha(220)

    def toggle(self):
        self._render = not self._render

    def render(self, display):
        if self._render:
            display.blit(self.surface, self.pos)


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================


class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.collision_rumble=False
        # self.collision_event = 0 #0:not collision 1:collision with static obstacle 2:collision with pedestrian
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        # if actor_type.find("Pedestrian")!=-1:
        #     self.collision_event=2
        # else:
        #     self.collision_event=1
        self.hud.notification('Collision with %r' % actor_type)
        self.collision_rumble=True
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


class CollisionEvaluator(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        weak_self = weakref.ref(self)
        self.collision_event = 0 #0:not collision 1:collision with static obstacle 2:collision with pedestrian
        self.other_actor_id = None
        self.sensor.listen(lambda event: CollisionEvaluator._on_collision(weak_self, event))

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        if actor_type.find("Pedestrian")!=-1:
            self.collision_event=2
            self.other_actor_id=event.other_actor.id
        else:
            self.collision_event=1


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================


class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # self.lane_invasion_event=False
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return

        lane_types = set(x.type for x in event.crossed_lane_markings)
        # for x in lane_types:
        #     if str(x) == "Other": return
        # self.lane_invasion_event = True
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))

class LaneInvasionEvaluator(object):
    def __init__(self, sensor):
        self.sensor = sensor
        self.lane_invasion_event=False
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: LaneInvasionEvaluator._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return

        lane_types = set(x.type for x in event.crossed_lane_markings)
        for x in lane_types:
            if str(x) == "Other": return
        self.lane_invasion_event = True


# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================


class GnssSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.0, z=2.8)), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude

class GnssSensorLink(object):
    def __init__(self, sensor):
        self.sensor = sensor
        self.lat = 0.0
        self.lon = 0.0
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)


# ==============================================================================
# -- RGB camera 360 ------------------------------------------------------------
# ==============================================================================


class RGBcamera (object):
    def __init__(self, parent_actor,hud=None):
        self.sensors = []
        self.hud=hud
        self._parent = parent_actor
        self._image = np.zeros([2048,128,3])
        world = self._parent.get_world()
        self._raw = None
        self.init = [False, False, False, False]
        for id in range(4):
            bp = world.get_blueprint_library().find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', "512")
            bp.set_attribute('image_size_y', "128")
            bp.set_attribute('fov', "90")
            bp.set_attribute('sensor_tick', "0.1")
            self.sensors.append(world.spawn_actor(bp, carla.Transform(carla.Location(x=0.17, z=1.16), carla.Rotation(yaw=-135+id*90)), 
                    attach_to=self._parent, attachment_type=carla.AttachmentType.Rigid))
            callback=Callback360img(self,id,cc.Raw)
            self.sensors[id].listen(callback)
        
    def render(self,display):
        array=cv2.resize(self._image, dsize=(128,self.hud.dim[0]))
        surface = pygame.surfarray.make_surface(array)
        display.blit(surface, (0, self.hud.dim[1]))

    def destroy(self):
        for sensor in self.sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()

# ==============================================================================
# -- Depth camera 360 ----------------------------------------------------------
# ==============================================================================


class Depthcamera (object):
    def __init__(self, parent_actor,hud=None):
        self.sensors = []
        self.hud=hud
        self._parent = parent_actor
        self._image = np.zeros([2048,128,3])
        self._raw = np.zeros([2048,128],np.float32)
        world = self._parent.get_world()
        self.init = [False, False, False, False]
        for id in range(4):
            bp = world.get_blueprint_library().find('sensor.camera.depth')
            bp.set_attribute('image_size_x', "512")
            bp.set_attribute('image_size_y', "128")
            bp.set_attribute('fov', "90")
            bp.set_attribute('sensor_tick', "0.1")
            self.sensors.append(world.spawn_actor(bp, carla.Transform(carla.Location(x=0.17, z=1.16), carla.Rotation(yaw=-135+id*90)), 
                    attach_to=self._parent, attachment_type=carla.AttachmentType.Rigid))
            callback=Callback360img(self,id,cc.LogarithmicDepth)
            self.sensors[id].listen(callback)
        
    def render(self,display):
        array=cv2.resize(self._image, dsize=(128,self.hud.dim[0]))
        surface = pygame.surfarray.make_surface(array)
        display.blit(surface, (0, self.hud.dim[1]))

    def destroy(self):
        for sensor in self.sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()

# ==============================================================================
# -- Semantic camera 360 -------------------------------------------------------
# ==============================================================================

class SemanticCamera (object):
    def __init__(self, parent_actor):
        self.sensors = []
        self._parent = parent_actor
        self._image = np.zeros([2048,128,3])
        world = self._parent.get_world()
        self._raw = None
        self.init = [False, False, False, False]
        for id in range(4):
            bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            bp.set_attribute('image_size_x', "512")
            bp.set_attribute('image_size_y', "128")
            bp.set_attribute('fov', "90")
            bp.set_attribute('sensor_tick', "0.1")
            self.sensors.append(world.spawn_actor(bp, carla.Transform(carla.Location(x=0.17, z=1.16), carla.Rotation(yaw=-135+id*90)), 
                    attach_to=self._parent, attachment_type=carla.AttachmentType.Rigid))
            callback=Callback360img(self,id,cc.Raw)
            self.sensors[id].listen(callback)

    def destroy(self):
        for sensor in self.sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()

# ==============================================================================
# -- Callback for 360 images 
# ==============================================================================


class Callback360img(object):
    def __init__(self, parent, id, transform):
        self.parent=parent
        self.id=id
        self.transform=transform

    
    def __call__(self, image):
        self.parent.init[self.id]=True
        if self.parent._raw is not None:
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width,4))
            array = array[:, :, :3]
            array = array[:, :, ::-1].astype(np.float32)
            normalized = (array[:,:,0] + array[:,:,1] * 256 + array[:,:,2] * 256 * 256) / float(256 * 256 * 256 - 1)
            # in_meters = 1000.0 * normalized
            self.parent._raw[512*self.id:512*(self.id+1)]=normalized.swapaxes(0, 1)
        image.convert(self.transform)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.parent._image[512*self.id:512*(self.id+1)]=array.swapaxes(0, 1)

# ==============================================================================
# -- LiDAR ------------------------------------------------------------
# ==============================================================================


class LidarSensor (object):
    def __init__(self, parent_actor,hud=None):
        self.sensor = None
        self.hud=hud
        self._parent = parent_actor
        self._pointcloud = None
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        bp.set_attribute('channels', "128")
        bp.set_attribute('range', "50")
        bp.set_attribute('upper_fov', "22.5")
        bp.set_attribute('lower_fov', "-22.5")
        bp.set_attribute('sensor_tick', "0.1")
        bp.set_attribute('points_per_second', "2621440")
        self.sensor = world.spawn_actor(bp, carla.Transform(carla.Location(x=0.17, z=1.16)), 
                attach_to=self._parent, attachment_type=carla.AttachmentType.Rigid)
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda lidar_data: LidarSensor._Lidar_callback(weak_self, lidar_data))
        
    def render(self,display):
        lidar_data = np.array(self._pointcloud[:, :2])
        lidar_data *= min(self.hud.dim) / (2.0 * 50.0) #50 = lidar_range
        lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
        surface = pygame.surfarray.make_surface(lidar_img)
        display.blit(surface, (0, 0))

    # def voxelize(self, voxel_size=0.25):
    #     points = self._pointcloud[:,:3]
    #     pcd = o3d.geometry.PointCloud()
    #     pcd.points = o3d.utility.Vector3dVector(points)
    #     voxel_grid=o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size)

    #     # Extract the voxel centroids
    #     voxel_centroids = np.asarray([voxel.grid_index for voxel in voxel_grid.get_voxels()])

    #     # Convert the voxel centroids to the original coordinates
    #     voxel_centroids = voxel_centroids * voxel_size + voxel_grid.origin
    #     self._pointcloud=voxel_centroids
    
    @staticmethod
    def _Lidar_callback(weak_self, lidar_data):
        self = weak_self()
        self._pointcloud = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
        self._pointcloud = np.reshape(self._pointcloud, (int(self._pointcloud.shape[0] / 4), 4))
        # self.voxelize()
        

class LidarSensorLink (object):
    def __init__(self, sensor):
        self.sensor = sensor
        self._pointcloud = None
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda lidar_data: LidarSensor._Lidar_callback(weak_self, lidar_data))



# ==============================================================================
# -- RadarSensor ---------------------------------------------------------------
# ==============================================================================


class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor._Radar_callback(weak_self, radar_data))

    @staticmethod
    def _Radar_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        # points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        # points = np.reshape(points, (len(radar_data), 4))

        current_rot = radar_data.transform.rotation
        for detect in radar_data:
            azi = math.degrees(detect.azimuth)
            alt = math.degrees(detect.altitude)
            # The 0.25 adjusts a bit the distance so the dots can
            # be properly seen
            fw_vec = carla.Vector3D(x=detect.depth - 0.25)
            carla.Transform(
                carla.Location(),
                carla.Rotation(
                    pitch=current_rot.pitch + alt,
                    yaw=current_rot.yaw + azi,
                    roll=current_rot.roll)).transform(fw_vec)

            def clamp(min_v, max_v, value):
                return max(min_v, min(value, max_v))

            norm_velocity = detect.velocity / self.velocity_range # range [-1, 1]
            r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
            g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
            b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
            self.debug.draw_point(
                radar_data.transform.location + fw_vec,
                size=0.075,
                life_time=0.06,
                persistent_lines=False,
                color=carla.Color(r, g, b))

# ==============================================================================
# -- CameraManager -------------------------------------------------------------
# ==============================================================================


class CameraManager(object):
    def __init__(self, parent_actor, hud, gamma_correction):
        self.sensor = None
        self.surface = None
        self._parent = parent_actor
        self.hud = hud
        self.recording = False
        self._image=None
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z
        Attachment = carla.AttachmentType

        if not self._parent.type_id.startswith("walker.pedestrian"):
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.0*bound_x, y=+0.0*bound_y, z=2.0*bound_z), carla.Rotation(pitch=8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=+0.8*bound_x, y=+0.0*bound_y, z=1.3*bound_z)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=+1.9*bound_x, y=+1.0*bound_y, z=1.2*bound_z)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-2.8*bound_x, y=+0.0*bound_y, z=4.6*bound_z), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-1.0, y=-1.0*bound_y, z=0.4*bound_z)), Attachment.Rigid)]
        else:
            self._camera_transforms = [
                (carla.Transform(carla.Location(x=-2.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
                (carla.Transform(carla.Location(x=2.5, y=0.5, z=0.0), carla.Rotation(pitch=-8.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=-4.0, z=2.0), carla.Rotation(pitch=6.0)), Attachment.SpringArmGhost),
                (carla.Transform(carla.Location(x=0, y=-2.5, z=-0.0), carla.Rotation(yaw=90.0)), Attachment.Rigid)]

        self.transform_index = 1
        self.sensors = [
            ['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
            ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
            ['sensor.camera.semantic_segmentation', cc.CityScapesPalette, 'Camera Semantic Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.instance_segmentation', cc.CityScapesPalette, 'Camera Instance Segmentation (CityScapes Palette)', {}],
            ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
        ]
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(hud.dim[0]))
                bp.set_attribute('image_size_y', str(hud.dim[1]))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, notify=False, force_respawn=True)

    def set_sensor(self, index, notify=True, force_respawn=False):
        index = index % len(self.sensors) if index>=0 else len(self.sensors)-1
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        self._image=None
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: CameraManager._parse_image(weak_self, image))
        if notify:
            self.hud.notification(self.sensors[index][2])
        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def previous_sensor(self):
        self.set_sensor(self.index - 1)

    def toggle_recording(self):
        self.recording = not self.recording
        self.hud.notification('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        if self.surface is not None:
            display.blit(self.surface, (0, 0))

    @staticmethod
    def _parse_image(weak_self, image):
        self = weak_self()
        if not self:
            return
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min(self.hud.dim) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.hud.dim[0], 0.5 * self.hud.dim[1])
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.hud.dim[0], self.hud.dim[1], 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            self._image=lidar_img
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            self._image=dvs_img.swapaxes(0, 1)
        elif self.sensors[self.index][0].startswith('sensor.camera.optical_flow'):
            image = image.get_color_coded_flow()
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._image=array.swapaxes(0, 1)
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            self._image=array.swapaxes(0, 1)
        self.surface=pygame.surfarray.make_surface(self._image)
        if self.recording:
            image.save_to_disk('_out/%08d' % image.frame)

# ==============================================================================
# -- PID Controller ------------------------------------------------------------
# ==============================================================================

class PID_Controller(object):
    def __init__(self, kp, ki, kd, min_value, max_value):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_value, self.min_value = max_value, min_value
        self.de, self.ie = 0, 0
        self.previous = 0

    def step(self, target_speed, current_speed, dt):
        error = target_speed - current_speed

        self.de = (self.previous-error) / dt

        #Reset integral part when the target speed changes significally
        self.ie = min(max(self.ie+error * dt,-1.0),1.0) if abs(error)<0.25 else 0.0

        self.previous=error
        return np.clip(self.kp * error + self.ki*self.ie + self.kd*self.de, self.min_value, self.max_value)
    
# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================


def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

def get_actor_blueprints(world, generation, filter="blue"):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2, 3]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []
        