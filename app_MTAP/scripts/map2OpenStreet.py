#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example script to generate traffic in the simulation"""

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

import carla

from carla import VehicleLightState as vls

import argparse
import logging
from utils import *
from numpy import random

def get_actor_blueprints(world, filter, generation):
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
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

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
        '-m', '--map',
        metavar='W',
        default="routes/Town04.xml",
        type=str)
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='Port to communicate with TM (default: 8000)')

    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    synchronous_master = False

    world = client.get_world()

    settings = world.get_settings()
    map,_ = ReadRouteFromXML(args.map)
    output = open(args.map.split(".")[0]+".osm","w")
    carla_map = world.get_map()

    output.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"+
        "<osm version=\"0.6\" generator=\"CGImap 0.7.5 (16168 thorn-03.openstreetmap.org)\" copyright=\"OpenStreetMap and contributors\""+
        " attribution=\"http://www.openstreetmap.org/copyright\" license=\"http://opendatacommons.org/licenses/odbl/1-0/\">\n"+
        " <bounds minlat=\"-0.003\" minlon=\"-0.0001\" maxlat=\"0.0001\" maxlon=\"0.004\"/>\n\n")
    for id, node in map["nodes"].items():
        location=carla.Location(node[0],node[1],node[2])
        geolocation=carla_map.transform_to_geolocation(location)
        output.write(" <node id=\"{}\" lat=\"{}\" lon=\"{}\" />\n".format(id,geolocation.latitude,geolocation.longitude))
    output.write("\n <way id=\"0\">\n")
    for node in map["way"]:
        output.write("  <nd ref=\"{}\"/>\n".format(node))
    output.write(" </way>\n\n</osm>")
    output.close()

if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
