import argparse
import os
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
from sensor_msgs.msg import Image
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

def compress_rosbag(input_path, output_path):
    """Compress a single rosbag by saving only specific topics."""
    print(f"Compressing rosbag: {input_path} to {output_path}")
    typestore = get_typestore(Stores.ROS2_JAZZY)
    with AnyReader([Path(input_path)], default_typestore=typestore) as bag, Writer(output_path, version=Writer.VERSION_LATEST) as output_bag:
        conn_loc=output_bag.add_connection(args.pose_topic, "nav_msgs/msg/Odometry", typestore=typestore)
        conn_status=output_bag.add_connection(args.pose_topic, "std_msgs/msg/Float32MultiArray", typestore=typestore)
        for connection, timestamp, rawdata in bag.messages():
            if connection.topic=="/localization": output_bag.write(conn_loc, timestamp,rawdata)
            if connection.topic=="/CLEAR_status": output_bag.write(conn_status, timestamp,rawdata)
    output_bag.close()
    bag.close()
    print(f"Compressed rosbag saved to {output_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Save only the localization topic and the CLEAR status')

    parser.add_argument('--rosbag', type=str, default=None,
                        help='Rosbag file (compress one)')
    parser.add_argument('--dir', type=str, default=None,
                        help='Directory with rosbag files (compress all)')

    args = parser.parse_args()

    # Read rosbag
    if args.rosbag is None and args.dir is None:
        raise ValueError("Please provide either a rosbag file or a directory with rosbag files.")
    if args.rosbag is not None:
        output_bag_path = args.rosbag + '_compressed'
        if not os.path.exists(output_bag_path):
            compress_rosbag(args.rosbag, output_bag_path)
    else:
        with ThreadPoolExecutor() as executor:
            futures = []
            for filename in os.listdir(args.dir):
                output_name = os.path.join(args.dir, filename+ '_compressed')
                if not filename.endswith('_compressed') and not os.path.exists(output_name):
                    input_path = os.path.join(args.dir, filename)
                    futures.append(executor.submit(compress_rosbag, input_path, output_name))
             # Wait for all threads to complete
            for future in futures:
                future.result()