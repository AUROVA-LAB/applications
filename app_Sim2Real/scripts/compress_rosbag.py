import argparse
import os
import numpy as np
import rosbag
from sensor_msgs.msg import Image
from concurrent.futures import ThreadPoolExecutor

def compress_rosbag(input_path, output_path):
    """Compress a single rosbag by saving only specific topics."""
    print(f"Compressing rosbag: {input_path} to {output_path}")
    bag = rosbag.Bag(input_path)
    output_bag = rosbag.Bag(output_path, "w")
    for topic, msg, t in bag.read_messages(topics=["/localization", "/CLEAR_status"]):
        output_bag.write(topic, msg, t)
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
        output_bag_path = args.rosbag.replace('.bag', '_compressed.bag')
        compress_rosbag(args.rosbag, output_bag_path)
    else:
        with ThreadPoolExecutor() as executor:
            futures = []
            for filename in os.listdir(args.dir):
                output_name = os.path.join(args.dir, filename.replace('.bag', '_compressed.bag'))
                if filename.endswith('.bag') and not filename.endswith('_compressed.bag') and not os.path.exists(output_name):
                    input_path = os.path.join(args.dir, filename)
                    futures.append(executor.submit(compress_rosbag, input_path, output_name))
             # Wait for all threads to complete
            for future in futures:
                future.result()