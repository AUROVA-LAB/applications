from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore
import sys
from pathlib import Path

if len(sys.argv) != 5:
    print(f"Usage: python crop_rosbag.py input.bag output.bag start_time end_time")
    print("start_time and end_time are in seconds, relative to the first message")
    sys.exit(1)

input_bag = sys.argv[1]
output_bag = sys.argv[2]
start_rel = float(sys.argv[3])
end_rel = float(sys.argv[4])
typestore = get_typestore(Stores.ROS2_JAZZY)
with AnyReader([Path(input_bag)], default_typestore=typestore) as bag:
    first_time = None
    # Find first timestamp
    for connection, timestamp, rawdata in bag.messages():
        first_time = timestamp/10**9
        break

    if first_time is None:
        print("Bag is empty.")
        sys.exit(1)

    bar_length = 40
    with Writer(output_bag, version=Writer.VERSION_LATEST) as outbag:
        conn_map={}
        for conn in bag.connections:
            conn_map[conn.id] = outbag.add_connection(conn.topic,conn.msgtype,typestore=typestore)
        for connection, timestamp, rawdata in bag.messages():
            rel_time = timestamp/10**9 - first_time
            if start_rel <= rel_time <= end_rel:
                outbag.write(conn_map[connection.id],timestamp,rawdata)
                progress = (rel_time - start_rel) / (end_rel - start_rel)
                filled_length = int(bar_length * progress)
                bar = '=' * filled_length + '-' * (bar_length - filled_length)
                print(f"\rProgress: [{bar}] {progress*100:.1f}% ({rel_time:.2f}s)", end='', flush=True)
            elif rel_time > end_rel:
                break
            else:
                progress = 0
                filled_length = int(bar_length * progress)
                bar = '=' * filled_length + '-' * (bar_length - filled_length)
                print(f"\rProgress: [{bar}] {progress*100:.1f}% ({rel_time:.2f}s)", end='', flush=True)
        print()  # Move to next line after progress bar
        print(f"Cropped bag saved to {output_bag}")