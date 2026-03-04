import rosbag
import sys

if len(sys.argv) != 5:
    print(f"Usage: python crop_rosbag.py input.bag output.bag start_time end_time")
    print("start_time and end_time are in seconds, relative to the first message")
    sys.exit(1)

input_bag = sys.argv[1]
output_bag = sys.argv[2]
start_rel = float(sys.argv[3])
end_rel = float(sys.argv[4])

with rosbag.Bag(input_bag, 'r') as bag:
    first_time = None
    # Find first timestamp
    for _, _, t in bag.read_messages():
        first_time = t.to_sec()
        break

    if first_time is None:
        print("Bag is empty.")
        sys.exit(1)

    bar_length = 40
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in bag.read_messages():
            rel_time = t.to_sec() - first_time
            if start_rel <= rel_time <= end_rel:
                outbag.write(topic, msg, t)
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