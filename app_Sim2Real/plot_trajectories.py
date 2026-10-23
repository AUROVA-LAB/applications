"""Create an animation of three trajectories from rosbag or text files."""

import argparse
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation

from scripts.trajectory_evaluation import extract_trajectory


def load_positions(file_path, odom_topic):
	"""Read one trajectory and return relative timestamps and XY positions."""
	timestamps, poses, _, _, _ = extract_trajectory(file_path, odom_topic)
	positions = np.asarray(poses["position"], dtype=float)
	if len(timestamps) == 0 or len(positions) == 0:
		raise ValueError(f"Trajectory is empty: {file_path}")
	return np.asarray(timestamps, dtype=float), positions[:, :2]


def plot_trajectories(file_paths, output_path, odom_topic, labels, fps):
	"""Save an XY animation, keeping each trajectory on its own time base."""
	trajectories = [load_positions(file_path, odom_topic) for file_path in file_paths]
	durations = [timestamps[-1] for timestamps, _ in trajectories]
	video_duration = max(durations)
	frame_times = np.linspace(0.0, video_duration, int(video_duration + 1))


	all_positions = np.vstack([positions for _, positions in trajectories])
	y_min, x_min = np.min(all_positions, axis=0)
	y_max, tmp = np.max(all_positions, axis=0)
	x_max=x_min*-1; x_min=tmp*-1
	span = max(x_max - x_min, y_max - y_min, 1.0)
	padding = span * 0.05

	fig, ax = plt.subplots(figsize=(10, 8))
	ax.set_xlim(x_min - padding, x_max + padding)
	ax.set_ylim(y_min - padding, y_max + padding)
	ax.set_aspect("equal", adjustable="box")
	ax.set_xlabel("X position (m)")
	ax.set_ylabel("Y position (m)")
	ax.grid(True, alpha=0.3)
	ax.set_title("Trajectory comparison")
	fig.subplots_adjust(left=0.08, right=0.98, bottom=0.08, top=0.92)

	colors = ("tab:blue", "tab:orange", "tab:green")
	lines = []
	markers = []
	for label, color in zip(labels, colors):
		line, = ax.plot([], [], color=color, linewidth=2, label=label)
		marker, = ax.plot([], [], "o", color=color, markersize=7)
		lines.append(line)
		markers.append(marker)
	ax.legend()
	time_text = ax.text(0.02, 0.97, "", transform=ax.transAxes, va="top")

	def update(frame_time):
		for (timestamps, positions), line, marker in zip(trajectories, lines, markers):
			# Each file has a different sample count and duration.
			end_index = np.searchsorted(timestamps, frame_time, side="right")
			end_index = max(1, min(end_index, len(positions)))
			visible_positions = positions[:end_index]
			line.set_data(-visible_positions[:, 1], visible_positions[:, 0])
			marker.set_data(-visible_positions[-1, 1], visible_positions[-1, 0])
		time_text.set_text(f"Time: {frame_time:.2f} s")
		return (*lines, *markers, time_text)

	animation = FuncAnimation(
		fig,
		update,
		frames=frame_times,
		interval=1000 / fps,
		blit=True,
		repeat=False,
	)
	output_path = Path(output_path)
	output_path.parent.mkdir(parents=True, exist_ok=True)
	animation.save(output_path, writer=FFMpegWriter(fps=fps))
	plt.close(fig)


def main():
	parser = argparse.ArgumentParser(description="Plot three trajectories into an MP4 video")
	parser.add_argument("trajectories", nargs=3, help="Three rosbag or txt trajectory files")
	parser.add_argument("--labels", nargs=3, default=None, help="Legend labels for the trajectories")
	parser.add_argument("--odom_topic", default="/localization", help="Odometry topic name")
	parser.add_argument("--output", default="trajectories.mp4", help="Output MP4 path")
	parser.add_argument("--fps", type=int, default=30, help="Video frames per second")
	args = parser.parse_args()

	if args.fps <= 0:
		parser.error("--fps must be greater than zero")
	labels = args.labels or [Path(file_path).stem for file_path in args.trajectories]
	plot_trajectories(args.trajectories, args.output, args.odom_topic, labels, args.fps)
	print(f"Saved trajectory video to {os.path.abspath(args.output)}")


if __name__ == "__main__":
	main()
