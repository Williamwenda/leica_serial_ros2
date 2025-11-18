#!/usr/bin/env python3
"""
ROS2 Bag Analysis Script for Leica Position Data
Analyzes teach and repeat trajectories from /leica/position topic.

Features:
  1) Visualize 3D trajectory of teach & repeat.
  2) Visualize x, y, z vs time (3 stacked subplots).
  3) Detect moving segments automatically and compute time-aligned RMSE on moving part.

Usage:
    python3 compute_rmse.py <teach_bag_path> <repeat_bag_path>
i.e. python3 compute_rmse.py test1/teach/ test1/repeat/
"""

import sys
from pathlib import Path
import sqlite3

import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ================================================================
# Bag reading
# ================================================================
def find_rosbag_in_dir(path: Path):
    """
    Given a directory such as 'test1/teach/',
    automatically locate the actual rosbag folder or file inside.

    Accepted cases:
      - test1/teach/*.mcap
      - test1/teach/*.db3
      - test1/teach/<bagname>/*.mcap
      - test1/teach/<bagname>/*.db3

    Returns:
        Path to the bag directory containing metadata.yaml
        (the directory, NOT the .mcap/db3 file)
    """
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Directory does not exist: {path}")

    # Direct pattern: test1/teach/*.mcap or *.db3
    direct = list(path.glob("*.mcap")) + list(path.glob("*.db3"))
    if direct:
        # return the directory (parent of the bag file)
        return path

    # Nested pattern: test1/teach/<bagname>/*.mcap or *.db3
    for sub in path.iterdir():
        if sub.is_dir():
            files = list(sub.glob("*.mcap")) + list(sub.glob("*.db3"))
            if files:
                return sub

    raise FileNotFoundError(f"No rosbag (.mcap/.db3) found in {path}")

def read_mcap_bag(bag_path, topic_name="/leica/position"):
    """
    Read position data from ROS2 bag directory (supports .mcap and .db3).

    Args:
        bag_path: Path to the bag directory.
        topic_name: Topic to read (default: /leica/position).

    Returns:
        timestamps: (N,) array, float seconds.
        positions:  (N,3) array, [x, y, z].
    """
    bag_path = Path(bag_path)

    db_files = list(bag_path.glob("*.db3"))
    mcap_files = list(bag_path.glob("*.mcap"))

    timestamps = []
    positions = []

    if mcap_files:
        try:
            from mcap_ros2.reader import read_ros2_messages
        except ImportError:
            print("Error: mcap-ros2-support not installed. Install with:")
            print("       pip install mcap-ros2-support")
            sys.exit(1)

        mcap_file = mcap_files[0]
        print(f"Reading from MCAP file: {mcap_file}")

        for msg in read_ros2_messages(str(mcap_file)):
            if msg.channel.topic == topic_name:
                point_msg = msg.ros_msg
                t = point_msg.header.stamp.sec + point_msg.header.stamp.nanosec * 1e-9
                timestamps.append(t)
                positions.append([
                    point_msg.point.x,
                    point_msg.point.y,
                    point_msg.point.z,
                ])

    elif db_files:
        db_file = db_files[0]
        print(f"Reading from DB3 file: {db_file}")

        conn = sqlite3.connect(str(db_file))
        cursor = conn.cursor()

        cursor.execute("SELECT id, type FROM topics WHERE name=?", (topic_name,))
        topic_info = cursor.fetchone()
        if not topic_info:
            print(f"Error: {topic_name} not found in bag.")
            conn.close()
            return np.array([]), np.array([])

        topic_id, msg_type = topic_info
        msg_class = get_message(msg_type)

        cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id=?", (topic_id,))
        for timestamp_ns, data in cursor.fetchall():
            msg = deserialize_message(data, msg_class)
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            timestamps.append(t)
            positions.append([
                msg.point.x,
                msg.point.y,
                msg.point.z,
            ])

        conn.close()
    else:
        print(f"Error: No .db3 or .mcap file found in {bag_path}")
        sys.exit(1)

    return np.array(timestamps), np.array(positions)

# ================================================================
# Moving-segment detection 
# ================================================================

def detect_moving_sequence(positions, tol=0.005, min_plateau=20, min_end_cluster=5):
    """
    Robust detection for single-segment motion:
        static  →  moving  →  static

    START:
        - Find the longest initial plateau where the position is nearly constant.
        - Start = first index right after that plateau.

    END:
        - Use step differences v[i] = ||p[i+1] - p[i]||.
        - Find last cluster of v > threshold, cluster length >= min_end_cluster.

    Returns:
        start_idx, end_idx (end_idx exclusive)
    """
    n = len(positions)
    if n < 5:
        return 0, n

    # -------------------------
    # 1. INITIAL PLATEAU DETECTION
    # -------------------------
    p0 = positions[0]
    d0 = np.linalg.norm(positions - p0, axis=1)

    # plateau ends when distance exceeds tolerance
    plateau_idx = np.where(d0 <= tol)[0]

    # must be contiguous from index 0
    end_plateau = 0
    for i in range(len(plateau_idx)):
        if plateau_idx[i] != i:
            break
        end_plateau = i

    # plateau must be long enough
    if end_plateau < min_plateau:
        start_idx = min_plateau
    else:
        start_idx = end_plateau + 1

    # -------------------------
    # 2. END DETECTION (unchanged logic)
    # -------------------------
    v = np.linalg.norm(positions[1:] - positions[:-1], axis=1)

    # noise estimate: smallest 30%
    v_sorted = np.sort(v)
    noise = np.median(v_sorted[: max(5, len(v)//3) ])
    thr = max(noise * 5, 1e-4)

    moving = v > thr

    end_idx = n
    i = len(moving) - 1
    while i >= 0:
        if moving[i]:
            j = i
            while j >= 0 and moving[j]:
                j -= 1
            if (i - j) >= min_end_cluster:
                end_idx = i + 1
                break
            i = j
        else:
            i -= 1

    # clamp
    start_idx = max(0, min(start_idx, n - 1))
    end_idx = max(start_idx + 1, min(end_idx, n))

    return start_idx, end_idx

# ================================================================
# RMSE (time-aligned)
# ================================================================

def extract_segment(time, pos, start_idx, end_idx):
    """Return time (starting at 0) and positions for a segment."""
    seg_t = time[start_idx:end_idx] - time[start_idx]
    seg_p = pos[start_idx:end_idx]
    return seg_t, seg_p


def compute_rmse_time_synced(t1, p1, t2, p2):
    """
    Compute RMSE between two 3D trajectories, time-aligned.

    - Time axes t1, t2 start at 0 (assumed).
    - Interpolate both onto a common grid [0, min(T1, T2)].

    Returns:
        rmse_total, rmse_x, rmse_y, rmse_z
    """
    if len(t1) < 2 or len(t2) < 2:
        return np.nan, np.nan, np.nan, np.nan

    T = min(t1[-1], t2[-1])  # common duration
    # use number of samples ~ min(len1,len2) for the grid
    n_grid = min(len(t1), len(t2))
    t_common = np.linspace(0.0, T, n_grid)

    p1_interp = np.zeros((n_grid, 3))
    p2_interp = np.zeros((n_grid, 3))

    for k in range(3):
        p1_interp[:, k] = np.interp(t_common, t1, p1[:, k])
        p2_interp[:, k] = np.interp(t_common, t2, p2[:, k])

    diff = p1_interp - p2_interp
    rmse_x = np.sqrt(np.mean(diff[:, 0] ** 2))
    rmse_y = np.sqrt(np.mean(diff[:, 1] ** 2))
    rmse_z = np.sqrt(np.mean(diff[:, 2] ** 2))
    rmse   = np.sqrt(np.mean(np.sum(diff ** 2, axis=1)))
    return rmse, rmse_x, rmse_y, rmse_z

# ================================================================
# Plotting
# ================================================================

def plot_3d_trajectory(teach_pos, repeat_pos, teach_moving, repeat_moving):
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    teach_start, teach_end = teach_moving
    repeat_start, repeat_end = repeat_moving

    # full
    ax.plot(teach_pos[:, 0], teach_pos[:, 1], teach_pos[:, 2],
            linewidth=1, alpha=0.3, label="Teach (full)")
    ax.plot(repeat_pos[:, 0], repeat_pos[:, 1], repeat_pos[:, 2],
            linewidth=1, alpha=0.3, label="Repeat (full)")

    # moving
    ax.plot(teach_pos[teach_start:teach_end, 0],
            teach_pos[teach_start:teach_end, 1],
            teach_pos[teach_start:teach_end, 2],
            linewidth=2, label="Teach (moving)")
    ax.plot(repeat_pos[repeat_start:repeat_end, 0],
            repeat_pos[repeat_start:repeat_end, 1],
            repeat_pos[repeat_start:repeat_end, 2],
            linewidth=2, label="Repeat (moving)")

    # markers
    ax.scatter(*teach_pos[teach_start], s=60, marker="o", label="Teach start")
    ax.scatter(*teach_pos[teach_end - 1], s=60, marker="s", label="Teach end")
    ax.scatter(*repeat_pos[repeat_start], s=60, marker="o", label="Repeat start")
    ax.scatter(*repeat_pos[repeat_end - 1], s=60, marker="s", label="Repeat end")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("3D Trajectory Comparison")
    ax.legend()
    ax.grid(True)
    return fig


def plot_xyz_vs_time(teach_time, teach_pos, repeat_time, repeat_pos,
                     teach_moving, repeat_moving):
    fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)

    # normalize time to start from 0 (for plotting only)
    teach_time_plot = teach_time - teach_time[0]
    repeat_time_plot = repeat_time - repeat_time[0]

    teach_start, teach_end = teach_moving
    repeat_start, repeat_end = repeat_moving

    labels = ["X Position (m)", "Y Position (m)", "Z Position (m)"]

    for i, ax in enumerate(axes):
        # full
        ax.plot(teach_time_plot, teach_pos[:, i], linewidth=1, alpha=0.3, label="Teach (full)")
        ax.plot(repeat_time_plot, repeat_pos[:, i], linewidth=1, alpha=0.3, label="Repeat (full)")

        # moving
        ax.plot(teach_time_plot[teach_start:teach_end],
                teach_pos[teach_start:teach_end, i],
                linewidth=2, label="Teach (moving)")
        ax.plot(repeat_time_plot[repeat_start:repeat_end],
                repeat_pos[repeat_start:repeat_end, i],
                linewidth=2, label="Repeat (moving)")

        # markers
        ax.scatter(teach_time_plot[teach_start], teach_pos[teach_start, i], s=40, marker="o")
        ax.scatter(teach_time_plot[teach_end - 1], teach_pos[teach_end - 1, i], s=40, marker="s")
        ax.scatter(repeat_time_plot[repeat_start], repeat_pos[repeat_start, i], s=40, marker="o")
        ax.scatter(repeat_time_plot[repeat_end - 1], repeat_pos[repeat_end - 1, i], s=40, marker="s")

        ax.set_ylabel(labels[i])
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

    axes[-1].set_xlabel("Time (s)")
    axes[0].set_title("Position vs Time Comparison")
    plt.tight_layout()
    return fig

# ================================================================
# Main
# ================================================================

def main():
    # Allow optional command-line override of bag paths
    if len(sys.argv) >= 3:
        teach_bag = find_rosbag_in_dir(Path(sys.argv[1]))
        repeat_bag = find_rosbag_in_dir(Path(sys.argv[2]))
    else:
        print("Usage: compute_rmse.py <teach_bag_path> <repeat_bag_path>")
        sys.exit(1)

    print("=" * 60)
    print("ROS2 Bag Analysis: Teach vs Repeat Trajectories")
    print("=" * 60)

    # 1) read data
    print("\n1. Reading data from bags...")
    teach_time, teach_pos = read_mcap_bag(teach_bag)
    repeat_time, repeat_pos = read_mcap_bag(repeat_bag)

    print(f"   Teach:  {len(teach_pos)} points")
    print(f"   Repeat: {len(repeat_pos)} points")

    # 2) moving sequence detection
    print("\n2. Detecting moving sequences (first static run / last static run)...")
    teach_start, teach_end = detect_moving_sequence(teach_pos)
    repeat_start, repeat_end = detect_moving_sequence(repeat_pos)

    print(f"   Teach idx:  {teach_start} -> {teach_end} "
          f"({teach_end - teach_start} samples)")
    print(f"   Repeat idx: {repeat_start} -> {repeat_end} "
          f"({repeat_end - repeat_start} samples)")
    print(f"   Teach time:  {teach_time[teach_start]:.3f}s -> {teach_time[teach_end-1]:.3f}s")
    print(f"   Repeat time: {repeat_time[repeat_start]:.3f}s -> {repeat_time[repeat_end-1]:.3f}s")

    # segments (for RMSE)
    teach_t_seg, teach_p_seg = extract_segment(teach_time, teach_pos, teach_start, teach_end)
    repeat_t_seg, repeat_p_seg = extract_segment(repeat_time, repeat_pos, repeat_start, repeat_end)

    # 3) RMSE (time-aligned)
    print("\n3. Computing time-aligned RMSE on moving sequences...")
    rmse, rmse_x, rmse_y, rmse_z = compute_rmse_time_synced(
        teach_t_seg, teach_p_seg, repeat_t_seg, repeat_p_seg
    )
    print(f"   Overall RMSE: {rmse:.6f} m")
    print(f"   RMSE X:       {rmse_x:.6f} m")
    print(f"   RMSE Y:       {rmse_y:.6f} m")
    print(f"   RMSE Z:       {rmse_z:.6f} m")

    # 4) plots
    print("\n4. Creating plots...")
    plot_3d_trajectory(teach_pos, repeat_pos, (teach_start, teach_end),
                       (repeat_start, repeat_end))
    plot_xyz_vs_time(teach_time, teach_pos, repeat_time, repeat_pos,
                     (teach_start, teach_end), (repeat_start, repeat_end))

    print("\nDone. Close the plot windows to exit.")
    plt.show()


if __name__ == "__main__":
    main()
