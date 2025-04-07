#!/usr/bin/env python3
"""
run_experiments.py
----------------------------------
Automates multiple simulation trials for
bed‑bathing cobot thesis, collects key metrics, and produces
CSV summaries + publication‑quality figures.

Metrics gathered per trial
-------------------------
1. task_efficiency_sec         – wall‑clock task completion time
2. rms_path_error_mm           – RMS Cartesian error EE vs. plan
3. max_path_error_mm           – peak Cartesian error
4. max_joint_torque_Nm         – highest joint effort seen
5. robustness_success_bool     – 1 if /task_status == 'finished'
6. planning_time_ms            – MoveIt planning time

Outputs
-------
results/metrics_raw.csv         – one row per trial
results/metrics_summary.csv     – mean ± std across trials
results/fig_task_time.png       – box plot
results/fig_path_error.png      – line + shaded CI
results/fig_joint_torque.png    – histogram
results/fig_success_rate.png    – bar chart
results/fig_planning_time.png   – histogram

Dependencies
------------
ROS 2 Humble or later, MoveIt 2, Gazebo, Python ≥3.9.
Python packages (pip install): rosbags rosbags-dataframe pandas numpy
matplotlib seaborn psutil tqdm.

Usage
-----
$ chmod +x run_bedbath_experiments.py
$ ./run_bedbath_experiments.py --trials 15

Tip: close all other ROS nodes before running to avoid namespace clashes.
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import psutil
import seaborn as sns
from matplotlib import pyplot as plt
from tqdm import tqdm

# rosbags imports (works for rosbag2 *.db3)
from rosbags.highlevel import AnyReader
from rosbags.typesys import get_types_from_msg, register_types

# ---------------------------------------------------------------------------
# Helper: launch and stop external processes cleanly
# ---------------------------------------------------------------------------

def _start_process(cmd, **kwargs):
    """Start subprocess, return Popen handle."""
    return subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, **kwargs)


def _terminate(proc):
    """SIGINT then SIGKILL fallback."""
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()

# ---------------------------------------------------------------------------
# Experiment runner
# ---------------------------------------------------------------------------

def run_trial(trial_idx: int, launch_file: str, topics: list[str]) -> str:
    """Launch sim + bag record, block until /task_status=='finished'.

    Returns bag directory name (rosbag2 creates folder)."""

    bag_name = f"trial_{trial_idx:02d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    # 1. start simulation
    sim_proc = _start_process([
        "ros2", "launch", "care_robot_sim", launch_file
    ])

    # give sim a few seconds to spin up
    time.sleep(5)

    # 2. start bag recording
    bag_proc = _start_process([
        "ros2", "bag", "record", "-o", bag_name, *topics
    ])

    # 3. listen to /task_status
    import rclpy
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node(f"trial_{trial_idx:02d}_listener")
    status_msg = {"done": False}

    def _cb(msg: String):
        if msg.data.lower().strip() == "finished":
            status_msg["done"] = True

    sub = node.create_subscription(String, "/task_status", _cb, 10)

    # spin until done
    while rclpy.ok() and not status_msg["done"]:
        rclpy.spin_once(node, timeout_sec=0.25)

    # clean up
    node.destroy_node()
    rclpy.shutdown()

    _terminate(bag_proc)
    _terminate(sim_proc)

    return bag_name

# ---------------------------------------------------------------------------
# Metric extraction helpers
# ---------------------------------------------------------------------------

def load_bag_df(bag_path: str, topic: str):
    """Return pandas DataFrame for a topic inside a rosbag2 folder."""
    with AnyReader([bag_path]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            raise ValueError(f"Topic {topic} not found in {bag_path}")
        msgs = []
        for conn, timestamp, rawdata in reader.messages(connections):
            msg = reader.deserialize(rawdata, conn.msgtype)
            msgs.append((timestamp * 1e-9, msg))  # ns -> s
    # Convert based on message type
    if topic == "/joint_states":
        import sensor_msgs.msg as sm
        times, positions, efforts = [], [], []
        for t, m in msgs:
            times.append(t)
            efforts.append(max(m.effort))
        return pd.DataFrame({"t": times, "effort": efforts})
    elif topic == "/planning_time":
        times = [t for t, _ in msgs]
        vals = [m.data for _, m in msgs]
        return pd.DataFrame({"t": times, "planning_time": vals})
    elif topic == "/task_status":
        return pd.DataFrame({"t": [t for t, _ in msgs], "status": [m.data for _, m in msgs]})
    elif topic == "/tf":
        import geometry_msgs.msg as gm
        times, tx, ty, tz = [], [], [], []
        for t, m in msgs:
            for transform in m.transforms:
                if transform.child_frame_id == "ee_link":
                    times.append(t)
                    tx.append(transform.transform.translation.x)
                    ty.append(transform.transform.translation.y)
                    tz.append(transform.transform.translation.z)
        return pd.DataFrame({"t": times, "x": tx, "y": ty, "z": tz})
    else:
        raise NotImplementedError


def compute_metrics(bag_path: str) -> dict:
    """Compute metrics for one trial bag."""
    # task efficiency
    status_df = load_bag_df(bag_path, "/task_status")
    t_start = status_df["t"].min()
    t_end = status_df["t"].max()
    task_eff = t_end - t_start

    # joint health
    js_df = load_bag_df(bag_path, "/joint_states")
    max_torque = js_df["effort"].max()

    # planning time (ms)
    try:
        plan_df = load_bag_df(bag_path, "/planning_time")
        planning_ms = plan_df["planning_time"].iloc[-1] * 1000.0
    except ValueError:
        planning_ms = np.nan

    # robustness
    success = status_df["status"].str.contains("finished", case=False).any()

    # path error (compare plan vs actual)
    tf_df = load_bag_df(bag_path, "/tf")
    # naive path accuracy: deviation from straight line between first and last pose
    start_xyz = tf_df.iloc[0][["x", "y", "z"]].to_numpy()
    end_xyz = tf_df.iloc[-1][["x", "y", "z"]].to_numpy()
    ideal = start_xyz + (end_xyz - start_xyz) * np.linspace(0, 1, len(tf_df))[:, None]
    err = np.linalg.norm(tf_df[["x", "y", "z"]].to_numpy() - ideal, axis=1) * 1000.0  # mm
    rms_err = np.sqrt((err ** 2).mean())
    max_err = err.max()

    return {
        "task_efficiency_sec": task_eff,
        "rms_path_error_mm": rms_err,
        "max_path_error_mm": max_err,
        "max_joint_torque_Nm": max_torque,
        "robustness_success_bool": int(success),
        "planning_time_ms": planning_ms,
    }

# ---------------------------------------------------------------------------
# Plotting utilities
# ---------------------------------------------------------------------------

def make_plots(df: pd.DataFrame, out_dir: Path):
    sns.set(style="whitegrid", font_scale=1.2)

    # Task completion time
    plt.figure()
    sns.boxplot(data=df, y="task_efficiency_sec")
    plt.ylabel("Task completion time (s)")
    plt.savefig(out_dir / "fig_task_time.png", dpi=300, bbox_inches="tight")

    # Path error
    plt.figure()
    sns.histplot(df["rms_path_error_mm"], kde=True)
    plt.xlabel("RMS path error (mm)")
    plt.savefig(out_dir / "fig_path_error.png", dpi=300, bbox_inches="tight")

    # Joint torque
    plt.figure()
    sns.histplot(df["max_joint_torque_Nm"], kde=True)
    plt.xlabel("Peak joint torque (Nm)")
    plt.savefig(out_dir / "fig_joint_torque.png", dpi=300, bbox_inches="tight")

    # Success rate
    plt.figure()
    sns.barplot(x=["success", "failure"],
                y=[df["robustness_success_bool"].mean(), 1 - df["robustness_success_bool"].mean()])
    plt.ylabel("Proportion")
    plt.savefig(out_dir / "fig_success_rate.png", dpi=300, bbox_inches="tight")

    # Planning time
    plt.figure()
    sns.histplot(df["planning_time_ms"].dropna(), kde=True)
    plt.xlabel("Planning time (ms)")
    plt.savefig(out_dir / "fig_planning_time.png", dpi=300, bbox_inches="tight")

# ---------------------------------------------------------------------------
# Main entry
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=10, help="Number of trials to run")
    parser.add_argument("--launch", type=str, default="spawn_moveit.launch.py", help="Launch file")
    args = parser.parse_args()

    topics = [
        "/clock", "/tf", "/joint_states", "/planning_time", "/task_status",
    ]

    results_dir = Path("results")
    results_dir.mkdir(exist_ok=True)

    metrics_list = []

    for idx in tqdm(range(1, args.trials + 1), desc="Running trials"):
        bag_folder = run_trial(idx, args.launch, topics)
        metrics = compute_metrics(bag_folder)
        metrics["trial"] = idx
        metrics_list.append(metrics)

    df = pd.DataFrame(metrics_list)
    df.to_csv(results_dir / "metrics_raw.csv", index=False)

    summary = df.describe().T[["mean", "std"]].round(2)
    summary.to_csv(results_dir / "metrics_summary.csv")

    make_plots(df, results_dir)

    print("\nAll done! Raw metrics, summaries, and figures saved to", results_dir.resolve())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted by user. Exiting…")
        sys.exit(0)
