#!/usr/bin/env python3
"""Records a rosbag of every topic in ARCHITECTURE.md's topic contract table,
for post-run debugging or lap-time/track-position analysis. Thin subprocess
wrapper around `ros2 bag record` rather than a rosbag2 API client, since the
CLI already handles storage-plugin selection and SIGINT-based clean shutdown
correctly.
"""
import argparse
import datetime
import subprocess
import sys

TOPICS = [
    "/zed/rgb/image_raw",
    "/zed/depth/image",
    "/zed/imu/data",
    "/zed/odom/vo",
    "/encoder/ticks",
    "/encoder/velocity_mps",
    "/scan",
    "/ekf/odom",
    "/lane/centerline",
    "/lane/curvature_radius_m",
    "/obstacle/avoidance_cmd",
    "/obstacle/costmap",
    "/safety/estop_status",
    "/safety/vehicle_state",
    "/safety/state_request",
    "/cmd_ackermann",
    "/diagnostics",
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", default=None,
        help="bag directory name (default: bags/<timestamp>)")
    parser.add_argument(
        "--topics", nargs="*", default=None,
        help="override the default topic contract list")
    args = parser.parse_args()

    output_dir = args.output_dir or (
        "bags/" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
    topics = args.topics or TOPICS

    cmd = ["ros2", "bag", "record", "-o", output_dir, *topics]
    print("Running:", " ".join(cmd))
    return subprocess.run(cmd).returncode


if __name__ == "__main__":
    sys.exit(main())
