#!/usr/bin/env python3
"""One-shot measurement: how long from a step-change in the synthetic
obstacle distance (sent to the interactive dashboard) until /cmd_ackermann's
speed actually reflects it. Reports the full input-to-actuation latency of
the live pipeline (mock camera/lidar -> obstacle_detection -> controller)."""
import json
import time
import urllib.request

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

DASHBOARD = "http://localhost:8088"


def post(path, body):
    req = urllib.request.Request(
        DASHBOARD + path, data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    urllib.request.urlopen(req, timeout=2).read()


class Watcher(Node):
    def __init__(self):
        super().__init__("latency_probe")
        self.first_speed = None
        self.trigger_t = None
        self.result_dt = None
        self.samples = []
        self.create_subscription(AckermannDriveStamped, "/cmd_ackermann", self._on_cmd, 50)

    def _on_cmd(self, msg):
        now = time.monotonic()
        self.samples.append((now, msg.drive.speed))
        if self.trigger_t is not None and self.result_dt is None:
            if self.first_speed is None:
                self.first_speed = msg.drive.speed
            elif msg.drive.speed < self.first_speed - 1.0:
                self.result_dt = now - self.trigger_t


def main():
    rclpy.init()
    node = Watcher()
    # settle at a clear baseline first
    post("/control", {"lane_bend": 0.0, "object_distance_m": 8.0, "object_angle_deg": 0.0,
                       "seq": int(time.time() * 1000)})
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)

    node.trigger_t = time.monotonic()
    post("/control", {"lane_bend": 0.0, "object_distance_m": 0.6, "object_angle_deg": 0.0,
                       "seq": int(time.time() * 1000) + 1})

    end = time.monotonic() + 2.0
    while time.monotonic() < end and node.result_dt is None:
        rclpy.spin_once(node, timeout_sec=0.01)

    if node.result_dt is not None:
        print(f"RESULT: input-to-actuation latency = {node.result_dt*1000:.1f} ms")
    else:
        print("RESULT: no speed drop observed within 2.0s window")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
