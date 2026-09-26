#!/usr/bin/env python3
"""Publishes synthetic ZED camera (RGB/depth/IMU/VO) and LiDAR scan data so
the perception/fusion/planning/control pipeline can be exercised end-to-end
without a real ZED 2i, RPLiDAR, or wheel encoder attached.

Values are NOT physically accurate -- this is a demo/testing double, not a
sensor simulator. It periodically injects a "close object" into the depth
image and LiDAR scan so obstacle_detection/recovery_manager have something
to react to.
"""
import math
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, LaserScan

IMG_W, IMG_H = 1280, 720

# Matches lane_detection's default LaneDetectorParams::src_points trapezoid
# exactly, so the perspective warp actually captures what we draw here.
LANE_BOTTOM_Y, LANE_TOP_Y = 700, 450
LANE_BOTTOM_LEFT_X, LANE_BOTTOM_RIGHT_X = 200, 1100
LANE_TOP_LEFT_X, LANE_TOP_RIGHT_X = 530, 750
CURVE_PERIOD_S = 20.0
CURVE_AMPLITUDE_PX = 550.0


class MockSensorsNode(Node):
    def __init__(self):
        super().__init__("mock_sensors")
        self.bridge = CvBridge()
        self.start_time = time.time()
        self.vo_x = 0.0

        self.rgb_pub = self.create_publisher(Image, "/zed/rgb/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/zed/depth/image", 10)
        self.imu_pub = self.create_publisher(Imu, "/zed/imu/data", 10)
        self.vo_pub = self.create_publisher(Odometry, "/zed/odom/vo", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/lidar/scan_raw", 10)

        self.create_timer(1.0 / 15.0, self.publish_camera)
        self.create_timer(1.0 / 100.0, self.publish_imu)
        self.create_timer(1.0 / 30.0, self.publish_vo)
        self.create_timer(1.0 / 10.0, self.publish_scan)

        self.get_logger().info("mock_sensors started -- publishing synthetic ZED + LiDAR data")

    def _elapsed(self) -> float:
        return time.time() - self.start_time

    @staticmethod
    def _lane_points(bend_amp: float):
        # Quadratic bend anchored at the vehicle (frac=0) and maximal at the
        # horizon (frac=1), like a real curve only visible ahead -- this is
        # what gives lane_detector's x=a*y^2+b*y+c fit a genuine, sweeping
        # `a` term instead of the near-zero (effectively straight) fit a pure
        # sideways translation of two straight lines would always produce.
        n = 40
        left_pts, right_pts = [], []
        for i in range(n + 1):
            frac = i / n
            y = int(LANE_BOTTOM_Y - frac * (LANE_BOTTOM_Y - LANE_TOP_Y))
            bend = bend_amp * frac * frac
            left_x = int(LANE_BOTTOM_LEFT_X + frac * (LANE_TOP_LEFT_X - LANE_BOTTOM_LEFT_X) + bend)
            right_x = int(LANE_BOTTOM_RIGHT_X + frac * (LANE_TOP_RIGHT_X - LANE_BOTTOM_RIGHT_X) + bend)
            left_pts.append((left_x, y))
            right_pts.append((right_x, y))
        return left_pts, right_pts

    def publish_camera(self):
        t = self._elapsed()
        frame = np.full((IMG_H, IMG_W, 3), (60, 60, 60), dtype=np.uint8)

        # Sweeps smoothly left-curve -> straight -> right-curve -> straight
        # every CURVE_PERIOD_S seconds so curvature_radius_m (and therefore
        # steering angle + the curvature-based speed profile) keep changing.
        bend_amp = CURVE_AMPLITUDE_PX * math.sin(2 * math.pi * t / CURVE_PERIOD_S)
        left_pts, right_pts = self._lane_points(bend_amp)
        cv2.polylines(frame, [np.array(left_pts, dtype=np.int32)], False, (255, 255, 255), 12)
        cv2.polylines(frame, [np.array(right_pts, dtype=np.int32)], False, (255, 255, 255), 12)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "zed_left_camera_optical_frame"
        self.rgb_pub.publish(img_msg)

        depth = np.full((IMG_H, IMG_W), 8.0, dtype=np.float32)
        # Cycle a close "obstacle" into view for a few seconds every ~10s so
        # the collision/avoidance path actually gets exercised.
        if int(t) % 10 < 3:
            depth[300:500, 500:800] = 1.0
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = img_msg.header
        self.depth_pub.publish(depth_msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "zed_imu_link"
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.angular_velocity.z = 0.02 * math.sin(self._elapsed() * 0.5)
        msg.linear_acceleration.z = 9.81
        self.imu_pub.publish(msg)

    def publish_vo(self):
        speed = 1.0  # m/s constant simulated forward speed
        dt = 1.0 / 30.0
        self.vo_x += speed * dt

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = self.vo_x
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.twist.twist.linear.x = speed
        self.vo_pub.publish(msg)

    def publish_scan(self):
        t = self._elapsed()
        n = 360
        angle_min, angle_max = -math.pi, math.pi
        increment = (angle_max - angle_min) / n
        ranges = [6.0] * n

        # Cycle a close object at roughly +45..+60 deg (front-left, REP-103
        # + = left) for a few seconds every ~8s so VFH has something to
        # steer away from.
        if int(t) % 8 < 3:
            for i in range(225, 241):
                ranges[i] = 0.8

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser_frame"
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = increment
        msg.range_min = 0.15
        msg.range_max = 12.0
        msg.ranges = ranges
        self.scan_pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = MockSensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
