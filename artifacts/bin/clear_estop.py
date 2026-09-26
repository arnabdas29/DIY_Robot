#!/usr/bin/env python3
"""Publishes a single explicit operator-clear request (/safety/state_request
= 9) to safety_manager's vehicle state machine.

This is intentionally the *only* way EMERGENCY_STOP is ever exited in this
stack -- see safety_manager/include/safety_manager/vehicle_state_machine.hpp:
"EMERGENCY_STOP can be entered from any state and can only be exited by an
explicit operator clear (never automatically)". Run this manually, only
after visually confirming the course is clear and the E-Stop cause has been
resolved.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

OPERATOR_CLEAR_REQUEST = 9


def main() -> int:
    rclpy.init()
    node = Node("clear_estop")
    pub = node.create_publisher(UInt8, "/safety/state_request", 10)

    # Give the publisher a moment to match with safety_manager's subscriber
    # before sending -- a single best-effort publish with no discovery delay
    # risks being silently dropped.
    time.sleep(0.5)

    msg = UInt8()
    msg.data = OPERATOR_CLEAR_REQUEST
    pub.publish(msg)
    node.get_logger().info("Published operator_clear_requested to /safety/state_request")

    time.sleep(0.2)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
