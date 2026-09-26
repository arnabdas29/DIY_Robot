#!/usr/bin/env python3
"""Live one-line-refresh dashboard of key topics for the mock demo.

Usage: python3 scripts/monitor_live.py
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from safety_manager.msg import VehicleState
from std_msgs.msg import Float32

STATE_NAMES = {0: "NORMAL", 1: "AVOID", 2: "BRAKE", 3: "REVERSE",
               4: "RECOVER", 5: "EMERGENCY_STOP"}


class Monitor(Node):
    def __init__(self):
        super().__init__("live_monitor")
        self.state = None
        self.reason = ""
        self.steer = 0.0
        self.speed = 0.0
        self.avoid_z = 0.0
        self.curvature = 0.0

        self.create_subscription(VehicleState, "/safety/vehicle_state", self._on_state, 10)
        self.create_subscription(AckermannDriveStamped, "/cmd_ackermann", self._on_cmd, 10)
        self.create_subscription(Twist, "/obstacle/avoidance_cmd", self._on_avoid, 10)
        self.create_subscription(Float32, "/lane/curvature_radius_m", self._on_curv,
                                  qos_profile_sensor_data)
        self.create_timer(0.5, self._print)

    def _on_state(self, msg):
        self.state = msg.state
        self.reason = msg.reason

    def _on_cmd(self, msg):
        self.steer = msg.drive.steering_angle
        self.speed = msg.drive.speed

    def _on_avoid(self, msg):
        self.avoid_z = msg.angular.z

    def _on_curv(self, msg):
        self.curvature = msg.data

    def _print(self):
        state_str = STATE_NAMES.get(self.state, "?")
        print(
            f"\rstate={state_str:<15} reason={self.reason:<20} "
            f"steer={self.steer:+.3f}rad speed={self.speed:5.2f}m/s "
            f"avoid.z={self.avoid_z:+.3f} curvature={self.curvature:9.1f}m   ",
            end="", flush=True,
        )


def main():
    rclpy.init()
    node = Monitor()
    print("Live monitor -- Ctrl+C to stop")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print()


if __name__ == "__main__":
    main()
