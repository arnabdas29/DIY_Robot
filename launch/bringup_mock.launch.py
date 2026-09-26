"""Full-stack bringup with SYNTHETIC sensor data instead of real hardware --
for exercising the perception/fusion/planning/control pipeline end-to-end
without a real ZED 2i, RPLiDAR, wheel encoder, or ESP32 attached.

DEMO/TESTING ONLY. The E-Stop link is mocked to always report "clear"
(mock_estop_link.py) -- this bypasses the one thing that must never be
bypassed on a real vehicle. Never use this launch file with real actuators
connected.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

SCRIPTS_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "scripts")
CLEAR_ESTOP_SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "src", "diagnostics", "scripts", "clear_estop.py")
MOCK_ESTOP_PTY = "/tmp/mock_estop_pty"


def generate_launch_description():
    mock_estop = ExecuteProcess(
        cmd=["python3", os.path.join(SCRIPTS_DIR, "mock_estop_link.py")],
        name="mock_estop_link",
        output="screen",
    )
    mock_sensors = ExecuteProcess(
        cmd=["python3", os.path.join(SCRIPTS_DIR, "mock_sensors.py")],
        name="mock_sensors",
        output="screen",
    )

    safety = Node(
        package="safety_manager",
        executable="estop_bridge_node",
        name="safety_manager",
        output="screen",
        # openSerial() is only attempted once at startup (no retry loop), so
        # mock_estop_link.py must have created this symlink before safety_manager
        # starts -- see the TimerAction stagger below.
        parameters=[{"serial_device": MOCK_ESTOP_PTY}],
    )
    fusion = Node(package="sensor_fusion", executable="ekf_node",
                  name="sensor_fusion_ekf", output="screen")
    lane = Node(package="lane_detection", executable="lane_detection_node",
                name="lane_detection", output="screen")
    obstacle = Node(package="obstacle_detection", executable="obstacle_avoidance_node",
                     name="obstacle_avoidance", output="screen")
    raceline = os.path.join(get_package_share_directory("planner"), "config", "raceline.yaml")
    plan = Node(package="planner", executable="local_planner_node",
                name="planner", output="screen",
                parameters=[{"raceline_yaml_path": raceline}])
    recovery = Node(package="recovery_manager", executable="recovery_manager_node",
                     name="recovery_manager", output="screen")
    ctrl = Node(package="controller", executable="pure_pursuit_node",
                name="pure_pursuit_controller", output="screen")
    encoder = Node(package="encoder_interface", executable="encoder_node",
                    name="encoder_interface", output="screen")
    lidar = Node(package="lidar_interface", executable="scan_filter_node",
                 name="lidar_interface", output="screen")
    diag = Node(package="diagnostics", executable="system_health_node",
                name="system_health", output="screen")

    # mock_estop_link.py must create /tmp/mock_estop_pty before safety_manager
    # tries to open it; a short stagger is simpler and more robust here than
    # trying to synchronize on the symlink's creation from within launch.
    delayed_safety = TimerAction(period=1.0, actions=[safety])
    delayed_rest = TimerAction(
        period=2.0,
        actions=[fusion, lane, obstacle, plan, recovery, ctrl, encoder, lidar, diag, mock_sensors])

    # EMERGENCY_STOP is a one-way latch that only clears on an explicit
    # operator action (by design, never inferred -- see
    # vehicle_state_machine.hpp) -- it WILL latch briefly during the startup
    # window before the mock link's first packet arrives, so auto-clear it
    # once things have settled rather than requiring a manual step every run.
    auto_clear_estop = TimerAction(
        period=4.0,
        actions=[ExecuteProcess(
            cmd=["python3", CLEAR_ESTOP_SCRIPT],
            name="auto_clear_estop",
            output="screen",
        )],
    )

    return LaunchDescription([mock_estop, delayed_safety, delayed_rest, auto_clear_estop])
