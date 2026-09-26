"""Full-stack bringup: safety_manager first, then perception, fusion, and
control. Use `mode:=speed` or `mode:=obstacle` to select the ZED feature
profile described in ARCHITECTURE.md.

Use `zed_source:=external` when the official zed-ros2-wrapper (zed_wrapper
package) is already running separately (e.g. `ros2 launch zed_wrapper
zed_camera.launch.py camera_model:=zed2i`) instead of our own zed_interface
node -- this skips launching zed_interface and remaps lane_detection /
obstacle_detection / sensor_fusion to the wrapper's real topics
(/zed2i/zed_node/rgb/color/rect/image, /depth/depth_registered, /imu/data,
/odom) instead of our internal /zed/... topic names."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def _include(pkg, launch_file, launch_arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory(pkg), f"/launch/{launch_file}"]),
        # IncludeLaunchDescription expects a list of (name, value) tuples,
        # not a dict -- .items() converts the caller-friendly dict shape.
        launch_arguments=(launch_arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    mode_arg = DeclareLaunchArgument(
        "mode", default_value="speed", description="'speed' or 'obstacle'")
    zed_source_arg = DeclareLaunchArgument(
        "zed_source", default_value="internal",
        description="'internal' (our zed_interface node opens the camera "
                     "directly via the SDK) or 'external' (a separately-launched "
                     "zed-ros2-wrapper is already publishing -- just remap to it)")
    zed_namespace_arg = DeclareLaunchArgument(
        "zed_wrapper_ns", default_value="/zed2i/zed_node",
        description="Topic namespace of the external zed-ros2-wrapper node")
    bypass_estop_hardware_arg = DeclareLaunchArgument(
        "bypass_estop_hardware", default_value="false",
        description="Bench-test only: run without E-Stop hardware attached (no permanent "
                     "EMERGENCY_STOP latch from a missing serial link). NEVER true for a real run.")
    enable_lidar_arg = DeclareLaunchArgument(
        "enable_lidar", default_value="true",
        description="Set false to skip lidar_interface entirely (no LiDAR hardware attached). "
                     "obstacle_detection still runs on ZED depth alone, but wall/side-clearance "
                     "coverage is reduced to the camera's narrower FOV.")

    is_internal = LaunchConfigurationEquals("zed_source", "internal")
    is_external = LaunchConfigurationNotEquals("zed_source", "internal")
    ns = LaunchConfiguration("zed_wrapper_ns")

    safety = _include("safety_manager", "safety_manager.launch.py",
                       {"bypass_estop_hardware": LaunchConfiguration("bypass_estop_hardware")})
    zed = _include("zed_interface", "zed_interface.launch.py",
                    {"mode": LaunchConfiguration("mode")}, condition=is_internal)
    lidar = _include("lidar_interface", "lidar_interface.launch.py",
                      condition=IfCondition(LaunchConfiguration("enable_lidar")))
    encoder = _include("encoder_interface", "encoder_interface.launch.py")
    fusion_internal = _include("sensor_fusion", "sensor_fusion.launch.py", condition=is_internal)
    fusion_external = _include(
        "sensor_fusion", "sensor_fusion.launch.py",
        {"vo_odom_topic": [ns, "/odom"], "imu_topic": [ns, "/imu/data"]}, condition=is_external)
    lane_internal = _include("lane_detection", "lane_detection.launch.py", condition=is_internal)
    lane_external = _include(
        "lane_detection", "lane_detection.launch.py",
        {"rgb_image_topic": [ns, "/rgb/color/rect/image"]}, condition=is_external)
    obstacle_internal = _include(
        "obstacle_detection", "obstacle_detection.launch.py", condition=is_internal)
    obstacle_external = _include(
        "obstacle_detection", "obstacle_detection.launch.py",
        {"depth_image_topic": [ns, "/depth/depth_registered"]}, condition=is_external)
    plan = _include("planner", "planner.launch.py",
                     {"mode": LaunchConfiguration("mode")})
    recovery = _include("recovery_manager", "recovery_manager.launch.py")
    ctrl = _include("controller", "controller.launch.py")
    diag = _include("diagnostics", "diagnostics.launch.py")

    # safety_manager must be alive before any actuation node starts publishing;
    # a short stagger avoids a race on node startup order without adding a
    # hard dependency between launch files.
    delayed_rest = TimerAction(
        period=1.5,
        actions=[
            zed, lidar, encoder,
            fusion_internal, fusion_external, lane_internal, lane_external,
            obstacle_internal, obstacle_external,
            plan, recovery, ctrl, diag])

    return LaunchDescription(
        [mode_arg, zed_source_arg, zed_namespace_arg, bypass_estop_hardware_arg, enable_lidar_arg,
         safety, delayed_rest])
