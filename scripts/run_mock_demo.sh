#!/usr/bin/env bash
# Activates the ROS2 Humble environment and launches the full autonomous_rc
# stack with SYNTHETIC sensor data (bringup_mock.launch.py) so it can be
# exercised end-to-end without real hardware attached.
#
# DEMO/TESTING ONLY -- see launch/bringup_mock.launch.py's module docstring.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "== run_mock_demo.sh =="
echo "WARNING: this uses synthetic sensor data and a mocked E-Stop link -- never run on a real vehicle."

if [ -d "$HOME/micromamba/envs/ros_env" ]; then
  echo "Activating sandbox RoboStack env (~/micromamba/envs/ros_env)"
  export MAMBA_ROOT_PREFIX="$HOME/micromamba"
  set +u
  eval "$("$HOME/.local/bin/micromamba" shell hook --shell bash)"
  micromamba activate ros_env
  set -u
  export PATH="/usr/local/cuda/bin:$PATH"
elif [ -f /opt/ros/humble/setup.bash ]; then
  echo "Activating system ROS2 Humble install (/opt/ros/humble)"
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "ERROR: no ROS2 Humble installation found (neither ~/micromamba/envs/ros_env nor /opt/ros/humble/setup.bash)." >&2
  exit 1
fi

if [ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  echo "ERROR: $WORKSPACE_ROOT/install/setup.bash not found -- run 'python3 build.py' first." >&2
  exit 1
fi
set +u
# shellcheck source=/dev/null
source "$WORKSPACE_ROOT/install/setup.bash"
set -u

echo "Launching mock-hardware stack..."
exec ros2 launch "$WORKSPACE_ROOT/launch/bringup_mock.launch.py"
