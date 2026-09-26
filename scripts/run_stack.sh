#!/usr/bin/env bash
# Activates the ROS2 Humble environment and launches the autonomous_rc stack,
# ultimately running every package's node binary via bringup.launch.py.
# Runs a real-hardware pre-flight check first (serial/USB device presence +
# the known E-Stop protocol-mismatch warning) so a bad launch fails loudly
# instead of silently latching EMERGENCY_STOP.
#
# Usage:
#   ./scripts/run_stack.sh [--skip-preflight] [--no-estop-hardware] [mode]
#   mode: "speed" (default) or "obstacle" -- selects the ZED feature profile.
#   --skip-preflight: skip ALL hardware pre-flight checks (e.g. CI/sandbox runs).
#   --no-estop-hardware: bench-test mode -- no E-Stop hardware attached. Skips
#     just the E-Stop preflight check and passes bypass_estop_hardware:=true so
#     estop_bridge_node won't permanently latch EMERGENCY_STOP from a missing
#     serial link. Other safety logic (obstacle/wall/cliff stops) is unaffected.
#     NEVER use this for an actual driving/competition run.
#
# Environment detection (in order):
#   1. This sandbox's userspace RoboStack env (~/micromamba/envs/ros_env),
#      set up because packages.ros.org is unreachable here but the RoboStack
#      conda mirror is -- see /memories/repo/autonomous_rc.md for how it was
#      built.
#   2. A normal system ROS2 install (/opt/ros/humble/setup.bash), which is
#      what the real Jetson Orin Nano target uses.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

SKIP_PREFLIGHT=0
NO_ESTOP_HARDWARE=0
while [ $# -gt 0 ]; do
  case "$1" in
    --skip-preflight) SKIP_PREFLIGHT=1; shift ;;
    --no-estop-hardware) NO_ESTOP_HARDWARE=1; shift ;;
    *) break ;;
  esac
done
MODE="${1:-speed}"

echo "== run_stack.sh: mode=${MODE} =="

preflight_check() {
  echo
  echo "== Hardware pre-flight check =="
  local warn=0

  if [ "$NO_ESTOP_HARDWARE" = "1" ]; then
    echo "[SKIP] E-Stop hardware check (--no-estop-hardware: bench-test mode, bypass_estop_hardware:=true will be passed)"
  elif [ -e /dev/ttyUSB_estop ] || [ -e /dev/ttyACM_estop ]; then
    echo "[OK  ] E-Stop serial device present"
  else
    warn=1
    echo "[WARN] No /dev/ttyUSB_estop (or ttyACM_estop) device found."
    echo "       *** KNOWN PROTOCOL MISMATCH ***"
    echo "       estop_bridge_node expects a WIRED USB-serial link speaking the"
    echo "       10-byte binary CRC8 StatusPacket protocol defined in"
    echo "       src/safety_manager/common/uart_protocol.hpp (START 0xAA / END 0x55)."
    echo "       The attached DIY_ESP32_Codes_3166 firmware (Master.ino +"
    echo "       SlaveMotorController.ino) instead uses a WiFi TCP PLAIN-TEXT"
    echo "       protocol between two separate ESP32 boards -- it does NOT speak"
    echo "       this UART protocol at all."
    echo "       If this device is missing/wrong, estop_bridge_node will fail to"
    echo "       open its serial link and, per its own fail-safe design, the"
    echo "       vehicle will remain permanently latched in EMERGENCY_STOP."
    echo "       No E-Stop hardware yet? Re-run with --no-estop-hardware instead of"
    echo "       --skip-preflight so the vehicle doesn't latch and other features"
    echo "       (lane/obstacle/planner/controller) can still be tested."
    echo "       See SYSTEM_FLOW.md (E-Stop hardware path) before relying on this"
    echo "       stack's safety layer with the real hardware."
  fi

  if command -v lsusb >/dev/null 2>&1; then
    if lsusb | grep -qi 'stereolabs\|zed'; then
      echo "[OK  ] ZED camera detected on USB"
    else
      echo "[WARN] No ZED camera detected on USB (lsusb) -- zed_interface will fail to open the camera."
    fi
  fi

  if [ -e /dev/ttyUSB_lidar ] || [ -e /dev/ttyACM_lidar ]; then
    echo "[OK  ] LiDAR serial device present"
  else
    echo "[WARN] No /dev/ttyUSB_lidar (or ttyACM_lidar) device found -- lidar_interface will fail to open its port."
  fi

  echo
  if [ "$warn" = "1" ]; then
    echo "Pre-flight found issues above. Re-run with --skip-preflight to bypass,"
    echo "or press Enter to continue anyway, Ctrl-C to abort."
    read -r _ || true
  else
    echo "Pre-flight checks passed."
  fi
  echo
}

if [ "$SKIP_PREFLIGHT" = "1" ]; then
  echo "(--skip-preflight given, skipping hardware checks)"
else
  preflight_check
fi

LAUNCH_ARGS=("mode:=${MODE}")
if [ "$NO_ESTOP_HARDWARE" = "1" ]; then
  echo "*** --no-estop-hardware: bypass_estop_hardware:=true -- E-Stop hardware fail-safe latch is DISABLED. Bench-testing only. ***"
  LAUNCH_ARGS+=("bypass_estop_hardware:=true")
fi

if [ -d "$HOME/micromamba/envs/ros_env" ]; then
  echo "Activating sandbox RoboStack env (~/micromamba/envs/ros_env)"
  export MAMBA_ROOT_PREFIX="$HOME/micromamba"
  # micromamba/conda activation scripts aren't `set -u` safe (reference
  # unset vars like CONDA_BUILD) -- relax nounset just for this section.
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
  echo "Build the workspace first -- see README.md." >&2
  exit 1
fi

if [ ! -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
  echo "ERROR: $WORKSPACE_ROOT/install/setup.bash not found -- run 'python3 build.py' first." >&2
  exit 1
fi
# shellcheck source=/dev/null
set +u
source "$WORKSPACE_ROOT/install/setup.bash"
set -u

echo "Launching full stack (safety_manager first, then perception/fusion/control)..."
exec ros2 launch "$WORKSPACE_ROOT/launch/bringup.launch.py" "${LAUNCH_ARGS[@]}"
