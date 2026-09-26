#!/usr/bin/env python3
"""Interactive web dashboard for the mock-hardware demo.

Publishes the same synthetic ZED/LiDAR topics as mock_sensors.py, but the
lane bend and object distance are driven by sliders in a browser page
instead of a fixed time-based sweep, so you can manually see how
lane_detection / obstacle_detection changes affect steering + speed.

No X11/DISPLAY is available in this environment -- this serves a small
local web UI instead (two panels: Controls + Driving View) that you open in
your own browser via VS Code's port forwarding.

Usage:
  python3 scripts/interactive_dashboard.py
  -> open http://localhost:8088 in your browser
"""
import json
import math
import os
import pty
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from safety_manager.msg import VehicleState
from sensor_msgs.msg import Image, Imu, LaserScan
from std_msgs.msg import Bool, Float32, UInt8

IMG_W, IMG_H = 1280, 720
# Matches lane_detection's default LaneDetectorParams::src_points trapezoid.
LANE_BOTTOM_Y, LANE_TOP_Y = 700, 450
LANE_BOTTOM_LEFT_X, LANE_BOTTOM_RIGHT_X = 200, 1100
LANE_TOP_LEFT_X, LANE_TOP_RIGHT_X = 530, 750
MAX_BEND_PX = 1400.0  # re-tuned 2026-09-26 after lane_detection's meters_per_pixel_x
# fix (was 3.7/500 Udacity/US-lane-width default, now 0.9144/(0.6*700) tied to
# this course's real 36in width) -- same pixel bend now reports a ~3.4x
# larger real-world radius, so the amplitude needed re-calibrating to still
# reach a genuinely tight (~5-6m) radius at full lock. See repo memory.
HTTP_PORT = 8088

# Absorbs mock_estop_link.py's job (emulates safety_manager's UART E-Stop
# heartbeat, see src/safety_manager/common/uart_protocol.hpp) directly into
# this process so the dashboard's "EMERGENCY STOP" button can actually flip
# the reported estop_pressed bit in real time instead of that being a
# separate, uncontrollable standalone script.
ESTOP_PTY_SYMLINK = "/tmp/mock_estop_pty"


def _crc8(data: bytes) -> int:
    """CRC-8/MAXIM, matching uart_protocol.hpp::crc8() exactly."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc & 0xFF


def _encode_estop_status(seq: int, estop_pressed: bool) -> bytes:
    """Matches uart_protocol.hpp::encodeStatus()'s 10-byte StatusPacket layout."""
    buf = bytearray(10)
    buf[0] = 0xAA  # kStartByte
    buf[1] = 0x01  # kPacketTypeStatus
    buf[2] = 1 if estop_pressed else 0
    buf[3] = 1  # radio_ok = True
    buf[4] = seq & 0xFF
    battery_mv = 7400
    buf[5] = battery_mv & 0xFF
    buf[6] = (battery_mv >> 8) & 0xFF
    buf[7] = (-55) & 0xFF  # rssi_dbm, two's complement
    buf[8] = _crc8(bytes(buf[0:8]))
    buf[9] = 0x55  # kEndByte
    return bytes(buf)


class EStopLinkThread(threading.Thread):
    """Background virtual-pty writer -- reads SHARED.estop_pressed every
    tick so the dashboard button controls it live, instead of the old
    always-False standalone mock_estop_link.py process."""

    def __init__(self):
        super().__init__(daemon=True)
        self._stop_event = threading.Event()

    def run(self):
        master_fd, slave_fd = pty.openpty()
        slave_path = os.ttyname(slave_fd)
        if os.path.islink(ESTOP_PTY_SYMLINK) or os.path.exists(ESTOP_PTY_SYMLINK):
            os.remove(ESTOP_PTY_SYMLINK)
        os.symlink(slave_path, ESTOP_PTY_SYMLINK)
        print(f"[estop_link] virtual ESP32 E-Stop link ready: {ESTOP_PTY_SYMLINK} -> {slave_path}",
              flush=True)

        seq = 0
        try:
            while not self._stop_event.is_set():
                os.write(master_fd, _encode_estop_status(seq, SHARED.get_estop_pressed()))
                seq = (seq + 1) % 256
                if seq == 0xAA:  # coincides with kStartByte, occasionally confuses the C++ resync
                    seq = (seq + 1) % 256
                time.sleep(0.1)
        finally:
            os.close(master_fd)
            os.close(slave_fd)
            if os.path.islink(ESTOP_PTY_SYMLINK):
                os.remove(ESTOP_PTY_SYMLINK)

    def stop(self):
        self._stop_event.set()


# Matches obstacle_avoidance_node's default camera params -- used to render a
# physically-plausible ground plane (close range near the bottom of the
# frame, receding toward the horizon) instead of a flat background, so the
# cliff/negative-obstacle detector has real ground to compare against.
CAMERA_VFOV_RAD = 1.204
CAMERA_MOUNT_HEIGHT_M = 0.195  # measured: camera+LiDAR mounted 19-20cm above ground
_FY = (IMG_H / 2.0) / math.tan(CAMERA_VFOV_RAD / 2.0)
_CY = IMG_H / 2.0


def _ground_plane_depth_by_row():
    d = np.full(IMG_H, 8.0, dtype=np.float32)
    for v in range(int(_CY) + 1, IMG_H):
        d[v] = min(8.0, CAMERA_MOUNT_HEIGHT_M * _FY / (v - _CY))
    return d


GROUND_DEPTH_BY_ROW = _ground_plane_depth_by_row()


class SharedState:
    """Thread-safe box shared between the ROS thread and the HTTP thread."""

    def __init__(self):
        self.lock = threading.Lock()
        # inputs, written by the web UI
        self.lane_bend = 0.0          # -1.0 (full left) .. +1.0 (full right)
        self.object_distance_m = 8.0  # 0.2 (touching) .. 8.0 (clear)
        self.object_angle_deg = 0.0   # -90 (front-right) .. +90 (front-left), REP-103: +Z/CCW=left
        self.object_width_deg = 8.0   # angular half-width of the blocking arc (wall scenario widens this)
        # each slider fires on every drag tick without debouncing, so a POST
        # can arrive out of order over the network -- guard with a
        # per-request sequence number so a stale request can never clobber a
        # newer one (was causing left-drags to silently "snap back").
        self.last_seq = -1
        # scenario runner status (see ScenarioRunner) -- separate from the
        # manual seq-guarded inputs above since a scenario re-asserts its own
        # values every tick and doesn't need ordering protection
        self.scenario_name = "none"
        self.scenario_active = False
        # outputs, written by ROS subscriptions
        self.steer = 0.0
        self.speed = 0.0
        self.state = 0
        self.reason = "init"
        self.curvature_m = 0.0
        self.avoid_bias = 0.0
        self.wall_detected = False
        self.front_clearance_m = 6.0
        self.left_clearance_m = 6.0
        self.right_clearance_m = 6.0
        self.cliff_detected = False
        self.cliff_distance_m = 6.0
        # scenario-only flag: simulates the floor disappearing ahead (a
        # track edge/drop-off) rather than an above-ground object
        self.cliff_active = False
        # Speed Course lap-distance tracking (see LAP_DISTANCE_M) --
        # distance_traveled_m integrates the REAL commanded /cmd_ackermann
        # speed over time, only while the speed_path scenario is active.
        self.distance_traveled_m = 0.0
        self.lap_finished = False
        # for courses with no real distance measurement (figure8,
        # multi_hairpin), the Course Map animates the car marker by elapsed
        # scripted time instead -- set every tick by ScenarioRunner._run
        self.scenario_elapsed_s = 0.0
        self.scenario_duration_s = 0.0
        # EMERGENCY STOP simulation: read by EStopLinkThread every tick and
        # encoded into the mock UART StatusPacket exactly like a real
        # physical E-Stop button press would be
        self.estop_pressed = False

    def get_inputs(self):
        with self.lock:
            return self.lane_bend, self.object_distance_m, self.object_angle_deg, self.object_width_deg

    def get_cliff_active(self):
        with self.lock:
            return self.cliff_active

    def get_estop_pressed(self):
        with self.lock:
            return self.estop_pressed

    def set_estop_pressed(self, pressed: bool):
        with self.lock:
            self.estop_pressed = bool(pressed)

    def set_inputs(self, lane_bend=None, object_distance_m=None, object_angle_deg=None, seq=None):
        with self.lock:
            if seq is not None:
                if seq <= self.last_seq:
                    return  # stale request, a newer one already applied
                self.last_seq = seq
            if lane_bend is not None:
                self.lane_bend = max(-1.0, min(1.0, float(lane_bend)))
            if object_distance_m is not None:
                self.object_distance_m = max(0.2, min(8.0, float(object_distance_m)))
            if object_angle_deg is not None:
                self.object_angle_deg = max(-90.0, min(90.0, float(object_angle_deg)))

    def set_scenario_inputs(self, lane_bend=None, object_distance_m=None, object_angle_deg=None,
                            object_width_deg=None):
        # used by ScenarioRunner: writes directly, bypassing the seq guard
        # (the runner re-asserts every tick, so ordering doesn't matter).
        with self.lock:
            if lane_bend is not None:
                self.lane_bend = max(-1.0, min(1.0, float(lane_bend)))
            if object_distance_m is not None:
                self.object_distance_m = max(0.2, min(8.0, float(object_distance_m)))
            if object_angle_deg is not None:
                self.object_angle_deg = max(-90.0, min(90.0, float(object_angle_deg)))
            if object_width_deg is not None:
                self.object_width_deg = max(4.0, min(120.0, float(object_width_deg)))

    def set_outputs(self, **kwargs):
        with self.lock:
            for k, v in kwargs.items():
                setattr(self, k, v)

    def add_distance(self, delta_m: float) -> bool:
        """Integrate traveled distance; returns True the instant the lap
        first completes (caller triggers the finish-line stop on that edge)."""
        with self.lock:
            if self.lap_finished:
                return False
            lap_distance_m = LAP_DISTANCE_BY_SCENARIO.get(self.scenario_name, 0.0)
            self.distance_traveled_m += delta_m
            if lap_distance_m > 0.0 and self.distance_traveled_m >= lap_distance_m:
                self.distance_traveled_m = lap_distance_m
                self.lap_finished = True
                return True
            return False

    def reset_distance(self):
        with self.lock:
            self.distance_traveled_m = 0.0
            self.lap_finished = False

    def snapshot(self):
        with self.lock:
            lap_distance_m = LAP_DISTANCE_BY_SCENARIO.get(self.scenario_name, 0.0)
            return dict(
                lane_bend=self.lane_bend, object_distance_m=self.object_distance_m,
                object_angle_deg=self.object_angle_deg, object_width_deg=self.object_width_deg,
                scenario_name=self.scenario_name, scenario_active=self.scenario_active,
                steer=self.steer, speed=self.speed, state=self.state,
                reason=self.reason, curvature_m=self.curvature_m,
                avoid_bias=self.avoid_bias, wall_detected=self.wall_detected,
                front_clearance_m=self.front_clearance_m, left_clearance_m=self.left_clearance_m,
                right_clearance_m=self.right_clearance_m,
                cliff_detected=self.cliff_detected, cliff_distance_m=self.cliff_distance_m,
                cliff_active=self.cliff_active,
                lap_distance_m=lap_distance_m, distance_traveled_m=self.distance_traveled_m,
                distance_remaining_m=max(0.0, lap_distance_m - self.distance_traveled_m),
                lap_finished=self.lap_finished,
                scenario_elapsed_s=self.scenario_elapsed_s, scenario_duration_s=self.scenario_duration_s,
                estop_pressed=self.estop_pressed,
            )


SHARED = SharedState()

# Scripted test scenarios: an object encountered straight-ahead/left/right, a
# sharp hairpin ("u-turn") lane that should slow the car, take a smooth
# turn, then recover speed once straight again, and a "wall" (object spans
# front + both sides at once, nothing passable -> full stop). Each keyframe
# is (t_seconds, lane_bend, object_distance_m, object_angle_deg,
# object_width_deg); values are linearly interpolated between keyframes.
SCENARIOS = {
    "object_ahead": [(0, 0.0, 8.0, 0.0, 8.0), (3, 0.0, 0.8, 0.0, 8.0),
                      (6, 0.0, 0.8, 0.0, 8.0), (9, 0.0, 8.0, 0.0, 8.0)],
    "object_left": [(0, 0.0, 8.0, 55.0, 8.0), (3, 0.0, 1.2, 55.0, 8.0),
                     (6, 0.0, 1.2, 55.0, 8.0), (9, 0.0, 8.0, 55.0, 8.0)],
    "object_right": [(0, 0.0, 8.0, -55.0, 8.0), (3, 0.0, 1.2, -55.0, 8.0),
                      (6, 0.0, 1.2, -55.0, 8.0), (9, 0.0, 8.0, -55.0, 8.0)],
    "uturn_left": [(0, 0.0, 8.0, 0.0, 8.0), (4, -1.0, 8.0, 0.0, 8.0),
                    (8, -1.0, 8.0, 0.0, 8.0), (12, 0.0, 8.0, 0.0, 8.0)],
    "uturn_right": [(0, 0.0, 8.0, 0.0, 8.0), (4, 1.0, 8.0, 0.0, 8.0),
                     (8, 1.0, 8.0, 0.0, 8.0), (12, 0.0, 8.0, 0.0, 8.0)],
    "wall_ahead": [(0, 0.0, 8.0, 0.0, 8.0), (3, 0.0, 0.35, 0.0, 100.0),
                    (7, 0.0, 0.35, 0.0, 100.0), (10, 0.0, 8.0, 0.0, 8.0)],
    "cliff_ahead": [(0, 0.0, 8.0, 0.0, 8.0), (10, 0.0, 8.0, 0.0, 8.0)],
}


def _ease(frac: float) -> float:
    # smoothstep (cosine ease in/out) -- avoids the sharp slope discontinuity
    # a plain linear ramp would have at each keyframe, so the spiral tightens
    # and unwinds smoothly like a real decreasing-radius turn.
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, frac)))


def _spiral_lap_keyframes(sign: float, t0: float, straight_s: float,
                          ease_in_s: float, hold_s: float, ease_out_s: float,
                          n: int = 10):
    # Models the attached "Speed Course" layout: a long straight leading into
    # a decreasing-radius spiral down to a tight hairpin apex, then the same
    # spiral unwinding back out -- one continuous-direction lap uses the same
    # turn sign at both ends (it's one loop, not a mirrored figure-8).
    kf = [(t0, 0.0), (t0 + straight_s, 0.0)]
    t = t0 + straight_s
    for i in range(1, n + 1):
        frac = i / n
        kf.append((t + ease_in_s * frac, sign * _ease(frac)))
    t += ease_in_s
    kf.append((t + hold_s, sign * 1.0))
    t += hold_s
    for i in range(1, n + 1):
        frac = i / n
        kf.append((t + ease_out_s * frac, sign * _ease(1.0 - frac)))
    t += ease_out_s
    return kf, t


def _build_multi_segment_course(signs, straight_s=3.0, ease_in_s=4.0, hold_s=1.0, ease_out_s=4.0):
    # Chains one spiral hairpin segment per entry in `signs` -- reused for
    # speed_path (same sign twice -> one dog-bone loop), figure8 (opposite
    # signs -> two crossing lobes), and multi_hairpin (alternating signs ->
    # a technical slalom that still closes into one simple loop overall).
    t = 0.0
    all_kf = []
    for sign in signs:
        kf, t = _spiral_lap_keyframes(sign, t, straight_s, ease_in_s, hold_s, ease_out_s)
        all_kf += kf
    all_kf.append((t, 0.0))
    return [(tt, b, 8.0, 0.0, 8.0) for tt, b in all_kf]


def _course_turn_gain(seg_integral, signs):
    # A real simple closed loop's total curvature integral is exactly 2*pi
    # (turning-tangent theorem) -- for same/alternating-sign hairpins that
    # net to a single loop (speed_path, multi_hairpin) that total is spread
    # across all segments. A figure-8's two lobes are each independently a
    # full closed loop in OPPOSITE senses (net turning = 0, they don't share
    # a single 2*pi budget) so each needs its own full 2*pi instead.
    net = sum(signs)
    if net != 0:
        return (2.0 * math.pi) / (seg_integral * abs(net))
    return (2.0 * math.pi) / seg_integral


_SEG_DEFAULTS = dict(ease_in_s=4.0, hold_s=1.0, ease_out_s=4.0)  # shared by all 3 courses below
_SEG_INTEGRAL = 0.5 * _SEG_DEFAULTS["ease_in_s"] + _SEG_DEFAULTS["hold_s"] + 0.5 * _SEG_DEFAULTS["ease_out_s"]

_SPEED_PATH_SIGNS = [-1.0, -1.0]
_FIGURE8_SIGNS = [-1.0, 1.0]
_MULTI_HAIRPIN_SIGNS = [-1.0, 1.0, -1.0, 1.0, -1.0]

SCENARIOS["speed_path"] = _build_multi_segment_course(_SPEED_PATH_SIGNS, straight_s=3.0, **_SEG_DEFAULTS)
SCENARIOS["figure8"] = _build_multi_segment_course(_FIGURE8_SIGNS, straight_s=2.0, **_SEG_DEFAULTS)
SCENARIOS["multi_hairpin"] = _build_multi_segment_course(_MULTI_HAIRPIN_SIGNS, straight_s=1.5, **_SEG_DEFAULTS)

# scenarios in this set loop indefinitely instead of stopping at the last keyframe
LOOPING_SCENARIOS = {"speed_path", "figure8", "multi_hairpin"}

# Speed Course: estimated from the attached real diagram (2 straights @ 44ft,
# plus 2 spiral hairpins built from the labeled radii 55.9/50/45.8/40/35.3/
# 30/20/10/4 ft, assuming ~45deg of turn per labeled radius -- see comment
# further below for the full derivation). figure8/multi_hairpin are pure
# continuous stress-test loops (like uturn_left/right) with no finish-line
# stop -- they have no real reference distance to measure against, and
# stopping mid-loop on an arbitrary number would cut short exactly the
# repeated-maneuver stress test they're meant to provide.
_SPIRAL_RADII_FT = [55.9, 50.0, 45.8, 40.0, 35.3, 30.0, 20.0, 10.0, 4.0]
_STRAIGHT_FT = 44.0
_FT_TO_M = 0.3048
LAP_DISTANCE_BY_SCENARIO = {
    "speed_path": (2 * _STRAIGHT_FT + 2 * sum(_SPIRAL_RADII_FT) * (math.pi / 4.0)) * _FT_TO_M,
}


def _build_course_shape(keyframes, turn_gain, n: int = 300):
    # Derives the actual top-down course SHAPE from the exact same bend(t)
    # keyframes driving a scenario, by forward-integrating a simple unicycle
    # model -- so the map the user sees is guaranteed geometrically
    # consistent with what the car is actually doing, not a separately
    # hand-drawn approximation.
    total_t = keyframes[-1][0]
    dt = total_t / n
    heading = 0.0
    x = y = 0.0
    pts = [(0.0, x, y)]
    dist = 0.0
    for i in range(1, n + 1):
        t = total_t * i / n
        bend = ScenarioRunner._interp(keyframes, t)[0]
        heading += bend * turn_gain * dt
        x += math.cos(heading)
        y += math.sin(heading)
        dist += 1.0
        pts.append((dist, x, y))
    return pts

# (start_s, end_s) window during which cliff_ahead makes the near-ground
# floor vanish (see ScenarioRunner._run and DashboardSensorsNode.publish_camera)
CLIFF_ACTIVE_WINDOW = (3.0, 7.0)


class ScenarioRunner:
    """Runs a keyframed scenario in a background thread, writing directly to
    SHARED (bypassing the manual-input seq guard) until it finishes or a new
    scenario preempts it."""

    def __init__(self):
        self._stop_event = threading.Event()
        self._thread = None

    def start(self, name: str) -> bool:
        if name not in SCENARIOS:
            return False
        self.stop()
        stop_event = threading.Event()
        self._stop_event = stop_event
        if name in LAP_DISTANCE_BY_SCENARIO:
            SHARED.reset_distance()
        SHARED.set_outputs(scenario_name=name, scenario_active=True)
        self._thread = threading.Thread(target=self._run, args=(name, stop_event), daemon=True)
        self._thread.start()
        return True

    def stop(self):
        if self._thread and self._thread.is_alive():
            self._stop_event.set()
            self._thread.join(timeout=1.0)
        SHARED.set_outputs(scenario_name="none", scenario_active=False, cliff_active=False)

    @staticmethod
    def _interp(keyframes, t):
        if t >= keyframes[-1][0]:
            return keyframes[-1][1:]
        for (t0, b0, d0, a0, w0), (t1, b1, d1, a1, w1) in zip(keyframes, keyframes[1:]):
            if t0 <= t <= t1:
                frac = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
                return (b0 + (b1 - b0) * frac, d0 + (d1 - d0) * frac,
                        a0 + (a1 - a0) * frac, w0 + (w1 - w0) * frac)
        return keyframes[-1][1:]

    def _run(self, name: str, stop_event: threading.Event):
        keyframes = SCENARIOS[name]
        start = time.monotonic()
        end_t = keyframes[-1][0]
        looping = name in LOOPING_SCENARIOS
        last_tick = start
        finished = False
        paused_since = None
        while not stop_event.is_set():
            now = time.monotonic()
            dt = now - last_tick
            last_tick = now

            # Freeze the scripted clock while E-Stopped -- actuation already
            # correctly stops (see pure_pursuit_node's motion_allowed gate),
            # but without this the scenario kept varying the synthetic camera
            # image in the background, making /lane/curvature_radius_m keep
            # changing even though the car wasn't moving (confusing, even
            # though perception legitimately not freezing is itself correct).
            estopped = SHARED.get_estop_pressed() or SHARED.state == 5
            if estopped:
                if paused_since is None:
                    paused_since = now
                time.sleep(0.1)
                continue
            elif paused_since is not None:
                start += (now - paused_since)
                paused_since = None

            t = now - start
            t_eff = (t % end_t) if (looping and end_t > 0) else t
            SHARED.set_outputs(scenario_elapsed_s=t_eff, scenario_duration_s=end_t)

            if name in LAP_DISTANCE_BY_SCENARIO and not finished:
                # integrate the REAL commanded speed (not scripted time) so
                # the finish-line stop reflects how far the car actually drove
                if SHARED.add_distance(SHARED.speed * dt):
                    finished = True

            if finished:
                # reached the finish line -- hold a full stop (reuses the
                # already-verified wall-detection path) until reset/stopped
                SHARED.set_scenario_inputs(0.0, 0.3, 0.0, 100.0)
            else:
                bend, dist, angle, width = self._interp(keyframes, t_eff)
                SHARED.set_scenario_inputs(bend, dist, angle, width)
                if name == "cliff_ahead":
                    lo, hi = CLIFF_ACTIVE_WINDOW
                    SHARED.set_outputs(cliff_active=(lo <= t_eff <= hi))

            if t_eff >= end_t and not looping and not finished:
                break
            time.sleep(0.1)
        SHARED.set_outputs(scenario_name="none", scenario_active=False, cliff_active=False)


SCENARIO_RUNNER = ScenarioRunner()

COURSE_PATHS = {
    "speed_path": _build_course_shape(
        SCENARIOS["speed_path"], _course_turn_gain(_SEG_INTEGRAL, _SPEED_PATH_SIGNS)),
    "figure8": _build_course_shape(
        SCENARIOS["figure8"], _course_turn_gain(_SEG_INTEGRAL, _FIGURE8_SIGNS)),
    "multi_hairpin": _build_course_shape(
        SCENARIOS["multi_hairpin"], _course_turn_gain(_SEG_INTEGRAL, _MULTI_HAIRPIN_SIGNS)),
}

class DashboardSensorsNode(Node):
    def __init__(self):
        super().__init__("interactive_dashboard")
        self.bridge = CvBridge()
        self.vo_x = 0.0

        self.rgb_pub = self.create_publisher(Image, "/zed/rgb/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/zed/depth/image", 10)
        self.imu_pub = self.create_publisher(Imu, "/zed/imu/data", 10)
        self.vo_pub = self.create_publisher(Odometry, "/zed/odom/vo", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/lidar/scan_raw", 10)

        self.create_subscription(AckermannDriveStamped, "/cmd_ackermann", self._on_cmd, 10)
        self.create_subscription(VehicleState, "/safety/vehicle_state", self._on_state, 10)
        self.create_subscription(Twist, "/obstacle/avoidance_cmd", self._on_avoid, 10)
        self.create_subscription(Float32, "/lane/curvature_radius_m", self._on_curv, 10)
        self.create_subscription(Bool, "/obstacle/wall_detected", self._on_wall, 10)
        self.create_subscription(Float32, "/obstacle/front_clearance_m", self._on_front, 10)
        self.create_subscription(Float32, "/obstacle/left_clearance_m", self._on_left, 10)
        self.create_subscription(Float32, "/obstacle/right_clearance_m", self._on_right, 10)
        self.create_subscription(Bool, "/obstacle/cliff_detected", self._on_cliff, 10)
        self.create_subscription(Float32, "/obstacle/cliff_distance_m", self._on_cliff_dist, 10)

        self.state_request_pub = self.create_publisher(UInt8, "/safety/state_request", 10)

        self.create_timer(1.0 / 15.0, self.publish_camera)
        self.create_timer(1.0 / 100.0, self.publish_imu)
        self.create_timer(1.0 / 30.0, self.publish_vo)
        self.create_timer(1.0 / 10.0, self.publish_scan)

        self.get_logger().info("interactive_dashboard sensors started")

    def request_operator_clear(self):
        # Same operator_clear_requested=9 mechanism as diagnostics'
        # clear_estop.py -- EMERGENCY_STOP is a one-way latch, per
        # vehicle_state_machine.cpp it only exits on this explicit request.
        msg = UInt8()
        msg.data = 9
        self.state_request_pub.publish(msg)

    def _on_cmd(self, msg):
        SHARED.set_outputs(steer=msg.drive.steering_angle, speed=msg.drive.speed)

    def _on_wall(self, msg):
        SHARED.set_outputs(wall_detected=msg.data)

    def _on_front(self, msg):
        SHARED.set_outputs(front_clearance_m=msg.data)

    def _on_left(self, msg):
        SHARED.set_outputs(left_clearance_m=msg.data)

    def _on_right(self, msg):
        SHARED.set_outputs(right_clearance_m=msg.data)

    def _on_cliff(self, msg):
        SHARED.set_outputs(cliff_detected=msg.data)

    def _on_cliff_dist(self, msg):
        SHARED.set_outputs(cliff_distance_m=msg.data)

    def _on_state(self, msg):
        SHARED.set_outputs(state=msg.state, reason=msg.reason)

    def _on_avoid(self, msg):
        SHARED.set_outputs(avoid_bias=msg.angular.z)

    def _on_curv(self, msg):
        SHARED.set_outputs(curvature_m=msg.data)

    @staticmethod
    def _lane_points(bend_amp: float):
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
        lane_bend, object_distance_m, object_angle_deg, object_width_deg = SHARED.get_inputs()
        bend_amp = lane_bend * MAX_BEND_PX

        frame = np.full((IMG_H, IMG_W, 3), (60, 60, 60), dtype=np.uint8)
        left_pts, right_pts = self._lane_points(bend_amp)
        cv2.polylines(frame, [np.array(left_pts, dtype=np.int32)], False, (255, 255, 255), 12)
        cv2.polylines(frame, [np.array(right_pts, dtype=np.int32)], False, (255, 255, 255), 12)

        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = "zed_left_camera_optical_frame"
        self.rgb_pub.publish(img_msg)

        depth = np.tile(GROUND_DEPTH_BY_ROW.reshape(-1, 1), (1, IMG_W))
        if object_distance_m < 7.9:
            # +angle (left, REP-103) -> smaller image x, matching the LiDAR mapping below
            half_px = min(IMG_W / 2, 150.0 * (object_width_deg / 8.0))
            center_x = int(IMG_W / 2 - (object_angle_deg / 90.0) * (IMG_W / 2 - half_px))
            x0, x1 = max(0, int(center_x - half_px)), min(IMG_W, int(center_x + half_px))
            depth[300:500, x0:x1] = object_distance_m
        if SHARED.get_cliff_active():
            # the floor vanishes ahead (track edge / drop-off): near-ground
            # rows read background-far instead of the expected close range
            depth[550:IMG_H, :] = 8.0
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header = img_msg.header
        self.depth_pub.publish(depth_msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "zed_imu_link"
        msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
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
        _, object_distance_m, object_angle_deg, object_width_deg = SHARED.get_inputs()
        n = 360
        angle_min, angle_max = -math.pi, math.pi
        increment = (angle_max - angle_min) / n
        ranges = [6.0] * n
        if object_distance_m < 7.9:
            # REP-103: 0 deg = straight ahead (+x), +90 deg = left (+y), -90 deg = right
            center_idx = int(round((math.radians(object_angle_deg) - angle_min) / increment))
            half_width = int(object_width_deg)  # 1 index == 1 degree (n=360 over 2*pi)
            for i in range(center_idx - half_width, center_idx + half_width + 1):
                ranges[i % n] = object_distance_m
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


PAGE_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Autonomous RC -- Interactive Demo</title>
<style>
  body { background:#1e1e1e; color:#ddd; font-family:sans-serif; display:flex; gap:20px; padding:20px; }
  .window { border:1px solid #444; border-radius:8px; background:#252526; box-shadow:0 4px 12px rgba(0,0,0,.4); }
  .titlebar { background:#333; padding:8px 12px; border-radius:8px 8px 0 0; font-weight:bold; display:flex; justify-content:space-between; }
  .content { padding:16px; }
  label { display:block; margin-top:14px; font-size:14px; }
  input[type=range] { width:100%; }
  .readout { color:#4fc3f7; font-family:monospace; }
  button.popout { background:#0e639c; color:white; border:none; border-radius:4px; padding:2px 8px; cursor:pointer; }
  .scenario-btn { background:#37373d; color:#ddd; border:1px solid #555; border-radius:4px; padding:4px 8px; cursor:pointer; font-size:12px; }
  .scenario-btn:hover { background:#0e639c; }
  #telemetry div { margin:4px 0; font-family:monospace; font-size:14px; }
  .state-NORMAL { color:#8bc34a; } .state-AVOID { color:#ffb300; }
  .state-BRAKE, .state-EMERGENCY_STOP { color:#f44336; } .state-REVERSE, .state-RECOVER { color:#4fc3f7; }
</style></head>
<body>
  <div class="window" style="width:380px;">
    <div class="titlebar">Controls <button class="popout" onclick="window.open('/controls','_blank','width=380,height=420')">pop out</button></div>
    <div class="content" id="controls-content"></div>
  </div>
  <div class="window" style="width:560px;">
    <div class="titlebar">Driving View <button class="popout" onclick="window.open('/view','_blank','width=600,height=760')">pop out</button></div>
    <div class="content" id="view-content"></div>
  </div>
  <div class="window" style="width:480px;">
    <div class="titlebar">Course Map <button class="popout" onclick="window.open('/coursemap','_blank','width=500,height=460')">pop out</button></div>
    <div class="content" id="course-content"></div>
  </div>
<script>
%CONTROLS_JS%
%VIEW_JS%
%COURSE_JS%
document.getElementById('controls-content').innerHTML = CONTROLS_HTML;
document.getElementById('view-content').innerHTML = VIEW_HTML;
document.getElementById('course-content').innerHTML = COURSE_HTML;
initControls(); initView(); initCourse();
</script>
</body></html>
"""

CONTROLS_ONLY_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Controls</title>
<style>
  body { background:#1e1e1e; color:#ddd; font-family:sans-serif; padding:16px; }
  label { display:block; margin-top:14px; font-size:14px; }
  input[type=range] { width:100%; }
  .readout { color:#4fc3f7; font-family:monospace; }
  .scenario-btn { background:#37373d; color:#ddd; border:1px solid #555; border-radius:4px; padding:4px 8px; cursor:pointer; font-size:12px; }
  .scenario-btn:hover { background:#0e639c; }
</style></head>
<body><div id="controls-content"></div>
<script>
%CONTROLS_JS%
document.getElementById('controls-content').innerHTML = CONTROLS_HTML;
initControls();
</script></body></html>
"""

VIEW_ONLY_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Driving View</title>
<style>
  body { background:#1e1e1e; color:#ddd; font-family:sans-serif; padding:16px; }
  #telemetry div { margin:4px 0; font-family:monospace; font-size:14px; }
  .state-NORMAL { color:#8bc34a; } .state-AVOID { color:#ffb300; }
  .state-BRAKE, .state-EMERGENCY_STOP { color:#f44336; } .state-REVERSE, .state-RECOVER { color:#4fc3f7; }
</style></head>
<body><div id="view-content"></div>
<script>
%VIEW_JS%
document.getElementById('view-content').innerHTML = VIEW_HTML;
initView();
</script></body></html>
"""

COURSE_ONLY_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Course Map</title>
<style>
  body { background:#1e1e1e; color:#ddd; font-family:sans-serif; padding:16px; }
</style></head>
<body><div id="course-content"></div>
<script>
%COURSE_JS%
document.getElementById('course-content').innerHTML = COURSE_HTML;
initCourse();
</script></body></html>
"""

CONTROLS_JS = """
const CONTROLS_HTML = `
  <label>Lane bend: left <span id="bend-val" class="readout">0.00</span> right
    <input type="range" id="bend" min="-100" max="100" value="0"></label>
  <label>Object distance (m): <span id="dist-val" class="readout">8.0</span>
    <input type="range" id="dist" min="2" max="80" value="80"></label>
  <label>Object angle: right <span id="angle-val" class="readout">0</span> deg left
    <input type="range" id="angle" min="-90" max="90" value="0"></label>
  <p style="font-size:12px;color:#888">Bend sweeps lane_detection's synthetic camera
  image from a hard left curve to a hard right curve. Distance controls the
  synthetic LiDAR/depth "obstacle" range (2.0m = right on top of it, 8.0m = clear).
  Angle moves that obstacle around the vehicle (REP-103: positive = left).</p>
  <hr style="border-color:#444">
  <div style="font-weight:bold;margin-bottom:6px">Test scenarios</div>
  <div style="display:flex;flex-wrap:wrap;gap:6px">
    <button class="scenario-btn" data-name="object_ahead">Object ahead</button>
    <button class="scenario-btn" data-name="object_left">Object left</button>
    <button class="scenario-btn" data-name="object_right">Object right</button>
    <button class="scenario-btn" data-name="uturn_left">U-turn left</button>
    <button class="scenario-btn" data-name="uturn_right">U-turn right</button>
    <button class="scenario-btn" data-name="wall_ahead">Wall ahead</button>
    <button class="scenario-btn" data-name="cliff_ahead">Cliff / drop-off</button>
    <button class="scenario-btn" data-name="speed_path" style="background:#1b5e20">Speed Course (loop)</button>
    <button class="scenario-btn" data-name="figure8" style="background:#1b5e20">Figure-8 (loop)</button>
    <button class="scenario-btn" data-name="multi_hairpin" style="background:#1b5e20">Multi-Hairpin (loop)</button>
    <button class="scenario-btn" data-name="stop" style="background:#5a2222">Stop / manual</button>
  </div>
  <div id="scenario-status" style="margin-top:8px;font-size:13px;color:#ffb300"></div>
  <hr style="border-color:#444">
  <div style="font-weight:bold;margin-bottom:6px">E-Stop (simulates the physical ESP32 link)</div>
  <button id="estop-btn" style="width:100%;background:#c62828;color:white;border:none;
    border-radius:4px;padding:10px;font-size:15px;font-weight:bold;cursor:pointer">
    EMERGENCY STOP</button>
  <button id="estop-clear-btn" style="width:100%;margin-top:6px;background:#2e7d32;color:white;
    border:none;border-radius:4px;padding:6px;cursor:pointer">Clear E-Stop</button>
  <div id="estop-status" style="margin-top:6px;font-size:13px"></div>
`;
function initControls() {
  const bend = document.getElementById('bend');
  const dist = document.getElementById('dist');
  const angle = document.getElementById('angle');
  const bendVal = document.getElementById('bend-val');
  const distVal = document.getElementById('dist-val');
  const angleVal = document.getElementById('angle-val');
  let seq = 0;
  function send() {
    const lane_bend = bend.value / 100.0;
    const object_distance_m = dist.value / 10.0;
    const object_angle_deg = Number(angle.value);
    bendVal.textContent = lane_bend.toFixed(2);
    distVal.textContent = object_distance_m.toFixed(1);
    angleVal.textContent = object_angle_deg.toFixed(0);
    // sequence number so an out-of-order network response can never clobber
    // a newer slider position (each field fires on every drag tick)
    fetch('/control', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({lane_bend, object_distance_m, object_angle_deg, seq: ++seq})});
  }
  bend.oninput = send; dist.oninput = send; angle.oninput = send;
  send();

  document.querySelectorAll('.scenario-btn').forEach(btn => {
    btn.onclick = () => {
      fetch('/scenario', {method:'POST', headers:{'Content-Type':'application/json'},
        body: JSON.stringify({name: btn.dataset.name})});
    };
  });

  document.getElementById('estop-btn').onclick = () => {
    fetch('/estop', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({pressed: true})});
  };
  document.getElementById('estop-clear-btn').onclick = () => {
    fetch('/estop', {method:'POST', headers:{'Content-Type':'application/json'},
      body: JSON.stringify({pressed: false})});
  };

  function pollScenario() {
    fetch('/status').then(r => r.json()).then(d => {
      const el = document.getElementById('scenario-status');
      el.textContent = d.scenario_active ? ('running: ' + d.scenario_name + ' ...') : '';
      // reflect the scenario-driven values on the sliders (read-only while active)
      if (d.scenario_active) {
        bend.value = Math.round(d.lane_bend * 100);
        dist.value = Math.round(d.object_distance_m * 10);
        angle.value = Math.round(d.object_angle_deg);
        bendVal.textContent = d.lane_bend.toFixed(2);
        distVal.textContent = d.object_distance_m.toFixed(1);
        angleVal.textContent = d.object_angle_deg.toFixed(0);
      }
      const estopEl = document.getElementById('estop-status');
      const STATE_NAMES = {0:"NORMAL",1:"AVOID",2:"BRAKE",3:"REVERSE",4:"RECOVER",5:"EMERGENCY_STOP"};
      estopEl.textContent = d.estop_pressed
        ? '\u26a0 E-STOP LINK REPORTING PRESSED (vehicle_state=' + (STATE_NAMES[d.state]||d.state) + ')'
        : 'link clear (vehicle_state=' + (STATE_NAMES[d.state]||d.state) + ')';
      estopEl.style.color = d.estop_pressed ? '#f44336' : (d.state === 0 ? '#8bc34a' : '#ffb300');
    }).catch(() => {});
  }
  setInterval(pollScenario, 300);
}
"""

VIEW_JS = """
const VIEW_HTML = `
  <canvas id="canvas" width="520" height="560" style="background:#333;border-radius:4px;"></canvas>
  <div id="telemetry">
    <div id="t-scenario" style="color:#ffb300"></div>
    <div>state: <span id="t-state" class="readout"></span> (<span id="t-reason"></span>)</div>
    <div>steering: <span id="t-steer" class="readout"></span> rad</div>
    <div>speed: <span id="t-speed" class="readout"></span> m/s</div>
    <div>lane curvature radius: <span id="t-curv" class="readout"></span> m</div>
    <div>obstacle steering bias: <span id="t-avoid" class="readout"></span> rad</div>
    <div>clearance F/L/R: <span id="t-clear" class="readout"></span> m</div>
    <div>cliff distance: <span id="t-cliff-dist" class="readout"></span> m</div>
    <div id="t-wall" style="font-weight:bold"></div>
    <div id="t-cliff" style="font-weight:bold"></div>
    <div>Speed Course distance remaining: <span id="t-dist-rem" class="readout"></span> / <span id="t-dist-total" class="readout"></span> m</div>
    <div id="t-finish" style="font-weight:bold;color:#8bc34a"></div>
  </div>
`;
function initView() {
  const canvas = document.getElementById('canvas');
  const ctx = canvas.getContext('2d');
  const STATE_NAMES = {0:"NORMAL",1:"AVOID",2:"BRAKE",3:"REVERSE",4:"RECOVER",5:"EMERGENCY_STOP"};
  let scroll = 0;

  function draw(d) {
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    // canvas y-axis points down, so rotate(+angle) is visually clockwise;
    // flip sign so +steer (REP-103 left) visually turns the view left
    const steerVis = -d.steer;
    // scrolling road, curving toward the current steering angle
    ctx.fillStyle = '#444';
    const bendPx = Math.max(-1, Math.min(1, steerVis / 0.55)) * 160;
    ctx.beginPath();
    ctx.moveTo(w/2 - 110, h);
    ctx.quadraticCurveTo(w/2 - 110 + bendPx*0.5, h*0.5, w/2 - 60 + bendPx, 0);
    ctx.lineTo(w/2 + 60 + bendPx, 0);
    ctx.quadraticCurveTo(w/2 + 110 + bendPx*0.5, h*0.5, w/2 + 110, h);
    ctx.closePath();
    ctx.fill();
    // scrolling dashed centerline (speed = scroll rate)
    scroll = (scroll + d.speed * 4) % 40;
    ctx.strokeStyle = '#ffeb3b';
    ctx.setLineDash([20, 20]);
    ctx.lineDashOffset = -scroll;
    ctx.beginPath();
    ctx.moveTo(w/2, h);
    ctx.quadraticCurveTo(w/2 + bendPx*0.5, h*0.5, w/2 + bendPx, 0);
    ctx.stroke();
    ctx.setLineDash([]);

    // obstacle marker: placed by the actual controlled angle (REP-103:
    // +angle = left = smaller canvas x) and distance (nearer -> lower)
    if (d.object_distance_m < 7.9) {
      const oy = h - 60 - (h - 140) * (1 - d.object_distance_m / 8.0);
      const ox = w/2 - (d.object_angle_deg / 90.0) * 150 + bendPx*0.3;
      ctx.fillStyle = 'rgba(244,67,54,0.85)';
      ctx.beginPath();
      ctx.arc(ox, oy, 18, 0, 2*Math.PI);
      ctx.fill();
    }

    // car icon, rotated by steering angle, colored by state
    const stateName = STATE_NAMES[d.state] || '?';
    const colors = {NORMAL:'#8bc34a', AVOID:'#ffb300', BRAKE:'#f44336',
      EMERGENCY_STOP:'#f44336', REVERSE:'#4fc3f7', RECOVER:'#4fc3f7'};
    ctx.save();
    ctx.translate(w/2, h - 60);
    ctx.rotate(steerVis);
    ctx.fillStyle = colors[stateName] || '#fff';
    ctx.fillRect(-16, -28, 32, 56);
    ctx.fillStyle = '#222';
    ctx.fillRect(-16, -28, 32, 10); // windshield
    ctx.restore();

    // speed bar
    ctx.fillStyle = '#555';
    ctx.fillRect(20, h - 20, 480, 10);
    ctx.fillStyle = '#4fc3f7';
    ctx.fillRect(20, h - 20, 480 * Math.min(1, d.speed / 8.0), 10);

    document.getElementById('t-state').textContent = stateName;
    document.getElementById('t-state').className = 'readout state-' + stateName;
    document.getElementById('t-reason').textContent = d.reason;
    document.getElementById('t-steer').textContent = d.steer.toFixed(3);
    document.getElementById('t-speed').textContent = d.speed.toFixed(2);
    document.getElementById('t-curv').textContent = d.curvature_m.toFixed(1);
    document.getElementById('t-avoid').textContent = d.avoid_bias.toFixed(3);
    document.getElementById('t-clear').textContent =
      d.front_clearance_m.toFixed(2) + ' / ' + d.left_clearance_m.toFixed(2) + ' / ' + d.right_clearance_m.toFixed(2);
    const wallEl = document.getElementById('t-wall');
    wallEl.textContent = d.wall_detected ? '\u26a0 WALL DETECTED -- STOPPED' : '';
    wallEl.style.color = d.wall_detected ? '#f44336' : '';
    document.getElementById('t-cliff-dist').textContent = d.cliff_distance_m.toFixed(2);
    const cliffEl = document.getElementById('t-cliff');
    cliffEl.textContent = d.cliff_detected ? '\u26a0 CLIFF / DROP-OFF DETECTED -- STOPPED' : '';
    cliffEl.style.color = d.cliff_detected ? '#f44336' : '';
    document.getElementById('t-dist-rem').textContent = d.distance_remaining_m.toFixed(1);
    document.getElementById('t-dist-total').textContent = d.lap_distance_m.toFixed(1);
    document.getElementById('t-finish').textContent =
      d.lap_finished ? '\u2714 FINISH LINE REACHED -- STOPPED' : '';
    document.getElementById('t-scenario').textContent =
      d.scenario_active ? ('\u25b6 scenario running: ' + d.scenario_name) : '';
  }

  function poll() {
    fetch('/status').then(r => r.json()).then(draw).catch(() => {});
  }
  setInterval(poll, 150);
  poll();
}
"""

COURSE_JS = """
const COURSE_HTML = `
  <canvas id="course-canvas" width="440" height="360" style="background:#333;border-radius:4px;"></canvas>
  <div style="font-size:12px;color:#888;margin-top:6px">Top-down shape of the active course
  (derived from the same turning profile the simulator drives, so it stays geometrically consistent).
  The dot shows the car's live position -- only moves while a looping course scenario is running.</div>
`;
const COURSE_PATHS = __COURSE_PATHS_JSON__;  // { scenario_name: [[cum_dist, x, y], ...], ... }
const COURSE_NAMES = {speed_path: 'Speed Course', figure8: 'Figure-8', multi_hairpin: 'Multi-Hairpin'};

function initCourse() {
  const canvas = document.getElementById('course-canvas');
  const ctx = canvas.getContext('2d');

  const fits = {};
  for (const name in COURSE_PATHS) {
    const pts = COURSE_PATHS[name];
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (const [, x, y] of pts) {
      minX = Math.min(minX, x); maxX = Math.max(maxX, x);
      minY = Math.min(minY, y); maxY = Math.max(maxY, y);
    }
    const pad = 30;
    const scale = Math.min((canvas.width - 2*pad) / (maxX - minX || 1),
                            (canvas.height - 2*pad) / (maxY - minY || 1));
    fits[name] = {minX, minY, scale, pad};
  }
  function toScreen(name, x, y) {
    const f = fits[name];
    return [f.pad + (x - f.minX) * f.scale, f.pad + (y - f.minY) * f.scale];
  }

  function drawTrack(name) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    const pts = COURSE_PATHS[name];
    ctx.strokeStyle = '#666';
    ctx.lineWidth = 14;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';
    ctx.beginPath();
    pts.forEach(([, x, y], i) => {
      const [sx, sy] = toScreen(name, x, y);
      if (i === 0) ctx.moveTo(sx, sy); else ctx.lineTo(sx, sy);
    });
    // close the loop visually -- the smooth easing profile (not a true
    // constant-radius arc) doesn't guarantee exact positional closure for
    // every sign pattern, so this draws an explicit "return to start" link
    ctx.closePath();
    ctx.stroke();
    ctx.strokeStyle = '#ffeb3b';
    ctx.lineWidth = 2;
    ctx.setLineDash([10, 10]);
    ctx.stroke();
    ctx.setLineDash([]);
    const [sx0, sy0] = toScreen(name, pts[0][1], pts[0][2]);
    ctx.fillStyle = '#fff';
    ctx.fillRect(sx0 - 10, sy0 - 3, 20, 6);
  }

  function drawCar(d) {
    const active = COURSE_PATHS.hasOwnProperty(d.scenario_name);
    const name = active ? d.scenario_name : 'speed_path';
    drawTrack(name);
    const pts = COURSE_PATHS[name];
    // distance-based progress where measured (speed_path); else fall back
    // to elapsed-scripted-time progress (figure8/multi_hairpin have no
    // real distance reference to integrate against)
    let frac = 0;
    if (active) {
      frac = d.lap_distance_m > 0
        ? Math.min(1, d.distance_traveled_m / d.lap_distance_m)
        : (d.scenario_duration_s > 0 ? (d.scenario_elapsed_s / d.scenario_duration_s) : 0);
    }
    const idx = Math.round(frac * (pts.length - 1));
    const [, x, y] = pts[idx];
    const [sx, sy] = toScreen(name, x, y);
    ctx.fillStyle = active ? '#4fc3f7' : '#777';
    ctx.beginPath();
    ctx.arc(sx, sy, 8, 0, 2 * Math.PI);
    ctx.fill();
    ctx.fillStyle = '#ddd';
    ctx.font = '12px sans-serif';
    const label = COURSE_NAMES[name] || name;
    let statusText = label + ' -- not running';
    if (active && d.lap_distance_m > 0) {
      statusText = label + ': ' + d.distance_traveled_m.toFixed(0) + 'm / ' + d.lap_distance_m.toFixed(0) + 'm'
                 + (d.lap_finished ? '  (FINISHED)' : '');
    } else if (active) {
      statusText = label + ': lap ' + Math.round(frac * 100) + '%';
    }
    ctx.fillText(statusText, 10, canvas.height - 10);
  }

  function poll() {
    fetch('/status').then(r => r.json()).then(drawCar).catch(() => {});
  }
  setInterval(poll, 200);
  poll();
}
""".replace("__COURSE_PATHS_JSON__", json.dumps(COURSE_PATHS))


def _render(template: str) -> bytes:
    return (
        template
        .replace("%CONTROLS_JS%", CONTROLS_JS)
        .replace("%VIEW_JS%", VIEW_JS)
        .replace("%COURSE_JS%", COURSE_JS)
        .encode("utf-8")
    )


class Handler(BaseHTTPRequestHandler):
    def _send(self, body: bytes, content_type: str, status: int = 200):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self._send(_render(PAGE_HTML), "text/html; charset=utf-8")
        elif self.path == "/controls":
            self._send(_render(CONTROLS_ONLY_HTML), "text/html; charset=utf-8")
        elif self.path == "/view":
            self._send(_render(VIEW_ONLY_HTML), "text/html; charset=utf-8")
        elif self.path == "/coursemap":
            self._send(_render(COURSE_ONLY_HTML), "text/html; charset=utf-8")
        elif self.path == "/status":
            self._send(json.dumps(SHARED.snapshot()).encode(), "application/json")
        else:
            self._send(b"not found", "text/plain", status=404)

    def do_POST(self):
        if self.path == "/control":
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length) or b"{}")
            SHARED.set_inputs(
                lane_bend=data.get("lane_bend"),
                object_distance_m=data.get("object_distance_m"),
                object_angle_deg=data.get("object_angle_deg"),
                seq=data.get("seq"),
            )
            self._send(b'{"ok":true}', "application/json")
        elif self.path == "/scenario":
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length) or b"{}")
            name = data.get("name", "")
            if name == "stop":
                SCENARIO_RUNNER.stop()
                self._send(b'{"ok":true}', "application/json")
            elif SCENARIO_RUNNER.start(name):
                self._send(b'{"ok":true}', "application/json")
            else:
                self._send(b'{"ok":false,"error":"unknown scenario"}', "application/json", status=400)
        elif self.path == "/estop":
            length = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(length) or b"{}")
            pressed = bool(data.get("pressed", False))
            SHARED.set_estop_pressed(pressed)
            if not pressed:
                # EMERGENCY_STOP is a one-way latch (see
                # vehicle_state_machine.cpp) -- give the mock link a couple
                # of packet cycles to report estop_pressed=0 first, then
                # send the explicit operator-clear it requires to exit.
                def _delayed_clear():
                    if DASHBOARD_NODE is not None:
                        DASHBOARD_NODE.request_operator_clear()
                threading.Timer(0.4, _delayed_clear).start()
            self._send(b'{"ok":true}', "application/json")
        else:
            self._send(b"not found", "text/plain", status=404)

    def log_message(self, fmt, *args):
        pass  # keep the node's ROS log output readable


DASHBOARD_NODE = None  # set in main(); lets the HTTP handler thread publish operator-clear requests


def main() -> int:
    global DASHBOARD_NODE
    rclpy.init()
    node = DashboardSensorsNode()
    DASHBOARD_NODE = node
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    estop_thread = EStopLinkThread()
    estop_thread.start()

    server = ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), Handler)
    node.get_logger().info(f"dashboard listening on http://0.0.0.0:{HTTP_PORT} (open it in your browser)")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        estop_thread.stop()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
