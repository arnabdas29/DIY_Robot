#!/usr/bin/env python3
"""GPU/CPU profiling harness for the Jetson Orin Nano.

There is no TensorRT-based perception model in this repo yet (see
ARCHITECTURE.md's ZED feature matrix -- Object Detection is optional/future
work), so this is not a TensorRT layer-by-layer profiler; it is the entry
point for that when one is added. For now it logs `tegrastats` samples to a
CSV for offline analysis of GPU/CPU/RAM headroom while the ROS2 stack is
running (e.g. to decide whether Speed Race Mode's NEURAL depth mode fits the
compute budget alongside lane_detection's CUDA path).
"""
import argparse
import csv
import re
import shutil
import subprocess
import sys
import time

RAM_RE = re.compile(r"RAM (\d+)/(\d+)MB")
GPU_RE = re.compile(r"GR3D_FREQ (\d+)%")
CPU_CORE_RE = re.compile(r"(\d+)%@")


def parse_line(line: str):
    ram = RAM_RE.search(line)
    gpu = GPU_RE.search(line)
    cores = CPU_CORE_RE.findall(line)
    return {
        "ram_used_mb": int(ram.group(1)) if ram else "",
        "ram_total_mb": int(ram.group(2)) if ram else "",
        "gpu_percent": int(gpu.group(1)) if gpu else "",
        "cpu_percent": (sum(int(c) for c in cores) / len(cores)) if cores else "",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=60.0)
    parser.add_argument("--interval-ms", type=int, default=500)
    parser.add_argument("--output", default="gpu_profile.csv")
    args = parser.parse_args()

    if not shutil.which("tegrastats"):
        print("tegrastats not found -- this script only works on a Jetson (JetPack)", file=sys.stderr)
        return 1

    proc = subprocess.Popen(
        ["tegrastats", "--interval", str(args.interval_ms)],
        stdout=subprocess.PIPE, text=True)

    start = time.monotonic()
    rows = []
    try:
        for line in proc.stdout:
            rows.append({"t_sec": time.monotonic() - start, **parse_line(line)})
            if time.monotonic() - start >= args.duration_sec:
                break
    finally:
        proc.terminate()

    with open(args.output, "w", newline="") as f:
        writer = csv.DictWriter(
            f, fieldnames=["t_sec", "cpu_percent", "gpu_percent", "ram_used_mb", "ram_total_mb"])
        writer.writeheader()
        writer.writerows(rows)

    print(f"Wrote {len(rows)} samples to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
