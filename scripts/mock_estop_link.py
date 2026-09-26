#!/usr/bin/env python3
"""Emulates the ESP32 E-Stop bridge's UART heartbeat protocol (see
src/safety_manager/common/uart_protocol.hpp) over a virtual pty, so
safety_manager can be exercised end-to-end without real ESP32 hardware
attached.

Always reports estop_pressed=False, radio_ok=True at 10Hz (well within
safety_manager's 0.3s link_timeout) -- this is a test/demo double for the
hardware fail-safe, never a substitute for it. Never point this at a real
vehicle's safety_manager instance.
"""
import os
import pty
import sys
import time

PTY_SYMLINK = "/tmp/mock_estop_pty"


def crc8(data: bytes) -> int:
    """CRC-8/MAXIM, matching uart_protocol.hpp::crc8() exactly."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc & 0xFF


def encode_status(seq: int) -> bytes:
    """Matches uart_protocol.hpp::encodeStatus()'s 10-byte StatusPacket layout."""
    buf = bytearray(10)
    buf[0] = 0xAA  # kStartByte
    buf[1] = 0x01  # kPacketTypeStatus
    buf[2] = 0  # estop_pressed = False
    buf[3] = 1  # radio_ok = True
    buf[4] = seq & 0xFF
    battery_mv = 7400
    buf[5] = battery_mv & 0xFF
    buf[6] = (battery_mv >> 8) & 0xFF
    buf[7] = (-55) & 0xFF  # rssi_dbm, two's complement
    buf[8] = crc8(bytes(buf[0:8]))
    buf[9] = 0x55  # kEndByte
    return bytes(buf)


def main() -> int:
    master_fd, slave_fd = pty.openpty()
    slave_path = os.ttyname(slave_fd)

    if os.path.islink(PTY_SYMLINK) or os.path.exists(PTY_SYMLINK):
        os.remove(PTY_SYMLINK)
    os.symlink(slave_path, PTY_SYMLINK)

    print(f"[mock_estop_link] virtual ESP32 link ready: {PTY_SYMLINK} -> {slave_path}", flush=True)
    print("[mock_estop_link] reporting estop_pressed=False, radio_ok=True at 10Hz", flush=True)

    seq = 0
    try:
        while True:
            os.write(master_fd, encode_status(seq))
            seq = (seq + 1) % 256
            if seq == 0xAA:  # coincides with kStartByte, occasionally confuses the C++ resync
                seq = (seq + 1) % 256
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        os.close(master_fd)
        os.close(slave_fd)
        if os.path.islink(PTY_SYMLINK):
            os.remove(PTY_SYMLINK)
    return 0


if __name__ == "__main__":
    sys.exit(main())
