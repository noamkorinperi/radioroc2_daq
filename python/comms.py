"""
================================================================
comms.py — USB CDC communication with STM32

Handles:
  - Serial port open/close
  - Packet receive (7-byte event packets)
  - Command send (mode, calibration, reset)

Packet format (STM32 → PC):
  [0]    0xAA          start byte
  [1]    channel       0-63, or 0xFF = histogram ready marker
  [2-5]  keV           float32, little-endian
  [6]    checksum      XOR of bytes [1..5]

Command format (PC → STM32):
  [0]    0xBB          start byte
  [1]    cmd_type      0x01=count, 0x02=time, 0x03=calib, 0x04=reset
  [2-5]  parameter     uint32 or float32, little-endian
  [6]    checksum      XOR of bytes [1..5]
================================================================
"""

import serial
import struct
import time
from typing import Optional


PKT_SIZE         = 7
PKT_START_EVENT  = 0xAA
PKT_START_CMD    = 0xBB
CH_HIST_READY    = 0xFF

CMD_SET_COUNT    = 0x01
CMD_SET_TIME     = 0x02
CMD_SET_CALIB    = 0x03
CMD_RESET        = 0x04


def _checksum(data: bytes) -> int:
    cs = 0
    for b in data:
        cs ^= b
    return cs


class USBComm:
    def __init__(self, port: str, baudrate: int = 115200):
        self._port     = port
        self._baudrate = baudrate
        self._serial: Optional[serial.Serial] = None
        self._buf = bytearray()

    def connect(self) -> bool:
        try:
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=0.1
            )
            time.sleep(0.5)   # allow USB CDC to enumerate
            return self._serial.is_open
        except serial.SerialException as e:
            print(f"[COMM] Serial error: {e}")
            return False

    def disconnect(self):
        if self._serial and self._serial.is_open:
            self._serial.close()

    # ----------------------------------------------------------
    # receive_packet
    #
    # Reads bytes from serial, accumulates in self._buf,
    # and extracts complete 7-byte packets.
    # Returns a dict with "type" key, or None if not ready.
    # ----------------------------------------------------------
    def receive_packet(self) -> Optional[dict]:
        if not self._serial or not self._serial.is_open:
            return None

        # Read available bytes into buffer
        waiting = self._serial.in_waiting
        if waiting:
            self._buf.extend(self._serial.read(waiting))

        # Scan for start byte
        while len(self._buf) >= PKT_SIZE:

            # Find start byte
            if self._buf[0] != PKT_START_EVENT:
                self._buf.pop(0)
                continue

            # Extract candidate packet
            pkt = self._buf[:PKT_SIZE]

            # Verify checksum
            expected_cs = _checksum(pkt[1:6])
            if pkt[6] != expected_cs:
                # Bad packet — discard start byte and retry
                self._buf.pop(0)
                continue

            # Valid packet — consume it
            self._buf = self._buf[PKT_SIZE:]

            channel = pkt[1]

            # Histogram-ready marker
            if channel == CH_HIST_READY:
                return {"type": "histogram_ready"}

            # Event packet
            energy_kev = struct.unpack_from("<f", pkt, 2)[0]
            return {
                "type":       "event",
                "channel":    channel,
                "energy_kev": energy_kev,
            }

        return None

    # ----------------------------------------------------------
    # _send_command — internal helper
    # ----------------------------------------------------------
    def _send_command(self, cmd: int, param_bytes: bytes):
        if not self._serial or not self._serial.is_open:
            return

        pkt = bytearray(PKT_SIZE)
        pkt[0] = PKT_START_CMD
        pkt[1] = cmd
        pkt[2:6] = param_bytes[:4]
        pkt[6] = _checksum(pkt[1:6])
        self._serial.write(pkt)

    def send_mode(self, mode: str, param: int):
        """Send COUNT or TIME mode command to STM32."""
        cmd = CMD_SET_COUNT if mode == "count" else CMD_SET_TIME
        self._send_command(cmd, struct.pack("<I", param))
        print(f"[COMM] Sent mode={mode}, param={param}")

    def send_calibration(self, s_t_ref: float):
        """Send updated calibration factor to STM32."""
        self._send_command(CMD_SET_CALIB, struct.pack("<f", s_t_ref))
        print(f"[COMM] Sent calibration S_T_REF={s_t_ref:.4f}")

    def send_reset(self):
        """Send reset command to STM32."""
        self._send_command(CMD_RESET, struct.pack("<I", 0))
        print("[COMM] Sent RESET")
