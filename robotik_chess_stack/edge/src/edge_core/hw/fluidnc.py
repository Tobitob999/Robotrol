from __future__ import annotations

import time
from typing import Optional

import serial


class FluidNCError(Exception):
    pass


class FluidNCAlarm(FluidNCError):
    pass


class FluidNC:
    def __init__(self, port: str, baudrate: int = 115200, timeout_s: float = 2.0):
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self._ser: Optional[serial.Serial] = None

    def connect(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout_s)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        time.sleep(0.2)

    def disconnect(self) -> None:
        if self._ser and self._ser.is_open:
            self._ser.close()
        self._ser = None

    def send_line(self, line: str, wait_ok: bool = True) -> str:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial not connected")
        payload = (line.strip() + "\n").encode("ascii", errors="ignore")
        self._ser.write(payload)
        self._ser.flush()
        if not wait_ok:
            return ""
        return self._read_until_ok()

    def poll_status(self) -> str:
        if not self._ser or not self._ser.is_open:
            raise RuntimeError("Serial not connected")
        self._ser.write(b"?")
        self._ser.flush()
        raw = self._ser.readline()
        return raw.decode("ascii", errors="ignore").strip()

    def unlock(self) -> None:
        self.send_line("$X")

    def home(self) -> None:
        self.send_line("$H")

    def reset(self) -> None:
        if not self._ser or not self._ser.is_open:
            return
        self._ser.write(b"\x18")
        self._ser.flush()

    def _read_until_ok(self) -> str:
        if not self._ser:
            raise RuntimeError("Serial not connected")
        deadline = time.time() + self.timeout_s
        lines = []
        while time.time() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line:
                continue
            lines.append(line)
            low = line.lower()
            if low.startswith("alarm"):
                raise FluidNCAlarm(line)
            if low.startswith("error"):
                raise FluidNCError(line)
            if low.startswith("ok"):
                return "\n".join(lines)
        raise TimeoutError("Timed out waiting for ok/error")
