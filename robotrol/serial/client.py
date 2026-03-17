"""
Robotrol v2.0 — Serial client.

Extracted from Robotrol_FluidNC_v7_3.py lines 208-563 (class SerialClient).
Handles serial connection, RX worker thread, listener dispatch.
No tkinter dependency.
"""

import threading
import time

import serial
import serial.tools.list_ports

from robotrol.config.constants import AXES, DEFAULT_ENDSTOP_LIMITS


class SerialClient:
    """
    Abstraction over serial connection.
    Supports backends: fluidnc, grbl, custom.
    """

    BACKEND_CAPS = {
        "fluidnc": {
            "name": "FluidNC",
            "supports_endstops": True,
            "supports_ota": True,
            "supports_axis_homing": True,
            "supports_global_homing": True,
            "supports_softlimits": True,
        },
        "grbl": {
            "name": "GRBL",
            "supports_endstops": False,
            "supports_ota": False,
            "supports_axis_homing": False,
            "supports_global_homing": True,
            "supports_softlimits": True,
        },
        "custom": {
            "name": "Custom",
            "supports_endstops": False,
            "supports_ota": False,
            "supports_axis_homing": False,
            "supports_global_homing": True,
            "supports_softlimits": False,
        },
    }

    DEBUG_SERIAL = True

    def __init__(self):
        self.ser = None

        self.rx_thread = None
        self.rx_running = False
        self.listeners: list = []
        self.lock = threading.Lock()

        # Axis Limits
        self.axis_limits = {ax: list(DEFAULT_ENDSTOP_LIMITS[ax]) for ax in AXES}

        # Backend
        self.backend_type = "fluidnc"
        self.backend_caps = self.BACKEND_CAPS[self.backend_type]

        # Status
        self._warned_not_connected = False
        self.last_status = None

    # ----------------------------------------------------------------
    #  Select backend (FluidNC / GRBL / custom)
    # ----------------------------------------------------------------
    def set_backend(self, backend_name: str):
        name = (backend_name or "").strip().lower()
        if name not in self.BACKEND_CAPS:
            name = "fluidnc"
        self.backend_type = name
        self.backend_caps = self.BACKEND_CAPS[name]
        if self.DEBUG_SERIAL:
            print(f"[Backend] -> {self.backend_caps['name']} ({self.backend_type})")

    @property
    def is_fluidnc(self) -> bool:
        return self.backend_type == "fluidnc"

    @property
    def is_grbl(self) -> bool:
        return self.backend_type == "grbl"

    @property
    def is_custom(self) -> bool:
        return self.backend_type == "custom"

    @property
    def supports_endstops(self) -> bool:
        return self.backend_caps.get("supports_endstops", False)

    @property
    def supports_ota(self) -> bool:
        return self.backend_caps.get("supports_ota", False)

    @property
    def supports_axis_homing(self) -> bool:
        return self.backend_caps.get("supports_axis_homing", False)

    @property
    def supports_global_homing(self) -> bool:
        return self.backend_caps.get("supports_global_homing", True)

    @property
    def supports_softlimits(self) -> bool:
        return self.backend_caps.get("supports_softlimits", False)

    def status_query_line(self) -> str:
        """Return the status query command for the active backend."""
        if self.backend_type == "custom":
            return "(TM)"
        return "?"

    # ----------------------------------------------------------------
    #  COM-Port Management
    # ----------------------------------------------------------------
    def connect(self, port: str, baud: int = 115200):
        """Open serial connection and start RX thread."""
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass

            self.ser = serial.Serial(
                port,
                baudrate=baud,
                timeout=0.05,
                write_timeout=0.2,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )

            time.sleep(0.2)

            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            with self.lock:
                self.ser.write(b"\r\n")

            time.sleep(0.1)

            # Start RX thread
            self.rx_running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()

            if self.DEBUG_SERIAL:
                print(f"[OK] Connected to {port} @ {baud} (Mega/GRBL)")

            self.send_line("$$")
            self.send_line(self.status_query_line())

        except Exception as e:
            self.ser = None
            print(f"[Error] Connection failed: {e}")
            raise

    def disconnect(self):
        """Cleanly close the serial connection."""
        try:
            self.rx_running = False
            if self.ser:
                if self.DEBUG_SERIAL:
                    print("[INFO] Serial disconnect()")
                self.ser.close()
            self.ser = None
        except Exception as e:
            print("[Error disconnect()]", e)

    # ----------------------------------------------------------------
    #  COM-Port listing
    # ----------------------------------------------------------------
    def list_ports(self):
        """Return all available COM ports."""
        return [p.device for p in serial.tools.list_ports.comports()]

    # ----------------------------------------------------------------
    #  Command sending
    # ----------------------------------------------------------------
    def send_line(self, line: str):
        """Send one line to serial."""
        if not self.ser:
            if not self._warned_not_connected:
                print("[Warn] Not connected (send_line ignored)")
                self._warned_not_connected = True
            return
        self._warned_not_connected = False

        try:
            text = line.strip()
            data = (text + "\n").encode("utf-8")
            with self.lock:
                self.ser.write(data)
            if self.DEBUG_SERIAL and text != "(TM)":
                print("TX>", text)
        except Exception as e:
            print(f"[Warn] Send error: {e}")

    def send_ctrl_x(self):
        """Send Ctrl+X (soft reset)."""
        if not self.ser:
            if not self._warned_not_connected:
                print("[Warn] Not connected (Ctrl+X ignored)")
                self._warned_not_connected = True
            return
        self._warned_not_connected = False
        try:
            with self.lock:
                self.ser.write(b"\x18")
            if self.DEBUG_SERIAL:
                print("TX> <Ctrl+X>")
        except Exception as e:
            print(f"[Warn] send_ctrl_x failed: {e}")

    # ----------------------------------------------------------------
    #  RX worker thread
    # ----------------------------------------------------------------
    def _rx_loop(self):
        """Continuously read serial data and dispatch to listeners."""
        buf = b""
        if self.DEBUG_SERIAL:
            print("[RX] Thread started")

        while self.rx_running and self.ser:
            try:
                data = self.ser.read(1024)
                if data:
                    buf += data
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        txt = raw.decode("utf-8", errors="replace").strip()
                        if not txt:
                            continue
                        if self.DEBUG_SERIAL:
                            if not (
                                txt.startswith("<")
                                or txt == "ok"
                                or txt.startswith("[MSG:")
                                or txt.startswith("[GC:")
                            ):
                                print("RX<", txt)

                        for cb in list(self.listeners):
                            try:
                                cb(txt)
                            except Exception as e:
                                print("[Warn Listener]", e)
                else:
                    time.sleep(0.02)

            except Exception as e:
                if self.DEBUG_SERIAL:
                    print("[RX-loop error]", e)
                time.sleep(0.1)

        if self.DEBUG_SERIAL:
            print("[RX] Thread ended")
