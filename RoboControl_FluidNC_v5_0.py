# ============================================================================================
# 🦾 RoboControl FluidNC v5.0
# Vollständige G-Code-Steuerzentrale mit Queue, Gamepad, Kinematik, Vision & OTA
# ============================================================================================
# Kompatibel mit:
#   • FluidNC v3.7–3.9 (ESP32)
#   • GRBL 1.1 / 1.2 (über Serial)
#
# Features:
#   • Echtzeit-Steuerung über FluidNC-/GRBL-Serial-Link
#   • Command Line + Live Queue (Pufferverwaltung, Prioritäten, Safe-Mischen)
#   • Gamepad-Steuerung mit 3 Geschwindigkeitsprofilen (Slow / Mid / Fast)
#   • DH6-Kinematik (FK/IK) + DLS-Inverse-Kinematik
#   • Live-TCP-Pose-Anzeige (Roll / Pitch / Yaw) per DH-FK
#   • Externer 3D-Visualizer (UDP Mirror → Port 9999)
#   • Vision-System:
#         - OpenCV Kameramodul
#         - Schachbrett-Erkennung
#         - BoardPose-Schätzung
#   • OTA / HTTP Config-Tool für FluidNC
#   • Dark-/Light-Theme umschaltbar
#
# Enthaltene Module / Dateien:
#   - tcp_world_kinematics_frame.py  ? DLS-IK + Welt-Koordinatensteuerung
#   - tcp_pose_module_v3.py          ? FK6 (DH-Modell), Roll/Pitch/Yaw, mm-Ausgabe
#   - tcp_world_kinematics_frame.py  ? TCP-Sequenzen (RET / TARGET / RET + Gripper)
#   - gamepad_block_v3.py            ? Gamepad-Tabs & Trigger-Integration
#   - robosim_visualizer_v90.py/exe  ? 3D-Visualizer (UDP 127.0.0.1:9999)
#   - camera_capturev_v1_1.py         ? Live-Kamera / OpenCV
#   - board_pose_v1.py               ? Schachbrett-Erkennung & Pose-Solve
#   - fluidnc_updater_v2.py          ? OTA-Updater, $$-Inspector, Config-Manager



import os
os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "1"
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog



# Serielle/IO/Gamepad/Netzwerk
import serial, serial.tools.list_ports
import threading, time, re, socket, pygame, subprocess, sys

# App-Module
from fluidnc_updater_v2 import FluidNCUpdaterFrame
from camera_capturev_v1_1 import CameraCapture
from board_pose_v1 import BoardPose
from tcp_pose_module_v3 import TcpPosePanel, GEOM_DH
from tcp_world_kinematics_frame import TcpWorldKinematicsTabs
# === Deaktiviert alle Popups ===
def _no_popup(*args, **kwargs):
    
    
    
    return None
messagebox.showinfo = _no_popup
messagebox.showwarning = _no_popup
messagebox.showerror = _no_popup

# =========================
# Konfiguration
# =========================
UDP_MIRROR = True
UDP_ADDR = ("127.0.0.1", 9999)
GC_RE = re.compile(r"\[GC:(.+)\]")
AXES = ["X", "Y", "Z", "A", "B", "C"]
AXIS_LIMITS_DEFAULT = {ax: (-180.0, 180.0) for ax in AXES}  # Fallback ±180°
MAX_FEED = 15000.0                     # mm/min (interpretiert als „Grad/min“)
MAX_HOME_FEED = 5000   # Beispielwert, frei änderbar
DEFAULT_TIMEOUT = 30.0                # s pro Befehl
MOTION_EPS = 0.01                     # min. Positionsänderung für Live-Move
# $$ Softlimits (Travel) – Grbl/FluidNC: $130..$134 → X..B max travel
SOFTMAX_RE = {
    "X": re.compile(r"^\$130=([0-9\.]+)"),
    "Y": re.compile(r"^\$131=([0-9\.]+)"),
    "Z": re.compile(r"^\$132=([0-9\.]+)"),
    "A": re.compile(r"^\$133=([0-9\.]+)"),
    "B": re.compile(r"^\$134=([0-9\.]+)"),
    "C": re.compile(r"^\$135=([0-9\.]+)"),
}

def map_speed(val_0_1000: int) -> int:
    """Linear: 0–1000 → 0–MAX_FEED."""
    val = max(0, min(1000, int(val_0_1000)))
    return int((val / 1000.0) * MAX_FEED)


# =========================
# Serial + UDP
# =========================
class SerialClient:
    """
    Einfache Abstraktion über die serielle Verbindung + UDP-Mirror.
    Unterstützt Backends:
      - fluidnc
      - grbl
      - custom
    """

    BACKEND_CAPS = {
        "fluidnc": {
            "name": "FluidNC",
            "supports_endstops": True,
            "supports_ota": True,
            "supports_axis_homing": True,   # $HX/$HY/...
            "supports_softlimits": True,
        },
        "grbl": {
            "name": "GRBL",
            "supports_endstops": False,     # kein Pn:
            "supports_ota": False,
            "supports_axis_homing": False,  # nur $H
            "supports_softlimits": True,
        },
        "custom": {
            "name": "Custom",
            "supports_endstops": False,
            "supports_ota": False,
            "supports_axis_homing": False,
            "supports_softlimits": False,
        },
    }

    DEBUG_SERIAL = True  # bei Bedarf auf False setzen, um Konsole zu entlasten

    def __init__(self):
        self.ser = None

        # Keine GUI-Referenzen hier!
        self.rx_thread = None
        self.rx_running = False
        self.listeners = []       # Muss existieren für TCP Panel
        self.lock = threading.Lock()

        # UDP Mirror
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if UDP_MIRROR else None

        # Axis Limits
        self.axis_limits = {ax: list(AXIS_LIMITS_DEFAULT[ax]) for ax in AXES}

        # Backend
        self.backend_type = "fluidnc"
        self.backend_caps = self.BACKEND_CAPS[self.backend_type]

        # Status
        self._warned_not_connected = False
        self.last_status = None

    # ------------------------------------------------------------
    # 🔀 Backend auswählen (FluidNC / GRBL / Custom)
    # ------------------------------------------------------------
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
    def supports_softlimits(self) -> bool:
        return self.backend_caps.get("supports_softlimits", False)

    # ------------------------------------------------------------
    # 📡 Zentrale Status-Parsing-Funktion (für TCP-Panel & Co.)
    # ------------------------------------------------------------
    def parse_status_line(self, line: str):
        """
        Parsed eine typische Statuszeile:
          <Idle|MPos:...|WPos:...|FS:...|Pn:...>
        und liefert (state, positions_dict).
        positions_dict nutzt MPos wenn vorhanden, sonst WPos.
        """
        state = None
        positions = {}

        if not (line.startswith("<") and line.endswith(">")):
            return state, positions

        payload = line[1:-1]
        parts = payload.split("|")
        if not parts:
            return state, positions

        state = parts[0]
        mpos = {}
        wpos = {}

        for part in parts[1:]:
            if part.startswith("MPos:"):
                nums = part[5:].split(",")
                for idx, ax in enumerate(AXES):
                    if idx < len(nums):
                        try:
                            mpos[ax] = float(nums[idx])
                        except ValueError:
                            pass
            elif part.startswith("WPos:"):
                nums = part[5:].split(",")
                for idx, ax in enumerate(AXES):
                    if idx < len(nums):
                        try:
                            wpos[ax] = float(nums[idx])
                        except ValueError:
                            pass

        pose_src = mpos or wpos
        for ax, val in pose_src.items():
            positions[ax] = val

        # für andere Teile merken
        self.last_status = {"state": state, "MPos": mpos, "WPos": wpos}
        return state, positions


    # ------------------------------------------------------------
    # 🔌 COM-Port-Management
    # ------------------------------------------------------------
    def connect(self, port, baud=115200):
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except:
                    pass

            # --------------------------
            # 🔥 EXPERTE: ARDUINO MEGA
            # Kein DTR/RTS, kein Reset
            # --------------------------
            self.ser = serial.Serial(
                port,
                baudrate=baud,
                timeout=0.05,
                write_timeout=0.2,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
                # Mega-spezifisch: DTR und RTS NICHT anfassen!
            )

            time.sleep(0.2)

            # 🔄 Buffer leeren
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            # 💡 Einmal \r\n schicken (Mega wird dadurch wach)
            with self.lock:
                self.ser.write(b"\r\n")

            time.sleep(0.1)

            # RX-Thread starten
            self.rx_running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()

            if self.DEBUG_SERIAL:
                print(f"[OK] Verbunden mit {port} @ {baud} (Mega/GRBL)")

            # 💡 Jetzt die erste echte Anfrage
            self.send_line("$$")
            self.send_line("?")

        except Exception as e:
            self.ser = None
            print(f"[Fehler] Verbindung fehlgeschlagen: {e}")
            raise






    def disconnect(self):
        """Sauberes Schließen der seriellen Verbindung."""
        try:
            self.rx_running = False
            if self.ser:
                if self.DEBUG_SERIAL:
                    print("[INFO] Serial disconnect()")
                self.ser.close()
            self.ser = None
        except Exception as e:
            print("[Fehler disconnect()]", e)

    # ------------------------------------------------------------
    # 🔍 COM-Port Liste abrufen
    # ------------------------------------------------------------
    def list_ports(self):
        """Liste aller verfügbaren COM-Ports zurückgeben."""
        return [p.device for p in serial.tools.list_ports.comports()]

    # ------------------------------------------------------------
    # 🌐 UDP Mirror (Visualizer / Status Broadcast)
    # ------------------------------------------------------------
    def _mirror_udp(self, line: str):
        if not self.udp_sock:
            return
        try:
            self.udp_sock.sendto((line + "\n").encode("utf-8"), UDP_ADDR)

            # einfaches M3 S-Parsing
            if line.startswith("M3") and "S" in line:
                try:
                    s_val = float(line.split("S", 1)[1].strip())
                    import json, time as _time
                    msg = json.dumps({
                        "type": "abs",
                        "S": s_val,
                        "timestamp": _time.time()
                    })
                    self.udp_sock.sendto(msg.encode("utf-8"), UDP_ADDR)
                except Exception:
                    pass
        except Exception as e:
            if self.DEBUG_SERIAL:
                print("[UDP Mirror error]", e)

    # ------------------------------------------------------------
    # ✉️ Senden von Befehlen
    # ------------------------------------------------------------
    def send_line(self, line: str):
        """Sendet eine Zeile an den seriellen Port (oder spiegelt über UDP)."""
        if not self.ser:
            if not self._warned_not_connected:
                print("[Warn] Not connected (send_line ignoriert)")
                self._warned_not_connected = True
            return
        self._warned_not_connected = False

        try:
            text = line.strip()
            data = (text + "\n").encode("utf-8")
            with self.lock:
                self.ser.write(data)
            # DEBUG nur, wenn es NICHT die Statusabfrage ist
            if self.DEBUG_SERIAL and text != "?":
                print("TX>", text)
            if self.udp_sock:
                self._mirror_udp(text)
        except Exception as e:
            print(f"[Warn] Sendefehler: {e}")

    def send_ctrl_x(self):
        """Sendet Ctrl+X (Soft Reset), ohne Log-Spam bei fehlender Verbindung."""
        if not self.ser:
            if not self._warned_not_connected:
                print("[Warn] Not connected (Ctrl+X ignoriert)")
                self._warned_not_connected = True
            return
        self._warned_not_connected = False
        try:
            with self.lock:
                self.ser.write(b"\x18")
            if self.DEBUG_SERIAL:
                print("TX> <Ctrl+X>")
        except Exception as e:
            print(f"[Warn] send_ctrl_x fehlgeschlagen: {e}")

    def _rx_loop(self):
        """Liest permanent Daten aus der seriellen Schnittstelle und verteilt sie an Listener."""
        buf = b""
        if self.DEBUG_SERIAL:
            print("[RX] Thread gestartet")

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
                                txt.startswith("<") or   # Statuszeilen <Idle|...>
                                txt == "ok" or
                                txt.startswith("[MSG:") or
                                txt.startswith("[GC:")
                            ):
                                print("RX<", txt)
                        # ---------------------------------

                        for cb in list(self.listeners):
                            try:
                                cb(txt)
                            except Exception as e:
                                print("[Warn Listener]", e)
                else:
                    time.sleep(0.02)

            except Exception as e:
                if self.DEBUG_SERIAL:
                    print("[RX-Loop Fehler]", e)
                time.sleep(0.1)

        if self.DEBUG_SERIAL:
            print("[RX] Thread beendet")

# =========================
# GUI Rahmen
# =========================
root = tk.Tk()
root.title("RoboControl FluidNC v5.0")
root.geometry("1150x1000")

style = ttk.Style()
try:
    style.theme_use("clam")

    # === ☀️ Standard: Light Mode aktiv ===
    light_bg = "#f0f0f0"
    light_fg = "black"

    style.configure(".", background=light_bg, foreground=light_fg)
    style.configure("TLabel", background=light_bg, foreground=light_fg)
    style.configure("TFrame", background=light_bg)
    style.configure("TButton", background="#e6e6e6", foreground="black")
    style.map("TButton",
        background=[("active", "#d0d0d0"), ("pressed", "#c0c0c0")],
        foreground=[("active", "black"), ("pressed", "black")]
    )
    style.configure("TNotebook", background=light_bg, foreground=light_fg)
    style.configure("TNotebook.Tab", background="#e6e6e6", foreground=light_fg)
    style.map("TNotebook.Tab",
        background=[("selected", "#ffffff")],
        foreground=[("selected", "black")]
    )
    style.configure("TEntry", fieldbackground="white", foreground="black")
    style.configure("Horizontal.TScale", background=light_bg)
    root.configure(bg=light_bg)

    current_theme = {"dark": False}

except Exception as e:
    print("[Warn]", e)

except Exception as e:
    print("[Warn]", e)
# === 🌗 Umschaltung Dark/Light Mode ===
current_theme = {"dark": False}  # Zustand merken (mutable für closure)

def toggle_theme():
    if current_theme["dark"]:
        # ---- LIGHT MODE aktivieren ----
        bg = "#f0f0f0"
        fg = "black"
        style.configure(".", background=bg, foreground=fg)
        style.configure("TLabel", background=bg, foreground=fg)
        style.configure("TFrame", background=bg)
        style.configure("TButton", background="#e6e6e6", foreground="black")
        style.map("TButton",
            background=[("active", "#d0d0d0"), ("pressed", "#c0c0c0")],
            foreground=[("active", "black"), ("pressed", "black")]
        )
        style.configure("TNotebook", background=bg, foreground=fg)
        style.configure("TNotebook.Tab", background="#e6e6e6", foreground=fg)
        style.map("TNotebook.Tab",
            background=[("selected", "#ffffff")],
            foreground=[("selected", "black")]
        )
        style.configure("TEntry", fieldbackground="white", foreground="black")
        style.configure("Horizontal.TScale", background=bg)
        root.configure(bg=bg)
        current_theme["dark"] = False

    else:
        # ---- DARK MODE aktivieren ----
        bg = "#2b2b2b"
        fg = "#e0e0e0"
        style.configure(".", background=bg, foreground=fg)
        style.configure("TLabel", background=bg, foreground=fg)
        style.configure("TFrame", background=bg)
        style.configure("TButton", background="#3c3f41", foreground="#ffffff")
        style.map("TButton",
            background=[("active", "#505354"), ("pressed", "#606364")],
            foreground=[("active", "#ffffff"), ("pressed", "#ffffff")]
        )
        style.configure("TNotebook", background=bg, foreground=fg)
        style.configure("TNotebook.Tab", background="#3a3a3a", foreground=fg)
        style.map("TNotebook.Tab",
            background=[("selected", "#4a4a4a")],
            foreground=[("selected", "#ffffff")]
        )
        style.configure("TEntry", fieldbackground="#3c3f41", foreground="#ffffff")
        style.configure("Horizontal.TScale", background=bg)
        root.configure(bg=bg)
        current_theme["dark"] = True

client = SerialClient()
# Header

top = ttk.Frame(root)
top.pack(fill=tk.X, pady=2)

def refresh_ports():
    ports = client.list_ports()
    combo_ports["values"] = ports
    if ports:
        combo_ports.set(ports[0])
    print("[INFO] Ports refreshed:", ports)

def connect():
    port = combo_ports.get().strip()
    if not port:
        print("[WARN] Kein Port ausgewählt")
        return

    baud = int(combo_baud.get())

    backend_map = {
        "FluidNC": "fluidnc",
        "GRBL": "grbl",
        "Custom": "custom"
    }
    client.set_backend(backend_map.get(combo_backend.get(), "fluidnc"))

    try:
        client.connect(port, baud)
        conn_lbl.configure(text="● Connected", foreground="#2e7d32")
        print(f"[INFO] Verbunden mit {port} @ {baud} Backend={client.backend_type}")
    except Exception as e:
        conn_lbl.configure(text="● Disconnected", foreground="#b71c1c")
        print("[ERROR] Connect:", e)

def disconnect():
    try:
        client.disconnect()
        conn_lbl.configure(text="● Disconnected", foreground="#b71c1c")
        print("[INFO] Disconnected")
    except Exception as e:
        print("[ERROR] Disconnect:", e)



# --- Port Auswahl ---
ttk.Label(top, text="Port:").pack(side=tk.LEFT, padx=(4, 2))
combo_ports = ttk.Combobox(top, width=18, state="readonly")
combo_ports.pack(side=tk.LEFT, padx=(0, 10))

# --- Baudrate ---
ttk.Label(top, text="Baud:").pack(side=tk.LEFT, padx=(0, 2))
combo_baud = ttk.Combobox(top, width=10, state="readonly",
                          values=[115200, 230400, 460800, 921600])
combo_baud.set(115200)
combo_baud.pack(side=tk.LEFT, padx=(0, 10))

# --- Backend Auswahl ---
ttk.Label(top, text="Backend:").pack(side=tk.LEFT, padx=(0, 2))
combo_backend = ttk.Combobox(top, width=10, state="readonly",
                              values=["FluidNC", "GRBL", "Custom"])
combo_backend.set("FluidNC")
combo_backend.pack(side=tk.LEFT, padx=(0, 10))

# --- Buttons ---
ttk.Button(top, text="🔄", width=3, command=refresh_ports).pack(side=tk.LEFT, padx=2)
ttk.Button(top, text="🔌 Connect", command=connect).pack(side=tk.LEFT, padx=2)
ttk.Button(top, text="❌ Disconnect", command=disconnect).pack(side=tk.LEFT, padx=2)
ttk.Button(top, text="🌗 Theme", width=12, command=toggle_theme).pack(side=tk.LEFT, padx=4)

# --- Statusanzeige ---
conn_lbl = ttk.Label(top, text="● Disconnected",
                     foreground="#b71c1c", width=14, anchor="e")
conn_lbl.pack(side=tk.RIGHT, padx=8)
# --- Ersten Port-Scan ausführen ---
refresh_ports()
# Execute App
class ExecuteApp(ttk.Frame):
    def __init__(self, master, client):
        super().__init__(master)
        self.GEOM_DH = GEOM_DH.copy()
        self.pack(fill=tk.BOTH, expand=True)
        self.client = client
        if not hasattr(self.client, "listeners"):
            self.client.listeners = []
        self.client.listeners.append(self.on_serial_line)
        self.paused = False
        self.repeat = tk.BooleanVar(value=False)
        self.repeat_count = tk.IntVar(value=0)
        self.repeat_times = tk.IntVar(value=1)   # Anzahl gewünschter Wiederholungen
        # State
        self.axis_positions = {ax: 0.0 for ax in AXES}
        self._user_editing = {ax: False for ax in AXES}
        self.mode_absolute = tk.BooleanVar(value=True)
        self.live_move = tk.BooleanVar(value=True)
        self.poll_positions = tk.BooleanVar(value=True)
        self.speed_val = tk.IntVar(value=500)
        self.accel_val = tk.IntVar(value=500)
        self.hw_limits = {}  # aus $$ ermittelte Softmax (0..max), je Achse
        # Feste interne Achsenlimits (Min/Max pro Achse)
        self.axis_limits = {
            "X": (-100.0, 100.0),
            "Y": (-100.0, 100.0),
            "Z": (-90.0, 90.0),
            "A": (-90.0, 90.0),
            "B": (-180.0, 180.0),
            "C": (-180.0, 180.0),
        }
        # Program/Queue
        self.program = []
        self.run_event = threading.Event()
        self.abort_event = threading.Event()
        self._awaiting_ack = False

        self._build_ui()

        # Worker-Thread
        self.worker_thread = threading.Thread(target=self.worker, daemon=True)
        self.worker_thread.start()

        # Status-Poll
        self.after(250, self._tick_status_poll)

    # ---------- UI ----------
    def _build_ui(self):
        outer = ttk.Frame(self); outer.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)
        self.axis_entry_order = []   # Reihenfolge für TAB

        # Positions + Speed/Accel
        wrap = ttk.Frame(outer)
        wrap.pack(fill=tk.BOTH, expand=True, pady=8)

        # 📁 Tabs für Achsensteuerung
        pos_tabs = ttk.Notebook(wrap, width=900)
        pos_tabs.pack(side=tk.LEFT, fill=tk.Y, expand=False, padx=(0, 2))

        # --- Tab 1: Manuelle Achssteuerung (bisheriger posf-Inhalt)
        tab_manual = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_manual, text="🕹 Manual")

        # ============================================================
        # 🧠 Vision – Kamera + Schachbrett-Erkennung
        # ============================================================
        tab_vision = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_vision, text="🧠 Vision")

        # Gemeinsamer Wrapper für saubere Ausrichtung
        vision_wrap = ttk.Frame(tab_vision)
        vision_wrap.pack(padx=6, pady=6)

        # Kamera und Pose-Fenster gleich groß
        self.cam = CameraCapture(vision_wrap, width=420, height=320)
        self.board_detector = BoardPose(vision_wrap, self.cam, pattern_size=(7, 7))

        # Nebeneinander mit oberer Kante bündig
        self.cam.get_frame().pack(side="left", padx=8, pady=6, anchor="n")
        self.board_detector.get_frame().pack(side="left", padx=8, pady=6, anchor="n")

        # --- Startknopf, der nach Start verschwindet ---
        btn_start = ttk.Button(tab_vision, text="▶ Kamera starten")
        btn_start.pack(pady=4)

        def start_cam():
            try:
                self.cam.start()
                btn_start.pack_forget()  # Button ausblenden
            except Exception as e:
                print("[Vision start error]", e)

        btn_start.config(command=start_cam)

        # ⚙️ Tab: FluidNC Config / OTA / $$ Inspector
        tab_updater = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_updater, text="⚙️ Config / OTA")

        # FluidNCUpdaterFrame einbinden
        self.updater_frame = FluidNCUpdaterFrame(tab_updater, default_ip="192.168.25.149")

        # Wenn das aktuelle Backend kein OTA kann (z.B. GRBL), ganze Sektion deaktivieren
        if not self.client.supports_ota:
            for child in tab_updater.winfo_children():
                try:
                    child.configure(state="disabled")
                except Exception:
                    pass



        # --- Höhe des OTA-Tabs begrenzen ---
        tab_updater.update_idletasks()
        tab_updater.configure(height=400)
        tab_updater.pack_propagate(False)
        
        # 🎮 Tab: Gamepad (nun unten in pos_tabs)
        tab_gamepad = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_gamepad, text="🎮 Gamepad")

        # Gamepad-Integration
        from gamepad_block_v3 import attach_gamepad_tab
        self.stop_gamepad = attach_gamepad_tab(tab_gamepad, self.client, self)

        # 🤖 Tab: Kinematics (DH6 / Weltkoordinaten)
        tab_kin = ttk.Frame(pos_tabs)
        # direkt rechts neben dem Gamepad-Tab einfügen
        pos_tabs.add(tab_kin, text="🤖 Kinematics")

        try:
            self.kinematics_tabs = TcpWorldKinematicsTabs(
                tab_kin,        # Parent
                self,           # ExecuteApp → für Axis-Access / G-Code
                self.client     # SerialClient
            )
            self.kinematics_tabs.pack(fill="both", expand=True)
        except TypeError:
            # Alternative Reihenfolge falls Modul anders definiert ist
            self.kinematics_tabs = TcpWorldKinematicsTabs(
                tab_kin,
                self.client,
                self
            )
            self.kinematics_tabs.pack(fill="both", expand=True)
        self.kinematics_tabs.pack(fill=tk.BOTH, expand=True)

        # Tab: Befehle
        tab_commands = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_commands, text="📘 Befehle")



        # Tab: Simulation / Pose
        tab_pose = ttk.Frame(pos_tabs)
        pos_tabs.add(tab_pose, text="🧩 Simulation / Pose")

        # =====================================================
        # 🧩 RoboSim Visualizer Integration (umgezogen aus oberem Tab)
        # =====================================================
        try:
            import psutil
        except Exception:
            psutil = None

        def _get_base_dir():
            """Ermittle Basisordner, egal ob Python oder gepackte EXE."""
            if getattr(sys, 'frozen', False):  # läuft als .exe (PyInstaller)
                return os.path.dirname(sys.executable)
            return os.path.dirname(os.path.abspath(__file__))

        BASE_DIR = _get_base_dir()
        VISUALIZER_SCRIPT = os.path.join(BASE_DIR, "robosim_visualizer_v90.py")
        VISUALIZER_EXE    = os.path.join(BASE_DIR, "robosim_visualizer_v90.exe")
        VISUALIZER_TAG    = "RoboSim Control Center"

        def _log_visualizer(msg: str):
            try:
                if hasattr(self, "log"):
                    self.log(msg)
                    return
            except Exception:
                pass
            print(msg)

        def _find_visualizer_process():
            """Suche laufende RoboSim-Prozesse, um Doppelstart zu verhindern."""
            if psutil is None:
                _log_visualizer("ℹ️ psutil fehlt; Prozess-Check deaktiviert.")
                return None
            try:
                for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
                    cmdline = " ".join(p.info.get("cmdline") or [])
                    if (
                        "robosim_visualizer_v90" in cmdline
                        or VISUALIZER_TAG in cmdline
                        or "RoboSim" in (p.info.get("name") or "")
                    ):
                        return p
            except Exception:
                pass
            return None

        def start_visualizer_window():
            """Startet den RoboSim-Visualizer als eigenen Prozess (einmalig)."""
            existing = _find_visualizer_process()
            if existing:
                _log_visualizer(f"ℹ️ Visualizer läuft bereits (PID={existing.pid}) – kein neuer Start.")
                return

            try:
                # Wenn EXE vorhanden (z. B. im dist-Ordner), diese bevorzugen
                if os.path.exists(VISUALIZER_EXE):
                    _log_visualizer(f"🚀 Starte Visualizer EXE: {VISUALIZER_EXE}")
                    subprocess.Popen([VISUALIZER_EXE], cwd=BASE_DIR)
                elif os.path.exists(VISUALIZER_SCRIPT):
                    if getattr(sys, "frozen", False):
                        _log_visualizer("❌ Visualizer EXE fehlt; im frozen-build kann kein .py gestartet werden.")
                        return
                    _log_visualizer(f"🚀 Starte Visualizer PY: {VISUALIZER_SCRIPT}")
                    log_path = os.path.join(BASE_DIR, "robosim_visualizer_start.log")
                    try:
                        log_fp = open(log_path, "a", encoding="utf-8")
                    except Exception:
                        log_fp = None
                    subprocess.Popen(
                        [sys.executable, VISUALIZER_SCRIPT],
                        cwd=BASE_DIR,
                        stdout=log_fp or subprocess.DEVNULL,
                        stderr=log_fp or subprocess.STDOUT,
                    )
                    if log_fp:
                        log_fp.close()
                        _log_visualizer(f"ℹ️ Visualizer-Startlog: {log_path}")
                else:
                    _log_visualizer(f"❌ Kein Visualizer gefunden in {BASE_DIR}")
                    return

                time.sleep(0.5)
                _log_visualizer("✅ Visualizer erfolgreich gestartet.")
            except Exception as e:
                _log_visualizer(f"❌ Visualizer-Start fehlgeschlagen: {e}")

        # ---- UI im Tab hinzufügen ----
        ttk.Label(tab_pose, text="RoboSim Visualizer", font=("Segoe UI", 12, "bold")).pack(pady=10)
        ttk.Button(
            tab_pose,
            text="🧭 Öffne Simulation (externer Prozess)",
            command=start_visualizer_window
        ).pack(pady=20)


        # =========================
        # 📘 Befehlsreferenz (umgezogen aus oberer Tab-Leiste)
        # =========================
        frame_text = ttk.Frame(tab_commands)
        frame_text.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)

        # Scrollbarer Textbereich
        text_commands = tk.Text(frame_text, wrap="word", height=6)
        text_commands.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        scrollbar = ttk.Scrollbar(frame_text, orient="vertical", command=text_commands.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        text_commands.config(yscrollcommand=scrollbar.set)

        # ---- Inhalt (aus oberem Tab kopiert) ----
        command_reference = """\
📘 FluidNC / G-Code Befehlsübersicht
===================================

🧭 Maschinensteuerung
---------------------
$X                  - Unlock / Entsperren nach Alarm
$H                  - Alle Achsen homen
$HX / $HY / $HZ     - Nur bestimmte Achse homen
$$                  - Zeigt aktuelle Parameter (Settings)
$G                  - Aktive G-Code Modi anzeigen
$I                  - Firmware-/Systeminformationen
!                   - Feed Hold (Pause)
~                   - Resume (Fortsetzen)
Ctrl+X (⛔)          - Software Reset / Not-Aus

⚙️ Bewegungsbefehle
---------------------
G90                 - Absolute Koordinaten
G91                 - Relative Koordinaten
G92 X0 Y0 Z0 A0 B0  - Nullpunkt setzen
G0 X10              - Schnelle Positionierung zu X10
G1 X10 F500         - Bewegung zu X10 mit Feedrate 500
G4 P2               - Pause 2 Sekunden

🔩 Achs- & Limit-Konfiguration
-------------------------------
$130..$134          - Max travel für X..B (Soft Limits)
$120..$124          - Beschleunigungen (mm/s²)
$110..$114          - Max feedrate pro Achse (mm/min)
$20 / $21           - Soft/Hard Limits aktivieren
$N / $N+            - Startup-Befehle konfigurieren

🖐️ Werkzeug / Greifer / Spindel
-------------------------------
M3 S0               - Greifer öffnen / Spindel an (S=0)
M3 S1000            - Greifer schließen / volle Drehzahl
M4                  - Spindel im Gegenuhrzeigersinn
M5                  - Spindel stoppen

🔍 Diagnose & Status
---------------------
?                   - Aktueller Status (MPos, Endstops)
$#                  - Koordinatensysteme anzeigen
$Help               - Hilfe / verfügbare Kommandos
$Startup/Show       - Startup-Datei anzeigen
$Erase/All          - Alle gespeicherten Einstellungen löschen
$CD / $Dir          - Dateisystem durchsuchen (ältere Versionen)
$PrintConfig        - Aktuelle geladene YAML anzeigen

💾 Sonstiges
------------
Ctrl-X              - Neustart / Reset
$Report/State       - Aktuellen Maschinenstatusbericht ausgeben
$Report/Startup     - Zeigt, welche Datei beim Start geladen wurde
"""

        text_commands.insert("1.0", command_reference)
        text_commands.configure(state="disabled")  # Nur Lesen

        
        # ============================================
        # 📏 Inhalt für Tab „Manual Axis Control“
        # ============================================
        posf = ttk.LabelFrame(tab_manual, text="Position / Manual Axis Control")
        posf.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)


        # ======================================================
        # HEADER-ZEILE: LiveMove | Absolute | Poll | ManualSpeed
        # ======================================================
        header = ttk.Frame(posf)
        header.pack(fill=tk.X, pady=(2, 4))

        # --- Live Move default ON ---
        self.live_move = tk.BooleanVar(value=True)
        ttk.Checkbutton(header, text="🟩 Live Move (on release)",
                        variable=self.live_move).pack(side=tk.LEFT, padx=(4, 10))

        # --- Absolute (G90) ---
        ttk.Checkbutton(header, text="Absolute (G90)",
                        variable=self.mode_absolute).pack(side=tk.LEFT, padx=(0, 10))

        # --- Poll Positions ---
        ttk.Checkbutton(header, text="Poll Positions (?)",
                        variable=self.poll_positions).pack(side=tk.LEFT, padx=(0, 25))


        # ======================================================
        # MANUAL SPEED FACTOR – in der gleichen Zeile
        # ======================================================
        style = ttk.Style()

        # ---- Stil für normale Buttons ----
        style.configure(
            "SpeedButton.TButton",
            padding=(6, 3),          # Innenpadding (links/rechts, oben/unten)
            font=("Segoe UI", 9),    # Schriftgröße
        )

        # ---- Stil für ausgewählten Button ----
        style.configure(
            "SpeedButtonSelected.TButton",
            padding=(6, 3),
            font=("Segoe UI", 9, "bold"),
            background="#333333",
            foreground="white"
        )

        # ---- Höhe/breite in Pixel erzwingen ----
        style.layout("SpeedButton.TButton", [
            ('Button.border', {'sticky': 'nswe', 'children': [
                ('Button.padding', {'sticky': 'nswe', 'children': [
                    ('Button.label', {'sticky': 'nswe'})
                ]})
            ]})
        ])

        style.layout("SpeedButtonSelected.TButton", style.layout("SpeedButton.TButton"))


        # Label
        ttk.Label(header, text="Manual Speed:", width=12).pack(side=tk.LEFT, padx=(4, 6))

        self.manual_speed_factor = tk.DoubleVar(value=1.0)
        factor_buttons = {}

        def _set_manual_factor(val):
            self.manual_speed_factor.set(val)

            # --- Buttons farblich setzen ---
            for denom, btn in factor_buttons.items():
                # float-safe comparison
                if abs(val - 1/denom) < 1e-9:
                    btn.configure(style="SpeedButtonSelected.TButton")
                else:
                    btn.configure(style="SpeedButton.TButton")

            # Nur loggen, wenn logger existiert
            if hasattr(self, "txt_log"):
                self.log(f"Manual Speed Factor: 1/{int(1/val)}")


        # ---- Buttons erzeugen ----
        for denom in (1, 5, 10, 20, 50, 100):
            b = ttk.Button(
                header,
                text=f"1/{denom}",
                style="SpeedButton.TButton",
                command=lambda d=denom: _set_manual_factor(1/d)
            )

            # ---- Größe & Abstand ----
            b.pack(side=tk.LEFT, padx=4, pady=2)
            b.configure(width=6)

            factor_buttons[denom] = b


        # ---- DEFAULT: 1/20 ----
        _set_manual_factor(1/20)



        # ======================================================
        # MANUAL AXIS CONTROL — Buttons oben, Slider unten
        # ======================================================
        self.axis_labels = {}
        self.axis_vars = {}
        self.axis_entries = {}
        self.axis_endstop_icons = {}
        self.axis_limit_labels = {}


        def clamp_with_limits(ax, v):
            try:
                val = float(v)
            except:
                val = 0.0
            lo, hi = self.axis_limits.get(ax, AXIS_LIMITS_DEFAULT[ax])
            return max(lo, min(hi, val))


        # ======================================================
        # Hilfsfunktion: Nach jeder Bewegung ALLE Felder aktualisieren
        # ======================================================
        def update_all_positions():
            """Aktualisiert alle Achsen anhand der letzten Statusdaten."""
            st = getattr(self.client, "last_status", None)
            if not st or "MPos" not in st:
                return
            for ax, pos in st["MPos"].items():
                if ax not in AXES:
                    continue
                self.axis_positions[ax] = pos
                self.axis_vars[ax].set(pos)
                if ax in self.axis_entries:
                    e = self.axis_entries[ax]
                    e.delete(0, tk.END)
                    e.insert(0, f"{pos:.3f}")
                if ax in self.axis_labels:
                    self.axis_labels[ax].config(text=f"{pos:.3f}")
        self.update_all_positions = update_all_positions



        # ======================================================
        # 1) BUTTONBLOCK (JE 6 REIHEN)
        # ======================================================
        for ax in AXES:

            row = ttk.Frame(posf)
            row.pack(fill=tk.X, pady=1)

            # Achsenlabel
            ttk.Label(row, text=ax, width=3).pack(side=tk.LEFT)

            # mpos-Anzeige
            lbl = ttk.Label(row, text="0.000", width=10, anchor="e")
            lbl.pack(side=tk.LEFT, padx=5)
            self.axis_labels[ax] = lbl

            # Endstop-Kreis
            c = tk.Canvas(row, width=14, height=14, highlightthickness=0)
            oval = c.create_oval(2, 2, 12, 12, fill="#999999")
            c.pack(side=tk.LEFT, padx=5)
            self.axis_endstop_icons[ax] = (c, oval, "unknown")


            # ---------------------------------------------------
            # Prozent-Buttons (Null statt "0%")
            # ---------------------------------------------------
            def send_percent(ax=ax, pct=0):
                lo, hi = self.axis_limits.get(ax, AXIS_LIMITS_DEFAULT[ax])
                target = lo + (hi - lo) * (pct + 100) / 200.0

                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G90")
                self.client.send_line(f"G1 {ax}{target:.3f} F{F:.0f}")

                self.log(f"{ax} → {target:.3f} ({pct:+d}%) @F={F}")
                self.update_all_positions()

            pct_frame = ttk.Frame(row, relief="solid", borderwidth=1)
            pct_frame.pack(side=tk.LEFT, padx=4)

            for pct in (-100, -50, 0, 50, 100):
                btn_text = "0" if pct == 0 else f"{pct:+d}%"
                b = ttk.Button(
                    pct_frame, text=btn_text,
                    width=6, style="FlatMini.TButton",
                    command=lambda ax=ax, pct=pct: send_percent(ax, pct)
                )
                if pct == 0:
                    b.configure(style="FlatMiniSelected.TButton")
                b.pack(side=tk.LEFT, padx=1)


            # ---------------------------------------------------
            # Jog Buttons (relativ)
            # ---------------------------------------------------
            jog_frame = ttk.Frame(row, relief="solid", borderwidth=1)
            jog_frame.pack(side=tk.LEFT, padx=4)

            neg = [10, 5, 1, 0.5, 0.1]
            pos = [0.1, 0.5, 1, 5, 10]

            def jog(ax=ax, dist=0):
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G91")
                self.client.send_line(f"G1 {ax}{dist:.3f} F{F:.0f}")
                self.client.send_line("G90")

                self.log(f"{ax} jog {dist:+} @F={F}")
                self.update_all_positions()

            for step in neg:
                ttk.Button(
                    jog_frame, text=f"{-step:g}",
                    width=4, style="FlatMini.TButton",
                    command=lambda ax=ax, step=step: jog(ax, -step)
                ).pack(side=tk.LEFT, padx=1)

            for step in pos:
                ttk.Button(
                    jog_frame, text=f"+{step:g}",
                    width=4, style="FlatMini.TButton",
                    command=lambda ax=ax, step=step: jog(ax, +step)
                ).pack(side=tk.LEFT, padx=1)

        # ======================================================
        # 2) SLIDERBLOCK
        # ======================================================
        for ax in AXES:

            slider_row = ttk.Frame(posf)
            slider_row.pack(fill=tk.X, pady=(3, 3))

            ttk.Label(slider_row, text=ax, width=9).pack(side=tk.LEFT)

            lo, hi = AXIS_LIMITS_DEFAULT[ax]
            var = tk.DoubleVar(value=0.0)
            self.axis_vars[ax] = var

            # --- Canvas + Null-Linie ---
            slider_wrap = tk.Frame(slider_row)
            slider_wrap.pack(side=tk.LEFT, padx=4)

            canvas = tk.Canvas(slider_wrap, width=600, height=22, highlightthickness=0)
            canvas.pack()

            # ---- Achsen-Skala im Canvas ----
            scale_width = 600
            scale_height = 22

            # Verhältnis Funktion (Wert → Pixel)
            def val_to_x(v):
                return (v - lo) / (hi - lo) * scale_width

            # Null-Linie
            zero_x = val_to_x(0)
            canvas.create_line(zero_x, 0, zero_x, scale_height, fill="black", width=2)

            # große Striche: immer bei 10er-Schritten
            for v in range(int(lo - lo % 10), int(hi) + 1, 10):
                x = val_to_x(v)
                canvas.create_line(x, scale_height - 14, x, scale_height, fill="black", width=2)

            # --- Slider ---
            s = ttk.Scale(canvas, from_=lo, to=hi, variable=var,
                          orient=tk.HORIZONTAL, length=600)
            s.place(x=0, y=0)

            # --- Entry ---
            ent = ttk.Entry(slider_row, width=7)
            ent.insert(0, "0.0")
            ent.pack(side=tk.LEFT, padx=4)
            self.axis_entries[ax] = ent
            self.axis_entry_order.append(ent)

            # -----------------------
            # Wert lesen + clamp
            # -----------------------
            def _get(ax=ax, ent=ent, var=var):
                try:
                    v = float(ent.get().replace(",", "."))
                except:
                    v = var.get()
                v = clamp_with_limits(ax, v)
                ent.delete(0, tk.END)
                ent.insert(0, f"{v:.3f}")
                return v

            # -----------------------
            # Slider->Entry Sync (HAT bisher gefehlt)
            # -----------------------
            def on_var_write(*_ignored, ax=ax, ent=ent, var=var):
                # während der aktiven Eingabe nicht überschreiben
                if getattr(self, "_user_editing", {}).get(ax, False):
                    return
                ent.delete(0, tk.END)
                ent.insert(0, f"{var.get():.3f}")

            var.trace_add("write", on_var_write)

            # --- ENTER sendet sofort ---
            def _on_enter(event=None, ax=ax, ent=ent, var=var):
                val = _get()
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G90")
                self.client.send_line(f"G1 {ax}{val:.3f} F{F:.0f}")
                self.log(f"{ax} → {val:.3f} @F={F}")
                # nach Bewegung aktualisieren
                self.update_position_display()
                return "break"
            ent.bind("<Return>", _on_enter)

            # --- TAB Navigation ---
            def _focus_and_select(w):
                w.focus_set()
                w.after(1, lambda: w.selection_range(0, tk.END))

            ent.bind("<FocusIn>", lambda e, ent=ent: ent.after(1, lambda: ent.selection_range(0, tk.END)))
            ent.bind("<Tab>", lambda e, ent=ent: (_focus_and_select(
                self.axis_entry_order[(self.axis_entry_order.index(ent)+1) % len(self.axis_entry_order)]
            ), "break")[1])
            ent.bind("<Shift-Tab>", lambda e, ent=ent: (_focus_and_select(
                self.axis_entry_order[(self.axis_entry_order.index(ent)-1) % len(self.axis_entry_order)]
            ), "break")[1])
            ent.bind("<ISO_Left_Tab>", lambda e, ent=ent: (_focus_and_select(
                self.axis_entry_order[(self.axis_entry_order.index(ent)-1) % len(self.axis_entry_order)]
            ), "break")[1])

            # --- Buttons ToQ / ToCLI / Send ---
            btn_block = ttk.Frame(slider_row)
            btn_block.pack(side=tk.LEFT, padx=2)

            def _btn_toQ(ax=ax, ent=ent, var=var):
                v = _get()
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.enqueue(f"G1 {ax}{v:.3f} F{F:.0f}")
                self.update_position_display()

            def _btn_toCLI(ax=ax, ent=ent, var=var):
                v = _get()
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.entry_cmd.delete(0, tk.END)
                self.entry_cmd.insert(0, f"G1 {ax}{v:.3f} F{F:.0f}")

            def _btn_send(ax=ax, ent=ent, var=var):
                v = _get()
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G90")
                self.client.send_line(f"G1 {ax}{v:.3f} F{F:.0f}")
                self.log(f"{ax} → {v:.3f} @F={F}")
                self.update_position_display()

            # --- Neue Funktion: komplette Pose → Queue ---
            def _btn_pose_toQ(ax=ax):
                # Alle Achsenwerte einsammeln
                parts = []
                for ax2 in AXES:
                    ent2 = self.axis_entries.get(ax2)
                    if not ent2:
                        continue
                    try:
                        v = float(ent2.get().replace(",", "."))
                    except:
                        v = self.axis_vars[ax2].get()

                    lo2, hi2 = self.axis_limits.get(ax2, AXIS_LIMITS_DEFAULT[ax2])
                    v = max(lo2, min(hi2, v))  # clamp

                    parts.append(f"{ax2}{v:.3f}")

                # Feedrate
                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()

                # Kompletten G1 erzeugen
                gline = "G1 " + " ".join(parts) + f" F{F:.0f}"

                # In Queue schieben
                self.enqueue(gline)
                self.log(f"Pose → Queue: {gline}")


            ttk.Button(btn_block, text="ToQ",   width=4, style="FlatMini.TButton", command=_btn_toQ).pack(side=tk.LEFT, padx=1)
            ttk.Button(btn_block, text="ToCLI", width=5, style="FlatMini.TButton", command=_btn_toCLI).pack(side=tk.LEFT, padx=1)
            ttk.Button(btn_block, text="Send",  width=5, style="FlatMini.TButton", command=_btn_send).pack(side=tk.LEFT, padx=1)



            # --- Slider LiveMove ---
            def _press(e, ax=ax):
                self._user_editing[ax] = True
                self._start_val = self.axis_vars[ax].get()

            def _release(e, ax=ax, ent=ent):
                self._user_editing[ax] = False
                val = self.axis_vars[ax].get()

                if not self.live_move.get():
                    return
                if abs(val - getattr(self, "_start_val", val)) < MOTION_EPS:
                    return

                F = map_speed(self.speed_val.get()) * self.manual_speed_factor.get()
                self.client.send_line("G90")
                self.client.send_line(f"G1 {ax}{val:.3f} F{F:.0f}")
                self.log(f"{ax} slide → {val:.3f} @F={F}")
                self.update_position_display()

            s.bind("<ButtonPress-1>", _press)
            s.bind("<ButtonRelease-1>", _release)

        # =====================================================
        # ⚙️ SPEED-Kontrolle – mit 3 MaxSpeed-Modi (Low/Mid/High)
        # =====================================================
        spdf = ttk.LabelFrame(wrap, text="Speed", width=240, height=220)
        spdf.pack(side=tk.LEFT, fill=tk.Y, padx=2, pady=2)
        spdf.pack_propagate(False)

        # --- interne Werte ---
        self.speed_val = tk.DoubleVar(value=500)
        self.max_feed = tk.DoubleVar(value=15000.0)
        self.speed_mode = tk.StringVar(value="Mid")

        # --- Header mit Titel + aktuellem Wert ---
        hdr = ttk.Frame(spdf)
        hdr.pack(fill="x", pady=(4, 2))
        ttk.Label(hdr, text="Speed:", font=("Segoe UI", 10, "bold")).pack(side=tk.LEFT, padx=(8, 2))
        self.lbl_speed_val = ttk.Label(hdr, text=f"{self.speed_val.get():.1f}", width=6, anchor="e")
        self.lbl_speed_val.pack(side=tk.LEFT, padx=(2, 8))

        # --- MaxSpeed-Mode Buttons (Low / Mid / High) ---
        mode_frame = ttk.Frame(spdf)
        mode_frame.pack(pady=(0, 4))
        style = ttk.Style()
        style.configure("SpeedMode.TButton", font=("Segoe UI", 8))
        def _set_mode(mode):
            modes = {"Low": 6000.0, "Mid": 15000.0, "High": 50000.0}
            self.speed_mode.set(mode)
            self.max_feed.set(modes[mode])
            self.log(f"Max-Speed-Mode: {mode} ({modes[mode]} mm/min)")
        for m in ("Low", "Mid", "High"):
            ttk.Button(mode_frame, text=m, width=6, style="SpeedMode.TButton",
                       command=lambda mm=m: _set_mode(mm)).pack(side=tk.LEFT, padx=2)

        # --- Speed-Slider (0–1000) ---
        speed_slider = ttk.Scale(
            spdf, from_=0, to=1000, variable=self.speed_val,
            orient=tk.HORIZONTAL, length=220
        )
        speed_slider.pack(padx=8, pady=(0, 6))

        # --- Prozent-Buttons ---
        pct_frame = ttk.Frame(spdf)
        pct_frame.pack(pady=(0, 0))
        style.configure("SpeedPct.TButton", font=("Segoe UI", 8))
        for pct in (0, 20, 40, 50, 60, 80, 100):
            ttk.Button(
                pct_frame,
                text=f"{pct}%",
                width=3,
                style="SpeedPct.TButton",
                command=lambda p=pct: self._set_speed_percent(p)
            ).pack(side=tk.LEFT, padx=0)

        # --- Label-Update ---
        def _update_speed_label(*_):
            self.lbl_speed_val.config(text=f"{self.speed_val.get():.1f}")
        self.speed_val.trace_add("write", _update_speed_label)

        # --- Speed-Mapping & interne Funktionen ---
        def map_speed(val: float) -> float:
            return (val / 1000.0) * self.max_feed.get()

        def _set_speed_percent_local(pct: int):
            val = max(0, min(1000, int(10 * pct)))
            self.speed_val.set(val)
            mm = map_speed(val)
            self.lbl_speed_val.config(text=f"{self.speed_val.get():.1f}")
            self.log(f"Speed auf {pct}% gesetzt ({val}/1000 → {mm:.0f} mm/min @ {self.speed_mode.get()})")

        self._set_speed_percent = _set_speed_percent_local

        # =====================================================
        # 🏠 HOMING / CONTROL – 3×4 Matrix IM Speed/Accel-Frame
        # =====================================================
        ctrl = ttk.LabelFrame(spdf, text="Homing / Control")
        ctrl.pack(fill=tk.X, padx=6, pady=(8, 4))

        btns = [
            ("🏠 $H", lambda: self.send_now("$H")),
            ("🏠 $HX", lambda: self.send_now("$HX")),
            ("🏠 $HY", lambda: self.send_now("$HY")),
            ("🏠 $HZ", lambda: self.send_now("$HZ")),
            ("🏠 $HA", lambda: self.send_now("$HA")),
            ("🏠 $HB", lambda: self.send_now("$HB")),
            ("🏠 $HC", lambda: self.send_now("$HB")),
            ("🔓 $X", lambda: self.send_now("$X")),
            ("⚙️ Zero", self.do_zero),
            ("↩ Home", self.goto_home),
            ("↩→Q", self.enqueue_home),
            ("🖐️ Auf", lambda: self.send_now("M3 S0")),
            ("✊ Zu", lambda: self.send_now("M3 S1000")),
            ("Pose 1", lambda: self.send_now("G1 X45 Y90 Z45 A0 B0 C0 F6000")),
            ("Pose 2", lambda: self.send_now("G1 X45 Y90 Z-45 A0 B0 C0 F6000")),
        ]
        # Buttons im Grid (3 Spalten × 4 Zeilen)
        for i, (txt, cmd) in enumerate(btns):
            r, c = divmod(i, 3)
            ttk.Button(ctrl, text=txt, width=8, command=cmd)\
                .grid(row=r, column=c, padx=2, pady=2, sticky="ew")

        for c in range(3):
            ctrl.columnconfigure(c, weight=1)

        # =====================================================
        # 🟩 Farbiger STATUS-BLOCK direkt unter Tilt
        # (eigener StringVar, nicht status_var überschreiben)
        # =====================================================
        self.status_block_var = tk.StringVar(value="Status: —")
        self.lbl_status_block = tk.Label(
            spdf,
            textvariable=self.status_block_var,
            font=("Segoe UI", 10, "bold"),
            width=22,
            height=2,
            relief="groove",
            bg="#777777",
            fg="white",
            anchor="w",
            padx=8
        )
        self.lbl_status_block.pack(fill="x", padx=8, pady=(4, 8))

        # =====================================================
        # 🧲 Endstop-Anzeige – kompakt unter Status
        # =====================================================
        endstop_frame = ttk.Frame(spdf)
        endstop_frame.pack(fill="x", padx=8, pady=(0, 4))

        self.endstop_indicators = {}

        for ax in AXES:
            col = ttk.Frame(endstop_frame)
            col.pack(side=tk.LEFT, expand=True, padx=3)

            # Kreis (Canvas)
            c = tk.Canvas(col, width=20, height=20, highlightthickness=0)
            oval = c.create_oval(3, 3, 17, 17, fill="#999999", outline="")
            c.pack()
            # Label unter Kreis
            ttk.Label(col, text=ax, font=("Segoe UI", 8)).pack(pady=(0, 0))

            self.endstop_indicators[ax] = (c, oval, "unknown")

        # =====================================================
        # 🌍 TCP Pose – zentrale 6DOF FK Anzeige
        # =====================================================

        self.tcp_panel = TcpPosePanel(
            master=spdf,
            client=self.client,
            geom_dh=GEOM_DH,
            title="TCP Pose",
            poll_interval_ms=150
        )
        self.tcp_panel.pack(
            fill="x",
            padx=8,
            pady=(6, 10)
        )

        # Panel aktivieren (Start des Update-Loops)
        try:
            self.tcp_panel.start()
        except:
            pass

        # ---- Set All Buttons ----
        allf = ttk.Frame(posf)
        allf.pack(pady=6)
        ttk.Button(allf, text="🚀 To Queue", command=_btn_pose_toQ).pack(side=tk.LEFT, padx=2)
        ttk.Button(allf, text="⚙️ Set All Now", command=self.set_all_now).pack(side=tk.LEFT, padx=2)




        # ========================
        # -- PROGRAM / QUEUE --
        # ========================
        prog = ttk.LabelFrame(outer, text="Program / Queue")
        prog.pack(fill=tk.BOTH, expand=True, pady=6)

        # ---- Command Line ----
        cline = ttk.Frame(prog)
        cline.pack(fill=tk.X, padx=2, pady=(2, 2))
        ttk.Label(cline, text="Command Line (G-Code):").pack(side=tk.LEFT)
        self.entry_cmd = ttk.Entry(cline)
        self.entry_cmd.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=6)
        ttk.Button(cline, text="→ Queue", command=self.cli_add_to_program).pack(side=tk.LEFT, padx=3)
        ttk.Button(cline, text="Send Now", command=self.cli_send_now).pack(side=tk.LEFT, padx=3)

        # ---- Log Window ----
        self.txt_log = tk.Text(prog, height=4)
        self.txt_log.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # ---- Queue List ----
        self.listbox_queue = tk.Listbox(prog, height=5)
        self.listbox_queue.pack(fill=tk.X, padx=6, pady=(0, 6))

        # ---- Queue Buttons ----
        pbtn = ttk.Frame(prog)
        pbtn.pack(fill=tk.X, padx=6, pady=(2, 8))
        ttk.Button(pbtn, text="Load (→ Queue)", command=self.load_program).pack(side=tk.LEFT)
        ttk.Button(pbtn, text="💾 Save Queue", command=self.save_program).pack(side=tk.LEFT, padx=2)
        ttk.Button(pbtn, text=" Clear Queue", command=self.clear_program).pack(side=tk.LEFT, padx=2)      
        ttk.Button(pbtn, text="▶ Run", command=self.start_run).pack(side=tk.LEFT, padx=2)
        ttk.Button(pbtn, text="⏸ Pause (!)", command=self.pause_run).pack(side=tk.LEFT, padx=2)
        ttk.Button(pbtn, text="▶ Resume (~)", command=self.resume_run).pack(side=tk.LEFT, padx=2)
        ttk.Button(pbtn, text="⛔ Stop / Abbruch", command=self.stop_abort).pack(side=tk.LEFT, padx=6)
        ttk.Checkbutton(pbtn, text="Repeat", variable=self.repeat).pack(side=tk.LEFT, padx=6)
        ttk.Label(pbtn, text="x").pack(side=tk.LEFT, padx=(2, 0))
        ttk.Entry(pbtn, textvariable=self.repeat_times, width=4, justify="center").pack(side=tk.LEFT, padx=(2, 8))
        ttk.Label(pbtn, text="Runs:").pack(side=tk.LEFT, padx=2)
        self.lbl_repeat_status = ttk.Label(pbtn, textvariable=self.repeat_count)
        self.lbl_repeat_status.pack(side=tk.LEFT)
        self._init_cli_history()
        self._init_queue_edit()


    # expose live tcp pose to children
    def get_current_tcp_mm(self):
        try:
            return self.tcp_panel.get_current_tcp_mm()
        except:
            return {"X_mm":0,"Y_mm":0,"Z_mm":0,"Roll_deg":0,"Pitch_deg":0,"Yaw_deg":0}


    # ---------- Helpers / Logging ----------
    def log(self, s: str):
        self.txt_log.insert(tk.END, s + "\n")
        self.txt_log.see(tk.END)

    def send_now(self, g: str):
        try:
            self.client.send_line(g)
            self.log("TX: " + g)
        except Exception as e:
            self.log("Sendefehler: " + str(e))

    def insert_delay(self):
        """Fragt eine Wartezeit ab und fügt G4 P<time> in die Queue ein."""
        try:
            t = tk.simpledialog.askfloat("Insert Delay", "Wartezeit in Sekunden:", minvalue=0.0, maxvalue=3600.0)
            if t is None:
                return
            g = f"G4 P{t:.3f} ; Pause {t:.3f}s"
            self.enqueue(g)
            self.log(f"⏱ Delay {t:.3f}s → Queue eingefügt")
        except Exception as e:
            self.log(f"Delay-Fehler: {e}")

    # ---------- CLI ----------
    def _parse_cli(self):
        txt = self.entry_cmd.get().strip()
        if not txt: raise ValueError("Eingabe leer.")
        return [ln.strip() for ln in txt.splitlines() if ln.strip()]

    def _init_cli_history(self):
        self.cli_history = []
        self.cli_hist_index = -1
        self.entry_cmd.bind("<Return>", self._on_cli_return)
        self.entry_cmd.bind("<Up>", self._on_cli_up)
        self.entry_cmd.bind("<Down>", self._on_cli_down)

    def _add_to_history(self, cmd: str):
        if cmd and (not self.cli_history or self.cli_history[-1] != cmd):
            self.cli_history.append(cmd)
        self.cli_hist_index = len(self.cli_history)

    def _on_cli_return(self, event=None):
        cmd = self.entry_cmd.get().strip()
        if not cmd: return "break"
        self._add_to_history(cmd)
        try:
            self.client.send_line(cmd)
            self.log(f"TX (CLI ⏎): {cmd}")
        except Exception as e:
            self.log(f"Sendefehler: {e}")
        self.entry_cmd.delete(0, tk.END)
        return "break"

    def _on_cli_up(self, event=None):
        if not self.cli_history: return "break"
        if self.cli_hist_index > 0: self.cli_hist_index -= 1
        self.entry_cmd.delete(0, tk.END)
        self.entry_cmd.insert(0, self.cli_history[self.cli_hist_index])
        return "break"

    def _on_cli_down(self, event=None):
        if not self.cli_history: return "break"
        if self.cli_hist_index < len(self.cli_history) - 1:
            self.cli_hist_index += 1
            self.entry_cmd.delete(0, tk.END)
            self.entry_cmd.insert(0, self.cli_history[self.cli_hist_index])
        else:
            self.cli_hist_index = len(self.cli_history)
            self.entry_cmd.delete(0, tk.END)
        return "break"

    def cli_add_to_program(self):
        try:
            lines = self._parse_cli()
            for ln in lines: self.enqueue(ln); self._add_to_history(ln)
            self.log(f"CLI: {len(lines)} zur Queue hinzugefügt.")
        except Exception as e:
            messagebox.showerror("CLI-Fehler", str(e))

    def cli_send_now(self):
        try:
            lines = self._parse_cli()
            sent = 0
            for ln in lines:
                try:
                    self.client.send_line(ln); self._add_to_history(ln)
                    self.log("TX (CLI): " + ln)
                    sent += 1
                except Exception as e:
                    self.log("Send Error: " + str(e))
            messagebox.showinfo("Send Now", f"{sent}/{len(lines)} gesendet.")
        except Exception as e:
            messagebox.showerror("CLI-Fehler", str(e))

    # ---------- Queue ops ----------
    def enqueue(self, gline: str):
        g = gline.strip()
        if not g: return
        self.program.append(g)
        self.listbox_queue.insert(tk.END, g)
        self.log("Queued: " + g)

    def clear_program(self):
        self.program.clear()
        self.listbox_queue.delete(0, tk.END)
        self.log("Queue cleared.")

    def load_program(self):
        path = filedialog.askopenfilename(filetypes=[("GCode/All", "*.gcode *.nc *.txt *.ngc *.tap *.*")])
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                lines = [ln.strip() for ln in f.readlines()]
            n = 0
            for ln in lines:
                if ln and not ln.startswith(";"):
                    self.enqueue(ln); n += 1
            self.log(f"Loaded {n} lines from {os.path.basename(path)}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def save_program(self):
        if not self.program:
            messagebox.showinfo("Save Queue", "Queue ist leer."); return
        path = filedialog.asksaveasfilename(defaultextension=".gcode", filetypes=[("GCode", "*.gcode"), ("Text", "*.txt")])
        if not path: return
        try:
            with open(path, "w", encoding="utf-8") as f:
                for ln in self.program: f.write(ln + "\n")
            self.log(f"Saved {len(self.program)} lines to {path}")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    # ---------- Queue Edit (DnD / Edit / Insert / Delete) ----------
    def _init_queue_edit(self):
        lb = self.listbox_queue
        lb.bind("<Delete>", self._queue_delete)
        lb.bind("<Insert>", self._queue_insert)
        lb.bind("<Double-Button-1>", self._queue_edit_item)

        # Drag&Drop Reorder
        lb.bind("<Button-1>", self._queue_on_click)
        lb.bind("<B1-Motion>", self._queue_on_drag)
        lb.bind("<ButtonRelease-1>", self._queue_on_drop)
        self._drag_index = None

    def _queue_delete(self, event=None):
        sel = self.listbox_queue.curselection()
        if not sel: return "break"
        idx = sel[0]
        self.listbox_queue.delete(idx)
        del self.program[idx]
        self.log(f"Deleted line {idx}")
        return "break"

    def _queue_insert(self, event=None):
        idx = self.listbox_queue.curselection()
        ins = idx[0] if idx else tk.END
        cmd = simpledialog.askstring("Insert G-Code", "G-Code Zeile:")
        if cmd:
            if ins == tk.END:
                self.program.append(cmd); self.listbox_queue.insert(tk.END, cmd)
            else:
                self.program.insert(ins, cmd); self.listbox_queue.insert(ins, cmd)
            self.log("Inserted: " + cmd)
        return "break"

    def _queue_edit_item(self, event=None):
        sel = self.listbox_queue.curselection()
        if not sel: return
        idx = sel[0]
        old = self.program[idx]
        new = simpledialog.askstring("Edit G-Code", "G-Code Zeile bearbeiten:", initialvalue=old)
        if new and new.strip() != old:
            self.program[idx] = new.strip()
            self.listbox_queue.delete(idx)
            self.listbox_queue.insert(idx, new.strip())
            self.listbox_queue.selection_set(idx)
            self.log(f"Edited line {idx}")

    def _queue_on_click(self, event):
        self._drag_index = self.listbox_queue.nearest(event.y)

    def _queue_on_drag(self, event):
        if self._drag_index is None: return
        i = self.listbox_queue.nearest(event.y)
        if i < 0 or i >= self.listbox_queue.size(): return
        if i == self._drag_index: return
        # Swap in listbox
        txt_from = self.listbox_queue.get(self._drag_index)
        txt_to   = self.listbox_queue.get(i)
        self.listbox_queue.delete(self._drag_index)
        self.listbox_queue.insert(self._drag_index, txt_to)
        self.listbox_queue.delete(i)
        self.listbox_queue.insert(i, txt_from)
        # Swap in program
        self.program[self._drag_index], self.program[i] = self.program[i], self.program[self._drag_index]
        self.listbox_queue.selection_clear(0, tk.END)
        self.listbox_queue.selection_set(i)
        self._drag_index = i

    def _queue_on_drop(self, event):
        self._drag_index = None

    def clamp_with_limits(self, ax, v):
        """Begrenzt eine Zielposition auf definierte Achsenlimits."""
        try:
            val = float(v)
        except Exception:
            val = 0.0
        lo, hi = self.axis_limits.get(ax, (-9999.0, 9999.0))
        return max(lo, min(hi, val))

    # ============================================================
    # ACHSENFUNKTIONEN: Queue / CLI / DirectSend
    # ============================================================

    def axis_to_queue(self, ax: str, val: float):
        """Achse mit Wert als G1-Befehl in Queue einfügen."""
        try:
            val = float(val)
            g = f"G1 {ax}{val:.3f} F{map_speed(int(self.speed_val.get()))}"
            self.enqueue(g)
            self.log(f"[ToQ] {g}")
        except Exception as e:
            self.log(f"[ToQ ERROR] {e}")

    def axis_to_cli(self, ax: str, val: float):
        """Achsenbewegung in CLI-Feld schreiben."""
        try:
            val = float(val)
            g = f"G1 {ax}{val:.3f} F{map_speed(int(self.speed_val.get()))}"
            self.entry_cmd.delete(0, tk.END)
            self.entry_cmd.insert(0, g)
            self.log(f"[ToCLI] {g}")
        except Exception as e:
            self.log(f"[ToCLI ERROR] {e}")

    def axis_direct_send(self, ax: str, val: float):
        """Direkt G1 senden (ohne Queue)."""
        try:
            val = float(val)
            f = map_speed(int(self.speed_val.get()))
            g = f"G1 {ax}{val:.3f} F{f}"

            self.client.send_line("$X")
            time.sleep(0.02)
            self.client.send_line("G90")
            time.sleep(0.02)

            self.client.send_line(g)
            self.axis_positions[ax] = val
            self.axis_vars[ax].set(val)
            self.update_position_display()

            self.log(f"[SEND] {g}")
        except Exception as e:
            self.log(f"[Send ERROR] {e}")

    def do_zero(self):
        try:
            self.client.send_line("G92 X0 Y0 Z0 A0 B0 C0")
            for ax in AXES: self.axis_positions[ax] = 0.0
            self.update_position_display()
            self.log("Zero All (G92)")
        except Exception as e:
            self.log("Zero-Fehler: " + str(e))

    def goto_home(self):
        try:
            # Original: gemappter Feed/Accel
            f = map_speed(int(self.speed_val.get()))
            a = map_speed(int(self.accel_val.get()))

            # ---- Nur für GOTO HOME limitieren ----
            f_home = min(f, MAX_HOME_FEED)

            self.client.send_line("$X"); time.sleep(0.02)   # entsperren
            self.client.send_line("G90"); time.sleep(0.02)  # absolut

            g = f"G1 X0 Y0 Z0 A0 B0 C0 F{f_home}"
            self.client.send_line(g)

            self.log(f"TX: {g} (Goto Home, F_user={f}, F_home={f_home}, A={a})")

        except Exception as e:
            self.log("Goto-Home-Fehler: " + str(e))

    def enqueue_home(self):
        f = map_speed(int(self.speed_val.get()))
        g = f"G90 G1 X0 Y0 Z0 A0 B0 C0 F{f}"
        self.enqueue(g)
        self.log("↩ To Home → Queue hinzugefügt")

    def enqueue_axis(self, ax: str, val: float):
        g = self._make_move_line({ax: val})
        self.enqueue(g)

    def set_all_to_queue(self):
        targets = {}
        for ax in AXES:
            try: v = float(self.axis_entries[ax].get())
            except: v = 0.0
            # clamp inkl. HW-Limits
            if ax in self.hw_limits:
                lo2, hi2 = self.hw_limits[ax]
                v = max(lo2, min(hi2, v))
            else:
                lo, hi = AXIS_LIMITS_DEFAULT[ax]
                v = max(lo, min(hi, v))
            targets[ax] = v
        g = self._make_move_line(targets)
        self.enqueue(g)
        self.log("To Queue: " + g)

    # ============================================================
    # POSE → QUEUE (für Gamepad & Button "🚀 To Queue")
    # ============================================================
    def add_current_pose_to_queue(self):
        """
        Read all axis entries from the Manual tab, clamp to limits,
        build a G1 line, and enqueue it.
        """
        try:
            parts = []

            for ax in AXES:
                ent = self.axis_entries.get(ax)
                if ent is None:
                    self.log(f"[add_current_pose_to_queue] Missing entry for {ax}")
                    return

                raw = ent.get().strip().replace(",", ".")
                if not raw:
                    self.log(f"[add_current_pose_to_queue] Empty entry for {ax}")
                    return
                try:
                    v = float(raw)
                except Exception:
                    self.log(f"[add_current_pose_to_queue] Invalid value for {ax}: {raw}")
                    return

                if ax in self.hw_limits:
                    lo, hi = self.hw_limits[ax]
                else:
                    lo, hi = self.axis_limits.get(ax, AXIS_LIMITS_DEFAULT[ax])

                v = max(lo, min(hi, v))
                parts.append(f"{ax}{v:.3f}")

            base_F = map_speed(int(self.speed_val.get()))
            msf = getattr(self, "manual_speed_factor", None)
            if msf is not None:
                base_F *= float(msf.get())
            F = max(1.0, base_F)

            gline = "G1 " + " ".join(parts) + f" F{F:.0f}"
            self.enqueue(gline)
            self.log(f"Pose -> Queue (Manual): {gline}")

        except Exception as e:
            self.log(f"[add_current_pose_to_queue ERROR] {e}")


    def set_all_now(self):
        targets = {}
        for ax in AXES:
            try: v = float(self.axis_entries[ax].get())
            except: v = 0.0
            if ax in self.hw_limits:
                lo2, hi2 = self.hw_limits[ax]
                v = max(lo2, min(hi2, v))
            else:
                lo, hi = AXIS_LIMITS_DEFAULT[ax]
                v = max(lo, min(hi, v))
            targets[ax] = v
        g = self._make_move_line(targets)
        try:
            self.client.send_line("G90" if self.mode_absolute.get() else "G91")
            time.sleep(0.01)
            self.client.send_line(g)
            self.log("TX (Set All Now): " + g)
        except Exception as e:
            self.log("Sendefehler Set All Now: " + str(e))

    # ---------- Worker / Steuerung ----------
    def start_run(self):
        if not self.program:
            self.log("⚠️ Queue ist leer.")
            return
        self.abort_event.clear()
        self.paused = False
        self.repeat_count.set(0)
        self.log("▶ RUN gestartet")
        self.run_event.set()

    def pause_run(self):
        """Pausiert aktuelle Bewegung (Feed Hold)."""
        try:
            self.client.send_line("!")
            self.paused = True
            self.log("⏸ Pause (Feed Hold gesendet)")
        except Exception as e:
            self.log(f"Pause-Fehler: {e}")

    def resume_run(self):
        """Setzt pausierte Bewegung fort."""
        if not self.paused:
            self.log("ℹ️ Keine Pause aktiv.")
            return
        try:
            self.client.send_line("~")
            self.paused = False
            self.log("▶ Resume (~) gesendet")
        except Exception as e:
            self.log(f"Resume-Fehler: {e}")

    def stop_abort(self):
        """Komplettabbruch: Queue stoppen + Events zurücksetzen und Steuerung sofort freigeben."""
        self.log("⛔ STOP/ABBRUCH gedrückt")
        self.abort_event.set()
        self.run_event.clear()
        self.paused = False
        self._awaiting_ack = False
        try:
            self.client.send_ctrl_x()  # Hard reset ist hier gewollt
            self.log("🛑 Ctrl-X (Reset) gesendet")
        except Exception as e:
            self.log(f"Abbruch-Fehler: {e}")
        self.log("✅ System bereit für neue Befehle")

    # Alias für Gamepad-Shortcut (Kompatibilität)
    def emergency_stop(self):
        """Alias für Stop, damit Gamepad-Aufruf kompatibel bleibt."""
        self.stop_abort()




    def worker(self):
        """Hintergrundthread für Queue-Ausführung."""
        while True:
            self.run_event.wait()
            if self.abort_event.is_set():
                time.sleep(0.05)
                continue
            if not self.program:
                time.sleep(0.05)
                continue

            lines = list(self.program)
            self.log(f"▶ Starte Programmausführung ({len(lines)} Zeilen)")
            self._awaiting_ack = False

            for i, g in enumerate(lines, start=1):
                # Pause/Abbruch prüfen
                while self.paused and not self.abort_event.is_set():
                    time.sleep(0.05)
                if self.abort_event.is_set() or not self.run_event.is_set():
                    break

                f_now = map_speed(int(self.speed_val.get()))
                a_now = map_speed(int(self.accel_val.get()))
                if "G1" in g or "G0" in g:
                    import re
                    g = re.sub(r"F[0-9\.]+", f"F{f_now}", g) if "F" in g else f"{g} F{f_now}"

                self.current_cmd = g
                self._awaiting_ack = True
                self.log(f"[{i}/{len(lines)}] TX: {g} (F={f_now}, A={a_now})")

                try:
                    self.client.send_line(g)
                except Exception as e:
                    self.log(f"❌ TX-Fehler: {e}")
                    self._awaiting_ack = False
                    continue

                deadline = time.time() + DEFAULT_TIMEOUT
                while time.time() < deadline:
                    if self.abort_event.is_set() or not self.run_event.is_set():
                        self._awaiting_ack = False
                        break
                    if not self._awaiting_ack:
                        break
                    time.sleep(0.02)
                else:
                    self.log(f"⚠️ Timeout bei: {g}")
                    self._awaiting_ack = False

                self.current_cmd = None

            # Ende/Abbruch/Repeat
            if self.abort_event.is_set():
                self.log("❌ Programmausführung abgebrochen")
                self.abort_event.clear()
                self.run_event.clear()
                self.paused = False
                self._awaiting_ack = False
                continue

            if self.repeat.get() and not self.abort_event.is_set():
                self.repeat_count.set(self.repeat_count.get() + 1)
                total = max(1, self.repeat_times.get())

                if self.repeat_count.get() < total:
                    self.log(f"🔁 Wiederhole Queue ({self.repeat_count.get()}/{total})")
                    continue
                else:
                    self.log(f"✅ Repeat abgeschlossen ({self.repeat_count.get()}/{total})")


            self.run_event.clear()
            self.paused = False
            self._awaiting_ack = False
            self.log("✅ Queue finished – bereit für neuen Start")

    def _update_status_block(self, state: str):
        """Färbt und beschriftet den Status-Block basierend auf dem Maschinenzustand."""
        st = (state or "—").lower()
        color = "#777777"   # neutral
        if "idle" in st:
            color = "#2e7d32"   # grün
        elif "run" in st:
            color = "#1565c0"   # blau
        elif "hold" in st:
            color = "#f9a825"   # gelb
        elif "alarm" in st or "error" in st:
            color = "#b71c1c"   # rot
        elif "home" in st:
            color = "#00897b"   # türkis
        elif "disconnect" in st:
            color = "#616161"   # grau

        self.status_block_var.set(f"Status: {state}")
        # tk.Label (nicht ttk) → bg funktioniert sicher
        self.lbl_status_block.configure(bg=color, fg=("black" if color == "#f9a825" else "white"))


    # ---------- RX / Status ----------
    def on_serial_line(self, line: str):
        if not line: return
        if line == "ok":
            if self._awaiting_ack: self._awaiting_ack = False
            return
        if line.startswith("error:"):
            self.log("RX: " + line)
            if self._awaiting_ack: self._awaiting_ack = False
            return


        # $$ softlimit parser (geht für FluidNC und GRBL)
        for ax, rx in SOFTMAX_RE.items():
            m = rx.match(line)
            if m:
                try:
                    max_travel = float(m.group(1))
                    self.hw_limits[ax] = (0.0, max_travel)
                    if ax in self.axis_limit_labels:
                        self.axis_limit_labels[ax].config(text=f"[0..{max_travel:.0f}]")
                except:
                    pass


        # =========================================================
        # 📡 Gemeinsamer Status-Parser für FluidNC & GRBL
        # Format typischerweise:
        #   <Idle|MPos:0.000,0.000,0.000|FS:0,0|Pn:X>
        #   <Run|MPos:...|FS:...>
        # =========================================================
        if line.startswith("<") and line.endswith(">"):
            payload = line[1:-1]
            parts = payload.split("|")
            if not parts:
                return

            state = parts[0]
            self._update_status_block(state)

            mpos = {}
            wpos = {}
            endstop_active = set()

            for part in parts[1:]:
                if part.startswith("MPos:"):
                    nums = part[5:].split(",")
                    for idx, ax in enumerate(AXES):
                        if idx < len(nums):
                            try:
                                mpos[ax] = float(nums[idx])
                            except ValueError:
                                pass

                elif part.startswith("WPos:"):
                    nums = part[5:].split(",")
                    for idx, ax in enumerate(AXES):
                        if idx < len(nums):
                            try:
                                wpos[ax] = float(nums[idx])
                            except ValueError:
                                pass

                elif part.startswith("Pn:") and self.client.supports_endstops:
                    endstop_active = set(part[3:])

            # --- Endstops nur bei FluidNC sinnvoll ---
            if self.client.supports_endstops and endstop_active is not None:
                for ax in AXES:
                    if ax in self.axis_endstop_icons:
                        c, oval, state_old = self.axis_endstop_icons[ax]
                        new_state = "tripped" if ax in endstop_active else "free"
                        if new_state != state_old:
                            color = "#b71c1c" if new_state == "tripped" else "#2e7d32"
                            c.itemconfig(oval, fill=color)
                            self.axis_endstop_icons[ax] = (c, oval, new_state)

                    if hasattr(self, "endstop_indicators") and ax in self.endstop_indicators:
                        c2, oval2, state2 = self.endstop_indicators[ax]
                        new_state = "tripped" if ax in endstop_active else "free"
                        if new_state != state2:
                            color = "#b71c1c" if new_state == "tripped" else "#2e7d32"
                            c2.itemconfig(oval2, fill=color)
                            self.endstop_indicators[ax] = (c2, oval2, new_state)

            # --- Achspositionen aktualisieren (MPos bevorzugt) ---
            pose_src = mpos or wpos
            abs_pose = {}

            updated = False
            for ax, val in pose_src.items():
                if ax not in AXES:
                    continue
                abs_pose[ax] = val
                if abs(val - self.axis_positions.get(ax, 0.0)) > MOTION_EPS and not self._user_editing.get(ax, False):
                    self.axis_positions[ax] = val
                    if ax in self.axis_vars:
                        self.axis_vars[ax].set(val)
                    updated = True

            # Status für andere Teile merken
            self.client.last_status = {"state": state, "MPos": mpos, "WPos": wpos}

            # --- UDP abs-Positionsbroadcast ---
            if UDP_MIRROR:
                try:
                    import json, time as _time

                    # Basis: das, was vom Controller kam
                    full_pose = dict(abs_pose)

                    # Fehlende Achsen mit letzter bekannter GUI-Position auffüllen
                    for ax in AXES:
                        if ax not in full_pose and ax in self.axis_positions:
                            full_pose[ax] = float(self.axis_positions[ax])

                    if full_pose:
                        msg = {"type": "abs", "timestamp": _time.time()}
                        msg.update(full_pose)
                        self.client.udp_sock.sendto((json.dumps(msg) + "\n").encode("utf-8"), UDP_ADDR)

                except Exception as e:
                    self.log(f"[UDP abs send fail] {e}")



        # Sonstiges
            if not (
                line.startswith("<") or
                line == "ok" or
                line.startswith("[MSG:") or
                line.startswith("[GC:") or
                "MPos:" in line or
                "WPos:" in line or
                "FS:"   in line or
                "Ov:"   in line
            ):
                self.log("RX: " + line)
    # ---------- $$ Parser ----------
    def request_and_parse_settings(self):
        """Sendet '$$' und parst die Rückgaben (Softlimits etc.)."""
        self.log("TX: $$ (Settings abfragen)")
        try:
            # Alte HW-Limits-Anzeige löschen
            for ax in self.hw_limits:
                self.axis_limit_labels[ax].configure(text="")
            self.hw_limits.clear()

            # Anfrage senden
            self.client.send_line("$$")
        except Exception as e:
            self.log("$$-Fehler: " + str(e))


    def update_position_display(self):
        for ax in AXES:
            self.axis_labels[ax].configure(text=f"{self.axis_positions[ax]:.3f}")


    # ============================================================
    # GAMEPAD → ExecuteApp Dispatcher
    # (wird von gamepad_block_v3_x aufgerufen)
    # ============================================================
    def on_gamepad_trigger(self, name, payload):
        """
        Zentraler Dispatcher für Gamepad-Aktionen.
        'name' ist ein String wie 'tcp_add_pose_to_queue',
        'payload' ein optionales Dict (derzeit kaum genutzt).
        """
        try:
            self.log(f"[GAMEPAD] Trigger → {name}  payload={payload}")
        except Exception:
            print(f"[GAMEPAD] Trigger → {name}  payload={payload}")

        # ---------------------------------------------------------
        # 1) aktuelle Pose zur Queue hinzufügen
        # ---------------------------------------------------------
        if name == "tcp_add_pose_to_queue":
            if hasattr(self, "add_current_pose_to_queue"):
                return self.add_current_pose_to_queue()
            self.log("⚠️ METHODE fehlt: add_current_pose_to_queue()")
            return

        # ---------------------------------------------------------
        # 2) Preview der TCP-Sequenz
        # ---------------------------------------------------------
        elif name == "tcp_seq_preview":
            # 1) Kinematics Tabs prüfen
            if hasattr(self, "kinematics_tabs"):
                for attr in (
                    "gamepad_preview_sequence",
                    "preview_sequence",
                    "preview_tcp_sequence",
                    "seq_preview",     # häufig
                    "preview",         # fallback
                ):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()

            # 2) Fallback: direkt ExecuteApp Preview
            if hasattr(self, "seq_preview"):
                return self.seq_preview()

            self.log("⚠️ Keine Preview-Funktion (weder in kinematics_tabs noch ExecuteApp).")
            return

        # ---------------------------------------------------------
        # 3) Sequenz ausführen
        # ---------------------------------------------------------
        elif name == "tcp_seq_execute_cli":
            if hasattr(self, "kinematics_tabs"):
                for attr in ("gamepad_execute_sequence", "execute_sequence_cli", "execute_tcp_sequence"):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()
            # Fallback: komplette Queue abfahren
            if hasattr(self, "start_run"):
                return self.start_run()
            self.log("⚠️ Keine Execute-Funktion für TCP-Sequenz gefunden.")
            return

        # ---------------------------------------------------------
        # 4) TCP als IK-Referenz setzen
        # ---------------------------------------------------------
        elif name == "tcp_set_tcp_as_reference":
            if hasattr(self, "kinematics_tabs"):
                for attr in ("set_tcp_as_reference", "use_current_tcp_as_ref"):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()
            self.log("⚠️ Keine Funktion zum Setzen der TCP-Referenz gefunden.")
            return

        # ---------------------------------------------------------
        # 5) Solve & Run komplette TCP-Sequenz
        # ---------------------------------------------------------
        elif name == "tcp_solve_seq_and_execute":
            if hasattr(self, "kinematics_tabs"):
                for attr in ("solve_and_execute_sequence", "solve_and_run_tcp_sequence"):
                    if hasattr(self.kinematics_tabs, attr):
                        return getattr(self.kinematics_tabs, attr)()
            self.log("⚠️ Keine Solve-&-Run-Funktion in kinematics_tabs gefunden.")
            return

        # ---------------------------------------------------------
        # 6) Unbekannt
        # ---------------------------------------------------------
        else:
            self.log(f"⚠️ Unbekannter Gamepad-Trigger: {name}")
            return



    def _tick_status_poll(self):
        if self.poll_positions.get():
            try: self.client.send_line("?")
            except Exception as e:
                print("[Warn]", e)
        cur = (self.status_block_var.get() or "")
        delay = 150 if ("Run" in cur or "Hold" in cur) else 250
        self.after(delay, self._tick_status_poll)

# ---- App-Instanz ----
execute_app = ExecuteApp(root, client)
execute_app.pack(fill="both", expand=True)

def on_close():
    try:
        # TCP-Panel stoppen
        if hasattr(execute_app, "tcp_panel"):
            try:
                execute_app.tcp_panel.stop()
            except:
                pass

        # Gamepad stoppen
        if hasattr(execute_app, "stop_gamepad"):
            try:
                execute_app.stop_gamepad()
            except Exception as e:
                print("[on_close] stop_gamepad error:", e)

        # Seriell trennen
        try:
            client.disconnect()
        except Exception as e:
            print("[on_close] client.disconnect error:", e)

    except Exception as e:
        print("[on_close ERROR]", e)

    root.destroy()
root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
