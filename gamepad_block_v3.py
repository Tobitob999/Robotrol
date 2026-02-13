# ============================================================
# 🎮 gamepad_block_full_v3_6.py — Neue GUI + Button-Mapping
# ============================================================
# Anforderungen / Integration:
# - attach_gamepad_tab(tab_gamepad, client, execute_app) aufrufen
# - client.send_line(...) verfügbar
# - execute_app.speed_val (0..1000) vorhanden
# - execute_app.on_gamepad_trigger(name, payload) vorhanden (für Actions)
#
# Features:
# - GUI-Layout: oben Log (5 Zeilen), darunter 3 Spalten nebeneinander:
#   (links)  Parameter-Tabelle (Tabs slow/normal/fast) + Button-Aktionen
#   (mitte)  Start/Stop + Speichern/Neu laden/Defaults
#   (rechts) Speed-Mode Buttons + farbige Statusanzeige des aktiven Modus
# - 3 Speed-Modi (slow/normal/fast) mit eigenem Parameterblock pro Achse (X,Y,Z,A,B,C)
# - JSON-Presets: gamepad_config.json (wird geladen/gespeichert; current_mode inkl.)
# - Jog-Mapping:
#     * X,Y über Achsen 0/1 (linker Stick)
#     * Z über Achse 3 (rechter Stick vertikal)
#     * B (Roll) über Achse 2 (rechter Stick horizontal)
#     * A (Yaw) über LB/RB (Buttons 4/5) als diskreter Jog
#     * C (aux) über D-Pad links/rechts als kleiner Jog
#     * Greifer via Buttons 0/1 → M3 S[0..1000], Schrittweite = step*C*1000, Debounce 0.25s
#     * Greifer fein via Buttons 2/3 (X/Y) → kleinere Schritte
# - Konfigurierbare Aktionen (nur freie Knöpfe):
#     * BACK  (Button 6)
#     * START (Button 7)
#     * D-Pad UP
#     * D-Pad DOWN
#   Mögliche Actions:
#     - none
#     - add_current_pose_to_queue
#     - preview_queue_sequence
#     - execute_queue_sequence
#     - set_tcp_as_kinematics_ref
#     - solve_and_run_tcp_sequence
#     - goto_home
#     - goto_zero
#     - tbd_1, tbd_2, tbd_3
#
# - Feedrate: map(0..1000)→0..15000 * speed_mode_factor (0.4/1.0/2.0)
# - Sauberer Start/Stop-Thread
# ============================================================

import os
import json
import time
import threading
import pygame
import tkinter as tk
from tkinter import ttk

CONFIG_PATH = "gamepad_config.json"

SPEED_MODES = {"slow": 0.4, "normal": 1.0, "fast": 2.0}
AXES = ["X", "Y", "Z", "A", "B", "C"]  # C = Aux / Fein-Jog + Gripper-Step

# Belegbare Knöpfe (nur Buttons, die nicht durch Achsen/Gripper verwendet werden)
BUTTON_ACTION_KEYS = ["back", "start", "dpad_up", "dpad_down"]

# Mögliche Actions (werden im Dropdown angezeigt & gespeichert)
ACTION_CHOICES = [
    "none",
    "add_current_pose_to_queue",
    "preview_queue_sequence",
    "execute_queue_sequence",
    "pose_to_mask_and_generate",
    "set_tcp_as_kinematics_ref",
    "solve_and_run_tcp_sequence",
    "goto_home",
    "goto_zero",
    "tbd_1",
    "tbd_2",
    "tbd_3",
]

# Default-Belegung für die Knöpfe
DEFAULT_BUTTON_ACTIONS = {
    "back": "pose_to_mask_and_generate",
    "start": "solve_and_run_tcp_sequence",
    "dpad_up": "add_current_pose_to_queue",
    "dpad_down": "none",
}


def DEFAULT_AXIS():
    return {"step": 0.5, "alpha": 0.25, "deadzone": 0.35, "speed_scale": 1.0, "invert": 1}


# lokales Feed-Mapping: 0..1000 → 0..15000 (wie im Hauptprogramm)
def map_speed_linear(val_0_1000: int, max_feed: float = 15000.0) -> int:
    v = max(0, min(1000, int(val_0_1000)))
    return int((v / 1000.0) * max_feed)



def _rpy_to_R(roll_deg, pitch_deg, yaw_deg):
    yaw_r = math.radians(yaw_deg)
    pitch_r = math.radians(pitch_deg)
    roll_r = math.radians(roll_deg)

    cy, sy = math.cos(yaw_r), math.sin(yaw_r)
    cp, sp = math.cos(pitch_r), math.sin(pitch_r)
    cr, sr = math.cos(roll_r), math.sin(roll_r)

    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]

# --------------------------------------------
# UI: Konfig-Tabelle mit Tabs je Speed-Modus + Button-Aktionen
# --------------------------------------------
class GamepadConfigUI(ttk.Frame):
    def __init__(self, master, config_provider=None):
        super().__init__(master)
        self.pack(fill=tk.BOTH, expand=True)
        self.config_provider = config_provider
        # Datenstruktur je Modus
        self.modes = {m: {ax: DEFAULT_AXIS().copy() for ax in AXES} for m in SPEED_MODES}
        self.current_mode = tk.StringVar(value="normal")
        self.entries = {m: {} for m in SPEED_MODES}      # entries[mode][ax][key] -> tk.StringVar
        self.invert_vars = {m: {} for m in SPEED_MODES}  # invert_vars[mode][ax] -> tk.BooleanVar

        # Button-Aktionen (BACK/START/D-Pad Up/Down)
        self.button_actions = DEFAULT_BUTTON_ACTIONS.copy()
        self.button_action_vars = {}  # key -> StringVar

        self._load_config()
        self._build_ui()

    def _build_ui(self):
        title = ttk.Label(self, text="🎮 Gamepad-Parameter", font=("Segoe UI", 11, "bold"))
        title.pack(anchor="w", pady=(0, 6))

        # Tabs slow/normal/fast
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill=tk.BOTH, expand=True)

        for mode in SPEED_MODES:
            frm = ttk.Frame(self.nb)
            self.nb.add(frm, text=f"{mode.capitalize()} (×{SPEED_MODES[mode]})")
            self._build_table(frm, mode)

        # beim Tabwechsel current_mode setzen
        def on_tab_changed(event=None):
            idx = self.nb.index(self.nb.select())
            m = list(SPEED_MODES.keys())[idx]
            self.current_mode.set(m)

        self.nb.bind("<<NotebookTabChanged>>", on_tab_changed)

        # initial korrekt setzen
        try:
            wanted = list(SPEED_MODES.keys()).index(self.current_mode.get())
            self.nb.select(wanted)
        except Exception:
            pass

        # --- Button-Aktions-Mapping direkt unterhalb der Tabs ---
        self._build_button_mapping(self)

    def _build_table(self, parent, mode):
        # Header
        head = ttk.Frame(parent)
        head.pack(fill=tk.X, padx=4)
        for name, w in zip(["Achse", "Step", "Alpha", "Deadzone", "Speed", "Invert"], [6, 8, 8, 9, 8, 8]):
            ttk.Label(head, text=name, width=w, font=("Segoe UI", 9, "bold")).pack(side=tk.LEFT, padx=3)

        body = ttk.Frame(parent)
        body.pack(fill=tk.BOTH, expand=True, padx=4, pady=(2, 6))
        self.entries[mode] = {}
        self.invert_vars[mode] = {}

        for ax, vals in self.modes[mode].items():
            row = ttk.Frame(body)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=ax, width=6).pack(side=tk.LEFT, padx=3)
            self.entries[mode][ax] = {}
            for key, w in zip(["step", "alpha", "deadzone", "speed_scale"], [8, 8, 9, 8]):
                var = tk.StringVar(value=f"{float(vals[key]):.3f}")
                ttk.Entry(row, textvariable=var, width=w, justify="center").pack(side=tk.LEFT, padx=3)
                self.entries[mode][ax][key] = var
            inv_var = tk.BooleanVar(value=(vals["invert"] == -1))
            ttk.Checkbutton(row, variable=inv_var).pack(side=tk.LEFT, padx=6)
            self.invert_vars[mode][ax] = inv_var

    def _build_button_mapping(self, parent):
        frm = ttk.LabelFrame(parent, text="Button Aktionen (frei belegbar)")
        frm.pack(fill=tk.X, padx=4, pady=(8, 4))

        nice_names = {
            "back": "BACK (Button 6)",
            "start": "START (Button 7)",
            "dpad_up": "D-Pad ↑",
            "dpad_down": "D-Pad ↓",
        }

        for key in BUTTON_ACTION_KEYS:
            row = ttk.Frame(frm)
            row.pack(fill=tk.X, pady=2)
            lbl_txt = nice_names.get(key, key)
            ttk.Label(row, text=lbl_txt, width=18).pack(side=tk.LEFT, padx=3)
            var = tk.StringVar(value=self.button_actions.get(key, "none"))
            cb = ttk.Combobox(row, textvariable=var, state="readonly", width=32)
            cb["values"] = ACTION_CHOICES
            cb.pack(side=tk.LEFT, padx=4)
            self.button_action_vars[key] = var

    # ---- Datei-Handling ----
    def _load_config(self):
        try:
            data = None
            if self.config_provider and hasattr(self.config_provider, "get_gamepad_config"):
                try:
                    data = self.config_provider.get_gamepad_config()
                except Exception:
                    data = None
            if not isinstance(data, dict) or not data:
                if os.path.exists(CONFIG_PATH):
                    with open(CONFIG_PATH, "r", encoding="utf-8") as f:
                        data = json.load(f)
            if isinstance(data, dict) and data:
                # Achs-Parameter
                for m in SPEED_MODES:
                    for ax, vals in data.get("modes", {}).get(m, {}).items():
                        if ax in self.modes[m]:
                            self.modes[m][ax].update(vals)
                # aktueller Mode
                if "current_mode" in data and data["current_mode"] in SPEED_MODES:
                    self.current_mode.set(data["current_mode"])
                # Button-Aktionen
                ba = data.get("button_actions", {})
                for k, v in ba.items():
                    if k in self.button_actions and v in ACTION_CHOICES:
                        self.button_actions[k] = v
        except Exception as e:
            print("⚠️ Fehler beim Laden:", e)

    def save_config(self):
        try:
            # GUI → Datenstruktur: Achsen
            for m in SPEED_MODES:
                for ax in AXES:
                    for k, var in self.entries[m][ax].items():
                        self.modes[m][ax][k] = float(var.get())
                    self.modes[m][ax]["invert"] = -1 if self.invert_vars[m][ax].get() else 1

            # Button-Aktionen aus GUI übernehmen
            btn_actions = {}
            for key, var in self.button_action_vars.items():
                val = var.get()
                if val not in ACTION_CHOICES:
                    val = "none"
                btn_actions[key] = val
                self.button_actions[key] = val

            data = {
                "modes": self.modes,
                "current_mode": self.current_mode.get(),
                "button_actions": btn_actions,
            }
            if self.config_provider and hasattr(self.config_provider, "save_gamepad_config"):
                try:
                    ok, msg = self.config_provider.save_gamepad_config(data)
                    return bool(ok), msg
                except Exception as e:
                    return False, f"Fehler beim Speichern: {e}"
            with open(CONFIG_PATH, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
            return True, f"Config gespeichert → {CONFIG_PATH}"
        except Exception as e:
            return False, f"Fehler beim Speichern: {e}"

    def reload_into_ui(self):
        # Datei → Struktur → Variablen
        self._load_config()

        # Achsen-Werte in die UI zurückspielen
        for m in SPEED_MODES:
            for ax in AXES:
                for k in ["step", "alpha", "deadzone", "speed_scale"]:
                    self.entries[m][ax][k].set(self.modes[m][ax][k])
                self.invert_vars[m][ax].set(self.modes[m][ax]["invert"] == -1)

        # Button-Aktionen in die UI zurückspielen
        for key, var in self.button_action_vars.items():
            var.set(self.button_actions.get(key, "none"))

        # Tab passend setzen
        try:
            idx = list(SPEED_MODES.keys()).index(self.current_mode.get())
            self.nb.select(idx)
        except Exception:
            pass
        return True, "Config neu geladen."

    def reset_defaults(self):
        # Achsen-Defaults
        for m in SPEED_MODES:
            for ax in AXES:
                vals = DEFAULT_AXIS()
                for k in ["step", "alpha", "deadzone", "speed_scale"]:
                    self.entries[m][ax][k].set(vals[k])
                    self.modes[m][ax][k] = vals[k]
                self.invert_vars[m][ax].set(False)
                self.modes[m][ax]["invert"] = 1

        # Button-Defaults
        for key in BUTTON_ACTION_KEYS:
            default_val = DEFAULT_BUTTON_ACTIONS.get(key, "none")
            self.button_actions[key] = default_val
            if key in self.button_action_vars:
                self.button_action_vars[key].set(default_val)

        self.current_mode.set("normal")
        try:
            self.nb.select(list(SPEED_MODES.keys()).index("normal"))
        except Exception:
            pass
        return True, "Standardwerte gesetzt."

    # ---- Laufzeit-Zugriff ----
    def get_params(self):
        m = self.current_mode.get()
        params = {}
        for ax, d in self.entries[m].items():
            params[ax] = {k: float(v.get()) for k, v in d.items()}
            params[ax]["invert"] = -1 if self.invert_vars[m][ax].get() else 1
        return params

    def get_speed_factor(self):
        return SPEED_MODES[self.current_mode.get()]

    def get_button_actions(self):
        acts = {}
        for key, var in self.button_action_vars.items():
            val = var.get()
            if val not in ACTION_CHOICES:
                val = "none"
            acts[key] = val
        return acts


# --------------------------------------------
# Attach in Tab + Thread/Loop
# --------------------------------------------
def attach_gamepad_tab(tab_gamepad, client, execute_app):
    # ------- Layout: Log oben -------
    log_box = tk.Text(tab_gamepad, height=5)
    log_box.pack(fill=tk.X, padx=10, pady=(8, 6))

    def log(msg: str):
        log_box.insert(tk.END, msg + "\n")
        log_box.see(tk.END)

    # ------- Hauptzeile: 3 Spalten -------
    row = ttk.Frame(tab_gamepad)
    row.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 8))

    left = ttk.Frame(row)
    left.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    mid = ttk.Frame(row)
    mid.pack(side=tk.LEFT, fill=tk.Y, padx=10)
    right = ttk.Frame(row)
    right.pack(side=tk.LEFT, fill=tk.Y)

    # (links) Konfig-Tabelle + Button-Aktionen
    cfg_frame = ttk.LabelFrame(left, text="Parameter je Modus + Button-Aktionen")
    cfg_frame.pack(fill=tk.BOTH, expand=True)
    config_ui = GamepadConfigUI(cfg_frame, config_provider=execute_app)
    if hasattr(execute_app, "set_gamepad_config_ui"):
        execute_app.set_gamepad_config_ui(config_ui)

    # (mitte) Start/Stop + File-Buttons
    ctl = ttk.LabelFrame(mid, text="Gamepad Steuerung")
    ctl.pack(fill=tk.X, pady=(0, 8))
    btn_start = ttk.Button(ctl, text="▶️ Start Gamepad")
    btn_start.pack(fill=tk.X, padx=8, pady=(10, 6))
    btn_stop = ttk.Button(ctl, text="⏹ Stop Gamepad", state="disabled")
    btn_stop.pack(fill=tk.X, padx=8, pady=(0, 10))

    filef = ttk.LabelFrame(mid, text="Konfiguration")
    filef.pack(fill=tk.X)
    btn_save = ttk.Button(filef, text="💾 Speichern", command=lambda: _on_save())
    btn_save.pack(fill=tk.X, padx=8, pady=4)
    btn_reload = ttk.Button(filef, text="🔁 Neu laden", command=lambda: _on_reload())
    btn_reload.pack(fill=tk.X, padx=8, pady=4)
    btn_defaults = ttk.Button(filef, text="♻ Standardwerte", command=lambda: _on_reset())
    btn_defaults.pack(fill=tk.X, padx=8, pady=(4, 8))

    # (rechts) Speed-Mode Buttons + Status
    spdf = ttk.LabelFrame(right, text="Speed Mode")
    spdf.pack(fill=tk.Y)
    speed_btns = {}

    # Styles für aktive Anzeige
    style = ttk.Style()
    style.configure("ModeActive.TButton", font=("Segoe UI", 9, "bold"))

    def _refresh_mode_buttons(active_mode):
        for m, b in speed_btns.items():
            b.configure(style="ModeActive.TButton" if m == active_mode else "TButton")
        # farbiger Mode-Status
        m = active_mode
        factor = SPEED_MODES[m]
        txt = f"Mode: {m.upper()} ×{factor}"
        color = {"slow": "#1565c0", "normal": "#2e7d32", "fast": "#ef6c00"}[m]
        lbl_mode.config(text=txt, background=color, foreground="white")

    def _switch_mode(m):
        config_ui.current_mode.set(m)
        # Notebook-Tab synchronisieren
        try:
            config_ui.nb.select(list(SPEED_MODES.keys()).index(m))
        except Exception:
            pass
        _refresh_mode_buttons(m)
        log(f"⚙️ Speed-Mode → {m} (×{SPEED_MODES[m]})")

    for m in SPEED_MODES:
        speed_btns[m] = ttk.Button(
            spdf, text=f"{m.capitalize()} (×{SPEED_MODES[m]})", command=lambda mm=m: _switch_mode(mm)
        )
        speed_btns[m].pack(fill=tk.X, padx=8, pady=4)

    lbl_mode = tk.Label(spdf, text="Mode: —", width=24, height=2, bg="#777", fg="white")
    lbl_mode.pack(fill=tk.X, padx=8, pady=(6, 8))
    _refresh_mode_buttons(config_ui.current_mode.get())

    # ------- Save/Reload/Defaults Handler -------
    def _on_save():
        ok, msg = config_ui.save_config()
        log(("✅ " if ok else "❌ ") + msg)

    def _on_reload():
        ok, msg = config_ui.reload_into_ui()
        log(("✅ " if ok else "❌ ") + msg)
        _refresh_mode_buttons(config_ui.current_mode.get())

    def _on_reset():
        ok, msg = config_ui.reset_defaults()
        log(("✅ " if ok else "❌ ") + msg)
        _refresh_mode_buttons(config_ui.current_mode.get())

    # ------- Gamepad-Thread/Loop -------
    running = False
    js = None
    thread = None

    def _do_action(action_name: str):
        """Dispatch für konfigurierbare Actions (BACK/START/D-Pad Up/Down)."""
        if not action_name or action_name == "none":
            return
        try:
            if action_name == "add_current_pose_to_queue":
                # Aktuelle TCP-Pose zur Queue hinzufügen
                execute_app.on_gamepad_trigger("tcp_add_pose_to_queue", {})
                log("➕ [Action] Aktuelle TCP-Pose → Queue")

            elif action_name == "preview_queue_sequence":
                execute_app.on_gamepad_trigger("tcp_seq_preview", {"dist_cm": 5.0, "grip": "keep"})
                log("👀 [Action] Preview TCP Sequence")

            elif action_name == "execute_queue_sequence":
                execute_app.on_gamepad_trigger("tcp_seq_execute_cli", {"dist_cm": 5.0, "grip": "keep"})
                log("🚀 [Action] Execute TCP Sequence")

            elif action_name == "pose_to_mask_and_generate":
                execute_app.on_gamepad_trigger("tcp_pose_to_mask_and_generate", {})
                log("Pose to Mask + Generate")

            elif action_name == "set_tcp_as_kinematics_ref":
                execute_app.on_gamepad_trigger("tcp_set_tcp_as_reference", {})
                log("📐 [Action] TCP als Kinematik-Referenz gesetzt")

            elif action_name == "solve_and_run_tcp_sequence":
                execute_app.on_gamepad_trigger("tcp_solve_seq_and_execute", {"grip": "keep"})
                log("🧠 [Action] Solve & Run TCP Sequence")

            elif action_name == "goto_home":
                feed = int(map_speed_linear(int(execute_app.speed_val.get())))
                client.send_line(f"G90 G1 X0 Y0 Z0 A0 B0 C0 F{feed}")
                log(f"🏠 [Action] Goto Home (F={feed})")

            elif action_name == "goto_zero":
                feed = int(map_speed_linear(int(execute_app.speed_val.get())))
                client.send_line(f"G90 G1 X0 Y0 Z0 A0 B0 C0 F{feed}")
                log(f"0️⃣ [Action] Goto Zero (F={feed})")

            elif action_name == "tbd_1":
                log("ℹ️ [Action] tbd_1 (noch nicht belegt)")

            elif action_name == "tbd_2":
                log("ℹ️ [Action] tbd_2 (noch nicht belegt)")

            elif action_name == "tbd_3":
                log("ℹ️ [Action] tbd_3 (noch nicht belegt)")

            else:
                log(f"⚠️ Unbekannte Aktion: {action_name}")
        except Exception as e:
            log(f"[Action {action_name}] Fehler: {e}")

    def gamepad_loop():
        nonlocal running, js
        poll_dt = 0.025
        smooth = {ax: 0.0 for ax in AXES}
        gripper_pos = 0.0
        last_grip_time = 0.0

        # Debounce für konfigurierbare Buttons
        button_last_time = {key: 0.0 for key in BUTTON_ACTION_KEYS}
        button_debounce = 0.3  # Sekunden

        while running:
            try:
                if js is None:
                    time.sleep(0.1)
                    continue

                pygame.event.pump()

                params = config_ui.get_params()
                spd_factor = config_ui.get_speed_factor()
                button_actions = config_ui.get_button_actions()

                # Read inputs
                axes_vals = [js.get_axis(i) for i in range(js.get_numaxes())]
                buttons = [js.get_button(i) for i in range(js.get_numbuttons())]
                hat = js.get_hat(0) if js.get_numhats() > 0 else (0, 0)


                # Trigger-Achsen (LT / RT)
                lt = js.get_axis(4) if js.get_numaxes() > 4 else 0.0
                rt = js.get_axis(5) if js.get_numaxes() > 5 else 0.0
                # bei manchen Controllern sind Trigger -1..1 → auf 0..1 normalisieren
                lt = (lt + 1.0) / 2.0
                rt = (rt + 1.0) / 2.0

                now = time.time()

                # Slow-Override standardmäßig aus
                speed_factor_override = 1.0
                if lt > 0.3:  # LT leicht gedrückt → Slow Mode
                    speed_factor_override = 0.1
                fixed_enabled_var = getattr(execute_app, "fixed_tcp_enabled", None)
                fixed_enabled = bool(fixed_enabled_var.get()) if fixed_enabled_var else False


                jog = {}
                # X/Y via Stick 0/1
                if len(axes_vals) >= 2:
                    ax0 = float(axes_vals[0])
                    ax1 = float(axes_vals[1])
                    if abs(ax0) > params["X"]["deadzone"]:
                        jog["X"] = params["X"]["invert"] * ax0 * params["X"]["step"]
                    if abs(ax1) > params["Y"]["deadzone"]:
                        jog["Y"] = params["Y"]["invert"] * (-ax1) * params["Y"]["step"]

                # Z via Achse 3 (rechter Stick vertikal)
                if len(axes_vals) > 3:
                    az = float(axes_vals[3])
                    if abs(az) > params["Z"]["deadzone"]:
                        jog["Z"] = params["Z"]["invert"] * (-az) * params["Z"]["step"]

                # Fixed mode step buttons disabled (GUI controls only)

                # B (Roll) via Achse 2 (rechter Stick horizontal)
                if len(axes_vals) > 2:
                    fixed_enabled_var = getattr(execute_app, "fixed_tcp_enabled", None)
                    fixed_enabled = bool(fixed_enabled_var.get()) if fixed_enabled_var else False
                    if not fixed_enabled:
                        ab = float(axes_vals[2])
                        if abs(ab) > params["B"]["deadzone"]:
                            jog["B"] = params["B"]["invert"] * ab * params["B"]["step"]

                # A (Yaw) via LB/RB (Buttons 4/5)
                if len(buttons) > 5:
                    fixed_enabled_var = getattr(execute_app, "fixed_tcp_enabled", None)
                    fixed_enabled = bool(fixed_enabled_var.get()) if fixed_enabled_var else False
                    if not fixed_enabled:
                        if buttons[4]:
                            jog["A"] = -params["A"]["invert"] * params["A"]["step"]
                        elif buttons[5]:
                            jog["A"] = params["A"]["invert"] * params["A"]["step"]

                # Greifer (0=open, 1=close)
                grip_delay = 0.25
                if len(buttons) > 1:
                    if buttons[0] and (now - last_grip_time > grip_delay):
                        gripper_pos -= params["C"]["invert"] * params["C"]["step"] * 1000.0
                        gripper_pos = max(0.0, gripper_pos)
                        client.send_line(f"M3 S{int(gripper_pos)}")
                        log(f"🖐️ Greifer auf → M3 S{int(gripper_pos)}")
                        last_grip_time = now
                    elif buttons[1] and (now - last_grip_time > grip_delay):
                        gripper_pos += params["C"]["invert"] * params["C"]["step"] * 1000.0
                        gripper_pos = min(1000.0, gripper_pos)
                        client.send_line(f"M3 S{int(gripper_pos)}")
                        log(f"✊ Greifer zu → M3 S{int(gripper_pos)}")
                        last_grip_time = now

                # C via D-Pad links/rechts (feiner Jog)
                fixed_enabled_var = getattr(execute_app, "fixed_tcp_enabled", None)
                fixed_enabled = bool(fixed_enabled_var.get()) if fixed_enabled_var else False
                if not fixed_enabled:
                    if hat[0] == -1:
                        jog["C"] = -params["C"]["invert"] * params["C"]["step"] * 0.1
                    elif hat[0] == 1:
                        jog["C"] = params["C"]["invert"] * params["C"]["step"] * 0.1

                # === Gripper fein öffnen/schließen mit X/Y (Button 2/3)
                if len(buttons) > 3:
                    if buttons[2] and (now - last_grip_time > grip_delay):
                        gripper_pos -= params["C"]["invert"] * params["C"]["step"] * 200.0
                        gripper_pos = max(0.0, gripper_pos)
                        client.send_line(f"M3 S{int(gripper_pos)}")
                        log(f"🖐️ Greifer fein auf → M3 S{int(gripper_pos)}")
                        last_grip_time = now
                    elif buttons[3] and (now - last_grip_time > grip_delay):
                        gripper_pos += params["C"]["invert"] * params["C"]["step"] * 200.0
                        gripper_pos = min(1000.0, gripper_pos)
                        client.send_line(f"M3 S{int(gripper_pos)}")
                        log(f"✊ Greifer fein zu → M3 S{int(gripper_pos)}")
                        last_grip_time = now

                # === Goto Zero mit RT (Right Trigger) — weiterhin direkt
                if rt > 0.8:  # stark gedrückt
                    feed = int(map_speed_linear(int(execute_app.speed_val.get())) * spd_factor * speed_factor_override)
                    client.send_line("G90 G1 X0 Y0 Z0 A0 B0 C0 F4000")
                    log(f"🏁 RT → Goto Zero All (F={feed})")
                    time.sleep(0.5)  # debounce

                # === Konfigurierbare Buttons: BACK/START ===
                BACK_BTN, START_BTN = 6, 7
                if len(buttons) > BACK_BTN and buttons[BACK_BTN]:
                    if now - button_last_time["back"] > button_debounce:
                        _do_action(button_actions.get("back", "none"))
                        button_last_time["back"] = now

                if len(buttons) > START_BTN and buttons[START_BTN]:
                    if now - button_last_time["start"] > button_debounce:
                        _do_action(button_actions.get("start", "none"))
                        button_last_time["start"] = now

                # === Konfigurierbare D-Pad UP/DOWN ===
                # UP: hat[1] = +1, DOWN: hat[1] = -1
                if not fixed_enabled:
                    if hat[1] == 1:
                        if now - button_last_time["dpad_up"] > button_debounce:
                            _do_action(button_actions.get("dpad_up", "none"))
                            button_last_time["dpad_up"] = now
                    elif hat[1] == -1:
                        if now - button_last_time["dpad_down"] > button_debounce:
                            _do_action(button_actions.get("dpad_down", "none"))
                            button_last_time["dpad_down"] = now

                # === Gl?ttung + Jog senden ===

                for ax in AXES:
                    target = float(jog.get(ax, 0.0))
                    a = float(params[ax]["alpha"])
                    smooth[ax] = (1.0 - a) * float(smooth[ax]) + a * target

                fixed_enabled_var = getattr(execute_app, "fixed_tcp_enabled", None)
                fixed_enabled = bool(fixed_enabled_var.get()) if fixed_enabled_var else False

                if fixed_enabled:
                    time.sleep(poll_dt)
                    continue
                else:
                    if any(abs(v) > 0.01 for v in smooth.values()):
                        feed = int(
                            map_speed_linear(int(execute_app.speed_val.get()))
                            * spd_factor
                            * speed_factor_override
                        )

                        # Nur die eigentliche Bewegung (G1 ...) bauen
                        move_parts = ["G1"]
                        for ax, val in smooth.items():
                            if abs(val) > 0.01:
                                s = float(params[ax]["speed_scale"])
                                move_parts.append(f"{ax}{(val * s):.3f}")
                        move_parts.append(f"F{feed}")
                        move_line = " ".join(move_parts)

                        # Wichtig: kurz auf G91, dann wieder zur?ck auf G90
                        client.send_line("G91")
                        client.send_line(move_line)
                        client.send_line("G90")

                        log(f"TX jog (rel): G91 / {move_line} / G90")


                time.sleep(poll_dt)
            except Exception as e:
                log(f"[Gamepad error] {e}")
                time.sleep(0.1)

    # ------- Start/Stop -------
    def start():
        nonlocal running, js, thread
        if running:
            return
        try:
            pygame.quit()
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() == 0:
                log("⚠️ Kein Gamepad gefunden.")
                return
            js = pygame.joystick.Joystick(0)
            js.init()
            log(f"🎮 Gamepad erkannt: {js.get_name()}")
            running = True
            thread = threading.Thread(target=gamepad_loop, daemon=True)
            thread.start()
            btn_start.config(state="disabled")
            btn_stop.config(state="normal")
        except Exception as e:
            log(f"Init-Fehler: {e}")

    def stop():
        nonlocal running
        if not running:
            return
        running = False
        log("🛑 Gamepad gestoppt")
        btn_start.config(state="normal")
        btn_stop.config(state="disabled")
        try:
            pygame.joystick.quit()
            pygame.quit()
        except Exception:
            pass

    btn_start.config(command=start)
    btn_stop.config(command=stop)

    # beim Start den gespeicherten Modus-Button highlighten
    _refresh_mode_buttons(config_ui.current_mode.get())

    # --- Automatisch starten (nur wenn ein Gamepad da ist) ---
    try:
        if not running:
            pygame.quit()
            pygame.init()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                start()
            else:
                log("ℹ️ Auto-Start übersprungen (kein Gamepad angeschlossen).")
    except Exception as e:
        log(f"Auto-Start Fehler: {e}")

    # Public: Rückgabe eines Stop-Callbacks (für App-Shutdown)
    return stop
