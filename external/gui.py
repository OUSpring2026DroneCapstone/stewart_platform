"""
Dear PyGui-based GUI (dashboard reskin):
 - Uses Dear PyGui for UI
 - Keeps pygame joystick backend and serial logic
 - Preserves original behavior and commands
 - Adds responsive dashboard layout + card styling
"""

import sys
import time
import serial
from serial.tools import list_ports
import pygame
from joystick import JoystickBackend
import dearpygui.dearpygui as dpg

BAUD_RATE = 115200
MAX_LOG_LINES = 2000

# ---------------- dashboard theme (dark + red accent) ----------------
COL_BG = (12, 14, 20)
COL_TOPBAR = (16, 18, 26)
COL_SIDEBAR = (16, 18, 26)
COL_CARD = (20, 24, 34)
COL_CARD_2 = (24, 29, 41)
COL_BORDER = (40, 48, 66)
COL_TEXT = (235, 240, 250)
COL_MUTED = (150, 160, 180)
COL_ACCENT = (235, 55, 78)
COL_ACCENT_HOVER = (255, 85, 105)
COL_GOOD = (70, 220, 170)
COL_WARN = (255, 185, 70)
COL_BAD = (255, 90, 100)

SIDEBAR_W = 240
SIDEBAR_PAD = 24
TOPBAR_H = 52
PAD = 20
LEFT_MARGIN = 30

# Pose display ranges
TRANS_RANGE = 1.0    # joystick axis range (-1..+1)
ROT_RANGE = 30.0     # max degrees for tilt/yaw display
NUM_LEGS = 6
LEG_MIN_IN = 0.0     # actuator min extension (inches)
LEG_MAX_IN = 8.0     # actuator max extension (inches)
LEG_WARN = 0.9       # fraction of range for yellow
LEG_CRIT = 0.97      # fraction of range for red


def main():
    pygame.init()

    # ---------------- serial (optional) ----------------
    ser = None
    def detect_serial_port():
        ports = list(list_ports.comports())
        if not ports:
            return None

        # Prefer common Arduino USB/ACM device names
        preferred = []
        for p in ports:
            desc = (p.description or "").lower()
            dev = (p.device or "").lower()
            if "arduino" in desc or "usb" in desc or "acm" in dev or "usb" in dev:
                preferred.append(p.device)

        if preferred:
            return preferred[0]
        return ports[0].device

    port = sys.argv[1] if len(sys.argv) >= 2 else detect_serial_port()
    if port:
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=0)
            time.sleep(2.0)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
        except Exception as e:
            available = ", ".join(p.device for p in list_ports.comports()) or "(none)"
            print(f"[Serial] Failed to open {port}: {e}")
            print(f"[Serial] Available ports: {available}")
            ser = None

    # ---------------- backend ----------------
    controller_mode = False

    def send_command(cmd: str):
        nonlocal controller_mode
        if ser:
            ser.write((cmd + "\n").encode())
            print(f">>> SENT: {cmd}")

        if cmd == "controller":
            controller_mode = True
            joystick.enable_controller()
        elif cmd == "stop":
            controller_mode = False
            joystick.disable_controller()
        elif cmd.startswith("start") or cmd == "center":
            controller_mode = False
            joystick.disable_controller()

    joystick = JoystickBackend(ser, on_command=send_command)

    # ---------------- state ----------------
    MAIN, START_MENU, PRESETS, RUNNING = "main", "start", "presets", "running"
    menu = MAIN
    active_preset = None
    waiting_for_center = False
    arduino_log: list[str] = []
    leg_extensions: list[float] = [0.0] * NUM_LEGS  # inches

    # ---------------- Dear PyGui setup ----------------
    dpg.create_context()
    dpg.create_viewport(title="Stewart Platform Control", width=1280, height=720, resizable=True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    is_fullscreen = False

    def toggle_fullscreen():
        nonlocal is_fullscreen
        is_fullscreen = not is_fullscreen
        dpg.set_viewport_properties(fullscreen=is_fullscreen)

    # ---------------- themes ----------------
    with dpg.theme() as theme_global:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, COL_BG)
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg, COL_CARD)
            dpg.add_theme_color(dpg.mvThemeCol_Text, COL_TEXT)
            dpg.add_theme_color(dpg.mvThemeCol_Border, COL_BORDER)
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_PopupRounding, 6)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 10, 6)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 0, 0)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, 6)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding, 4)

    with dpg.theme() as theme_topbar:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_TOPBAR)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, PAD, PAD)

    with dpg.theme() as theme_sidebar:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_SIDEBAR)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, SIDEBAR_PAD, PAD)

    with dpg.theme() as theme_card:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_CARD)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, PAD, PAD)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 6)

    with dpg.theme() as theme_card_inner:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_CARD_2)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 10, 10)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 4)

    with dpg.theme() as theme_btn_primary:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, COL_ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)

    with dpg.theme() as theme_btn_sidebar:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (255, 255, 255, 8))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 255, 255, 20))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (255, 255, 255, 30))
            dpg.add_theme_color(dpg.mvThemeCol_Text, COL_TEXT)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)

    # progress bar themes
    def _bar_theme(fill, bg=(255, 255, 255, 15)):
        with dpg.theme() as t:
            with dpg.theme_component(dpg.mvProgressBar):
                dpg.add_theme_color(dpg.mvThemeCol_PlotHistogram, fill)
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, bg)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 3)
        return t

    theme_bar_accent = _bar_theme(COL_ACCENT)
    theme_bar_good = _bar_theme(COL_GOOD)
    theme_bar_warn = _bar_theme(COL_WARN)
    theme_bar_bad = _bar_theme(COL_BAD)

    dpg.bind_theme(theme_global)

    # ---------------- layout helpers ----------------
    def viewport_size():
        return dpg.get_viewport_client_width(), dpg.get_viewport_client_height()

    def apply_layout():
        w, h = viewport_size()
        aw = w - LEFT_MARGIN  # available width after left margin
        dpg.configure_item("root", width=aw, height=h)
        dpg.configure_item("topbar", width=aw, height=TOPBAR_H)
        dpg.configure_item("main_area", width=aw, height=h - TOPBAR_H)

        # content sizes
        content_w = max(320, aw - SIDEBAR_W)
        content_h = max(240, h - TOPBAR_H)

        dpg.configure_item("sidebar", width=SIDEBAR_W, height=content_h)
        dpg.configure_item("content", width=content_w, height=content_h)

    def on_viewport_resize(sender, app_data):
        apply_layout()

    dpg.set_viewport_resize_callback(on_viewport_resize)

    # ---------------- popup helper ----------------
    def show_popup(messages):
        if isinstance(messages, str):
            messages = [messages]
        if dpg.does_item_exist("popup"):
            dpg.delete_item("popup")
        w, h = viewport_size()
        pw, ph = 520, 240
        px, py = (w - pw) // 2, (h - ph) // 2
        with dpg.window(modal=True, no_title_bar=True, tag="popup", pos=(px, py), width=pw, height=ph):
            dpg.add_text("Connection Required", color=COL_TEXT)
            dpg.add_spacer(height=6)
            dpg.add_separator()
            dpg.add_spacer(height=8)
            for msg in messages:
                dpg.add_text(f"• {msg}", color=COL_TEXT)
            dpg.add_spacer(height=12)
            dpg.bind_item_theme(dpg.add_button(label="OK", width=140, callback=lambda: dpg.delete_item("popup")), theme_btn_primary)

    def connection_error_for(action_label):
        missing = []
        serial_required = {"CENTER", "STOP", "DEMO", "FIGURE 8", "RESTART", "ORBIT", "WAVE"}
        controller_actions = {"CONTROLLER"}
        if action_label in serial_required or action_label in controller_actions:
            if not ser:
                missing.append("Serial port not connected")
        if action_label in controller_actions:
            if not joystick.available:
                missing.append("Joystick not connected")
        return missing or None

    # ---------------- UI build ----------------
    with dpg.window(tag="root", no_title_bar=True, no_resize=True, no_move=True, pos=(LEFT_MARGIN, 0)):
        # TOP BAR
        with dpg.child_window(tag="topbar", border=False):
            dpg.bind_item_theme("topbar", theme_topbar)
            dpg.add_spacer(height=PAD)
            with dpg.group(horizontal=True, indent=PAD):
                dpg.add_text("Stewart Platform", color=COL_TEXT)
                dpg.add_spacer(width=16)
                dpg.add_text(tag="status_serial", default_value="Serial: DISCONNECTED", color=COL_WARN)
                dpg.add_spacer(width=16)
                dpg.add_text(tag="status_joy", default_value="Joystick: DISCONNECTED", color=COL_WARN)
                dpg.add_spacer(width=16)
                dpg.add_text(tag="status_mode", default_value="Mode: —", color=COL_MUTED)

        # MAIN AREA: sidebar + content
        with dpg.child_window(tag="main_area", border=False):
            with dpg.table(header_row=False, resizable=False, policy=dpg.mvTable_SizingFixedFit,
                           borders_innerV=False, borders_outerV=False, borders_innerH=False, borders_outerH=False):
                dpg.add_table_column(width_fixed=True, init_width_or_weight=SIDEBAR_W)
                dpg.add_table_column(width_fixed=False)

                with dpg.table_row():
                    # SIDEBAR
                    with dpg.child_window(tag="sidebar", border=False):
                        dpg.bind_item_theme("sidebar", theme_sidebar)

                        with dpg.group(indent=SIDEBAR_PAD):
                            dpg.add_spacer(height=10)
                            dpg.add_text(tag="menu_title", default_value="MAIN", color=COL_TEXT)
                            dpg.add_spacer(height=12)

                            # buttons container (rebuilt)
                            with dpg.group(tag="buttons_container"):
                                pass

                    # CONTENT
                    with dpg.child_window(tag="content", border=False):
                        # ===== Row 1: Pose + Actuators =====
                        with dpg.table(header_row=False, resizable=False, policy=dpg.mvTable_SizingStretchProp,
                                       borders_innerV=False, borders_outerV=False, borders_innerH=False, borders_outerH=False):
                            dpg.add_table_column(init_width_or_weight=3)
                            dpg.add_table_column(init_width_or_weight=2)

                            with dpg.table_row():
                                # --- Pose Display ---
                                with dpg.child_window(tag="card_pose", border=False, height=230):
                                    dpg.bind_item_theme("card_pose", theme_card)
                                    dpg.add_spacer(height=PAD)
                                    with dpg.group(indent=PAD):
                                        dpg.add_text("Platform Pose", color=COL_MUTED)
                                        dpg.add_spacer(height=6)

                                        pose_axes = [
                                            ("pose_roll",  "Roll"),
                                            ("pose_pitch", "Pitch"),
                                            ("pose_yaw",   "Yaw"),
                                            ("pose_x",     "X"),
                                            ("pose_y",     "Y"),
                                            ("pose_z",     "Z"),
                                        ]
                                        for ax_tag, ax_label in pose_axes:
                                            with dpg.group(horizontal=True):
                                                dpg.add_text(f"{ax_label:>5s}", color=COL_MUTED)
                                                dpg.add_spacer(width=6)
                                                bar = dpg.add_progress_bar(tag=ax_tag, default_value=0.5,
                                                                           overlay="0.0", width=-(PAD + 60))
                                                dpg.bind_item_theme(bar, theme_bar_accent)
                                                dpg.add_text("", tag=f"{ax_tag}_val", color=COL_TEXT)

                                # --- Actuator Status ---
                                with dpg.child_window(tag="card_legs", border=False, height=230):
                                    dpg.bind_item_theme("card_legs", theme_card)
                                    dpg.add_spacer(height=PAD)
                                    with dpg.group(indent=PAD):
                                        dpg.add_text("Actuators", color=COL_MUTED)
                                        dpg.add_spacer(height=6)

                                        for i in range(NUM_LEGS):
                                            with dpg.group(horizontal=True):
                                                dpg.add_text(f"Leg {i+1}", color=COL_MUTED)
                                                dpg.add_spacer(width=4)
                                                bar = dpg.add_progress_bar(tag=f"leg_{i}", default_value=0.0,
                                                                           overlay="--", width=-(PAD + 60))
                                                dpg.bind_item_theme(bar, theme_bar_good)
                                                dpg.add_text("", tag=f"leg_{i}_val", color=COL_TEXT)

                        dpg.add_spacer(height=6)

                        # ===== Row 2: Log + Controls =====
                        with dpg.table(header_row=False, resizable=False, policy=dpg.mvTable_SizingStretchProp,
                                       borders_innerV=False, borders_outerV=False, borders_innerH=False, borders_outerH=False):

                            dpg.add_table_column(init_width_or_weight=2)
                            dpg.add_table_column(init_width_or_weight=1)

                            with dpg.table_row():
                                # --- Arduino Log Card ---
                                with dpg.child_window(tag="card_log", border=False, height=-1):
                                    dpg.bind_item_theme("card_log", theme_card)
                                    dpg.add_spacer(height=PAD)
                                    with dpg.group(indent=PAD):
                                        with dpg.group(horizontal=True):
                                            dpg.add_text("Arduino Log", color=COL_TEXT)
                                            dpg.add_spacer(width=10)
                                            dpg.add_text(tag="running_label", default_value="", color=COL_GOOD)

                                        dpg.add_spacer(height=8)
                                        dpg.add_separator()
                                        dpg.add_spacer(height=10)

                                        with dpg.child_window(tag="log_box", border=False, height=-1):
                                            dpg.bind_item_theme("log_box", theme_card_inner)
                                            dpg.add_text(tag="arduino_log_text", default_value="", color=COL_MUTED)

                                # --- Controls Card ---
                                with dpg.child_window(tag="card_controls", border=False, height=-1):
                                    dpg.bind_item_theme("card_controls", theme_card)
                                    dpg.add_spacer(height=PAD)
                                    with dpg.group(indent=PAD):
                                        with dpg.group(horizontal=True):
                                            dpg.add_text("Controls", color=COL_TEXT)
                                            dpg.add_spacer(width=10)
                                            dpg.add_text(tag="mode_label", default_value="", color=COL_ACCENT)

                                        dpg.add_spacer(height=8)
                                        dpg.add_separator()
                                        dpg.add_spacer(height=10)

                                        dpg.add_text(tag="controls_text", default_value="", color=COL_MUTED)


    # Responsive sizing initial
    apply_layout()

    # ---------------- menu logic ----------------
    def toggle_mode(sender=None, app_data=None, user_data=None):
        joystick.toggle_mode()
        dpg.configure_item("mode_toggle", label=("D-PAD MODE" if joystick.dpad_mode else "ANALOG MODE"))
        print(f"[Mode] Switched to {'D-PAD' if joystick.dpad_mode else 'ANALOG'}")

    def on_button(label):
        nonlocal menu, waiting_for_center, active_preset
        if label == "START" and menu == MAIN:
            menu = START_MENU
            rebuild_buttons()
            return

        errs = connection_error_for(label)
        if errs:
            show_popup(errs)

        if menu == MAIN:
            if label == "CENTER":
                send_command("center")
            elif label == "STOP":
                send_command("stop")

        elif menu == START_MENU:
            if label == "CONTROLLER":
                send_command("controller")
                menu = MAIN
                rebuild_buttons()
            elif label == "PRESETS":
                menu = PRESETS
                rebuild_buttons()
            elif label == "BACK":
                menu = MAIN
                rebuild_buttons()

        elif menu == PRESETS:
            if label in {"DEMO", "FIGURE 8", "ORBIT", "WAVE"}:
                preset_map = {"DEMO": "demo", "FIGURE 8": "figure8", "ORBIT": "orbit", "WAVE": "wave"}
                active_preset = preset_map[label]
                send_command("center")
                waiting_for_center = True
                menu = RUNNING
                dpg.configure_item("running_label", default_value=f"RUNNING: {active_preset.upper()}")
                rebuild_buttons()
            elif label == "BACK":
                menu = START_MENU
                rebuild_buttons()

        elif menu == RUNNING:
            if label == "RESTART":
                send_command("center")
                waiting_for_center = True
            elif label == "STOP":
                send_command("stop")
                waiting_for_center = False
                menu = MAIN
                rebuild_buttons()

    def on_button_callback(sender, app_data, user_data):
        on_button(user_data)

    def rebuild_buttons():
        if dpg.does_item_exist("buttons_container"):
            dpg.delete_item("buttons_container")

        with dpg.group(tag="buttons_container", parent="sidebar", indent=SIDEBAR_PAD):
            labels = []
            if menu == MAIN:
                labels = ["START", "CENTER", "CALIBRATE", "STOP"]
                dpg.configure_item("menu_title", default_value="MAIN")
            elif menu == START_MENU:
                labels = ["CONTROLLER", "PRESETS", "BACK"]
                dpg.configure_item("menu_title", default_value="START MENU")
            elif menu == PRESETS:
                labels = ["DEMO", "FIGURE 8", "ORBIT", "WAVE", "BACK"]
                dpg.configure_item("menu_title", default_value="PRESETS")
            elif menu == RUNNING:
                labels = ["RESTART", "STOP"]
                dpg.configure_item("menu_title", default_value="RUNNING")

            for i, lab in enumerate(labels):
                btn = dpg.add_button(label=lab, height=34, width=SIDEBAR_W - (SIDEBAR_PAD * 2),
                                     callback=on_button_callback, user_data=lab)
                if lab == "STOP":
                    dpg.bind_item_theme(btn, theme_btn_primary)
                else:
                    dpg.bind_item_theme(btn, theme_btn_sidebar)

                dpg.add_spacer(height=4)

            # joystick mode toggle (only if joystick exists)
            if joystick.available:
                dpg.add_spacer(height=6)
                dpg.add_separator()
                dpg.add_spacer(height=10)
                dpg.add_text("JOYSTICK", color=COL_TEXT)
                dpg.add_spacer(height=8)
                mt = dpg.add_button(
                    tag="mode_toggle",
                    label=("D-PAD MODE" if joystick.dpad_mode else "ANALOG MODE"),
                    height=34,
                    width=SIDEBAR_W - (SIDEBAR_PAD * 2),
                    callback=toggle_mode
                )
                dpg.bind_item_theme(mt, theme_btn_sidebar)

    rebuild_buttons()

    def build_controls_text():
        if not joystick.available:
            return (
                "Connect a joystick to enable controller shortcuts.\n\n"
                "A: Enable controller\n"
                "B: Stop\n"
                "X: Center\n"
                "Y: Run demo"
            )
        if joystick.dpad_mode:
            lines = [
                "D-Pad: Move X/Y position",
                "LB/RB: Move Z (up/down)",
                "",
                "A: Enable controller",
                "B: Stop movement",
                "X: Center platform",
                "Y: Run demo",
            ]
        else:
            lines = [
                "Left Stick: Move X/Y position",
                "Right Stick UP: Tilt FORWARD (10°)",
                "Right Stick DOWN: Tilt BACK (10°)",
                "Right Stick LEFT: Tilt LEFT (10°)",
                "Right Stick RIGHT: Tilt RIGHT (10°)",
                "",
                "A: Enable controller",
                "B: Stop movement",
                "X: Center platform",
                "Y: Run demo",
            ]
        return "\n".join(lines)

    # ---------------- main loop ----------------
    clock = pygame.time.Clock()
    while dpg.is_dearpygui_running():
        events = pygame.event.get()

        # F11 fullscreen
        for event in events:
            if event.type == pygame.KEYDOWN and event.key == pygame.K_F11:
                toggle_fullscreen()

        joystick.handle_events(events)
        joystick.tick()

        # Serial feedback
        if ser:
            try:
                waiting = ser.in_waiting
            except Exception:
                waiting = 0
            if waiting:
                lines = ser.read(waiting).decode(errors="replace").splitlines()
                for line in lines:
                    line = line.strip()
                    if not line:
                        continue
                    print("[ARDUINO]", line)
                    arduino_log.append(line)
                    if len(arduino_log) > MAX_LOG_LINES:
                        arduino_log.pop(0)

                    if waiting_for_center and "Centered. Motors disabled." in line:
                        waiting_for_center = False
                        if active_preset:
                            send_command(f"start {active_preset}")
                            menu = RUNNING
                            dpg.configure_item("running_label", default_value=f"RUNNING: {active_preset.upper()}")

                    # Parse actuator extensions: "  M1: 4.500" or "  M1: start=0.5, end=0.6, max=0.7"
                    for mi in range(NUM_LEGS):
                        prefix = f"M{mi+1}:"
                        if prefix in line:
                            try:
                                after = line.split(prefix, 1)[1].strip()
                                val = float(after.split(",")[0].split("=")[-1])
                                leg_extensions[mi] = val
                            except (ValueError, IndexError):
                                pass

        # Update top status
        dpg.configure_item("status_serial", default_value=f"Serial: {'CONNECTED' if ser else 'DISCONNECTED'}",
                           color=(COL_GOOD if ser else COL_WARN))
        dpg.configure_item("status_joy", default_value=f"Joystick: {'CONNECTED' if joystick.available else 'DISCONNECTED'}",
                           color=(COL_GOOD if joystick.available else COL_WARN))
        mode_text = "D-PAD" if joystick.available and joystick.dpad_mode else ("ANALOG" if joystick.available else "—")
        dpg.configure_item("status_mode", default_value=f"Mode: {mode_text}", color=COL_MUTED)

        # Update pose display (derived from joystick command state)
        # Rotation axes: alt = pitch, yaw = yaw, roll ≈ 0 (not directly controlled)
        pose_vals = {
            "pose_roll":  (0.0, ROT_RANGE),
            "pose_pitch": (joystick.alt_f if joystick.available else 0.0, ROT_RANGE),
            "pose_yaw":   (joystick.yaw_f if joystick.available else 0.0, ROT_RANGE),
            "pose_x":     (joystick.ax_f if joystick.available else 0.0, TRANS_RANGE),
            "pose_y":     (joystick.ay_f if joystick.available else 0.0, TRANS_RANGE),
            "pose_z":     (joystick.az_f if joystick.available else 0.0, TRANS_RANGE),
        }
        for ax_tag, (val, rng) in pose_vals.items():
            bar_frac = (val / rng + 1.0) / 2.0  # map -range..+range → 0..1
            bar_frac = max(0.0, min(1.0, bar_frac))
            if rng > 1:
                unit = "\u00b0"  # degree symbol
                display = f"{val:+.1f}{unit}"
            else:
                display = f"{val:+.3f}"
            dpg.configure_item(ax_tag, default_value=bar_frac, overlay=display)
            dpg.configure_item(f"{ax_tag}_val", default_value=display)

        # Update actuator bars
        leg_range = LEG_MAX_IN - LEG_MIN_IN
        for i in range(NUM_LEGS):
            ext = leg_extensions[i]
            frac = (ext - LEG_MIN_IN) / leg_range if leg_range > 0 else 0.0
            frac = max(0.0, min(1.0, frac))

            if frac >= LEG_CRIT:
                theme = theme_bar_bad
            elif frac >= LEG_WARN:
                theme = theme_bar_warn
            else:
                theme = theme_bar_good

            label = f"{ext:.2f}\""
            dpg.configure_item(f"leg_{i}", default_value=frac, overlay=label)
            dpg.configure_item(f"leg_{i}_val", default_value=label)
            dpg.bind_item_theme(f"leg_{i}", theme)

        # Update card labels/text
        if menu == RUNNING and active_preset:
            dpg.configure_item("running_label", default_value=f"RUNNING: {active_preset.upper()}")
        else:
            dpg.configure_item("running_label", default_value="")

        mode_label = "MODE: D-PAD (discrete positions)" if joystick.available and joystick.dpad_mode else \
                     ("MODE: ANALOG (continuous + directional tilt)" if joystick.available else "MODE: —")
        dpg.configure_item("mode_label", default_value=mode_label)

        dpg.configure_item("controls_text", default_value=build_controls_text())
        dpg.configure_item("arduino_log_text", default_value="\n".join(arduino_log))
        dpg.set_y_scroll("log_box", dpg.get_y_scroll_max("log_box"))

        dpg.render_dearpygui_frame()
        clock.tick(30)

    # Cleanup
    if ser:
        try:
            ser.write(b"stop\n")
        except Exception:
            pass
        ser.close()
    pygame.quit()
    dpg.destroy_context()


if __name__ == "__main__":
    main()
