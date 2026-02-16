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
MAX_LOG_LINES = 18

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

SIDEBAR_W = 300
TOPBAR_H = 72
PAD = 16


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
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 14)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 12)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 14)
            dpg.add_theme_style(dpg.mvStyleVar_PopupRounding, 12)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 14, 10)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 0, 0)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 10, 10)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding, 12)

    with dpg.theme() as theme_topbar:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_TOPBAR)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, PAD, PAD)

    with dpg.theme() as theme_sidebar:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_SIDEBAR)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, PAD, PAD)

    with dpg.theme() as theme_card:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_CARD)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, PAD, PAD)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 16)

    with dpg.theme() as theme_card_inner:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_CARD_2)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 12, 12)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 14)

    with dpg.theme() as theme_btn_primary:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, COL_ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 14)

    with dpg.theme() as theme_btn_ghost:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (0, 0, 0, 0))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 255, 255, 25))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (255, 255, 255, 35))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 12)

    with dpg.theme() as theme_btn_sidebar:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (255, 255, 255, 18))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255, 255, 255, 30))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (255, 255, 255, 40))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 14)

    dpg.bind_theme(theme_global)

    # ---------------- layout helpers ----------------
    def viewport_size():
        return dpg.get_viewport_client_width(), dpg.get_viewport_client_height()

    def apply_layout():
        w, h = viewport_size()
        dpg.configure_item("root", width=w, height=h)
        dpg.configure_item("topbar", width=w, height=TOPBAR_H)
        dpg.configure_item("main_area", width=w, height=h - TOPBAR_H)

        # content sizes
        content_w = max(320, w - SIDEBAR_W)
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
    with dpg.window(tag="root", no_title_bar=True, no_resize=True, no_move=True, pos=(0, 0)):
        # TOP BAR
        with dpg.child_window(tag="topbar", border=False):
            dpg.bind_item_theme("topbar", theme_topbar)

            with dpg.group(horizontal=True):
                dpg.add_text("Stewart Platform", color=COL_TEXT)
                dpg.add_spacer(width=12)
                dpg.add_text("Control Dashboard", color=COL_MUTED)

            dpg.add_spacer(height=8)

            with dpg.group(horizontal=True):
                # status pills
                dpg.add_text("●", color=(0, 0, 0, 0))
                dpg.add_text(tag="status_serial", default_value="Serial: DISCONNECTED", color=COL_WARN)
                dpg.add_spacer(width=18)
                dpg.add_text(tag="status_joy", default_value="Joystick: DISCONNECTED", color=COL_WARN)
                dpg.add_spacer(width=18)
                dpg.add_text(tag="status_mode", default_value="Mode: —", color=COL_MUTED)
                dpg.add_spacer(width=24)

                # right-side actions
                dpg.add_spacer(width=24)
                btn_fs = dpg.add_button(label="⛶ Fullscreen", width=150, callback=lambda: toggle_fullscreen())
                dpg.bind_item_theme(btn_fs, theme_btn_ghost)

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

                        # red accent strip (left edge)
                        with dpg.drawlist(width=8, height=-1):
                            dpg.draw_rectangle((0, 0), (8, 2000), fill=COL_ACCENT, thickness=0)

                        dpg.add_spacer(height=4)
                        dpg.add_text("MENU", color=COL_MUTED)
                        dpg.add_spacer(height=6)
                        dpg.add_separator()
                        dpg.add_spacer(height=10)

                        dpg.add_text(tag="menu_title", default_value="MAIN", color=COL_TEXT)
                        dpg.add_spacer(height=10)

                        # buttons container (rebuilt)
                        with dpg.group(tag="buttons_container"):
                            pass

                        dpg.add_spacer(height=10)
                        dpg.add_separator()
                        dpg.add_spacer(height=10)

                        dpg.add_text("Shortcuts", color=COL_TEXT)
                        dpg.add_spacer(height=6)
                        dpg.add_text("F11: Fullscreen", color=COL_MUTED)
                        dpg.add_text("A/B/X/Y: Controller actions", color=COL_MUTED)

                    # CONTENT
                    with dpg.child_window(tag="content", border=False):
                        # --- dashboard grid ---
                        with dpg.table(header_row=False, resizable=False, policy=dpg.mvTable_SizingStretchProp,
                                       borders_innerV=False, borders_outerV=False, borders_innerH=False, borders_outerH=False):

                            dpg.add_table_column(init_width_or_weight=1)
                            dpg.add_table_column(init_width_or_weight=1)
                            dpg.add_table_column(init_width_or_weight=1)

                            # ===== Row 1: metrics =====
                            with dpg.table_row():
                                for tag, title in [("metric_serial", "Serial"), ("metric_joy", "Joystick"), ("metric_mode", "Mode")]:
                                    with dpg.child_window(tag=tag, border=False, height=90):
                                        dpg.bind_item_theme(tag, theme_card)
                                        dpg.add_text(title, color=COL_MUTED)
                                        dpg.add_spacer(height=6)
                                        dpg.add_text("", tag=f"{tag}_value", color=COL_TEXT)

                            # spacer row
                            with dpg.table_row():
                                dpg.add_spacer(height=10)
                                dpg.add_spacer(height=10)
                                dpg.add_spacer(height=10)

                        # ===== Row 2: log + controls (2 columns) =====
                        with dpg.table(header_row=False, resizable=False, policy=dpg.mvTable_SizingStretchProp,
                                       borders_innerV=False, borders_outerV=False, borders_innerH=False, borders_outerH=False):

                            dpg.add_table_column(init_width_or_weight=2)
                            dpg.add_table_column(init_width_or_weight=1)

                            with dpg.table_row():
                                # --- Arduino Log Card ---
                                with dpg.child_window(tag="card_log", border=False, height=340):
                                    dpg.bind_item_theme("card_log", theme_card)

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
                                with dpg.child_window(tag="card_controls", border=False, height=340):
                                    dpg.bind_item_theme("card_controls", theme_card)

                                    with dpg.group(horizontal=True):
                                        dpg.add_text("Controls", color=COL_TEXT)
                                        dpg.add_spacer(width=10)
                                        dpg.add_text(tag="mode_label", default_value="", color=COL_ACCENT)

                                    dpg.add_spacer(height=8)
                                    dpg.add_separator()
                                    dpg.add_spacer(height=10)

                                    dpg.add_text(tag="controls_text", default_value="", color=COL_MUTED)

                        # ===== Row 3: optional bottom card =====
                        dpg.add_spacer(height=14)
                        with dpg.child_window(tag="card_bottom", border=False, height=160):
                            dpg.bind_item_theme("card_bottom", theme_card)
                            dpg.add_text("Notes / Activity", color=COL_TEXT)
                            dpg.add_spacer(height=8)
                            dpg.add_separator()
                            dpg.add_spacer(height=10)
                            dpg.add_text("Tip: Put controller hints, warnings, or last command sent here.", color=COL_MUTED)

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

        with dpg.group(tag="buttons_container", parent="sidebar"):
            # left padding hack (keeps buttons from touching edge)
            dpg.add_spacer(height=2)

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
                btn = dpg.add_button(label=lab, height=46, width=SIDEBAR_W - (PAD * 2),
                                     callback=on_button_callback, user_data=lab)
                dpg.bind_item_theme(btn, theme_btn_ghost)

                # Make STOP pop slightly
                if lab == "STOP":
                    dpg.bind_item_theme(btn, theme_btn_primary)

                dpg.add_spacer(height=10)

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
                    height=40,
                    width=SIDEBAR_W - (PAD * 2),
                    callback=toggle_mode
                )
                dpg.bind_item_theme(mt, theme_btn_ghost)

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

        # Update top status
        dpg.configure_item("status_serial", default_value=f"Serial: {'CONNECTED' if ser else 'DISCONNECTED'}",
                           color=(COL_GOOD if ser else COL_WARN))
        dpg.configure_item("status_joy", default_value=f"Joystick: {'CONNECTED' if joystick.available else 'DISCONNECTED'}",
                           color=(COL_GOOD if joystick.available else COL_WARN))
        mode_text = "D-PAD" if joystick.available and joystick.dpad_mode else ("ANALOG" if joystick.available else "—")
        dpg.configure_item("status_mode", default_value=f"Mode: {mode_text}", color=COL_MUTED)

        # Update metric cards
        dpg.configure_item("metric_serial_value",
                           default_value=("CONNECTED" if ser else "DISCONNECTED"),
                           color=(COL_GOOD if ser else COL_WARN))

        dpg.configure_item("metric_joy_value",
                           default_value=("CONNECTED" if joystick.available else "DISCONNECTED"),
                           color=(COL_GOOD if joystick.available else COL_WARN))

        metric_mode_text = "—"
        if joystick.available:
            metric_mode_text = "D-PAD" if joystick.dpad_mode else "ANALOG"

        dpg.configure_item("metric_mode_value",
                           default_value=metric_mode_text,
                           color=COL_TEXT)

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
