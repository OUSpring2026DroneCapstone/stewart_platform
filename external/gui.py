"""
Dear PyGui-based GUI (hybrid):
 - Uses Dear PyGui for UI
 - Keeps pygame joystick backend and serial logic
 - Preserves original behavior and commands
"""

import sys
import time
import serial
import pygame
from joystick import JoystickBackend
import dearpygui.dearpygui as dpg

BAUD_RATE = 115200

# Theme colors (modern dark)
BG_MAIN = (22, 20, 33)
BG_LEFT = (32, 29, 46)
BG_CARD = (42, 38, 61)
ACCENT = (0, 215, 170)
TEXT_MAIN = (235, 240, 248)
TEXT_MUTED = (168, 176, 194)

# Arduino log cap
MAX_LOG_LINES = 18


def main():
    # Init pygame for joystick backend

    pygame.init()

    # ---------------- serial (optional) ----------------
    ser = None
    port = sys.argv[1] if len(sys.argv) >= 2 else None
    if port:
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=0)
            time.sleep(2.0)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
        except Exception as e:
            print(f"[Serial] Failed to open {port}: {e}")
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

    # ---------------- Dear PyGui setup ----------------
    dpg.create_context()
    dpg.create_viewport(title="Stewart Platform Control", width=1200, height=650)
    dpg.setup_dearpygui()
    arduino_log: list[str] = []
    dpg.show_viewport()

    # Simple theme
    with dpg.theme() as app_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, BG_MAIN)
            dpg.add_theme_color(dpg.mvThemeCol_Text, TEXT_MAIN)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 8)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 10, 10)
    dpg.bind_theme(app_theme)

    # Root windows
    with dpg.window(no_title_bar=True, no_resize=True, no_move=True, pos=(0, 0), width=1200, height=60):
        dpg.add_text("Stewart Platform", color=TEXT_MAIN)
        dpg.add_text(" ")

    with dpg.window(tag="left_panel", no_title_bar=True, pos=(0, 60), width=380, height=590):
        dpg.add_text(tag="menu_title", default_value="MAIN", color=TEXT_MUTED)
        dpg.add_separator()

    with dpg.window(tag="viz_panel", no_title_bar=True, pos=(380 + 40, 100), width=520, height=470):
        dpg.add_text("ARDUINO LOG", color=(140, 150, 200))
        dpg.add_separator()
        dpg.add_text(tag="arduino_log_text", default_value="", color=(90, 110, 140))
        dpg.add_spacer(height=12)
        dpg.add_text("CONTROLS", color=(140, 150, 200))
        dpg.add_separator()
        dpg.add_text(tag="controls_text", default_value="", color=TEXT_MUTED)
        dpg.add_spacer(height=8)
        dpg.add_text(tag="running_label", default_value="", color=TEXT_MUTED)
        dpg.add_text(tag="mode_label", default_value="", color=(138, 150, 190))

    # Popup helper
    def show_popup(messages):
        if isinstance(messages, str):
            messages = [messages]
        # Delete existing popup if present
        if dpg.does_item_exist("popup"):
            dpg.delete_item("popup")
        with dpg.window(modal=True, no_title_bar=True, tag="popup", pos=(340, 220), width=520, height=220):
            dpg.add_text("Connection Required", color=TEXT_MAIN)
            dpg.add_separator()
            for msg in messages:
                dpg.add_text(msg, color=TEXT_MAIN)
            dpg.add_spacer(height=8)
            dpg.add_button(label="OK", width=120, callback=lambda *args, **kwargs: dpg.delete_item("popup"))

    def connection_error_for(action_label):
        missing = []
        serial_required = {"CENTER", "STOP", "DEMO", "FIGURE 8", "RESTART"}
        controller_actions = {"CONTROLLER"}
        if action_label in serial_required or action_label in controller_actions:
            if not ser:
                missing.append("Serial port not connected")
        if action_label in controller_actions:
            if not joystick.available:
                missing.append("Joystick not connected")
        return missing or None

    # Build left panel buttons depending on menu
    def rebuild_buttons():
        if dpg.does_item_exist("buttons_container"):
            dpg.delete_item("buttons_container")
        # Use a simple group as container for reliable rebuilds
        with dpg.group(tag="buttons_container", parent="left_panel"):
            labels = []
            if menu == MAIN:
                labels = ["START", "CENTER", "CALIBRATE", "STOP"]
                dpg.configure_item("menu_title", default_value="MAIN")
            elif menu == START_MENU:
                labels = ["CONTROLLER", "PRESETS", "BACK"]
                dpg.configure_item("menu_title", default_value="START MENU")
            elif menu == PRESETS:
                labels = ["DEMO", "FIGURE 8", "BACK"]
                dpg.configure_item("menu_title", default_value="PRESETS")
            elif menu == RUNNING:
                labels = ["RESTART", "STOP"]
                dpg.configure_item("menu_title", default_value="RUNNING")

            for lab in labels:
                dpg.add_button(label=lab, height=42, width=260,
                               callback=on_button_callback, user_data=lab)
                dpg.add_spacer(height=12)

            if joystick.available:
                dpg.add_separator()
                dpg.add_text("JOYSTICK MODE", color=TEXT_MUTED)
                dpg.add_button(label=("D-PAD MODE" if joystick.dpad_mode else "ANALOG MODE"), height=36, width=220,
                               callback=toggle_mode, tag="mode_toggle")

    def toggle_mode(sender=None, app_data=None, user_data=None):
        joystick.toggle_mode()
        dpg.configure_item("mode_toggle", label=("D-PAD MODE" if joystick.dpad_mode else "ANALOG MODE"))
        print(f"[Mode] Switched to {'D-PAD' if joystick.dpad_mode else 'ANALOG'}")

    def on_button_callback(sender, app_data, user_data):
        # Route Dear PyGui button callbacks to our label-based handler
        on_button(user_data)

    def on_button(label):
        nonlocal menu, waiting_for_center, active_preset
        if label == "START" and menu == MAIN:
            menu = START_MENU
            rebuild_buttons()
            return

        # Show connection warnings but DO NOT block actions
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
            if label == "DEMO":
                active_preset = "demo"
                send_command("center")
                waiting_for_center = True
                menu = RUNNING
                dpg.configure_item("running_label", default_value=f"RUNNING: {active_preset.upper()}")
                rebuild_buttons()
            elif label == "FIGURE 8":
                active_preset = "figure8"
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

    rebuild_buttons()

    def build_controls_text():
        if not joystick.available:
            return (
                "Connect a joystick to enable controller shortcuts.\n"
                "A: Enable controller\nB: Stop\nX: Center\nY: Run demo"
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

    # Main render loop with backend processing
    clock = pygame.time.Clock()
    while dpg.is_dearpygui_running():
        # Process joystick events and ticking
        events = pygame.event.get()
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

        # Update labels
        if menu == RUNNING and active_preset:
            dpg.configure_item("running_label", default_value=f"RUNNING: {active_preset.upper()}")
        else:
            dpg.configure_item("running_label", default_value="")

        mode_text = "MODE: D-PAD (discrete positions)" if joystick.dpad_mode else "MODE: ANALOG (continuous + directional tilt)"
        dpg.configure_item("controls_text", default_value=build_controls_text())
        dpg.configure_item("arduino_log_text", default_value="\n".join(arduino_log))
        dpg.configure_item("mode_label", default_value=mode_text)

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
