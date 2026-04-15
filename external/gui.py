"""
Dear PyGui-based GUI (dashboard reskin):
 - Uses Dear PyGui for UI
 - Serial command interface to Arduino
 - Responsive dashboard layout + card styling
"""

import sys
import time
import serial
from serial.tools import list_ports
import dearpygui.dearpygui as dpg
import re


BAUD_RATE = 115200
MAX_LOG_LINES = 2000

last_reconnect_attampt = 0.0
# ---------------- white + dark red theme ----------------
# White base
COL_BG        = (255, 255, 255)
COL_TOPBAR    = (210, 210, 215)
COL_SIDEBAR   = (210, 210, 215)

# Cards (slightly off-white)
COL_CARD      = (255, 255, 255)
COL_CARD_2    = (210, 210, 215)

# Borders/separators (light gray)
COL_BORDER    = (160, 160, 172)

# Text
COL_TEXT      = (20, 20, 25)
COL_MUTED     = (100, 100, 110)

# Accent (dark red)
COL_ACCENT        = (155, 26, 44)
COL_ACCENT_HOVER  = (185, 34, 56)
COL_BAD           = (210, 55, 70)
COL_WARN          = (230, 165, 80)
COL_GOOD          = (85, 200, 160)

SIDEBAR_W = 150
SIDEBAR_PAD = 10
TOPBAR_H = 52
PAD = 5
LEFT_MARGIN = 30
RIGHT_MARGIN = 30
SCROLL_W = 16  # extra right padding to avoid scrollbar overlap

# Pose display ranges
TRANS_RANGE = 1.0    # joystick axis range (-1..+1)
ROT_RANGE = 30.0     # max degrees for tilt/yaw display
NUM_LEGS = 6
LEG_MIN_IN = 0.0     # actuator min extension (inches)
LEG_MAX_IN = 8.0     # actuator max extension (inches)
LEG_WARN = 0.9       # fraction of range for yellow
LEG_CRIT = 0.97      # fraction of range for red


def main():
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
        nonlocal controller_mode, ser
        if ser:
            try:
                ser.write((cmd + "\n").encode())
                print(f">>> SENT: {cmd}")
            except Exception as e:
                print(f"[Serial] Write failed: {e} — port disconnected")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None


        if cmd == "controller":
            controller_mode = True
        elif cmd == "stop":
            controller_mode = False
        elif cmd.startswith("start") or cmd == "center":
            controller_mode = False

    # ---------------- state ----------------
    MAIN, START_MENU, PRESETS, RUNNING = "main", "start", "presets", "running"
    menu = MAIN
    active_preset = None
    last_reconnect_attempt = 0.0
    waiting_for_center = False
    arduino_log: list[str] = []
    serial_buf = ""
    leg_extensions: list[float] = [0.0] * NUM_LEGS  # inches
    # Serial parsing state for multi-line metric blocks
    pending_tag: str | None = None
    pending_values: list[float] = []
    latest_meas_vel: list[float] | None = None

    def finalize_pending():
        nonlocal pending_tag, pending_values, leg_extensions, latest_meas_vel
        if not pending_tag:
            return
        vals = pending_values[:]
        if pending_tag == "POS_IN" and vals:
            # Update available indices; keep the rest unchanged
            for i, v in enumerate(vals):
                if i >= NUM_LEGS:
                    break
                v = max(LEG_MIN_IN, min(LEG_MAX_IN, v))
                leg_extensions[i] = v
        elif pending_tag == "MEAS_VEL" and vals:
            # Pad/trim to NUM_LEGS for plotting
            filled = [0.0] * NUM_LEGS
            for i in range(min(NUM_LEGS, len(vals))):
                filled[i] = vals[i]
            latest_meas_vel = filled
        # reset
        pending_tag = None
        pending_values = []

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
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_CARD)
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg, COL_CARD)

            dpg.add_theme_color(dpg.mvThemeCol_Text, COL_TEXT)
            dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, (120, 120, 130))

            dpg.add_theme_color(dpg.mvThemeCol_Border, COL_BORDER)
            dpg.add_theme_color(dpg.mvThemeCol_Separator, COL_BORDER)
            dpg.add_theme_color(dpg.mvThemeCol_SeparatorHovered, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_SeparatorActive, COL_ACCENT)

            # light frame backgrounds
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (195, 195, 202))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (182, 182, 190))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (168, 168, 178))

            # scrollbars
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarBg, (210, 210, 215))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrab, (150, 150, 162))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabHovered, (130, 130, 143))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabActive, (110, 110, 125))
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
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 8)

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
            dpg.add_theme_color(dpg.mvThemeCol_Button, (0, 0, 0, 10))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (0, 0, 0, 18))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (0, 0, 0, 30))
            dpg.add_theme_color(dpg.mvThemeCol_Text, COL_TEXT)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)

    # progress bar themes
    def _bar_theme(fill, bg=(0, 0, 0, 20)):
        with dpg.theme() as t:
            with dpg.theme_component(dpg.mvProgressBar):
                dpg.add_theme_color(dpg.mvThemeCol_PlotHistogram, fill)
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, bg)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 3)
        return t

    theme_bar_accent = _bar_theme((135, 22, 40))
    theme_bar_good = _bar_theme(COL_GOOD)
    theme_bar_warn = _bar_theme(COL_WARN)
    theme_bar_bad = _bar_theme(COL_BAD)

    dpg.bind_theme(theme_global)

    # ---------------- layout helpers ----------------
    def viewport_size():
        return dpg.get_viewport_client_width(), dpg.get_viewport_client_height()

    def compute_top_row_h():
        # Scale top row height with viewport/content height, clamped for usability
        w, h = viewport_size()
        content_h = max(240, h - TOPBAR_H)
        val = int(content_h * 0.38)
        return max(220, min(360, val))

    def apply_layout():
        w, h = viewport_size()
        aw = w - LEFT_MARGIN - RIGHT_MARGIN  # available width after left & right margins
        dpg.configure_item("root", width=aw, height=h)
        dpg.configure_item("topbar", width=aw, height=TOPBAR_H)
        dpg.configure_item("main_area", width=aw, height=h - TOPBAR_H)

        # content sizes
        content_w = max(320, aw - SIDEBAR_W)
        content_h = max(240, h - TOPBAR_H)

        dpg.configure_item("sidebar", width=SIDEBAR_W)
        dpg.configure_item("content", width=content_w, height=content_h)
        # Dynamically size top row cards and telemetry plot based on viewport
        top_h = compute_top_row_h()
        if dpg.does_item_exist("card_pose"):
            dpg.configure_item("card_pose", height=top_h)
        if dpg.does_item_exist("card_legs"):
            dpg.configure_item("card_legs", height=top_h)
        if dpg.does_item_exist("card_telemetry"):
            dpg.configure_item("card_telemetry", height=top_h)
            plot_h = max(160, top_h - (PAD*4 + 60))
            # adjust heights for tabbed plots if present
            if dpg.does_item_exist("plot_pose"):
                dpg.configure_item("plot_pose", height=plot_h)
            if dpg.does_item_exist("plot_legs"):
                dpg.configure_item("plot_legs", height=plot_h)
            if dpg.does_item_exist("plot_snapshot"):
                dpg.configure_item("plot_snapshot", height=plot_h)

        # adjust bottom-right deep telemetry plots heights
        if dpg.does_item_exist("card_deep"):
            card_h = dpg.get_item_height("card_deep") or 240
            plot_h2 = max(160, card_h - (PAD*4 + 60))
            if dpg.does_item_exist("plot_vel"):
                dpg.configure_item("plot_vel", height=plot_h2)
            if dpg.does_item_exist("plot_accel"):
                dpg.configure_item("plot_accel", height=plot_h2)
            if dpg.does_item_exist("plot_sat"):
                dpg.configure_item("plot_sat", height=plot_h2)

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
        serial_required = {"CENTER", "STOP", "DEMO", "FIGURE 8", "RESTART", "ORBIT", "WAVE", "CONTROLLER"}
        if action_label in serial_required:
            if not ser:
                missing.append("Serial port not connected")
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
                dpg.add_text("Fans On", color=COL_GOOD)

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

                            # buttons container (rebuilt)
                            with dpg.group(tag="buttons_container"):
                                pass

                    # CONTENT
                    with dpg.child_window(tag="content", border=False):

                        # =========================
                        # TOP ROW: Pose | Actuators | Telemetry
                        # =========================
                        TOP_ROW_H = compute_top_row_h()

                        with dpg.table(header_row=False, resizable=False, policy=dpg.mvTable_SizingStretchProp,
                                       borders_innerV=False, borders_outerV=False, borders_innerH=False, borders_outerH=False):
                            dpg.add_table_column(init_width_or_weight=3)  # pose
                            dpg.add_table_column(init_width_or_weight=2)  # legs
                            dpg.add_table_column(init_width_or_weight=2)  # telemetry

                            with dpg.table_row():

                                # --- Pose Display ---
                                with dpg.table_cell():
                                    with dpg.child_window(tag="card_pose", border=False, height=TOP_ROW_H):
                                        dpg.bind_item_theme("card_pose", theme_card)
                                        dpg.add_spacer(height=PAD)
                                        with dpg.group(indent=PAD):
                                            dpg.add_text("Platform Pose", color=COL_MUTED)
                                            dpg.add_spacer(height=3)

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
                                                    bar = dpg.add_progress_bar(
                                                        tag=ax_tag,
                                                        default_value=0.5,
                                                        overlay="0.0",
                                                        width=-(PAD + 60 + SCROLL_W),
                                                    )
                                                    dpg.bind_item_theme(bar, theme_bar_accent)
                                                    dpg.add_text("", tag=f"{ax_tag}_val", color=COL_TEXT)

                                # --- Actuators ---
                                with dpg.table_cell():
                                    with dpg.child_window(tag="card_legs", border=False, height=TOP_ROW_H):
                                        dpg.bind_item_theme("card_legs", theme_card)
                                        dpg.add_spacer(height=PAD)
                                        with dpg.group(indent=PAD):
                                            dpg.add_text("Actuators", color=COL_MUTED)
                                            dpg.add_spacer(height=6)

                                            for i in range(NUM_LEGS):
                                                with dpg.group(horizontal=True):
                                                    dpg.add_text(f"Leg {i+1}", color=COL_MUTED)
                                                    dpg.add_spacer(width=4)
                                                    bar = dpg.add_progress_bar(
                                                        tag=f"leg_{i}",
                                                        default_value=0.0,
                                                        overlay="--",
                                                        width=-(PAD + 60 + SCROLL_W),
                                                    )
                                                    dpg.bind_item_theme(bar, theme_bar_good)
                                                    dpg.add_text("", tag=f"leg_{i}_val", color=COL_TEXT)

                                # --- Telemetry ---
                                with dpg.table_cell():
                                    with dpg.child_window(tag="card_telemetry", border=False, height=TOP_ROW_H):
                                        dpg.bind_item_theme("card_telemetry", theme_card)
                                        dpg.add_spacer(height=PAD)
                                        with dpg.group(indent=PAD):
                                            dpg.add_text("Telemetry", color=COL_TEXT)
                                            dpg.add_spacer(height=8)
                                            dpg.add_separator()
                                            dpg.add_spacer(height=10)

                                            plot_h = max(160, TOP_ROW_H - (PAD*4 + 60))
                                            with dpg.tab_bar():

                                                # ---- A) Pose Timeseries ----
                                                with dpg.tab(label="Pose"):
                                                    with dpg.plot(tag="plot_pose", height=plot_h, width=-(SCROLL_W + PAD)):
                                                        dpg.add_plot_legend()
                                                        dpg.add_plot_axis(dpg.mvXAxis, tag="plot_pose_x", label="time (s)")
                                                        dpg.add_plot_axis(dpg.mvYAxis, tag="plot_pose_y", label="deg / norm")

                                                        dpg.add_line_series([], [], label="Roll (deg)",  parent="plot_pose_y", tag="series_roll")
                                                        dpg.add_line_series([], [], label="Pitch (deg)", parent="plot_pose_y", tag="series_pitch")
                                                        dpg.add_line_series([], [], label="Yaw (deg)",   parent="plot_pose_y", tag="series_yaw")
                                                        dpg.add_line_series([], [], label="X (norm)",    parent="plot_pose_y", tag="series_x")
                                                        dpg.add_line_series([], [], label="Y (norm)",    parent="plot_pose_y", tag="series_y")
                                                        dpg.add_line_series([], [], label="Z (norm)",    parent="plot_pose_y", tag="series_z")

                                                # ---- B) Leg Length Timeseries ----
                                                with dpg.tab(label="Legs"):
                                                    with dpg.plot(tag="plot_legs", height=plot_h, width=-(SCROLL_W + PAD)):
                                                        dpg.add_plot_legend()
                                                        dpg.add_plot_axis(dpg.mvXAxis, tag="plot_legs_x", label="time (s)")
                                                        dpg.add_plot_axis(dpg.mvYAxis, tag="plot_legs_y", label="inches")

                                                        for i in range(NUM_LEGS):
                                                            dpg.add_line_series([], [], label=f"Leg {i+1}", parent="plot_legs_y", tag=f"series_leg_{i}")

                                                # ---- C) Leg Snapshot Bars ----
                                                with dpg.tab(label="Snapshot"):
                                                    with dpg.plot(tag="plot_snapshot", height=plot_h, width=-(SCROLL_W + PAD)):
                                                        dpg.add_plot_axis(dpg.mvXAxis, tag="plot_snap_x", label="leg")
                                                        dpg.add_plot_axis(dpg.mvYAxis, tag="plot_snap_y", label="inches")

                                                        # x positions 1..6, y values update
                                                        dpg.add_bar_series([1,2,3,4,5,6], [0,0,0,0,0,0], weight=0.8, parent="plot_snap_y", tag="bars_legs")

                        dpg.add_spacer(height=6)

                        # =========================
                        # BOTTOM ROW: Log | Platform Visualization
                        # =========================
                        with dpg.table(header_row=False, resizable=False, policy=dpg.mvTable_SizingStretchProp,
                                       borders_innerV=False, borders_outerV=False, borders_innerH=False, borders_outerH=False):
                            dpg.add_table_column(init_width_or_weight=3)  # log big
                            dpg.add_table_column(init_width_or_weight=2)  # viz

                            with dpg.table_row():

                                # --- Arduino Log ---
                                with dpg.table_cell():
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

                                # --- Deep Telemetry (replacing Platform Visualization) ---
                                with dpg.table_cell():
                                    with dpg.child_window(tag="card_deep", border=False, height=-1):
                                        dpg.bind_item_theme("card_deep", theme_card)
                                        dpg.add_spacer(height=PAD)
                                        with dpg.group(indent=PAD):
                                            dpg.add_text("Deep Telemetry", color=COL_TEXT)
                                            dpg.add_spacer(height=8)
                                            dpg.add_separator()
                                            dpg.add_spacer(height=10)

                                            # Tabs: Velocity | Acceleration | Saturation %
                                            with dpg.tab_bar():
                                                # Velocity
                                                with dpg.tab(label="Velocity"):
                                                    with dpg.plot(tag="plot_vel", height=220, width=-(SCROLL_W + PAD)):
                                                        dpg.add_plot_legend()
                                                        dpg.add_plot_axis(dpg.mvXAxis, tag="plot_vel_x", label="time (s)")
                                                        dpg.add_plot_axis(dpg.mvYAxis, tag="plot_vel_y", label="in/s")
                                                        for i in range(NUM_LEGS):
                                                            dpg.add_line_series([], [], label=f"Leg {i+1}", parent="plot_vel_y", tag=f"series_vel_{i}")

                                                # Acceleration
                                                with dpg.tab(label="Acceleration"):
                                                    with dpg.plot(tag="plot_accel", height=220, width=-(SCROLL_W + PAD)):
                                                        dpg.add_plot_legend()
                                                        dpg.add_plot_axis(dpg.mvXAxis, tag="plot_accel_x", label="time (s)")
                                                        dpg.add_plot_axis(dpg.mvYAxis, tag="plot_accel_y", label="in/s^2")
                                                        for i in range(NUM_LEGS):
                                                            dpg.add_line_series([], [], label=f"Leg {i+1}", parent="plot_accel_y", tag=f"series_acc_{i}")

                                                # Saturation %
                                                with dpg.tab(label="Saturation %"):
                                                    with dpg.plot(tag="plot_sat", height=220, width=-(SCROLL_W + PAD)):
                                                        dpg.add_plot_legend()
                                                        dpg.add_plot_axis(dpg.mvXAxis, tag="plot_sat_x", label="time (s)")
                                                        dpg.add_plot_axis(dpg.mvYAxis, tag="plot_sat_y", label="fraction (0-1)")
                                                        for i in range(NUM_LEGS):
                                                            dpg.add_line_series([], [], label=f"Leg {i+1}", parent="plot_sat_y", tag=f"series_sat_{i}")


    # Responsive sizing initial
    apply_layout()

    # ---------------- menu logic ----------------
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

    def sidebar_height(n_buttons):
        # 10 top spacer + n*(34 btn + 4 spacer) + 10 bottom pad
        return 10 + n_buttons * 38 + 55

    def rebuild_buttons():
        if dpg.does_item_exist("buttons_container"):
            dpg.delete_item("buttons_container")

        with dpg.group(tag="buttons_container", parent="sidebar", indent=SIDEBAR_PAD):
            labels = []
            if menu == MAIN:
                labels = ["START", "CENTER", "CALIBRATE", "STOP"]

            elif menu == START_MENU:
                labels = ["CONTROLLER", "PRESETS", "BACK"]

            elif menu == PRESETS:
                labels = ["DEMO", "FIGURE 8", "ORBIT", "WAVE", "BACK"]

            elif menu == RUNNING:
                labels = ["RESTART", "STOP"]


            for i, lab in enumerate(labels):
                btn = dpg.add_button(label=lab, height=34, width=SIDEBAR_W - (SIDEBAR_PAD * 2),
                                     callback=on_button_callback, user_data=lab)
                if lab == "STOP":
                    dpg.bind_item_theme(btn, theme_btn_primary)
                else:
                    dpg.bind_item_theme(btn, theme_btn_sidebar)

                dpg.add_spacer(height=4)

            dpg.configure_item("sidebar", width=SIDEBAR_W, height=sidebar_height(len(labels)))


    rebuild_buttons()

    # ---------------- telemetry buffers ----------------
    MAX_SAMPLES = 300
    t_hist = []
    x_hist  = []
    y_hist  = []
    roll_hist  = []
    pitch_hist = []
    yaw_hist   = []
    z_hist     = []

    legs_hist = [[] for _ in range(NUM_LEGS)]
    vel_hist  = [[] for _ in range(NUM_LEGS)]
    acc_hist  = [[] for _ in range(NUM_LEGS)]
    sat_hist  = [[] for _ in range(NUM_LEGS)]

    # ---------------- simple 3D projection helpers ----------------
    def rot_y(angle_deg):
        import math
        a = math.radians(angle_deg)
        c, s = math.cos(a), math.sin(a)
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]

    def rot_x(angle_deg):
        import math
        a = math.radians(angle_deg)
        c, s = math.cos(a), math.sin(a)
        return [[1, 0, 0], [0, c, -s], [0, s, c]]

    def rot_z(angle_deg):
        import math
        a = math.radians(angle_deg)
        c, s = math.cos(a), math.sin(a)
        return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

    def mat_mul_vec(m, v):
        return [m[0][0]*v[0] + m[0][1]*v[1] + m[0][2]*v[2],
                m[1][0]*v[0] + m[1][1]*v[1] + m[1][2]*v[2],
                m[2][0]*v[0] + m[2][1]*v[1] + m[2][2]*v[2]]

    def project_point(px, py, w, h, p, scale=100, z_cam=3.0):
        # simple perspective projection onto drawlist
        import math
        x, y, z = p
        z += z_cam
        f = scale / max(0.1, z)
        sx = px + w*0.5 + x * f
        sy = py + h*0.5 - y * f
        return sx, sy

    def hex_points(radius, z):
        import math
        pts = []
        for i in range(6):
            ang = math.pi/3 * i
            pts.append([radius*math.cos(ang), radius*math.sin(ang), z])
        return pts

    def draw_platform(pitch_deg, yaw_deg, tx, ty, tz):
        # geometry
        r_base = 1.0
        r_top = 0.9
        base_z = 0.0
        top_h = 1.1 + tz  # z translation moves top plate up/down

        base = hex_points(r_base, base_z)
        top = hex_points(r_top, top_h)

        # rotate top plate (pitch about X, yaw about Z)
        rx = rot_x(pitch_deg)
        rz = rot_z(yaw_deg)
        def apply(v):
            v1 = mat_mul_vec(rx, v)
            v2 = mat_mul_vec(rz, v1)
            # apply XY translation
            return [v2[0] + tx, v2[1] + ty, v2[2]]
        top_r = [apply(v) for v in top]

        # draw
        w = dpg.get_item_width("cube_drawlist") or 240
        h = dpg.get_item_height("cube_drawlist") or 240
        px = 0
        py = 0
        dpg.delete_item("cube_drawlist", children_only=True)

        # frame rect to make widget visible
        dpg.draw_rectangle((1,1), (max(2,w-2), max(2,h-2)), color=COL_BORDER, thickness=1.0, parent="cube_drawlist")

        # axes for orientation reference
        origin = [0,0,base_z]
        axis_len = 0.8
        axes = {
            "X": ([axis_len,0,base_z], (200, 100, 255)),
            "Y": ([0,axis_len,base_z], (100, 255, 200)),
            "Z": ([0,0,axis_len+base_z], (255, 200, 100)),
        }
        for _, (end, col) in axes.items():
            x1,y1 = project_point(px, py, w, h, origin)
            x2,y2 = project_point(px, py, w, h, end)
            dpg.draw_line((x1,y1), (x2,y2), color=col, thickness=2.0, parent="cube_drawlist")

        # base edges
        for i in range(6):
            a = base[i]
            b = base[(i+1) % 6]
            x1,y1 = project_point(px, py, w, h, a)
            x2,y2 = project_point(px, py, w, h, b)
            dpg.draw_line((x1,y1), (x2,y2), color=COL_MUTED, thickness=2.0, parent="cube_drawlist")

        # top edges
        for i in range(6):
            a = top_r[i]
            b = top_r[(i+1) % 6]
            x1,y1 = project_point(px, py, w, h, a)
            x2,y2 = project_point(px, py, w, h, b)
            dpg.draw_line((x1,y1), (x2,y2), color=COL_ACCENT, thickness=2.0, parent="cube_drawlist")

        # legs (color by extension fraction)
        leg_range = LEG_MAX_IN - LEG_MIN_IN
        for i in range(6):
            a = base[i]
            b = top_r[i]
            x1,y1 = project_point(px, py, w, h, a)
            x2,y2 = project_point(px, py, w, h, b)
            ext = leg_extensions[i]
            frac = (ext - LEG_MIN_IN) / leg_range if leg_range > 0 else 0.0
            frac = max(0.0, min(1.0, frac))
            if frac >= LEG_CRIT:
                col = COL_BAD
            elif frac >= LEG_WARN:
                col = COL_WARN
            else:
                col = COL_GOOD
            dpg.draw_line((x1,y1), (x2,y2), color=col, thickness=2.0, parent="cube_drawlist")

    # ---------------- main loop ----------------
    while dpg.is_dearpygui_running():

        # Serial feedback
        if ser:
                try:
                    waiting = ser.in_waiting
                except Exception as e:
                    print(f"[Serial] Port lost: {e}")
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                    waiting = 0

                if waiting:
                    serial_buf += ser.read(waiting).decode(errors="replace")

                if ser:
                    while "\n" in serial_buf:
                        line, serial_buf = serial_buf.split("\n", 1)
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

                        # Parse metrics from Arduino logs
                        # A) Legacy per-motor format: "M1: 4.500" or "M1: start=..., end=..., max=..."
                        matched_legacy = False
                        for mi in range(NUM_LEGS):
                            prefix = f"M{mi+1}:"
                            if prefix in line:
                                try:
                                    after = line.split(prefix, 1)[1].strip()
                                    val = float(after.split(",")[0].split("=")[-1])
                                    # clamp to valid range
                                    leg_extensions[mi] = max(LEG_MIN_IN, min(LEG_MAX_IN, val))
                                    matched_legacy = True
                                except (ValueError, IndexError):
                                    pass

                        if matched_legacy:
                            continue

                        # B) Tagged block formats: <POS_IN>, <MEAS_VEL>, <CMD_VEL_PER_SEC>
                        # Start of a block
                        if "<POS_IN>" in line:
                            finalize_pending()
                            pending_tag = "POS_IN"
                            pending_values = []
                        if "<MEAS_VEL>" in line:
                            finalize_pending()
                            pending_tag = "MEAS_VEL"
                            pending_values = []
                        if "<CMD_VEL_PER_SEC>" in line:
                            finalize_pending()
                            pending_tag = "CMD_VEL_PER_SEC"
                            pending_values = []

                        # Accumulate numeric values for current block
                        if pending_tag is not None:
                            if pending_tag == "POS_IN":
                                # Capture all numbers on comma-delimited lines, else single plausible value
                                if "," in line:
                                    nums = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", line)
                                    for n in nums:
                                        val = float(n)
                                        if LEG_MIN_IN - 0.1 <= val <= LEG_MAX_IN + 0.1:
                                            pending_values.append(val)
                                else:
                                    m = re.search(r"[-+]?(?:\d*\.\d+|\d+)", line)
                                    if m:
                                        val = float(m.group(0))
                                        if LEG_MIN_IN - 0.1 <= val <= LEG_MAX_IN + 0.1:
                                            pending_values.append(val)
                                # If we gathered 6 values, finalize immediately
                                if len(pending_values) >= NUM_LEGS:
                                    finalize_pending()
                            else:
                                # Other tags may include all values on the same line
                                nums = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", line)
                                if nums:
                                    pending_values.extend(float(n) for n in nums)
                                # Consume when we have enough values
                                if len(pending_values) >= NUM_LEGS:
                                    finalize_pending()

        # Auto-reconnect if disconnected
        if not ser:
            now_r = time.monotonic()
            if now_r - last_reconnect_attempt > 3.0:  # try every 3 seconds
                last_reconnect_attempt = now_r
                reconnect_port = detect_serial_port()
                if reconnect_port:
                    try:
                        ser = serial.Serial(reconnect_port, BAUD_RATE, timeout=0)
                        time.sleep(0.5)
                        try:
                            ser.reset_input_buffer()
                        except Exception:
                            pass
                        serial_buf = ""
                        print(f"[Serial] Reconnected on {reconnect_port}")
                    except Exception as e:
                        print(f"[Serial] Reconnect failed: {e}")
                        ser = None

        # Update top status
        dpg.configure_item("status_serial", default_value=f"Serial: {'CONNECTED' if ser else 'DISCONNECTED'}",
                           color=(COL_GOOD if ser else COL_WARN))

        # Update pose display
        pose_vals = {
            "pose_roll":  (0.0, ROT_RANGE),
            "pose_pitch": (0.0, ROT_RANGE),
            "pose_yaw":   (0.0, ROT_RANGE),
            "pose_x":     (0.0, TRANS_RANGE),
            "pose_y":     (0.0, TRANS_RANGE),
            "pose_z":     (0.0, TRANS_RANGE),
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

        # Update Telemetry buffers and plots (Pose + Legs + Snapshot)
        now = time.monotonic()

        # pose
        roll  = 0.0
        xval  = 0.0
        yval  = 0.0
        pitch = 0.0
        yaw   = 0.0
        zval  = 0.0

        t_hist.append(now)
        x_hist.append(xval)
        y_hist.append(yval)
        roll_hist.append(roll)
        pitch_hist.append(pitch)
        yaw_hist.append(yaw)
        z_hist.append(zval)

        # legs
        for i in range(NUM_LEGS):
            legs_hist[i].append(leg_extensions[i])

        # compute velocity, acceleration, saturation fraction
        if len(t_hist) >= 2:
            if latest_meas_vel is not None:
                for i in range(NUM_LEGS):
                    vel_hist[i].append(latest_meas_vel[i])
            else:
                dt = max(1e-6, t_hist[-1] - t_hist[-2])
                for i in range(NUM_LEGS):
                    v = (legs_hist[i][-1] - legs_hist[i][-2]) / dt
                    vel_hist[i].append(v)
        else:
            for i in range(NUM_LEGS):
                vel_hist[i].append(0.0)

        if len(t_hist) >= 3:
            dt2 = max(1e-6, t_hist[-1] - t_hist[-2])
            for i in range(NUM_LEGS):
                a = (vel_hist[i][-1] - vel_hist[i][-2]) / dt2 if len(vel_hist[i]) >= 2 else 0.0
                acc_hist[i].append(a)
        else:
            for i in range(NUM_LEGS):
                acc_hist[i].append(0.0)

        leg_range = max(1e-6, LEG_MAX_IN - LEG_MIN_IN)
        for i in range(NUM_LEGS):
            frac = (leg_extensions[i] - LEG_MIN_IN) / leg_range
            frac = max(0.0, min(1.0, frac))
            sat_hist[i].append(frac)

        # trim
        if len(t_hist) > MAX_SAMPLES:
            t_hist.pop(0)
            x_hist.pop(0)
            y_hist.pop(0)
            roll_hist.pop(0)
            pitch_hist.pop(0)
            yaw_hist.pop(0)
            z_hist.pop(0)
            for i in range(NUM_LEGS):
                legs_hist[i].pop(0)
                vel_hist[i].pop(0)
                acc_hist[i].pop(0)
                sat_hist[i].pop(0)

        # normalize time to start at 0
        if t_hist:
            t0 = t_hist[0]
            xs = [t - t0 for t in t_hist]

            # A) Pose plot
            dpg.set_value("series_roll",  [xs, roll_hist])
            dpg.set_value("series_pitch", [xs, pitch_hist])
            dpg.set_value("series_yaw",   [xs, yaw_hist])
            dpg.set_value("series_x",     [xs, x_hist])
            dpg.set_value("series_y",     [xs, y_hist])
            dpg.set_value("series_z",     [xs, z_hist])

            # B) Leg timeseries plot (top tabs)
            for i in range(NUM_LEGS):
                dpg.set_value(f"series_leg_{i}", [xs, legs_hist[i]])

            # Bottom-right: Velocity, Acceleration, Saturation %
            for i in range(NUM_LEGS):
                dpg.set_value(f"series_vel_{i}", [xs, vel_hist[i]])
                dpg.set_value(f"series_acc_{i}", [xs, acc_hist[i]])
                dpg.set_value(f"series_sat_{i}", [xs, sat_hist[i]])

        # C) Snapshot bar chart (instant)
        dpg.set_value("bars_legs", [[1,2,3,4,5,6], leg_extensions])

        # Bottom-right plots sizing handled in apply_layout()

        # Update card labels/text
        if menu == RUNNING and active_preset:
            dpg.configure_item("running_label", default_value=f"RUNNING: {active_preset.upper()}")
        else:
            dpg.configure_item("running_label", default_value="")
        dpg.configure_item("arduino_log_text", default_value="\n".join(arduino_log))
        dpg.set_y_scroll("log_box", dpg.get_y_scroll_max("log_box"))

        dpg.render_dearpygui_frame()
        time.sleep(1 / 30)

    # Cleanup
    if ser:
        try:
            ser.write(b"stop\n")
        except Exception:
            pass
        ser.close()
    dpg.destroy_context()


if __name__ == "__main__":
    main()