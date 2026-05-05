"""
Simulated GUI — no hardware required.
Uses the exact geometry from platform.h and the exact waypoints from
runPreset() in main.cpp.  Leg lengths are computed with the same IK
formula used on the Arduino (translate top anchors, measure 3-D distance
to base anchors, subtract zero_length).

All presets run at the same speed as the real hardware (8 s / segment).

Usage:
    python test/gui_simulated.py
"""

import os
import time
import math
import dearpygui.dearpygui as dpg


# ─────────────────────── display constants ───────────────────────
MAX_LOG_LINES = 2000
COL_BG            = (61, 31, 36)
COL_TOPBAR        = (73, 34, 40)
COL_SIDEBAR       = (51, 27, 31)
COL_CARD          = (236, 226, 210)
COL_CARD_2        = (217, 202, 183)
COL_STEWART_BG    = (226, 213, 194)
COL_LOG_BG        = (224, 211, 192)
COL_BORDER        = (125, 93, 82)
COL_TEXT          = (34, 25, 23)
COL_MUTED         = (106, 79, 72)
COL_TEXT_INV      = (245, 235, 221)
COL_TEXT_INV_SOFT = (215, 194, 173)
COL_PANEL_DARK    = (35, 23, 24)
COL_PANEL_DARK_2  = (51, 31, 32)
COL_ACCENT        = (123, 29, 43)
COL_ACCENT_HOVER  = (148, 43, 59)
COL_BAD           = (92, 17, 28)
COL_WARN          = (160, 98, 79)
COL_GOOD          = (191, 159, 123)
COL_PLOT_BG       = (244, 236, 223)
COL_PLOT_BORDER   = (116, 84, 75)

POSE_SERIES_COLORS = (
    (122, 31, 45),
    (176, 118, 92),
    (86, 61, 58),
)

LEG_SERIES_COLORS = (
    (122, 31, 45),
    (171, 91, 74),
    (205, 169, 130),
    (83, 57, 54),
    (148, 56, 69),
    (232, 220, 202),
)

SIDEBAR_PAD = 10
TOPBAR_H    = 60
BOTTOMBAR_H = 68
PAD         = 5
LEFT_MARGIN = 24
RIGHT_MARGIN= 24
SCROLL_W    = 16
STEWART_VIEW_YAW   = 18.0
STEWART_VIEW_PITCH = -60.0
ITEM_SPACING_Y = 6
CARD_TOP_INSET = 0
SECTION_GAP    = 10
TOP_ROW_RATIO  = 0.42
BOTTOM_LEFT_WEIGHT  = 2.8
BOTTOM_RIGHT_WEIGHT = 2.8
RIGHT_CARD_OVERHEAD = CARD_TOP_INSET + 76
LOG_CARD_OVERHEAD = 0
BOTTOM_ROW_CLEARANCE = 10
CARD_RADIUS = 12
CARD_INNER_RADIUS = 10
LAYOUT_FIT_PAD = 14
BOTTOM_CARD_FOOT = 16

TRANS_RANGE = 1.0    # normalised display range (-1..+1)
ROT_RANGE   = 30.0   # degrees
NUM_LEGS    = 6
LEG_MIN_IN  = 0.0
LEG_MAX_IN  = 8.0
LEG_WARN    = 0.9
LEG_CRIT    = 0.97

# Physical pairs share a platform joint: (1,4), (2,3), (5,6).
# Display order maps those to GUI labels (1,2), (3,4), (5,6).
# Index into leg_extensions[] → GUI label number
_LEG_DISPLAY = [0, 3, 1, 2, 4, 5]   # C++ indices in display order
_LEG_LABELS  = [1, 2, 3, 4, 5, 6]   # corresponding GUI label

# ──────────── platform geometry (from include/platform.h) ─────────
# All values in inches, matching the Arduino source exactly.

_PLAT_0 = [0.0, 0.0, 17.4775 - 0.625]   # top-plate centre at home (= [0, 0, 16.8525])

_BASES = [
    [ 4.8125,       11.464,        0.0],  # base_1
    [11.98812,      -5.18221723,   0.0],  # base_2
    [ 5.17514616,  -11.98104893,   0.0],  # base_3
    [-4.8125,       11.464,        0.0],  # base_4
    [-11.98812,     -5.18221723,   0.0],  # base_5
    [-5.17514616,  -11.98104893,   0.0],  # base_6
]

_PLATS = [
    [ 0.75,         11.469,        0.0],  # plat_1
    [ 9.00827306,   -8.1549931,    0.0],  # plat_2
    [ 7.94761289,   -9.21565328,   0.0],  # plat_3
    [-0.75,         11.469,        0.0],  # plat_4
    [-9.00827306,   -8.1549931,    0.0],  # plat_5
    [-7.94761289,   -9.21565328,   0.0],  # plat_6
]

_BASE_RING_ORDER = tuple(sorted(range(NUM_LEGS),
                                key=lambda i: math.atan2(_BASES[i][1], _BASES[i][0])))
_PLAT_RING_ORDER = tuple(sorted(range(NUM_LEGS),
                                key=lambda i: math.atan2(_PLATS[i][1], _PLATS[i][0])))

# zero_length: leg length at T=[0,0,0], identity rotation
# mirrors the Arduino calculation: sqrt(sum((plat_0+plat_1 - base_1)^2))
def _calc_zero():
    total = 0.0
    for i in range(3):
        d = _PLAT_0[i] + _PLATS[0][i] - _BASES[0][i]
        total += d * d
    return math.sqrt(total)

_ZERO_LEN = _calc_zero()


def _rotate_point(px, py, pz, roll_deg, pitch_deg, yaw_deg=0.0):
    """Apply Rz(yaw) * Ry(pitch) * Rx(roll) to a platform-frame point."""
    roll  = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw   = math.radians(yaw_deg)
    cr, sr = math.cos(roll),  math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw),   math.sin(yaw)

    # Roll about X.
    x1 = px
    y1 = py * cr - pz * sr
    z1 = py * sr + pz * cr

    # Pitch about Y.
    x2 = x1 * cp + z1 * sp
    y2 = y1
    z2 = -x1 * sp + z1 * cp

    # Yaw about Z.
    rx = x2 * cy - y2 * sy
    ry = x2 * sy + y2 * cy
    rz = z2
    return rx, ry, rz


def ik(tx, ty, tz, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """Return 6 leg extensions (inches, clamped) for translation + rotation."""
    result = []
    for i in range(NUM_LEGS):
        rx, ry, rz = _rotate_point(*_PLATS[i], roll_deg, pitch_deg, yaw_deg)
        wx = _PLAT_0[0] + tx + rx
        wy = _PLAT_0[1] + ty + ry
        wz = _PLAT_0[2] + tz + rz
        bx, by, bz = _BASES[i]
        raw = math.sqrt((wx - bx)**2 + (wy - by)**2 + (wz - bz)**2)
        ext = raw - _ZERO_LEN
        result.append(max(LEG_MIN_IN, min(LEG_MAX_IN, ext)))
    return result


# ──────────── preset waypoint tables ────
# Each entry: (p0, p1, duration_seconds)
# p0/p1 are [tx, ty, tz] for translation-only presets, or
# [tx, ty, tz, roll_deg, pitch_deg, yaw_deg] when rotation is needed.
# Missing rotation components are treated as 0.

T0        = [0.0,  0.0,  0.0]
T1        = [0.0,  0.0,  2.0]
TX        = [5.0,  0.0,  2.0]
TY        = [0.0,  5.0,  2.0]
TZ        = [0.0,  0.0,  7.0]
T_CENTER  = [0.0,  0.0,  2.0]

_DUR = 8.0   # default segment duration (seconds)

PRESET_SEGS = {
    "demo": [
        (T0,       T1,       _DUR),
        (T1,       TX,       _DUR),
        (TX,       T1,       _DUR),
        (T1,       TY,       _DUR),
        (TY,       T1,       _DUR),
        (T1,       TZ,       5.0),   # shorter last segment
    ],
    "figure8": [
        (T_CENTER,            [4.5,  0.0, 3.0], _DUR),
        ([4.5,  0.0, 3.0],   [0.0,  4.5, 1.5], _DUR),
        ([0.0,  4.5, 1.5],   [-4.5, 0.0, 3.0], _DUR),
        ([-4.5, 0.0, 3.0],   [0.0, -4.5, 1.5], _DUR),
        ([0.0, -4.5, 1.5],   [4.5,  0.0, 3.0], _DUR),
        ([4.5,  0.0, 3.0],   T_CENTER,          _DUR),
    ],
    "orbit": [
        (T_CENTER,             [6.0,   0.0,  2.5], _DUR),
        ([6.0,   0.0,  2.5],   [4.2,   4.2,  4.0], _DUR),
        ([4.2,   4.2,  4.0],   [0.0,   6.0,  5.5], _DUR),
        ([0.0,   6.0,  5.5],   [-4.2,  4.2,  4.0], _DUR),
        ([-4.2,  4.2,  4.0],   [-6.0,  0.0,  2.5], _DUR),
        ([-6.0,  0.0,  2.5],   [-4.2, -4.2,  1.2], _DUR),
        ([-4.2, -4.2,  1.2],   [0.0,  -6.0,  0.5], _DUR),
        ([0.0,  -6.0,  0.5],   [4.2,  -4.2,  1.2], _DUR),
        ([4.2,  -4.2,  1.2],   [6.0,   0.0,  2.5], _DUR),
        ([6.0,   0.0,  2.5],   T_CENTER,            _DUR),
    ],
    "wave": [
        # Exaggerated multi-axis wave that still stays inside the simulated
        # 8 in leg stroke envelope: translation orbits in X/Y, Z rides a high/
        # low sweep, roll/pitch keep the tilt-wave feel, and yaw now swings too.
        (T_CENTER,                                [ 2.5,     0.0,     4.2,     0.0,    -18.0,    16.0], _DUR),
        ([ 2.5,     0.0,     4.2,     0.0,    -18.0,    16.0], [ 1.7678,  1.7678,  4.9071,  9.8995, -12.7279, 11.3137], _DUR),
        ([ 1.7678,  1.7678,  4.9071,  9.8995, -12.7279, 11.3137], [ 0.0,     2.5,     5.2,    14.0,     0.0,     0.0], _DUR),
        ([ 0.0,     2.5,     5.2,    14.0,     0.0,     0.0], [-1.7678,  1.7678,  4.9071,  9.8995,  12.7279,-11.3137], _DUR),
        ([-1.7678,  1.7678,  4.9071,  9.8995,  12.7279,-11.3137], [-2.5,     0.0,     4.2,     0.0,    18.0,   -16.0], _DUR),
        ([-2.5,     0.0,     4.2,     0.0,    18.0,   -16.0], [-1.7678, -1.7678,  3.4929, -9.8995,  12.7279,-11.3137], _DUR),
        ([-1.7678, -1.7678,  3.4929, -9.8995,  12.7279,-11.3137], [ 0.0,    -2.5,     3.2,   -14.0,     0.0,     0.0], _DUR),
        ([ 0.0,    -2.5,     3.2,   -14.0,     0.0,     0.0], [ 1.7678, -1.7678,  3.4929, -9.8995, -12.7279, 11.3137], _DUR),
        ([ 1.7678, -1.7678,  3.4929, -9.8995, -12.7279, 11.3137], [ 2.5,     0.0,     4.2,     0.0,   -18.0,    16.0], _DUR),
        ([ 2.5,     0.0,     4.2,     0.0,   -18.0,    16.0], T_CENTER, _DUR),
    ],
}

# Precompute cumulative durations for fast lookup
_PRESET_TOTAL = {name: sum(d for _, _, d in segs) for name, segs in PRESET_SEGS.items()}
_PRESET_BOUNDS = {}
for _name, _segs in PRESET_SEGS.items():
    _bounds = [0.0]
    _elapsed = 0.0
    for _, _, _dur in _segs:
        _elapsed += _dur
        _bounds.append(_elapsed)
    _PRESET_BOUNDS[_name] = tuple(_bounds)


def _time_label(seconds: float) -> str:
    whole = round(seconds)
    if abs(seconds - whole) < 1e-9:
        return str(int(whole))
    return f"{seconds:.1f}"


def _rolling_window_ticks(x_start: float, x_end: float, step: float = 2.0):
    first_tick = int(math.ceil(x_start / step) * step)
    ticks = tuple(
        (_time_label(float(t)), float(t))
        for t in range(first_tick, int(math.floor(x_end)) + 1, int(step))
    )
    if ticks:
        return ticks
    return ((_time_label(x_start), x_start), (_time_label(x_end), x_end))


def _preset_segment_info(name: str, elapsed: float):
    bounds = _PRESET_BOUNDS[name]
    total = bounds[-1]
    if total <= 0.0:
        return 0, 0.0, 0.0, 0.0

    t_in_cycle = elapsed % total
    for idx in range(len(bounds) - 1):
        seg_start = bounds[idx]
        seg_end = bounds[idx + 1]
        if t_in_cycle <= seg_end or idx == len(bounds) - 2:
            return idx, seg_start, seg_end, t_in_cycle - seg_start
    return len(bounds) - 2, bounds[-2], bounds[-1], 0.0


def _preset_segment_status(name: str, elapsed: float) -> str:
    bounds = _PRESET_BOUNDS[name]
    total = bounds[-1]
    if total <= 0.0:
        return "Segmented preset view"

    idx, seg_start, seg_end, seg_elapsed = _preset_segment_info(name, elapsed)
    seg_dur = max(0.0, seg_end - seg_start)
    return (
        f"Segment {idx + 1}/{len(bounds) - 1}"
        f"  {_time_label(seg_elapsed)}/{_time_label(seg_dur)} s"
    )


def _smoothstep(t: float) -> float:
    """Cubic Hermite ease-in/ease-out: zero velocity at both ends of each segment.
    Mirrors the P-controller behaviour on the real Arduino (ramps up then
    decelerates to the target, so velocity is zero at each waypoint)."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def _preset_position(name: str, elapsed: float):
    """Smoothstep through preset waypoints; loops when done.
    Returns [tx, ty, tz] or [tx, ty, tz, roll, pitch, yaw]
    depending on waypoint length."""
    segs  = PRESET_SEGS[name]
    total = _PRESET_TOTAL[name]
    t     = elapsed % total
    for p0, p1, dur in segs:
        if t <= dur:
            frac = _smoothstep(t / dur)
            n = max(len(p0), len(p1))
            def _v(p, i): return p[i] if i < len(p) else 0.0
            return [_v(p0, i) + frac * (_v(p1, i) - _v(p0, i)) for i in range(n)]
        t -= dur
    return list(segs[-1][1])


# ─────────────────────────────────────────────────────────────────
def main():
    # ──────────────── state ────────────────
    MAIN, START_MENU, PRESETS, RUNNING = "main", "start", "presets", "running"
    menu           = MAIN
    active_preset  = None
    preset_start_t = 0.0
    arduino_log: list[str] = [
        "[SIM] IK visualization — platform geometry active",
        "[SIM] IK computed from platform.h geometry",
        f"[SIM] zero_length = {_ZERO_LEN:.4f} in",
    ]
    leg_extensions: list[float] = ik(*T_CENTER)

    def send_command(cmd: str):
        print(f">>> [SIM] {cmd}")

    # ──────────────── DPG setup ────────────────
    dpg.create_context()
    dpg.create_viewport(title="Stewart Platform Control [SIMULATED]",
                        width=1200, height=680, resizable=True)
    dpg.setup_dearpygui()
    dpg.show_viewport()

    with dpg.font_registry():
        if os.path.exists(r"C:\Windows\Fonts\segoeui.ttf"):
            with dpg.font(r"C:\Windows\Fonts\segoeui.ttf", 17, tag="font_ui_base"):
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
            with dpg.font(r"C:\Windows\Fonts\segoeui.ttf", 16, tag="font_ui_label"):
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
        if os.path.exists(r"C:\Windows\Fonts\seguisb.ttf"):
            with dpg.font(r"C:\Windows\Fonts\seguisb.ttf", 20, tag="font_ui_section_title"):
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
            with dpg.font(r"C:\Windows\Fonts\seguisb.ttf", 18, tag="font_ui_topbar_title"):
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
            with dpg.font(r"C:\Windows\Fonts\seguisb.ttf", 16, tag="font_ui_value"):
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
            with dpg.font(r"C:\Windows\Fonts\seguisb.ttf", 13, tag="font_ui_badge"):
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)
        if os.path.exists(r"C:\Windows\Fonts\consola.ttf"):
            with dpg.font(r"C:\Windows\Fonts\consola.ttf", 13, tag="font_log_text"):
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Default)

    if dpg.does_item_exist("font_ui_base"):
        dpg.bind_font("font_ui_base")

    def bind_font_if_exists(item, font_tag):
        if dpg.does_item_exist(item) and dpg.does_item_exist(font_tag):
            dpg.bind_item_font(item, font_tag)

    # ──────────────── themes ────────────────
    with dpg.theme() as theme_global:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, COL_BG)
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_BG)
            dpg.add_theme_color(dpg.mvThemeCol_PopupBg, COL_CARD)
            dpg.add_theme_color(dpg.mvThemeCol_Text, COL_TEXT)
            dpg.add_theme_color(dpg.mvThemeCol_TextDisabled, COL_MUTED)
            dpg.add_theme_color(dpg.mvThemeCol_Border, COL_BORDER)
            dpg.add_theme_color(dpg.mvThemeCol_Separator, COL_BORDER)
            dpg.add_theme_color(dpg.mvThemeCol_SeparatorHovered, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_SeparatorActive, COL_ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, COL_CARD_2)
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (199, 182, 161))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (185, 165, 143))
            dpg.add_theme_color(dpg.mvThemeCol_TitleBg, COL_TOPBAR)
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive, COL_TOPBAR)
            dpg.add_theme_color(dpg.mvThemeCol_TitleBgCollapsed, COL_PANEL_DARK)
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarBg, COL_PANEL_DARK)
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrab, (116, 74, 72))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabHovered, (142, 92, 88))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabActive, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_Header, (183, 160, 139))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, (197, 173, 149))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderActive, (168, 136, 116))
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, CARD_RADIUS)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 7)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, CARD_RADIUS)
            dpg.add_theme_style(dpg.mvStyleVar_PopupRounding, 10)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 10, 6)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 0, 0)
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8, ITEM_SPACING_Y)
            dpg.add_theme_style(dpg.mvStyleVar_CellPadding, 4, 2)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding, 8)

    with dpg.theme() as theme_topbar:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_TOPBAR)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, PAD, PAD)

    with dpg.theme() as theme_bottombar:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg,  (25, 16, 17))
            dpg.add_theme_color(dpg.mvThemeCol_Border,   COL_ACCENT)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding,   8)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding,  16, 10)
            dpg.add_theme_style(dpg.mvStyleVar_ChildBorderSize,  2)

    with dpg.theme() as theme_window_shell:
        with dpg.theme_component(dpg.mvWindowAppItem):
            dpg.add_theme_style(dpg.mvStyleVar_WindowBorderSize, 0)
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 0)

    with dpg.theme() as theme_card:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_CARD)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, PAD, PAD)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, CARD_RADIUS)

    with dpg.theme() as theme_card_inner:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_CARD_2)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 10, 10)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, CARD_INNER_RADIUS)

    with dpg.theme() as theme_stewart_stage:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_STEWART_BG)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 6, 6)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, CARD_INNER_RADIUS)

    with dpg.theme() as theme_log_box:
        with dpg.theme_component(dpg.mvChildWindow):
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, COL_LOG_BG)
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 14, 12)
            dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, CARD_INNER_RADIUS)

    with dpg.theme() as theme_btn_primary:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, COL_ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_Text, COL_TEXT_INV)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 8)

    with dpg.theme() as theme_btn_sidebar:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, COL_PANEL_DARK)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, COL_PANEL_DARK_2)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, COL_ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_Text, COL_TEXT_INV)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 8)

    # STOP — danger, brightest/strongest
    with dpg.theme() as theme_btn_stop:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (185, 32, 50))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,  (210, 48, 68))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,   (150, 22, 38))
            dpg.add_theme_color(dpg.mvThemeCol_Text,           COL_TEXT_INV)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding,  8)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding,   20, 6)

    # CENTER / CALIBRATE — secondary, muted filled
    with dpg.theme() as theme_btn_secondary:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (52, 34, 36))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,  (70, 47, 49))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,   COL_ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_Text,           COL_TEXT_INV_SOFT)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding,  8)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding,   14, 8)

    # SETTINGS — same grey fill as secondary buttons
    with dpg.theme() as theme_btn_ghost:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button,        (52, 34, 36))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,  (70, 47, 49))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive,   COL_ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_Text,           COL_TEXT_INV_SOFT)
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding,  8)
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding,   14, 8)

    def _badge_theme(fill, text_color):
        with dpg.theme() as t:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, fill)
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, fill)
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, fill)
                dpg.add_theme_color(dpg.mvThemeCol_Text, text_color)
                dpg.add_theme_color(dpg.mvThemeCol_Border, fill)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 14)
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 12, 6)
        return t

    theme_badge_running = _badge_theme(COL_ACCENT_HOVER, COL_TEXT_INV)
    theme_badge_fans = _badge_theme(COL_GOOD, COL_TEXT)
    theme_badge_ready = _badge_theme((101, 58, 61), COL_TEXT_INV_SOFT)

    with dpg.theme() as theme_tabs:
        with dpg.theme_component(dpg.mvTab):
            dpg.add_theme_color(dpg.mvThemeCol_Text, COL_TEXT_INV)
            dpg.add_theme_color(dpg.mvThemeCol_Tab, COL_PANEL_DARK)
            dpg.add_theme_color(dpg.mvThemeCol_TabHovered, COL_ACCENT_HOVER)
            dpg.add_theme_color(dpg.mvThemeCol_TabActive, COL_ACCENT)
            dpg.add_theme_color(dpg.mvThemeCol_TabUnfocused, COL_PANEL_DARK_2)
            dpg.add_theme_color(dpg.mvThemeCol_TabUnfocusedActive, (94, 47, 54))
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 12, 7)
            dpg.add_theme_style(dpg.mvStyleVar_TabRounding, 8)

    with dpg.theme() as theme_plot:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvPlotCol_FrameBg, COL_PLOT_BG,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_PlotBg, COL_PLOT_BG,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_PlotBorder, COL_PLOT_BORDER,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_LegendBg, COL_CARD,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_color(dpg.mvPlotCol_LegendBorder, COL_PLOT_BORDER,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_PlotPadding, 6, 4,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LabelPadding, 8, 6,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LegendPadding, 10, 8,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LegendInnerPadding, 7, 6,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_LegendSpacing, 8, 4,
                                category=dpg.mvThemeCat_Plots)
            dpg.add_theme_style(dpg.mvPlotStyleVar_FitPadding, 0.02, 0.06,
                                category=dpg.mvThemeCat_Plots)

    def _bar_theme(fill, bg=(74, 47, 45, 70)):
        with dpg.theme() as t:
            with dpg.theme_component(dpg.mvProgressBar):
                dpg.add_theme_color(dpg.mvThemeCol_PlotHistogram, fill)
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, bg)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)
        return t

    def _line_series_theme(color, weight=2.0):
        with dpg.theme() as t:
            with dpg.theme_component(dpg.mvLineSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, color,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_color(dpg.mvPlotCol_Fill, color,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_LineWeight, weight,
                                    category=dpg.mvThemeCat_Plots)
        return t

    def _bar_series_theme(fill):
        with dpg.theme() as t:
            with dpg.theme_component(dpg.mvBarSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Fill, fill,
                                    category=dpg.mvThemeCat_Plots)
                dpg.add_theme_color(dpg.mvPlotCol_Line, COL_ACCENT,
                                    category=dpg.mvThemeCat_Plots)
        return t

    theme_bar_accent = _bar_theme(COL_ACCENT)
    theme_bar_good   = _bar_theme(COL_GOOD)
    theme_bar_warn   = _bar_theme(COL_WARN)
    theme_bar_bad    = _bar_theme(COL_BAD)
    pose_series_themes = tuple(_line_series_theme(color, weight=2.2)
                               for color in POSE_SERIES_COLORS)
    leg_series_themes = tuple(_line_series_theme(color, weight=1.9)
                              for color in LEG_SERIES_COLORS)
    theme_snapshot_bar = _bar_series_theme(COL_ACCENT_HOVER)

    dpg.bind_theme(theme_global)

    # ──────────────── layout helpers ────────────────
    def viewport_size():
        return dpg.get_viewport_client_width(), dpg.get_viewport_client_height()

    def compute_top_row_h():
        _, h = viewport_size()
        usable_content_h = max(
            240,
            h - TOPBAR_H - BOTTOMBAR_H - BOTTOM_ROW_CLEARANCE - LAYOUT_FIT_PAD,
        )
        return max(220, min(360, int(usable_content_h * TOP_ROW_RATIO)))

    def sync_log_box_height(fallback_card_h: int | None = None):
        if not dpg.does_item_exist("log_box"):
            return
        card_h = dpg.get_item_height("card_log") or (fallback_card_h or 0)
        header_h = dpg.get_item_height("log_header") or 26
        if card_h <= 0:
            return
        log_h = max(120, int(card_h - header_h - LOG_CARD_OVERHEAD))
        dpg.configure_item("log_box", height=log_h)

    def apply_layout():
        w, h = viewport_size()
        content_h = max(240, h - TOPBAR_H - BOTTOMBAR_H - BOTTOM_ROW_CLEARANCE)
        content_inner_h = max(220, content_h - LAYOUT_FIT_PAD)
        aw = w - LEFT_MARGIN - RIGHT_MARGIN
        dpg.configure_item("bg",        width=w, height=h)
        dpg.configure_item("root",      width=w, height=h)
        dpg.configure_item("topbar",    pos=(LEFT_MARGIN, 0), width=aw, height=TOPBAR_H)
        dpg.configure_item("main_area", pos=(LEFT_MARGIN, TOPBAR_H),
                           width=aw, height=content_h)
        dpg.configure_item("content", width=max(320, aw), height=content_h)
        dpg.configure_item("bottom_controls",
                           pos=(LEFT_MARGIN, h - BOTTOMBAR_H),
                           width=aw, height=BOTTOMBAR_H)
        top_h = compute_top_row_h()
        bottom_h = max(220, content_inner_h - top_h - SECTION_GAP)
        bottom_card_h = max(180, bottom_h - BOTTOM_CARD_FOOT)
        right_gap = SECTION_GAP
        right_available_h = max(120, bottom_card_h - right_gap)
        right_top_h = right_available_h // 2
        right_bottom_h = right_available_h - right_top_h
        for tag in ("card_pose", "card_legs", "card_visual"):
            if dpg.does_item_exist(tag):
                dpg.configure_item(tag, height=top_h)
        if dpg.does_item_exist("card_log"):
            dpg.configure_item("card_log", height=bottom_card_h)
            sync_log_box_height(bottom_card_h)
        if dpg.does_item_exist("card_telemetry"):
            dpg.configure_item("card_telemetry", height=right_top_h)
        if dpg.does_item_exist("card_telemetry"):
            ph = max(100, right_top_h - RIGHT_CARD_OVERHEAD)
            for tag in ("plot_pose", "plot_legs", "plot_snapshot"):
                if dpg.does_item_exist(tag):
                    dpg.configure_item(tag, height=ph)
        if dpg.does_item_exist("card_deep"):
            dpg.configure_item("card_deep", height=right_bottom_h)
            ph2 = max(100, right_bottom_h - RIGHT_CARD_OVERHEAD)
            for tag in ("plot_vel", "plot_accel", "plot_sat"):
                if dpg.does_item_exist(tag):
                    dpg.configure_item(tag, height=ph2)
        if dpg.does_item_exist("stewart_stage"):
            visual_h = max(180, top_h - (PAD * 2))
            dpg.configure_item("stewart_stage", height=visual_h)
            stage_w = dpg.get_item_width("stewart_stage") or 0
            card_w = dpg.get_item_width("card_visual") or 0
            usable_w = max(
                stage_w,
                (card_w - (PAD * 2)) if card_w > 0 else 0,
                aw * 0.26,
                320,
            )
            if dpg.does_item_exist("stewart_drawlist"):
                dpg.configure_item("stewart_drawlist",
                                   width=int(max(220, usable_w)),
                                   height=visual_h)

    dpg.set_viewport_resize_callback(lambda s, a: apply_layout())

    # ──────────────── UI build ────────────────
    with dpg.window(tag="bg", no_title_bar=True, no_resize=True,
                    no_move=True, no_scrollbar=True,
                    no_scroll_with_mouse=True, pos=(0, 0)):
        pass
    dpg.bind_item_theme("bg", theme_window_shell)

    with dpg.window(tag="root", no_title_bar=True, no_resize=True,
                    no_move=True, no_scrollbar=True,
                    no_scroll_with_mouse=True, pos=(0, 0)):
        dpg.bind_item_theme("root", theme_window_shell)

        with dpg.child_window(tag="topbar", border=False,
                              no_scrollbar=True, no_scroll_with_mouse=True):
            dpg.bind_item_theme("topbar", theme_topbar)
            with dpg.table(header_row=False, resizable=False,
                           policy=dpg.mvTable_SizingStretchProp,
                           borders_innerV=False, borders_outerV=False,
                           borders_innerH=False, borders_outerH=False):
                dpg.add_table_column(init_width_or_weight=9)
                dpg.add_table_column(init_width_or_weight=1.8)
                dpg.add_table_column(init_width_or_weight=1.5)
                dpg.add_table_column(init_width_or_weight=1.4)
                with dpg.table_row():
                    with dpg.table_cell():
                        with dpg.group(indent=PAD):
                            dpg.add_spacer(height=10)
                            title = dpg.add_text("Stewart Platform", tag="topbar_title",
                                                 color=COL_TEXT_INV)
                            bind_font_if_exists(title, "font_ui_topbar_title")
                    for label, tag, theme in (
                        ("RUNNING", "badge_running", theme_badge_running),
                        ("FANS ON", "badge_fans", theme_badge_fans),
                        ("READY", "badge_ready", theme_badge_ready),
                    ):
                        with dpg.table_cell():
                            dpg.add_spacer(height=7)
                            badge = dpg.add_button(label=label, tag=tag, width=-1, height=30)
                            dpg.bind_item_theme(badge, theme)
                            bind_font_if_exists(badge, "font_ui_badge")

        with dpg.child_window(tag="main_area", border=False,
                              no_scrollbar=True, no_scroll_with_mouse=True):
            with dpg.child_window(tag="content", border=False,
                                  no_scrollbar=True, no_scroll_with_mouse=True):
                TOP_ROW_H = compute_top_row_h()
                with dpg.table(header_row=False, resizable=False,
                               policy=dpg.mvTable_SizingStretchProp,
                               borders_innerV=False, borders_outerV=False,
                               borders_innerH=False, borders_outerH=False):
                    dpg.add_table_column(init_width_or_weight=BOTTOM_LEFT_WEIGHT)
                    dpg.add_table_column(init_width_or_weight=BOTTOM_RIGHT_WEIGHT)
                    dpg.add_table_column(init_width_or_weight=2)

                    with dpg.table_row():
                        # Pose card
                        with dpg.table_cell():
                            with dpg.child_window(tag="card_pose",
                                                  border=False, height=TOP_ROW_H,
                                                  no_scrollbar=True,
                                                  no_scroll_with_mouse=True):
                                dpg.bind_item_theme("card_pose", theme_card)
                                dpg.add_spacer(height=PAD)
                                with dpg.group(indent=PAD):
                                    title = dpg.add_text("Platform Pose", tag="title_pose",
                                                         color=COL_TEXT)
                                    bind_font_if_exists(title, "font_ui_section_title")
                                    dpg.add_spacer(height=6)
                                    with dpg.table(header_row=False, resizable=False,
                                                   policy=dpg.mvTable_SizingStretchProp,
                                                   borders_innerV=False,
                                                   borders_outerV=False,
                                                   borders_innerH=False,
                                                   borders_outerH=False):
                                        dpg.add_table_column(init_width_or_weight=78,
                                                             width_fixed=True)
                                        dpg.add_table_column(init_width_or_weight=1.0)
                                        dpg.add_table_column(init_width_or_weight=86,
                                                             width_fixed=True)
                                        for ax_tag, ax_label in [
                                            ("pose_roll",  "Roll"),
                                            ("pose_pitch", "Pitch"),
                                            ("pose_yaw",   "Yaw"),
                                            ("pose_x",     "X"),
                                            ("pose_y",     "Y"),
                                            ("pose_z",     "Z"),
                                        ]:
                                            with dpg.table_row():
                                                with dpg.table_cell():
                                                    axis_label = dpg.add_text(ax_label,
                                                                              color=COL_MUTED)
                                                    bind_font_if_exists(axis_label,
                                                                        "font_ui_label")
                                                with dpg.table_cell():
                                                    bar = dpg.add_progress_bar(
                                                        tag=ax_tag, default_value=0.5,
                                                        overlay="0.0", width=-1)
                                                    dpg.bind_item_theme(bar, theme_bar_accent)
                                                with dpg.table_cell():
                                                    axis_value = dpg.add_text(
                                                        "", tag=f"{ax_tag}_val",
                                                        color=COL_TEXT)
                                                    bind_font_if_exists(axis_value,
                                                                        "font_ui_value")

                        # Actuators card
                        with dpg.table_cell():
                            with dpg.child_window(tag="card_legs",
                                                  border=False, height=TOP_ROW_H,
                                                  no_scrollbar=True,
                                                  no_scroll_with_mouse=True):
                                dpg.bind_item_theme("card_legs", theme_card)
                                dpg.add_spacer(height=PAD)
                                with dpg.group(indent=PAD):
                                    title = dpg.add_text("Actuators", tag="title_legs",
                                                         color=COL_TEXT)
                                    bind_font_if_exists(title, "font_ui_section_title")
                                    dpg.add_spacer(height=8)
                                    with dpg.table(header_row=False, resizable=False,
                                                   policy=dpg.mvTable_SizingStretchProp,
                                                   borders_innerV=False,
                                                   borders_outerV=False,
                                                   borders_innerH=False,
                                                   borders_outerH=False):
                                        dpg.add_table_column(init_width_or_weight=76,
                                                             width_fixed=True)
                                        dpg.add_table_column(init_width_or_weight=1.0)
                                        dpg.add_table_column(init_width_or_weight=72,
                                                             width_fixed=True)
                                        for lbl, cpp_i in zip(_LEG_LABELS, _LEG_DISPLAY):
                                            with dpg.table_row():
                                                with dpg.table_cell():
                                                    leg_label = dpg.add_text(f"Leg {lbl}",
                                                                             color=COL_MUTED)
                                                    bind_font_if_exists(leg_label,
                                                                        "font_ui_label")
                                                with dpg.table_cell():
                                                    bar = dpg.add_progress_bar(
                                                        tag=f"leg_{cpp_i}",
                                                        default_value=0.0,
                                                        overlay="--",
                                                        width=-1)
                                                    dpg.bind_item_theme(bar, theme_bar_good)
                                                with dpg.table_cell():
                                                    leg_value = dpg.add_text(
                                                        "", tag=f"leg_{cpp_i}_val",
                                                        color=COL_TEXT)
                                                    bind_font_if_exists(leg_value,
                                                                        "font_ui_value")

                        # Stewart preview card
                        with dpg.table_cell():
                            with dpg.child_window(tag="card_visual",
                                                  border=False, height=TOP_ROW_H,
                                                  no_scrollbar=True,
                                                  no_scroll_with_mouse=True):
                                dpg.bind_item_theme("card_visual", theme_card)
                                visual_h = max(180, TOP_ROW_H - (PAD * 2))
                                with dpg.child_window(tag="stewart_stage",
                                                       border=False,
                                                       height=visual_h,
                                                       width=-1,
                                                       no_scrollbar=True,
                                                       no_scroll_with_mouse=True):
                                    dpg.bind_item_theme("stewart_stage", theme_stewart_stage)
                                    with dpg.drawlist(tag="stewart_drawlist",
                                                      width=320, height=visual_h):
                                        pass

                dpg.add_spacer(height=SECTION_GAP)

                with dpg.table(header_row=False, resizable=False,
                               policy=dpg.mvTable_SizingStretchProp,
                               borders_innerV=False, borders_outerV=False,
                               borders_innerH=False, borders_outerH=False):
                    dpg.add_table_column(init_width_or_weight=1)
                    dpg.add_table_column(init_width_or_weight=1)

                    with dpg.table_row():
                        with dpg.table_cell():
                            with dpg.child_window(tag="card_log", border=False,
                                                  height=-1, width=-1,
                                                  no_scrollbar=True,
                                                  no_scroll_with_mouse=True):
                                dpg.bind_item_theme("card_log", theme_card)
                                dpg.add_spacer(height=PAD)
                                with dpg.group(indent=PAD):
                                    with dpg.table(tag="log_header",
                                                   header_row=False, resizable=False,
                                                   policy=dpg.mvTable_SizingStretchProp,
                                                   borders_innerV=False,
                                                   borders_outerV=False,
                                                   borders_innerH=False,
                                                   borders_outerH=False):
                                        dpg.add_table_column(init_width_or_weight=3.0)
                                        dpg.add_table_column(init_width_or_weight=2.0)
                                        with dpg.table_row():
                                            with dpg.table_cell():
                                                log_title = dpg.add_text(
                                                    "Arduino Log", tag="title_log",
                                                    color=COL_TEXT)
                                                bind_font_if_exists(log_title,
                                                                    "font_ui_section_title")
                                            with dpg.table_cell():
                                                running = dpg.add_text(
                                                    tag="running_label",
                                                    default_value="",
                                                    color=COL_GOOD)
                                                bind_font_if_exists(running,
                                                                    "font_ui_value")
                                    dpg.add_spacer(height=10)
                                    with dpg.child_window(tag="log_box",
                                                          border=False,
                                                          height=-1, width=-1):
                                        dpg.bind_item_theme("log_box", theme_log_box)
                                        log_text = dpg.add_text(tag="arduino_log_text",
                                                                default_value="", color=COL_MUTED)
                                        bind_font_if_exists(log_text, "font_log_text")

                        with dpg.table_cell():
                            with dpg.group():
                                with dpg.child_window(tag="card_telemetry",
                                                      border=False, height=220, width=-1,
                                                      no_scrollbar=True,
                                                      no_scroll_with_mouse=True):
                                    dpg.bind_item_theme("card_telemetry", theme_card)
                                    with dpg.group():
                                        dpg.add_spacer(height=CARD_TOP_INSET)
                                        plot_h = 160
                                        with dpg.tab_bar(tag="telemetry_tabs"):
                                            with dpg.tab(label="Pose"):
                                                with dpg.plot(tag="plot_pose",
                                                              height=plot_h,
                                                              width=-1):
                                                    dpg.add_plot_legend(
                                                        location=dpg.mvPlot_Location_NorthWest,
                                                        no_buttons=True,
                                                        no_highlight_item=True,
                                                        no_highlight_axis=True)
                                                    dpg.add_plot_axis(dpg.mvXAxis, tag="plot_pose_x", label="time (s)")
                                                    dpg.add_plot_axis(dpg.mvYAxis, tag="plot_pose_y", label="in / norm")
                                                    for lbl, stag in [
                                                        ("X (in)",  "series_x"),
                                                        ("Y (in)",  "series_y"),
                                                        ("Z (in)",  "series_z"),
                                                    ]:
                                                        dpg.add_line_series([], [], label=lbl,
                                                                            parent="plot_pose_y", tag=stag)
                                            with dpg.tab(label="Legs"):
                                                with dpg.plot(tag="plot_legs",
                                                              height=plot_h,
                                                              width=-1):
                                                    dpg.add_plot_legend(
                                                        location=dpg.mvPlot_Location_NorthWest,
                                                        no_buttons=True,
                                                        no_highlight_item=True,
                                                        no_highlight_axis=True)
                                                    dpg.add_plot_axis(dpg.mvXAxis, tag="plot_legs_x", label="time (s)")
                                                    dpg.add_plot_axis(dpg.mvYAxis, tag="plot_legs_y", label="inches")
                                                    for lbl, cpp_i in zip(_LEG_LABELS, _LEG_DISPLAY):
                                                        dpg.add_line_series([], [], label=f"Leg {lbl}",
                                                                            parent="plot_legs_y", tag=f"series_leg_{cpp_i}")
                                            with dpg.tab(label="Snapshot"):
                                                with dpg.plot(tag="plot_snapshot",
                                                              height=plot_h,
                                                              width=-1):
                                                    dpg.add_plot_axis(dpg.mvXAxis, tag="plot_snap_x", label="leg")
                                                    dpg.add_plot_axis(dpg.mvYAxis, tag="plot_snap_y", label="inches")
                                                    dpg.add_bar_series([1,2,3,4,5,6], [0]*6,
                                                                       weight=0.8,
                                                                       parent="plot_snap_y",
                                                                       tag="bars_legs")

                                dpg.add_spacer(height=SECTION_GAP)

                                with dpg.child_window(tag="card_deep", border=False, height=220, width=-1,
                                                      no_scrollbar=True,
                                                      no_scroll_with_mouse=True):
                                    dpg.bind_item_theme("card_deep", theme_card)
                                    with dpg.group():
                                        dpg.add_spacer(height=CARD_TOP_INSET)
                                        plot_h = 160
                                        with dpg.tab_bar(tag="deep_tabs"):
                                            with dpg.tab(label="Velocity"):
                                                with dpg.plot(tag="plot_vel", height=plot_h,
                                                              width=-1):
                                                    dpg.add_plot_legend(
                                                        location=dpg.mvPlot_Location_NorthWest,
                                                        no_buttons=True,
                                                        no_highlight_item=True,
                                                        no_highlight_axis=True)
                                                    dpg.add_plot_axis(dpg.mvXAxis, tag="plot_vel_x", label="time (s)")
                                                    dpg.add_plot_axis(dpg.mvYAxis, tag="plot_vel_y", label="in/s")
                                                    for i in range(NUM_LEGS):
                                                        dpg.add_line_series([], [], label=f"Leg {i+1}",
                                                                            parent="plot_vel_y", tag=f"series_vel_{i}")
                                            with dpg.tab(label="Acceleration"):
                                                with dpg.plot(tag="plot_accel", height=plot_h,
                                                              width=-1):
                                                    dpg.add_plot_legend(
                                                        location=dpg.mvPlot_Location_NorthWest,
                                                        no_buttons=True,
                                                        no_highlight_item=True,
                                                        no_highlight_axis=True)
                                                    dpg.add_plot_axis(dpg.mvXAxis, tag="plot_accel_x", label="time (s)")
                                                    dpg.add_plot_axis(dpg.mvYAxis, tag="plot_accel_y", label="in/s^2")
                                                    for i in range(NUM_LEGS):
                                                        dpg.add_line_series([], [], label=f"Leg {i+1}",
                                                                            parent="plot_accel_y", tag=f"series_acc_{i}")
                                            with dpg.tab(label="Saturation %"):
                                                with dpg.plot(tag="plot_sat", height=plot_h,
                                                              width=-1):
                                                    dpg.add_plot_legend(
                                                        location=dpg.mvPlot_Location_NorthWest,
                                                        no_buttons=True,
                                                        no_highlight_item=True,
                                                        no_highlight_axis=True)
                                                    dpg.add_plot_axis(dpg.mvXAxis, tag="plot_sat_x", label="time (s)")
                                                    dpg.add_plot_axis(dpg.mvYAxis, tag="plot_sat_y", label="fraction (0-1)")
                                                    for i in range(NUM_LEGS):
                                                        dpg.add_line_series([], [], label=f"Leg {i+1}",
                                                                            parent="plot_sat_y", tag=f"series_sat_{i}")

        with dpg.child_window(tag="bottom_controls", border=True):
            dpg.bind_item_theme("bottom_controls", theme_bottombar)
            with dpg.group(tag="buttons_container", horizontal=True):
                pass

    apply_layout()

    for tag in ("telemetry_tabs", "deep_tabs"):
        if dpg.does_item_exist(tag):
            dpg.bind_item_theme(tag, theme_tabs)

    for tag in ("plot_pose", "plot_legs", "plot_snapshot",
                "plot_vel", "plot_accel", "plot_sat"):
        if dpg.does_item_exist(tag):
            dpg.bind_item_theme(tag, theme_plot)

    for tag, theme in zip(("series_x", "series_y", "series_z"), pose_series_themes):
        if dpg.does_item_exist(tag):
            dpg.bind_item_theme(tag, theme)

    for prefix in ("series_leg_", "series_vel_", "series_acc_", "series_sat_"):
        for idx, theme in enumerate(leg_series_themes):
            tag = f"{prefix}{idx}"
            if dpg.does_item_exist(tag):
                dpg.bind_item_theme(tag, theme)

    if dpg.does_item_exist("bars_legs"):
        dpg.bind_item_theme("bars_legs", theme_snapshot_bar)

    # ──────────────── menu / button logic ────────────────
    def on_button(label):
        nonlocal menu, active_preset, preset_start_t
        if label == "START" and menu == MAIN:
            menu = START_MENU;  rebuild_buttons();  return
        if label == "SETTINGS":
            arduino_log.append("[SIM] Settings panel placeholder — not implemented")
            if len(arduino_log) > MAX_LOG_LINES:
                del arduino_log[:-MAX_LOG_LINES]
            return

        if menu == MAIN:
            if label == "CENTER":
                active_preset = None;  reset_telemetry_buffers();  send_command("center")
            elif label == "STOP":
                active_preset = None;  reset_telemetry_buffers();  send_command("stop")

        elif menu == START_MENU:
            if label == "CONTROLLER":
                send_command("controller");  menu = MAIN;  rebuild_buttons()
            elif label == "PRESETS":
                menu = PRESETS;  rebuild_buttons()
            elif label == "BACK":
                menu = MAIN;  rebuild_buttons()

        elif menu == PRESETS:
            preset_map = {"DEMO": "demo", "FIGURE 8": "figure8",
                          "ORBIT": "orbit", "WAVE": "wave"}
            if label in preset_map:
                active_preset  = preset_map[label]
                preset_start_t = time.monotonic()
                reset_telemetry_buffers()
                menu = RUNNING
                dpg.configure_item("running_label",
                                   default_value=f"RUNNING: {active_preset.upper()}")
                rebuild_buttons()
                arduino_log.append(f"[SIM] Starting preset: {active_preset}")
            elif label == "BACK":
                menu = START_MENU;  rebuild_buttons()

        elif menu == RUNNING:
            if label == "RESTART":
                preset_start_t = time.monotonic();  reset_telemetry_buffers();  send_command("center")
            elif label == "STOP":
                active_preset = None
                reset_telemetry_buffers()
                send_command("stop");  menu = MAIN;  rebuild_buttons()

    def on_button_callback(sender, app_data, user_data):
        on_button(user_data)

    _btn_theme_map = {
        "STOP":      theme_btn_stop,
        "START":     theme_btn_primary,
        "CENTER":    theme_btn_secondary,
        "CALIBRATE": theme_btn_secondary,
        "SETTINGS":  theme_btn_ghost,
    }

    def rebuild_buttons():
        if dpg.does_item_exist("buttons_container"):
            dpg.delete_item("buttons_container")
        layout = {
            # Left group: START / CENTER / CALIBRATE  —  big gap  —  STOP  —  gap  — SETTINGS
            MAIN: [
                ("spacer", 0.06, None),
                ("button", 1.00, "START"),
                ("spacer", 0.04, None),
                ("button", 0.88, "CENTER"),
                ("spacer", 0.04, None),
                ("button", 1.00, "CALIBRATE"),
                ("spacer", 2.10, None),
                ("button", 1.30, "STOP"),
                ("spacer", 1.10, None),
                ("button", 0.82, "SETTINGS"),
                ("spacer", 0.06, None),
            ],
            START_MENU: [
                ("spacer", 0.55, None),
                ("button", 1.20, "CONTROLLER"),
                ("button", 1.00, "PRESETS"),
                ("button", 0.90, "BACK"),
                ("spacer", 0.22, None),
                ("button", 1.00, "SETTINGS"),
                ("spacer", 0.55, None),
            ],
            PRESETS: [
                ("spacer", 0.18, None),
                ("button", 0.95, "DEMO"),
                ("button", 1.05, "FIGURE 8"),
                ("button", 0.90, "ORBIT"),
                ("button", 0.90, "WAVE"),
                ("button", 0.90, "BACK"),
                ("spacer", 0.28, None),
                ("button", 1.00, "SETTINGS"),
                ("spacer", 0.18, None),
            ],
            RUNNING: [
                ("spacer", 0.65, None),
                ("button", 1.05, "RESTART"),
                ("spacer", 0.28, None),
                ("button", 1.30, "STOP"),
                ("spacer", 0.22, None),
                ("button", 1.00, "SETTINGS"),
                ("spacer", 0.65, None),
            ],
        }.get(menu, [])
        with dpg.table(tag="buttons_container", parent="bottom_controls",
                       header_row=False, resizable=False,
                       policy=dpg.mvTable_SizingStretchProp,
                       borders_innerV=False, borders_outerV=False,
                       borders_innerH=False, borders_outerH=False):
            for kind, weight, _ in layout:
                dpg.add_table_column(init_width_or_weight=weight)
            with dpg.table_row():
                for kind, _, lab in layout:
                    with dpg.table_cell():
                        if kind == "button":
                            btn = dpg.add_button(label=lab, height=48, width=-1,
                                                 callback=on_button_callback,
                                                 user_data=lab)
                            dpg.bind_item_theme(btn, _btn_theme_map.get(lab, theme_btn_sidebar))
                            bind_font_if_exists(btn, "font_ui_label")

    rebuild_buttons()

    # ──────────────── telemetry buffers ────────────────
    # Sampled at 30 Hz for smooth motion plots.
    # When a preset is active, plots use a rolling 10-second window driven by
    # continuous elapsed time instead of resetting at segment boundaries.
    MAX_SAMPLES  = 1800        # 60 seconds at 30 Hz
    TELEM_HZ     = 1 / 30      # 30 Hz
    last_telem_t = -TELEM_HZ   # force first sample immediately
    t_hist    = []
    x_hist    = [];  y_hist  = [];  z_hist  = []
    legs_hist = [[] for _ in range(NUM_LEGS)]
    vel_hist  = [[] for _ in range(NUM_LEGS)]
    acc_hist  = [[] for _ in range(NUM_LEGS)]
    sat_hist  = [[] for _ in range(NUM_LEGS)]
    last_log_t = 0.0

    def reset_telemetry_buffers():
        nonlocal last_telem_t
        t_hist.clear()
        x_hist.clear();  y_hist.clear();  z_hist.clear()
        for histories in (legs_hist, vel_hist, acc_hist, sat_hist):
            for hist in histories:
                hist.clear()
        last_telem_t = -TELEM_HZ

    def rot_x(angle_deg):
        a = math.radians(angle_deg)
        c, s = math.cos(a), math.sin(a)
        return [[1, 0, 0], [0, c, -s], [0, s, c]]

    def rot_z(angle_deg):
        a = math.radians(angle_deg)
        c, s = math.cos(a), math.sin(a)
        return [[c, -s, 0], [s, c, 0], [0, 0, 1]]

    def mat_mul_vec(m, v):
        return [
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
        ]

    def project_stewart_points(points, w, h):
        rz = rot_z(STEWART_VIEW_YAW)
        rx = rot_x(STEWART_VIEW_PITCH)
        z_center = _PLAT_0[2] * 0.52
        transformed = []
        for px, py, pz in points:
            local = [px, py, pz - z_center]
            v1 = mat_mul_vec(rz, local)
            v2 = mat_mul_vec(rx, v1)
            transformed.append(v2)

        xs = [p[0] for p in transformed]
        ys = [p[1] for p in transformed]
        span_x = max(1.0, max(xs) - min(xs))
        span_y = max(1.0, max(ys) - min(ys))
        pad = 18.0
        usable_w = max(80.0, w - pad * 2.0)
        usable_h = max(80.0, h - pad * 2.0)
        scale = min(usable_w / span_x, usable_h / span_y)
        cx = 0.5 * (max(xs) + min(xs))
        cy = 0.5 * (max(ys) + min(ys))
        return [
            (w * 0.5 + (px - cx) * scale, h * 0.53 - (py - cy) * scale)
            for px, py, _ in transformed
        ]

    def draw_stewart_view(tx, ty, tz, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        if not dpg.does_item_exist("stewart_drawlist"):
            return

        card_w = dpg.get_item_width("card_visual") or 0
        stage_w = dpg.get_item_width("stewart_stage") or 0
        stage_h = dpg.get_item_height("stewart_stage") or 0
        w = max(220,
                stage_w,
                (card_w - (PAD * 2)) if card_w > 0 else 0,
                dpg.get_item_width("stewart_drawlist") or 0,
                320)
        h = max(180, stage_h or dpg.get_item_height("stewart_drawlist") or 220)
        dpg.configure_item("stewart_drawlist", width=w, height=h)
        dpg.delete_item("stewart_drawlist", children_only=True)

        base_pts = [list(_BASES[i]) for i in range(NUM_LEGS)]
        top_pts = []
        for i in range(NUM_LEGS):
            rx, ry, rz = _rotate_point(*_PLATS[i], roll_deg, pitch_deg, yaw_deg)
            top_pts.append([_PLAT_0[0] + tx + rx,
                             _PLAT_0[1] + ty + ry,
                             _PLAT_0[2] + tz + rz])
        base_center = [0.0, 0.0, 0.0]
        top_center = [_PLAT_0[0] + tx, _PLAT_0[1] + ty, _PLAT_0[2] + tz]
        axis_len = 2.2
        ax_x = _rotate_point(axis_len, 0.0, 0.0, roll_deg, pitch_deg, yaw_deg)
        ax_y = _rotate_point(0.0, axis_len, 0.0, roll_deg, pitch_deg, yaw_deg)
        ax_z = _rotate_point(0.0, 0.0, axis_len, roll_deg, pitch_deg, yaw_deg)
        projected = project_stewart_points(
            base_pts + top_pts + [
                base_center,
                top_center,
                top_center,
                [top_center[0] + ax_x[0], top_center[1] + ax_x[1], top_center[2] + ax_x[2]],
                [top_center[0] + ax_y[0], top_center[1] + ax_y[1], top_center[2] + ax_y[2]],
                [top_center[0] + ax_z[0], top_center[1] + ax_z[1], top_center[2] + ax_z[2]],
            ],
            w, h
        )
        base_proj = projected[:NUM_LEGS]
        top_proj = projected[NUM_LEGS:NUM_LEGS * 2]
        base_center_proj = projected[NUM_LEGS * 2]
        top_center_proj = projected[NUM_LEGS * 2 + 1]
        axis_origin = projected[NUM_LEGS * 2 + 2]
        axis_points = projected[NUM_LEGS * 2 + 3:NUM_LEGS * 2 + 6]
        axis_cols = (COL_ACCENT_HOVER, COL_GOOD, COL_MUTED)
        for axis_end, axis_col in zip(axis_points, axis_cols):
            dpg.draw_line(axis_origin, axis_end,
                          color=axis_col, thickness=1.8, parent="stewart_drawlist")

        dpg.draw_line(base_center_proj, top_center_proj,
                      color=(150, 124, 114), thickness=0.9, parent="stewart_drawlist")

        for ring, pts, color, thickness in (
            (_BASE_RING_ORDER, base_proj, (112, 88, 82), 1.8),
            (_PLAT_RING_ORDER, top_proj, COL_ACCENT, 3.6),
        ):
            for idx, start in enumerate(ring):
                end = ring[(idx + 1) % len(ring)]
                dpg.draw_line(pts[start], pts[end],
                              color=color, thickness=thickness, parent="stewart_drawlist")

        leg_range = max(1e-6, LEG_MAX_IN - LEG_MIN_IN)
        for i in range(NUM_LEGS):
            frac = max(0.0, min(1.0, (leg_extensions[i] - LEG_MIN_IN) / leg_range))
            leg_col = (COL_BAD if frac >= LEG_CRIT else
                       COL_WARN if frac >= LEG_WARN else
                       COL_GOOD)
            dpg.draw_line(base_proj[i], top_proj[i],
                          color=leg_col, thickness=2.2, parent="stewart_drawlist")
            dpg.draw_circle(base_proj[i], 3.0, color=(92, 69, 66),
                            fill=COL_STEWART_BG, parent="stewart_drawlist")
            dpg.draw_circle(top_proj[i], 3.2, color=COL_TEXT,
                            fill=COL_ACCENT_HOVER, parent="stewart_drawlist")

        dpg.draw_circle(base_center_proj, 3.2, color=(92, 69, 66),
                        fill=COL_STEWART_BG, parent="stewart_drawlist")
        dpg.draw_circle(top_center_proj, 3.6, color=COL_TEXT,
                        fill=COL_ACCENT, parent="stewart_drawlist")

    # ──────────────── main loop ────────────────
    while dpg.is_dearpygui_running():
        now = time.monotonic()

        # ── Compute current platform position ──
        elapsed = 0.0
        preset_total = 0.0
        if active_preset:
            elapsed = now - preset_start_t
            preset_total = _PRESET_TOTAL[active_preset]
            T = _preset_position(active_preset, elapsed)
        else:
            T = list(T_CENTER)

        tx, ty, tz = T[0], T[1], T[2]
        roll  = T[3] if len(T) > 3 else 0.0
        pitch = T[4] if len(T) > 4 else 0.0
        yaw   = T[5] if len(T) > 5 else 0.0
        leg_extensions[:] = ik(tx, ty, tz, roll, pitch, yaw)
        draw_stewart_view(tx, ty, tz, roll, pitch, yaw)

        # ── Fake log every 2 s ──
        if now - last_log_t >= 2.0:
            last_log_t = now
            pos_str = ", ".join(f"{v:.3f}" for v in leg_extensions)
            arduino_log.append(f"[SIM] <POS_IN> {pos_str}")
            if active_preset:
                arduino_log.append(
                    f"[SIM] T=[{tx:.2f}, {ty:.2f}, {tz:.2f}] in")
            if len(arduino_log) > MAX_LOG_LINES:
                arduino_log = arduino_log[-MAX_LOG_LINES:]

        # ── Pose display ──
        # Bar fill: (val / rng + 1) / 2  maps  -rng..+rng  →  0..1
        # X/Y: ±LEG_MAX_IN range, centred at 0
        # Z:   centred at home height (T_CENTER[2]=2.0), ±5.5 in
        pose_vals = [
            ("pose_roll",  roll,     ROT_RANGE,   "{:+.1f}°"),
            ("pose_pitch", pitch,    ROT_RANGE,   "{:+.1f}°"),
            ("pose_yaw",   yaw,      ROT_RANGE,   "{:+.1f}°"),
            ("pose_x",     tx,       LEG_MAX_IN,  '{:+.2f}"'),
            ("pose_y",     ty,       LEG_MAX_IN,  '{:+.2f}"'),
            ("pose_z",     tz - 2.0, 5.5,         '{:+.2f}"'),
        ]
        for ax_tag, val, rng, fmt in pose_vals:
            frac = max(0.0, min(1.0, (val / rng + 1.0) / 2.0))
            display = fmt.format(val)
            dpg.configure_item(ax_tag, default_value=frac, overlay=display)
            dpg.configure_item(f"{ax_tag}_val", default_value=display)

        # ── Actuator bars ──
        leg_range = max(1e-6, LEG_MAX_IN - LEG_MIN_IN)
        for i in range(NUM_LEGS):
            ext  = leg_extensions[i]
            frac = max(0.0, min(1.0, (ext - LEG_MIN_IN) / leg_range))
            theme = (theme_bar_bad  if frac >= LEG_CRIT else
                     theme_bar_warn if frac >= LEG_WARN else theme_bar_good)
            lbl = f"{ext:.2f}\""
            dpg.configure_item(f"leg_{i}", default_value=frac, overlay=lbl)
            dpg.configure_item(f"leg_{i}_val", default_value=lbl)
            dpg.bind_item_theme(f"leg_{i}", theme)

        # Telemetry sampling for the rolling history plots.
        if now - last_telem_t >= TELEM_HZ:
            last_telem_t = now   # reset to current time — no runaway catch-up

            t_hist.append(elapsed if active_preset else now)
            x_hist.append(tx);  y_hist.append(ty);  z_hist.append(tz)
            for i in range(NUM_LEGS):
                legs_hist[i].append(leg_extensions[i])

            # Velocity and acceleration via central differences on the smoothstep
            # trajectory (analytical, no frame-timing noise).
            # EPS = 50 ms half-window.  We skip samples within EPS of a loop
            # boundary; at those points the preset returns to its start waypoint
            # so velocity and acceleration are physically zero anyway.
            EPS = 0.05
            if active_preset:
                el   = now - preset_start_t
                total = _PRESET_TOTAL[active_preset]
                t_in  = el % total          # position within current loop cycle
                near_boundary = t_in < EPS or t_in > total - EPS

                if near_boundary:
                    for i in range(NUM_LEGS):
                        vel_hist[i].append(0.0)
                        acc_hist[i].append(0.0)
                else:
                    T_m    = _preset_position(active_preset, el - EPS)
                    T_p    = _preset_position(active_preset, el + EPS)
                    legs_m = ik(*T_m)
                    legs_p = ik(*T_p)
                    for i in range(NUM_LEGS):
                        vel_hist[i].append(abs((legs_p[i] - legs_m[i]) / (2 * EPS)))
                        acc_hist[i].append(
                            (legs_p[i] - 2 * leg_extensions[i] + legs_m[i]) / (EPS * EPS))
            else:
                for i in range(NUM_LEGS):
                    vel_hist[i].append(0.0)
                    acc_hist[i].append(0.0)

            # Saturation fraction (0–1 of full stroke)
            for i in range(NUM_LEGS):
                sat_hist[i].append(max(0.0, min(1.0,
                    (leg_extensions[i] - LEG_MIN_IN) / leg_range)))

            # Trim to MAX_SAMPLES
            if len(t_hist) > MAX_SAMPLES:
                t_hist.pop(0);  x_hist.pop(0);  y_hist.pop(0);  z_hist.pop(0)
                for i in range(NUM_LEGS):
                    legs_hist[i].pop(0);  vel_hist[i].pop(0)
                    acc_hist[i].pop(0);   sat_hist[i].pop(0)

            # Push the latest history into the plots.
            if t_hist:
                if active_preset:
                    xs = list(t_hist)
                else:
                    t0 = t_hist[0]
                    xs = [t - t0 for t in t_hist]
                dpg.set_value("series_x", [xs, x_hist])
                dpg.set_value("series_y", [xs, y_hist])
                dpg.set_value("series_z", [xs, z_hist])
                for i in range(NUM_LEGS):
                    dpg.set_value(f"series_leg_{i}", [xs, legs_hist[i]])
                    dpg.set_value(f"series_vel_{i}", [xs, vel_hist[i]])
                    dpg.set_value(f"series_acc_{i}", [xs, acc_hist[i]])
                    dpg.set_value(f"series_sat_{i}", [xs, sat_hist[i]])

                # X axes: use a rolling 10-second window for presets; otherwise
                # keep the rolling 60-second operator view.
                if active_preset and preset_total > 0.0:
                    x_end = max(10.0, xs[-1])
                    x_start = max(0.0, x_end - 10.0)
                    ticks = _rolling_window_ticks(x_start, x_end, step=2.0)
                else:
                    x_end   = float(int(xs[-1]) + 1)
                    x_start = max(0.0, x_end - 60.0)
                    tick_first = int(math.ceil(x_start / 10.0)) * 10
                    ticks = tuple(
                        (str(t), float(t))
                        for t in range(tick_first, int(x_end) + 1, 10)
                    )
                    if not ticks:
                        ticks = ((str(int(x_start)), x_start),)
                for ax in ("plot_pose_x", "plot_legs_x",
                           "plot_vel_x", "plot_accel_x", "plot_sat_x"):
                    if dpg.does_item_exist(ax):
                        dpg.set_axis_limits(ax, x_start, x_end)
                        dpg.set_axis_ticks(ax, ticks)

                # Snapshot X axis: fixed to exactly the 6 actuators
                dpg.set_axis_limits("plot_snap_x", 0.5, 6.5)

                # Y axes: fixed inch ranges so all plots are consistent
                dpg.set_axis_limits("plot_pose_y", -7.0, 8.0)   # X/Y/Z translation (in)
                dpg.set_axis_limits("plot_legs_y",  0.0, 8.0)   # leg extensions (in)
                dpg.set_axis_limits("plot_snap_y",  0.0, 8.0)   # snapshot bars (in)
                dpg.set_axis_limits("plot_sat_y",   0.0, 1.0)   # saturation fraction

                # Velocity Y axis: always 0 at bottom (speed is non-negative),
                # top scales to 10% above current max so the range fits the data.
                flat = [v for vh in vel_hist for v in vh]
                v_max = max(flat) if flat else 0.0
                dpg.set_axis_limits("plot_vel_y", 0.0, max(0.1, v_max * 1.1))

        dpg.set_value("bars_legs", [[1,2,3,4,5,6], leg_extensions])

        if menu == RUNNING and active_preset:
            dpg.configure_item("running_label",
                               default_value=f"RUNNING: {active_preset.upper()}")
        else:
            dpg.configure_item("running_label", default_value="")
        sync_log_box_height()
        dpg.configure_item("arduino_log_text", default_value="\n".join(arduino_log))
        dpg.set_y_scroll("log_box", dpg.get_y_scroll_max("log_box"))

        dpg.render_dearpygui_frame()
        time.sleep(1 / 30)

    dpg.destroy_context()


if __name__ == "__main__":
    main()
