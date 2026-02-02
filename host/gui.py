# gui.py
import sys
import serial
import pygame
import time

from joystick import JoystickBackend

BAUD_RATE = 115200

# ---------------- colors ----------------
BG_MAIN = (16, 20, 28)
BG_LEFT = (24, 30, 42)
BG_CARD = (34, 42, 58)

BTN = (60, 78, 105)
BTN_HOVER = (82, 104, 135)
BTN_BORDER = (120, 135, 160)

TEXT_MAIN = (235, 240, 248)
TEXT_MUTED = (170, 180, 195)

VIZ_BG = (245, 246, 248)
VIZ_BORDER = (190, 195, 205)


def main():
    pygame.init()
    screen = pygame.display.set_mode((1200, 650))
    pygame.display.set_caption("Stewart Platform Control")
    clock = pygame.time.Clock()

    font_title = pygame.font.SysFont("Segoe UI", 22, bold=True)
    font = pygame.font.SysFont("Segoe UI", 16)
    font_small = pygame.font.SysFont("Segoe UI", 13)

    # ---------------- serial (optional) ----------------
    ser = None
    port = sys.argv[1] if len(sys.argv) >= 2 else None

    if port:
        try:
            ser = serial.Serial(port, BAUD_RATE, timeout=0)
            # Give the Arduino time to reset after opening serial on Windows
            time.sleep(2.0)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
        except Exception as e:
            print(f"[Serial] Failed to open {port}: {e}")
            ser = None

    # ---------------- joystick backend ----------------
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

    # ---------------- UI STATES ----------------
    MAIN = "main"
    START_MENU = "start"
    PRESETS = "presets"
    RUNNING = "running"

    menu = MAIN
    active_preset = None

    # 🔴 NEW: wait-for-center flag
    waiting_for_center = False

    # ---------------- layout ----------------
    LEFT_W = 380
    HEIGHT = 650

    left_panel = pygame.Rect(0, 0, LEFT_W, HEIGHT)
    viz_panel = pygame.Rect(LEFT_W + 40, 80, 520, HEIGHT - 160)

    def make_buttons(labels):
        btns = []
        btn_w, btn_h = 260, 46
        start_y = 260
        for i, label in enumerate(labels):
            btns.append(
                (label,
                 pygame.Rect(60, start_y + i * (btn_h + 18), btn_w, btn_h))
            )
        return btns

    mouse_was_down = False

    # ---------------- main loop ----------------
    running = True
    while running:
        mouse_pos = pygame.mouse.get_pos()
        mouse_down = pygame.mouse.get_pressed()[0]
        clicked = mouse_down and not mouse_was_down
        mouse_was_down = mouse_down

        events = pygame.event.get()
        joystick.handle_events(events)

        for event in events:
            if event.type == pygame.QUIT:
                running = False

        joystick.tick()

        # 🔴 NEW: listen for Arduino center completion
        if ser:
            waiting = 0
            try:
                waiting = ser.in_waiting
            except serial.SerialException:
                # Can occur on Windows if the device is resetting or driver is busy
                waiting = 0
            except Exception:
                waiting = 0

            if waiting:
                lines = ser.read(waiting).decode(errors="replace").splitlines()
                for line in lines:
                    print("[ARDUINO]", line)

                    if waiting_for_center and "Centered. Motors disabled." in line:
                        waiting_for_center = False
                        if active_preset:
                            send_command(f"start {active_preset}")
                            menu = RUNNING

        # ---------------- draw ----------------
        screen.fill(BG_MAIN)
        pygame.draw.rect(screen, BG_LEFT, left_panel)

        screen.blit(font_title.render("Stewart Platform", True, TEXT_MAIN), (40, 32))

        # ---- buttons by state ----
        if menu == MAIN:
            buttons = make_buttons(["START", "CENTER", "STOP"])

        elif menu == START_MENU:
            buttons = make_buttons(["CONTROLLER", "PRESETS", "BACK"])

        elif menu == PRESETS:
            buttons = make_buttons(["DEMO", "FIGURE 8", "BACK"])

        elif menu == RUNNING:
            buttons = make_buttons(["RESTART", "STOP"])

        for label, rect in buttons:
            hover = rect.collidepoint(mouse_pos)
            color = BTN_HOVER if hover else BTN

            pygame.draw.rect(screen, color, rect, border_radius=10)
            pygame.draw.rect(screen, BTN_BORDER, rect, 2, border_radius=10)

            txt = font.render(label, True, TEXT_MAIN)
            screen.blit(
                txt,
                (rect.centerx - txt.get_width() // 2,
                 rect.centery - txt.get_height() // 2)
            )

            if clicked and hover:
                if menu == MAIN:
                    if label == "START":
                        menu = START_MENU
                    elif label == "CENTER":
                        send_command("center")
                    elif label == "STOP":
                        send_command("stop")

                elif menu == START_MENU:
                    if label == "CONTROLLER":
                        send_command("controller")
                        menu = MAIN
                    elif label == "PRESETS":
                        menu = PRESETS
                    elif label == "BACK":
                        menu = MAIN

                elif menu == PRESETS:
                    if label == "DEMO":
                        active_preset = "demo"
                        send_command("center")
                        waiting_for_center = True
                        menu = RUNNING
                    elif label == "FIGURE 8":
                        active_preset = "figure8"
                        send_command("center")
                        waiting_for_center = True
                        menu = RUNNING
                    elif label == "BACK":
                        menu = START_MENU

                elif menu == RUNNING:
                    if label == "STOP":
                        send_command("stop")
                        waiting_for_center = False
                        menu = MAIN
                    elif label == "RESTART":
                        send_command("center")
                        waiting_for_center = True

        # ---- running label ----
        if menu == RUNNING and active_preset:
            tag = font.render(
                f"RUNNING: {active_preset.upper()}",
                True,
                TEXT_MUTED
            )
            screen.blit(tag, (40, 210))

        # ---- joystick mode toggle (bottom of left panel) ----
        if joystick.available:
            mode_y = HEIGHT - 180
            mode_label = font.render("JOYSTICK MODE", True, TEXT_MUTED)
            screen.blit(mode_label, (40, mode_y))

            mode_toggle_rect = pygame.Rect(60, mode_y + 35, 260, 46)
            mode_hover = mode_toggle_rect.collidepoint(mouse_pos)
            mode_color = BTN_HOVER if mode_hover else BTN

            pygame.draw.rect(screen, mode_color, mode_toggle_rect, border_radius=10)
            pygame.draw.rect(screen, BTN_BORDER, mode_toggle_rect, 2, border_radius=10)

            mode_text = "D-PAD MODE" if joystick.dpad_mode else "ANALOG MODE"
            mode_txt = font.render(mode_text, True, TEXT_MAIN)
            screen.blit(
                mode_txt,
                (mode_toggle_rect.centerx - mode_txt.get_width() // 2,
                 mode_toggle_rect.centery - mode_txt.get_height() // 2)
            )

            if clicked and mode_hover:
                joystick.toggle_mode()
                print(f"[Mode] Switched to {'D-PAD' if joystick.dpad_mode else 'ANALOG'}")

        # ---- visualization panel ----
        pygame.draw.rect(screen, VIZ_BG, viz_panel, border_radius=14)
        pygame.draw.rect(screen, VIZ_BORDER, viz_panel, 2, border_radius=14)

        # ---- control scheme display ----
        if joystick.available:
            info_y = viz_panel.top + 30
            info_x = viz_panel.left + 30

            # Title
            controls_title = font_title.render("CONTROLS", True, (40, 50, 65))
            screen.blit(controls_title, (info_x, info_y))

            info_y += 50

            if joystick.dpad_mode:
                # D-PAD MODE controls
                controls = [
                    ("D-Pad:", "Move X/Y position", TEXT_MAIN),
                    ("LB/RB:", "Move Z (up/down)", TEXT_MAIN),
                    ("", "", TEXT_MAIN),
                    ("A Button:", "Enable controller", TEXT_MUTED),
                    ("B Button:", "Stop movement", TEXT_MUTED),
                    ("X Button:", "Center platform", TEXT_MUTED),
                    ("Y Button:", "Run demo", TEXT_MUTED),
                ]
            else:
                # ANALOG MODE controls
                controls = [
                    ("Left Stick:", "Move X/Y position", TEXT_MAIN),
                    ("Right Stick UP:", "Tilt FORWARD (10°)", (100, 180, 255)),
                    ("Right Stick DOWN:", "Tilt BACK (10°)", (100, 180, 255)),
                    ("Right Stick LEFT:", "Tilt LEFT (10°)", (100, 180, 255)),
                    ("Right Stick RIGHT:", "Tilt RIGHT (10°)", (100, 180, 255)),
                    ("", "", TEXT_MAIN),
                    ("A Button:", "Enable controller", TEXT_MUTED),
                    ("B Button:", "Stop movement", TEXT_MUTED),
                    ("X Button:", "Center platform", TEXT_MUTED),
                    ("Y Button:", "Run demo", TEXT_MUTED),
                ]

            for label, description, color in controls:
                if label:
                    label_txt = font.render(label, True, color)
                    screen.blit(label_txt, (info_x, info_y))

                    if description:
                        desc_txt = font_small.render(description, True, TEXT_MUTED)
                        screen.blit(desc_txt, (info_x + 150, info_y + 2))

                info_y += 30

            # Mode indicator at bottom
            mode_indicator_y = viz_panel.bottom - 50
            mode_text = "MODE: D-PAD (discrete positions)" if joystick.dpad_mode else "MODE: ANALOG (continuous + directional tilt)"
            mode_indicator = font.render(mode_text, True, (80, 100, 130))
            screen.blit(mode_indicator, (info_x, mode_indicator_y))

        pygame.display.flip()
        clock.tick(30)

    if ser:
        ser.write(b"stop\n")
        ser.close()
    pygame.quit()


if __name__ == "__main__":
    main()
