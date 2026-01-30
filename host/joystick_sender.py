import sys
import time
import os
import serial
import pygame

SERIAL_PORT = None
BAUD_RATE = 115200

# Scale factors
TRANS_SCALE = 1.0
ALT_SCALE_DEG = 10.0
YAW_SCALE_DEG = 30.0

# Axis alignment + smoothing
INVERT_X = False
INVERT_Y = False
INVERT_Z = False
SWAP_XY = False
SMOOTH_ALPHA = 0.25


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def main():
    global INVERT_X, INVERT_Y, INVERT_Z, SWAP_XY, SMOOTH_ALPHA

    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode((1080, 480))
    pygame.display.set_caption('Stewart Platform Sender')
    font = pygame.font.SysFont(None, 24)

    if pygame.joystick.get_count() == 0:
        print('No joystick detected.')
        return 1

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f'Using joystick: {js.get_name()}')

    # Port selection
    port = sys.argv[1] if len(sys.argv) >= 2 else os.environ.get('SERIAL_PORT')
    if not port:
        print('Usage: python host/joystick_sender.py COMx  (e.g., COM5)')
        return 1

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0)
        try:
            ser.setDTR(False)
            ser.setRTS(False)
        except Exception:
            pass
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f'Failed to open serial {port}: {e}')
        return 1

    def send_cmd(cmd: str):
        nonlocal controller_armed, command_pause_until
        cmd = cmd.strip().lower()
        msg = (cmd + "\n").encode("ascii")
        try:
            ser.write(msg)
            print(f">>> SENT: {cmd}")

            # Pause joystick streaming briefly so Arduino can read the command
            command_pause_until = time.monotonic() + 0.35

            # Track when it's safe/meaningful to stream J frames
            if cmd == "controller":
                controller_armed = True
            elif cmd in ("stop", "center", "start"):
                # start runs scripted sequence (no J frames needed)
                controller_armed = False

        except Exception as e:
            print(f"Failed to send {cmd}: {e}")

    clock = pygame.time.Clock()
    print("Controls:")
    print("  A=controller, B=stop, X=center, Y=start, Back=toggle D-pad/Joystick")
    print("  Keys: M=mode, [/]=smoothing, X/Y/Z invert, O swap X/Y, ESC quit")

    last_rx = ''
    dpad_mode = True
    ax_f = ay_f = az_f = alt_f = yaw_f = 0.0
    controller_armed = False
    command_pause_until = 0.0

    # Cooldown so a held button doesn't spam commands
    last_cmd_time = {"controller": 0.0, "stop": 0.0, "center": 0.0, "start": 0.0}
    CMD_COOLDOWN = 0.40  # seconds

    def can_send(name: str) -> bool:
        now = time.monotonic()
        if now - last_cmd_time.get(name, 0.0) >= CMD_COOLDOWN:
            last_cmd_time[name] = now
            return True
        return False

    def draw_button(rect, text, mouse_pos, mouse_down):
        x, y, w, h = rect
        hover = (x <= mouse_pos[0] <= x+w and y <= mouse_pos[1] <= y+h)
        color = (70, 90, 120) if hover else (50, 65, 90)
        if hover and mouse_down:
            color = (35, 45, 60)
        pygame.draw.rect(screen, color, rect, border_radius=10)
        pygame.draw.rect(screen, (120, 130, 150), rect, width=2, border_radius=10)
        label = font.render(text, True, (230, 230, 230))
        screen.blit(label, (x + 12, y + (h - label.get_height()) // 2))
        return hover

    try:
        was_mouse_down = False
        while True:
            mouse_pos = pygame.mouse.get_pos()
            mouse_down = pygame.mouse.get_pressed(num_buttons=3)[0]
            clicked = (mouse_down and not was_mouse_down)
            was_mouse_down = mouse_down
            # ---- Events (keyboard + controller buttons) ----
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        raise KeyboardInterrupt

                    # Keep your existing keyboard controls
                    elif event.key == pygame.K_m:
                        dpad_mode = not dpad_mode
                        print(f"Mode: {'D-pad' if dpad_mode else 'Joystick'}")

                    elif event.key == pygame.K_x:
                        INVERT_X = not INVERT_X; print(f"Invert X: {INVERT_X}")
                    elif event.key == pygame.K_y:
                        INVERT_Y = not INVERT_Y; print(f"Invert Y: {INVERT_Y}")
                    elif event.key == pygame.K_z:
                        INVERT_Z = not INVERT_Z; print(f"Invert Z: {INVERT_Z}")
                    elif event.key == pygame.K_o:
                        SWAP_XY = not SWAP_XY; print(f"Swap X/Y: {SWAP_XY}")
                    elif event.key == pygame.K_LEFTBRACKET:
                        SMOOTH_ALPHA = max(0.05, SMOOTH_ALPHA - 0.05); print(f"Alpha: {SMOOTH_ALPHA:.2f}")
                    elif event.key == pygame.K_RIGHTBRACKET:
                        SMOOTH_ALPHA = min(0.95, SMOOTH_ALPHA + 0.05); print(f"Alpha: {SMOOTH_ALPHA:.2f}")

                    # Optional keyboard shortcuts for commands:
                    elif event.key == pygame.K_1:
                        if can_send("center"): send_cmd("center")
                    elif event.key == pygame.K_2:
                        if can_send("controller"): send_cmd("controller")
                    elif event.key == pygame.K_3:
                        if can_send("start"): send_cmd("start")
                    elif event.key == pygame.K_4:
                        if can_send("stop"): send_cmd("stop")

                elif event.type == pygame.JOYBUTTONDOWN:
                    # Xbox mapping: A=0 B=1 X=2 Y=3 LB=4 RB=5 Back=6 Start=7
                    b = event.button

                    if b == 0:  # A
                        if can_send("controller"): send_cmd("controller")
                    elif b == 1:  # B
                        if can_send("stop"): send_cmd("stop")
                    elif b == 2:  # X
                        if can_send("center"): send_cmd("center")
                    elif b == 3:  # Y
                        if can_send("start"): send_cmd("start")
                    elif b == 6:  # Back
                        dpad_mode = not dpad_mode
                        print(f"Mode: {'D-pad' if dpad_mode else 'Joystick'}")
                    elif b == 7:  # Start
                        if can_send("controller"): send_cmd("controller")

            # ---- Input mapping (same as yours) ----
            if dpad_mode:
                hx, hy = js.get_hat(0)
                lb = 1.0 if js.get_button(4) else 0.0
                rb = 1.0 if js.get_button(5) else 0.0

                ax_raw = float(hx)
                ay_raw = float(hy)
                az_raw = (rb - lb)

                alt = 0.0
                yaw = 0.0

                # enable flag
                azi = 1.0 if (hx != 0 or hy != 0 or abs(az_raw) > 0.0) else 0.0
                dpad_active = (azi > 0.5)

            else:
                ax_raw = clamp(js.get_axis(0)) * TRANS_SCALE
                ay_raw = clamp(-js.get_axis(1)) * TRANS_SCALE
                lt = js.get_axis(2); rt = js.get_axis(5)
                lt = (lt + 1.0) * 0.5; rt = (rt + 1.0) * 0.5
                az_raw = clamp(rt - lt)
                rx = clamp(js.get_axis(3)); ry = clamp(-js.get_axis(4))
                alt = clamp(ry) * ALT_SCALE_DEG
                yaw = clamp(rx) * YAW_SCALE_DEG

                # IMPORTANT: make azi enable when ANY motion intent exists
                azi = 1.0 if (abs(ax_raw) > 0.05 or abs(ay_raw) > 0.05 or abs(az_raw) > 0.05 or abs(alt) > 0.5 or abs(yaw) > 0.5) else 0.0

            # Apply alignment
            ax_m, ay_m = (ay_raw, ax_raw) if SWAP_XY else (ax_raw, ay_raw)
            ax = ax_m * (-1.0 if INVERT_X else 1.0)
            ay = ay_m * (-1.0 if INVERT_Y else 1.0)
            az = az_raw * (-1.0 if INVERT_Z else 1.0)

            # Smoothing
            if dpad_mode:
                ax_f, ay_f, az_f, alt_f, yaw_f = ax, ay, az, 0.0, 0.0
            else:
                ax_f = ax_f + SMOOTH_ALPHA * (ax - ax_f)
                ay_f = ay_f + SMOOTH_ALPHA * (ay - ay_f)
                az_f = az_f + SMOOTH_ALPHA * (az - az_f)
                alt_f = alt_f + SMOOTH_ALPHA * (alt - alt_f)
                yaw_f = yaw_f + SMOOTH_ALPHA * (yaw - yaw_f)

            # Send line when active
            dead_ax = 0.05
            dead_ang = 0.5
            if dpad_mode:
                active = dpad_active
            else:
                active = (abs(ax_f) > dead_ax or abs(ay_f) > dead_ax or abs(az_f) > dead_ax or abs(alt_f) > dead_ang or abs(yaw_f) > dead_ang)

            if controller_armed and active and time.monotonic() >= command_pause_until:
                line = f"J {ax_f:.3f} {ay_f:.3f} {az_f:.3f} {azi:.1f} {alt_f:.1f} {yaw_f:.1f}\n"
                ser.write(line.encode('ascii'))

            # Arduino responses
            waiting = getattr(ser, 'in_waiting', 0)
            if waiting:
                data = ser.read(waiting)
                text = data.decode('utf-8', errors='replace').strip()
                if text:
                    last_rx = text.splitlines()[-1]
                    print(f"ARDUINO: {last_rx}")

            # UI
            screen.fill((20, 20, 20))

            def blit_line(y, msg):
                surf = font.render(msg, True, (230, 230, 230))
                screen.blit(surf, (12, y))

            blit_line(10,  f"Port: {port}  |  Baud: {BAUD_RATE}")
            blit_line(40,  f"Mode: {'D-pad' if dpad_mode else 'Joystick'}   (Back toggles)")
            blit_line(70,  f"A=controller  B=stop  X=center  Y=start")
            blit_line(100, f"Axes: ax={ax_f:.2f} ay={ay_f:.2f} az={az_f:.2f}  alt={alt_f:.1f} yaw={yaw_f:.1f}")
            blit_line(130, f"SwapXY:{SWAP_XY}  InvX:{INVERT_X} InvY:{INVERT_Y} InvZ:{INVERT_Z}  Smooth:{SMOOTH_ALPHA:.2f}")
            blit_line(200, f"Arduino: {last_rx}")
            # Clickable command buttons
            btn_center = (12, 240, 160, 44)
            btn_controller = (182, 240, 160, 44)
            btn_start = (352, 240, 160, 44)
            btn_stop = (522, 240, 160, 44)

            h_center = draw_button(btn_center, "CENTER", mouse_pos, mouse_down)
            h_controller = draw_button(btn_controller, "CONTROLLER", mouse_pos, mouse_down)
            h_start = draw_button(btn_start, "START", mouse_pos, mouse_down)
            h_stop = draw_button(btn_stop, "STOP", mouse_pos, mouse_down)

            if clicked:
                if h_center and can_send("center"):
                    send_cmd("center")
                elif h_controller and can_send("controller"):
                    send_cmd("controller")
                elif h_start and can_send("start"):
                    send_cmd("start")
                elif h_stop and can_send("stop"):
                    send_cmd("stop")

            pygame.display.flip()
            clock.tick(30)

    except KeyboardInterrupt:
        print('\nStopping...')
        try:
            ser.write(b"stop\n")
        except Exception:
            pass
    finally:
        ser.close()
        pygame.quit()

    return 0


if __name__ == '__main__':
    sys.exit(main())
