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

        ser.write(b"center\n")
        time.sleep(4.0)           # centering move time (adjust if needed)
        ser.write(b"controller\n")
        time.sleep(0.2)
        
    except Exception as e:
        print(f'Failed to open serial {port}: {e}')
        return 1

    clock = pygame.time.Clock()
    print("Streaming controller input only... Keys: M=mode, [/]=smoothing, X/Y/Z invert, O swap X/Y, ESC quit")
    last_rx = ''
    command_pause_until = 0.0
    dpad_mode = True
    ax_f = ay_f = az_f = alt_f = yaw_f = 0.0

    # No auto-enable self-test; controller-only movement

    try:
        while True:
            # Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        raise KeyboardInterrupt
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
                # No mouse buttons; controller-only

            # Input mapping
            if dpad_mode:
                hx, hy = js.get_hat(0)         # dpad: hx=-1/0/1, hy=-1/0/1
                lb = 1.0 if js.get_button(4) else 0.0
                rb = 1.0 if js.get_button(5) else 0.0

                # D-pad controls X/Y only (one axis at a time)
                ax_raw = float(hx)             # right=+1, left=-1
                ay_raw = float(hy)             # up=+1, down=-1

                # RB/LB controls Z only
                az_raw = (rb - lb)             # RB=+1 (up), LB=-1 (down)

                # NO tilt/yaw in dpad mode
                alt = 0.0
                yaw = 0.0

                # enable flag (just "1" when anything is pressed)
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
                # Use azi as an enable flag; true when triggers indicate Z intent
                azi = 1.0 if (abs(az_raw) > 0.05) else 0.0

            # Apply alignment
            ax_m, ay_m = (ay_raw, ax_raw) if SWAP_XY else (ax_raw, ay_raw)
            ax = ax_m * (-1.0 if INVERT_X else 1.0)
            ay = ay_m * (-1.0 if INVERT_Y else 1.0)
            az = az_raw * (-1.0 if INVERT_Z else 1.0)

            # Smoothing: off for D-pad, on for Joystick
            if dpad_mode:
                ax_f, ay_f, az_f, alt_f, yaw_f = ax, ay, az, 0.0, 0.0
            else:
                ax_f = ax_f + SMOOTH_ALPHA * (ax - ax_f)
                ay_f = ay_f + SMOOTH_ALPHA * (ay - ay_f)
                az_f = az_f + SMOOTH_ALPHA * (az - az_f)
                alt_f = alt_f + SMOOTH_ALPHA * (alt - alt_f)
                yaw_f = yaw_f + SMOOTH_ALPHA * (yaw - yaw_f)

            # Send line only when actively pressing controls
            dead_ax = 0.05
            dead_ang = 0.5
            if dpad_mode:
                active = dpad_active
            else:
                active = (abs(ax_f) > dead_ax or abs(ay_f) > dead_ax or abs(az_f) > dead_ax or abs(alt_f) > dead_ang or abs(yaw_f) > dead_ang)
            if active and time.monotonic() >= command_pause_until:
                line = f"J {ax_f:.3f} {ay_f:.3f} {az_f:.3f} {azi:.1f} {alt_f:.1f} {yaw_f:.1f}\n"
                ser.write(line.encode('ascii'))

            # Label
            thresh = 0.1
            parts = []
            if dpad_mode:
                if not (hx != 0 or hy != 0 or (rb - lb) != 0):
                    label = "Idle"
                else:
                    if hy > 0: parts.append("Back")  # inverted mapping applied above
                    elif hy < 0: parts.append("Forward")
                    if hx > 0: parts.append("Right")
                    elif hx < 0: parts.append("Left")
                    if rb > lb: parts.append("Up")
                    elif lb > rb: parts.append("Down")
                    label = ", ".join(parts) if parts else "Active"
            else:
                if ay_f > thresh: parts.append("Forward")
                elif ay_f < -thresh: parts.append("Back")
                if ax_f > thresh: parts.append("Right")
                elif ax_f < -thresh: parts.append("Left")
                if az_f > thresh: parts.append("Up")
                elif az_f < -thresh: parts.append("Down")
                label = "Idle" if not parts else ", ".join(parts)
            print(f"[{'D-pad' if dpad_mode else 'Joystick'}] {label} | ax={ax_f:.2f} ay={ay_f:.2f} az={az_f:.2f} alt={alt_f:.1f} yaw={yaw_f:.1f} (alpha={SMOOTH_ALPHA:.2f})")

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
            blit_line(10, f"Port: {port}  |  Baud: {BAUD_RATE}")
            blit_line(40, f"Label: {label}")
            blit_line(70, f"Axes: ax={ax_f:.2f} ay={ay_f:.2f} az={az_f:.2f}  alt={alt_f:.1f} yaw={yaw_f:.1f}")
            blit_line(100, "Keys: M=mode  [/]=smoothing  X/Y/Z=invert  O=swap X/Y  ESC=quit")
            blit_line(200, f"Mode: {'D-pad' if dpad_mode else 'Joystick'}   SwapXY:{SWAP_XY} InvX:{INVERT_X} InvY:{INVERT_Y} InvZ:{INVERT_Z}  Smooth:{SMOOTH_ALPHA:.2f}   Arduino: {last_rx}")
            pygame.display.flip()

            clock.tick(30)
    except KeyboardInterrupt:
        print('\nStopping...'); ser.write(b"stop\n")
    finally:
        ser.close(); pygame.quit()
    return 0


if __name__ == '__main__':
    sys.exit(main())
