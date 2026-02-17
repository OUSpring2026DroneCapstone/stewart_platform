# joystick.py
import time
import pygame

TRANS_SCALE = 1.0
ALT_SCALE_DEG = 10.0
YAW_SCALE_DEG = 30.0
SMOOTH_ALPHA = 0.25


def clamp(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


class JoystickBackend:
    def __init__(self, ser=None, on_command=None):
        """
        ser: pyserial Serial object or None
        on_command: callback like on_command("stop")
        """
        self.ser = ser
        self.on_command = on_command

        pygame.joystick.init()
        self.available = pygame.joystick.get_count() > 0
        self.js = None

        self.enabled = False
        self.pause_until = 0.0
        self.dpad_mode = True

        self.ax_f = self.ay_f = self.az_f = 0.0
        self.alt_f = self.yaw_f = 0.0

        if self.available:
            self.js = pygame.joystick.Joystick(0)
            self.js.init()
            print(f"[Joystick] Connected: {self.js.get_name()}")
        else:
            print("[Joystick] Not connected")

    # ---------------- controller state ----------------
    def enable_controller(self):
        self.enabled = True
        self.pause_until = time.monotonic() + 0.35

    def disable_controller(self):
        self.enabled = False

    def toggle_mode(self):
        self.dpad_mode = not self.dpad_mode

    # ---------------- joystick buttons ----------------
    def handle_events(self, events):
        if not self.available or not self.on_command:
            return

        for event in events:
            if event.type == pygame.JOYBUTTONDOWN:
                b = event.button

                # Xbox mapping
                if b == 0:        # A
                    self.on_command("controller")
                elif b == 1:      # B
                    self.on_command("stop")
                elif b == 2:      # X
                    self.on_command("center")
                elif b == 3:      # Y
                    self.on_command("start demo")

    # ---------------- main update ----------------
    def tick(self):
        """
        Called every frame by GUI.
        Polls joystick and updates filtered state for the GUI.
        Sends J frames to serial only when controller is enabled and serial is available.
        """
        # Always require a connected joystick to poll axes
        if not self.available:
            return

        active = False

        # Poll axes regardless of serial/controller state so GUI updates
        if self.dpad_mode:
            hx, hy = self.js.get_hat(0)
            lb = self.js.get_button(4)
            rb = self.js.get_button(5)

            ax = float(hx)
            ay = float(hy)
            az = rb - lb
            alt = yaw = 0.0
            active = (hx != 0 or hy != 0 or az != 0)

        else:
            ax = clamp(self.js.get_axis(0))
            ay = clamp(-self.js.get_axis(1))

            lt = (self.js.get_axis(2) + 1.0) * 0.5
            rt = (self.js.get_axis(5) + 1.0) * 0.5
            az = clamp(rt - lt)

            rx = clamp(self.js.get_axis(3))
            ry = clamp(-self.js.get_axis(4))
            alt = ry * ALT_SCALE_DEG
            yaw = rx * YAW_SCALE_DEG

            active = any(abs(v) > 0.05 for v in (ax, ay, az, alt, yaw))

        # Update filtered values for GUI
        self.ax_f += SMOOTH_ALPHA * (ax - self.ax_f)
        self.ay_f += SMOOTH_ALPHA * (ay - self.ay_f)
        self.az_f += SMOOTH_ALPHA * (az - self.az_f)
        self.alt_f += SMOOTH_ALPHA * (alt - self.alt_f)
        self.yaw_f += SMOOTH_ALPHA * (yaw - self.yaw_f)

        # Only send serial frames when controller is enabled, not paused, and serial available
        if not self.enabled or time.monotonic() < self.pause_until or not self.ser:
            return
        if not active:
            return

        line = (
            f"J {self.ax_f:.3f} {self.ay_f:.3f} {self.az_f:.3f} "
            f"1.0 {self.alt_f:.1f} {self.yaw_f:.1f}\n"
        )
        self.ser.write(line.encode())
