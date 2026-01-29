# Stewart Platform: Spacebar Start + Joystick Control

This firmware now supports:

- Spacebar start for the scripted path
- Live joystick control streamed from a PC

## How It Works

- On boot, the Arduino calibrates, then waits.
- Press SPACE (in any serial terminal) to run the predefined motion path.
- Send joystick frames over serial with the protocol: `J ax ay az azi alt yaw` followed by `\n`.
  - `ax, ay, az` in `[-1, 1]` map to translations (Arduino maps to ±3\")
  - `azi` in degrees (heading); `alt` in degrees (tilt magnitude)
  - `yaw` in degrees (rotation about Z)
- Send `S` to stop immediately.

## PC Joystick Sender (Windows)

Minimal setup:

```powershell
# Install dependencies once
pip install pygame pyserial

# Run with your COM port
python host/joystick_sender.py COM5
```

## Notes

- In joystick mode, motors enable on first frame and disable when `S` is sent.
- You can tune ranges in the Arduino code (`±3\"` translation, tilt/yaw scales) and in the Python sender.
- If your controller axes differ, adjust the axis indices in `host/joystick_sender.py`.
If you’re unsure of the COM port, check Windows Device Manager → Ports (COM & LPT) while the board is plugged in.
