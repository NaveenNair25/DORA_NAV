#!/usr/bin/env python3
"""
Simple keyboard control for Ranger robot via DORA
Based on the adora keyboard control script
"""

import json
import sys
import termios
import tty

def get_key():
    """Get a single keypress from stdin"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    print("========================================")
    print("Ranger Keyboard Control")
    print("========================================")
    print("Controls:")
    print("  w/s: forward/backward")
    print("  a/d: turn left/right")
    print("  q/e: strafe left/right (parallel mode)")
    print("  x: stop")
    print("  ESC: quit")
    print("========================================")

    linear_speed = 0.5  # m/s
    angular_speed = 0.3  # rad/s

    try:
        while True:
            key = get_key()

            cmd = {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
            }

            if key == 'w':
                cmd["linear"]["x"] = linear_speed
                print("Forward")
            elif key == 's':
                cmd["linear"]["x"] = -linear_speed
                print("Backward")
            elif key == 'a':
                cmd["angular"]["z"] = angular_speed
                print("Turn Left")
            elif key == 'd':
                cmd["angular"]["z"] = -angular_speed
                print("Turn Right")
            elif key == 'q':
                cmd["linear"]["y"] = linear_speed
                print("Strafe Left")
            elif key == 'e':
                cmd["linear"]["y"] = -linear_speed
                print("Strafe Right")
            elif key == 'x':
                print("Stop")
            elif key == '\x1b':  # ESC
                print("Exiting...")
                break
            else:
                continue

            # Output JSON command
            print(json.dumps(cmd))

    except KeyboardInterrupt:
        print("\nExiting...")

if __name__ == "__main__":
    main()
