#!/usr/bin/env python3
import re
import sys
from pathlib import Path

try:
    import serial.tools.list_ports
except ImportError:
    print(
        "error: pyserial not installed. install with: pixi install or pixi add pyserial"
    )
    sys.exit(1)


def find_serial_ports():
    ports = serial.tools.list_ports.comports()
    filtered = []
    for p in ports:
        device = p.device
        if sys.platform == "linux":
            if device.startswith("/dev/ttyACM") or device.startswith("/dev/ttyUSB"):
                filtered.append((device, p.description))
        elif sys.platform == "darwin":
            if device.startswith("/dev/cu.") or device.startswith("/dev/tty."):
                filtered.append((device, p.description))
        else:
            if "COM" in device.upper():
                filtered.append((device, p.description))
    return filtered


def update_justfile(port):
    justfile_path = Path(__file__).parent.parent / "justfile"

    if not justfile_path.exists():
        print(f"error: justfile not found at {justfile_path}")
        sys.exit(1)

    content = justfile_path.read_text()

    pattern = r'^PORT := ".*"'
    replacement = f'PORT := "{port}"'

    updated_content = re.sub(pattern, replacement, content, flags=re.MULTILINE)

    if content == updated_content:
        print("warning: no PORT variable found in justfile")
        sys.exit(1)

    justfile_path.write_text(updated_content)
    print(f"updated justfile: PORT = {port}")


def main():
    ports = find_serial_ports()

    if not ports:
        print("no serial ports found")
        sys.exit(1)

    print("available serial ports:")
    for idx, (device, description) in enumerate(ports, 1):
        print(f"  {idx}. {device} - {description}")

    while True:
        try:
            choice = input("\nselect port number: ").strip()
            idx = int(choice)
            if 1 <= idx <= len(ports):
                selected_port = ports[idx - 1][0]
                update_justfile(selected_port)
                break
            else:
                print(f"invalid selection. enter number between 1 and {len(ports)}")
        except ValueError:
            print("invalid input. enter a number")
        except KeyboardInterrupt:
            print("\naborted")
            sys.exit(0)


if __name__ == "__main__":
    main()
