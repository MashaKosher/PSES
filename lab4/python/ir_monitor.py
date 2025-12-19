#!/usr/bin/env python3
"""
IR Remote Control Monitor
=========================
Receives and displays decoded IR signals from STM32 via serial port.

Usage:
    python ir_monitor.py [port] [baudrate]

Example:
    python ir_monitor.py /dev/ttyUSB0 115200
    python ir_monitor.py COM3 115200
"""

import sys
import serial
import serial.tools.list_ports
from datetime import datetime


# Common NEC remote control codes (can be extended)
KNOWN_CODES = {
    # Example codes - update with your remote's actual codes
    0x00FF6897: "0",
    0x00FF30CF: "1",
    0x00FF18E7: "2",
    0x00FF7A85: "3",
    0x00FF10EF: "4",
    0x00FF38C7: "5",
    0x00FF5AA5: "6",
    0x00FF42BD: "7",
    0x00FF4AB5: "8",
    0x00FF52AD: "9",
    0x00FFA25D: "CH-",
    0x00FF629D: "CH",
    0x00FFE21D: "CH+",
    0x00FF22DD: "|<<",
    0x00FF02FD: ">>|",
    0x00FFC23D: ">||",
    0x00FFE01F: "-",
    0x00FFA857: "+",
    0x00FF906F: "EQ",
    0x00FF9867: "100+",
    0x00FFB04F: "200+",
}


def list_serial_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found!")
        return []

    print("\nAvailable serial ports:")
    print("-" * 50)
    for port in ports:
        print(f"  {port.device}: {port.description}")
    print("-" * 50)
    return [p.device for p in ports]


def parse_ir_line(line):
    """Parse a line of IR data from the STM32."""
    line = line.strip()

    if line == "REPEAT":
        return {"type": "repeat"}

    if line.startswith("CODE:"):
        try:
            # Format: CODE:0xXXXXXXXX ADDR:0xXX CMD:0xXX
            parts = line.split()
            code_str = parts[0].split(":")[1]
            addr_str = parts[1].split(":")[1]
            cmd_str = parts[2].split(":")[1]

            code = int(code_str, 16)
            addr = int(addr_str, 16)
            cmd = int(cmd_str, 16)

            return {
                "type": "code",
                "raw": code,
                "address": addr,
                "command": cmd
            }
        except (IndexError, ValueError) as e:
            return {"type": "error", "message": str(e), "raw": line}

    if line == "IR Decoder Ready":
        return {"type": "ready"}

    return {"type": "unknown", "raw": line}


def format_output(data):
    """Format parsed IR data for display."""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]

    if data["type"] == "ready":
        return f"[{timestamp}] ✓ STM32 IR Decoder Ready"

    if data["type"] == "repeat":
        return f"[{timestamp}] ↺ REPEAT"

    if data["type"] == "code":
        code = data["raw"]
        addr = data["address"]
        cmd = data["command"]

        # Look up known button name
        button = KNOWN_CODES.get(code, "Unknown")

        output = f"[{timestamp}] 📡 IR Code Received:\n"
        output += f"    Raw Code:  0x{code:08X}\n"
        output += f"    Address:   0x{addr:02X} ({addr})\n"
        output += f"    Command:   0x{cmd:02X} ({cmd})\n"
        output += f"    Button:    {button}"
        return output

    if data["type"] == "error":
        return f"[{timestamp}] ⚠ Parse Error: {data['message']} - Raw: {data['raw']}"

    return f"[{timestamp}] ? {data.get('raw', 'Unknown data')}"


def main():
    # Default settings
    port = None
    baudrate = 115200

    # Parse command line arguments
    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3:
        baudrate = int(sys.argv[2])

    # If no port specified, list available ports and ask user
    if not port:
        available_ports = list_serial_ports()
        if not available_ports:
            print("\nPlease connect your STM32 board and try again.")
            sys.exit(1)

        print("\nEnter the serial port to use (e.g., /dev/ttyUSB0 or COM3):")
        port = input("> ").strip()

    print(f"\n{'='*60}")
    print("IR Remote Control Monitor")
    print(f"{'='*60}")
    print(f"Port:     {port}")
    print(f"Baudrate: {baudrate}")
    print(f"{'='*60}")
    print("\nPress Ctrl+C to exit\n")
    print("Waiting for IR signals...")
    print("-" * 60)

    try:
        with serial.Serial(port, baudrate, timeout=1) as ser:
            while True:
                try:
                    line = ser.readline()
                    if line:
                        decoded = line.decode('utf-8', errors='ignore').strip()
                        if decoded:
                            data = parse_ir_line(decoded)
                            print(format_output(data))
                            print("-" * 60)

                except serial.SerialException as e:
                    print(f"\nSerial error: {e}")
                    break
                except UnicodeDecodeError:
                    continue

    except serial.SerialException as e:
        print(f"\nCould not open port {port}: {e}")
        print("Please check:")
        print("  1. The STM32 board is connected")
        print("  2. The correct port is specified")
        print("  3. No other application is using the port")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nExiting...")
        sys.exit(0)


if __name__ == "__main__":
    main()

