"""
IR Receiver Module
==================
Handles serial communication with STM32 IR decoder.
Runs in a background thread and dispatches IR codes to callbacks.
"""

import threading
import serial
import serial.tools.list_ports
import re
from typing import Callable, Optional


class IRReceiver:
    """Receives and parses IR codes from STM32 via serial port."""

    # IR Code mappings
    CODE_GIF1_TOGGLE = 0x20DF8877  # Top-left corner
    CODE_GIF2_TOGGLE = 0x20DF48B7  # Top-right corner
    CODE_GIF3_TOGGLE = 0x20DFC837  # Bottom-left corner
    CODE_GIF4_TOGGLE = 0x20DF28D7  # Bottom-right corner
    CODE_PLAY_PAUSE = 0x20DFA857   # Play/Pause track
    CODE_RESET_TRACK = 0x20DF6897  # Reset track to start

    def __init__(self, port: Optional[str] = None, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.callback: Optional[Callable[[int], None]] = None

        # Regex to parse IR code from STM32 output
        # Format: CODE:0xXXXXXXXX ADDR:0xXX CMD:0xXX
        self.code_pattern = re.compile(r'CODE:(0x[0-9A-Fa-f]+)')

    def list_ports(self) -> list:
        """List available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def connect(self, port: Optional[str] = None) -> bool:
        """Connect to serial port."""
        if port:
            self.port = port

        if not self.port:
            available = self.list_ports()
            if available:
                self.port = available[0]
            else:
                print("No serial ports available")
                return False

        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=1)
            print(f"Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from serial port."""
        self.stop()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("Disconnected")

    def set_callback(self, callback: Callable[[int], None]):
        """Set callback function to be called when IR code is received."""
        self.callback = callback

    def _parse_line(self, line: str) -> Optional[int]:
        """Parse a line from serial and extract IR code."""
        line = line.strip()

        if line == "REPEAT":
            return None  # Ignore repeat codes for now

        match = self.code_pattern.search(line)
        if match:
            try:
                code = int(match.group(1), 16)
                return code
            except ValueError:
                pass

        return None

    def _receiver_thread(self):
        """Background thread that reads serial data."""
        while self.running:
            try:
                if self.serial and self.serial.is_open:
                    line = self.serial.readline()
                    if line:
                        decoded = line.decode('utf-8', errors='ignore').strip()
                        if decoded:
                            code = self._parse_line(decoded)
                            if code is not None and self.callback:
                                self.callback(code)
            except serial.SerialException:
                print("Serial connection lost")
                self.running = False
            except Exception as e:
                print(f"Error in receiver thread: {e}")

    def start(self):
        """Start the receiver thread."""
        if not self.serial or not self.serial.is_open:
            if not self.connect():
                return False

        self.running = True
        self.thread = threading.Thread(target=self._receiver_thread, daemon=True)
        self.thread.start()
        print("IR Receiver started")
        return True

    def stop(self):
        """Stop the receiver thread."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
            self.thread = None
        print("IR Receiver stopped")


# For testing without hardware
class MockIRReceiver:
    """Mock IR receiver for testing without hardware."""

    CODE_GIF1_TOGGLE = IRReceiver.CODE_GIF1_TOGGLE
    CODE_GIF2_TOGGLE = IRReceiver.CODE_GIF2_TOGGLE
    CODE_GIF3_TOGGLE = IRReceiver.CODE_GIF3_TOGGLE
    CODE_GIF4_TOGGLE = IRReceiver.CODE_GIF4_TOGGLE
    CODE_PLAY_PAUSE = IRReceiver.CODE_PLAY_PAUSE
    CODE_RESET_TRACK = IRReceiver.CODE_RESET_TRACK

    def __init__(self):
        self.callback: Optional[Callable[[int], None]] = None

    def set_callback(self, callback: Callable[[int], None]):
        self.callback = callback

    def simulate_code(self, code: int):
        """Simulate receiving an IR code."""
        if self.callback:
            self.callback(code)

    def connect(self, port: Optional[str] = None) -> bool:
        return True

    def disconnect(self):
        pass

    def start(self):
        return True

    def stop(self):
        pass

