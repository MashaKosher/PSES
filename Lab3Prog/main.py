import sys
import serial
import serial.tools.list_ports
import tkinter as tk
from tkinter import ttk, messagebox


BAUDRATE = 115200
FRAME_START = 0xFF


def find_default_port() -> str | None:
    """
    Try to find an ST-LINK Virtual COM Port automatically.
    Returns port name or None.
    """
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "stlink" in desc or "st-link" in desc or "stm" in desc:
            return p.device
    # Fallback: first available port
    ports = list(serial.tools.list_ports.comports())
    return ports[0].device if ports else None


class RGBViewerApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Lab3 RGB Viewer")
        self.root.geometry("360x320")
        self.root.resizable(False, False)

        self.ser: serial.Serial | None = None
        self.running = False

        self._build_ui()
        default_port = find_default_port()
        if default_port:
            self.port_var.set(default_port)

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=10)
        main.pack(fill="both", expand=True)

        # Port selection
        port_frame = ttk.Frame(main)
        port_frame.pack(fill="x", pady=(0, 10))

        ttk.Label(port_frame, text="Serial port:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_entry = ttk.Entry(port_frame, textvariable=self.port_var, width=18)
        self.port_entry.pack(side="left", padx=5)

        self.connect_btn = ttk.Button(
            port_frame, text="Connect", command=self.on_connect_clicked
        )
        self.connect_btn.pack(side="left", padx=5)

        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(port_frame, textvariable=self.status_var).pack(side="left", padx=5)

        # Color preview
        self.canvas = tk.Canvas(main, width=320, height=220, highlightthickness=1)
        self.canvas.pack(fill="both", expand=True)
        self.rect = self.canvas.create_rectangle(
            0, 0, 400, 400, fill="#000000", outline=""
        )

        # Last RGB label
        self.rgb_var = tk.StringVar(value="R: 0  G: 0  B: 0")
        ttk.Label(main, textvariable=self.rgb_var).pack(pady=(8, 0))

    def on_connect_clicked(self) -> None:
        if self.ser and self.ser.is_open:
            # Disconnect
            self.running = False
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.connect_btn.config(text="Connect")
            self.status_var.set("Disconnected")
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Please enter serial port name (e.g. COM3).")
            return

        try:
            self.ser = serial.Serial(port, BAUDRATE, timeout=0.05)
        except Exception as e:
            messagebox.showerror("Error", f"Cannot open port {port}:\n{e}")
            self.ser = None
            return

        self.running = True
        self.connect_btn.config(text="Disconnect")
        self.status_var.set(f"Connected to {port}")
        self.root.after(10, self.poll_serial)

    def poll_serial(self) -> None:
        if not self.running or not self.ser or not self.ser.is_open:
            return

        try:
            # Consume all available data; pick last complete frame
            last_rgb = None
            while self.ser.in_waiting >= 4:
                b = self.ser.read(1)
                if not b:
                    break
                if b[0] != FRAME_START:
                    continue
                data = self.ser.read(3)
                if len(data) != 3:
                    break
                r, g, b_val = data[0], data[1], data[2]
                last_rgb = (r, g, b_val)

            if last_rgb is not None:
                self.update_color(*last_rgb)

        except Exception as e:
            self.status_var.set(f"Serial error: {e}")
            self.running = False
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.connect_btn.config(text="Connect")
            return

        self.root.after(10, self.poll_serial)

    def update_color(self, r: int, g: int, b: int) -> None:
        color = f"#{r:02x}{g:02x}{b:02x}"
        self.canvas.itemconfig(self.rect, fill=color)
        self.rgb_var.set(f"R: {r:3d}   G: {g:3d}   B: {b:3d}")


def main() -> None:
    root = tk.Tk()
    app = RGBViewerApp(root)
    root.mainloop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)


