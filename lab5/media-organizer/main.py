"""
Media Organizer - IR Remote Controlled Desktop App
===================================================
Displays 4 corner GIFs controlled by IR remote.
Plays audio track with background noise interference.
"""

import os
import sys
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
from typing import Optional, List

from ir_receiver import IRReceiver, MockIRReceiver
from media_controller import MediaController


class AnimatedGIF:
    """Handles animated GIF display in tkinter."""

    def __init__(self, label: tk.Label, path: str):
        self.label = label
        self.path = path
        self.frames: List[ImageTk.PhotoImage] = []
        self.delays: List[int] = []
        self.current_frame = 0
        self.playing = False
        self.after_id: Optional[str] = None
        self.target_size: Optional[tuple] = None

    def load(self, target_size: Optional[tuple] = None):
        """Load GIF frames."""
        self.target_size = target_size
        self.frames = []
        self.delays = []

        if not os.path.exists(self.path):
            print(f"GIF not found: {self.path}")
            return False

        try:
            img = Image.open(self.path)
            for frame_num in range(getattr(img, 'n_frames', 1)):
                img.seek(frame_num)
                frame = img.copy()

                if target_size:
                    frame = frame.resize(target_size, Image.Resampling.LANCZOS)

                self.frames.append(ImageTk.PhotoImage(frame))
                delay = img.info.get('duration', 100)
                self.delays.append(max(delay, 20))

            print(f"Loaded {len(self.frames)} frames from {self.path}")
            return True
        except Exception as e:
            print(f"Failed to load GIF {self.path}: {e}")
            return False

    def resize(self, new_size: tuple):
        """Reload GIF with new size."""
        if new_size != self.target_size:
            self.stop()
            self.load(new_size)
            if self.playing:
                self.play()

    def play(self):
        """Start animation."""
        if not self.frames:
            return
        
        # Don't start another loop if already playing
        if self.playing and self.after_id:
            return
        
        # Cancel any existing animation
        if self.after_id:
            self.label.after_cancel(self.after_id)
            self.after_id = None
        
        self.playing = True
        
        # Display first frame immediately
        self.label.config(image=self.frames[self.current_frame])
        
        # Start animation loop
        self._schedule_next_frame()

    def stop(self):
        """Stop animation."""
        self.playing = False
        if self.after_id:
            self.label.after_cancel(self.after_id)
            self.after_id = None

    def _schedule_next_frame(self):
        """Schedule the next frame."""
        if not self.playing or not self.frames:
            return
        
        delay = self.delays[self.current_frame] if self.delays else 100
        self.after_id = self.label.after(delay, self._animate)

    def _animate(self):
        """Animate to next frame."""
        if not self.playing or not self.frames:
            return

        self.current_frame = (self.current_frame + 1) % len(self.frames)
        self.label.config(image=self.frames[self.current_frame])
        
        # Schedule next frame
        self._schedule_next_frame()


class MediaOrganizer:
    """Main application class."""

    def __init__(self, use_mock_ir: bool = False):
        self.root = tk.Tk()
        self.root.title("Media Organizer")
        self.root.configure(bg='black')

        # Set initial window size
        self.root.geometry("800x600")
        self.root.minsize(400, 300)

        # Assets directory
        self.assets_dir = os.path.join(os.path.dirname(__file__), "assets")

        # State
        self.corner_visible = [False, False, False, False]
        self.corner_gifs: List[Optional[AnimatedGIF]] = [None, None, None, None]
        self.no_signal_gif: Optional[AnimatedGIF] = None
        self.corner_labels: List[Optional[tk.Label]] = [None, None, None, None]
        self.no_signal_label: Optional[tk.Label] = None

        # IR Receiver
        if use_mock_ir:
            self.ir_receiver = MockIRReceiver()
        else:
            self.ir_receiver = IRReceiver()

        # Media Controller
        self.media_controller = MediaController(self.assets_dir)

        # Setup UI
        self._setup_ui()
        self._setup_ir()
        self._setup_media()

        # Bind resize event
        self.root.bind('<Configure>', self._on_resize)

        # Bind keyboard for testing
        self._bind_keyboard_shortcuts()

        # Schedule initial display update after window is ready
        self.root.after(100, self._initial_display)

    def _setup_ui(self):
        """Setup the tkinter UI."""
        # Main container
        self.container = tk.Frame(self.root, bg='black')
        self.container.pack(fill=tk.BOTH, expand=True)

        # Configure grid weights for equal sizing
        self.container.grid_rowconfigure(0, weight=1)
        self.container.grid_rowconfigure(1, weight=1)
        self.container.grid_columnconfigure(0, weight=1)
        self.container.grid_columnconfigure(1, weight=1)

        # Create corner labels (initially hidden)
        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        for i, (row, col) in enumerate(positions):
            label = tk.Label(self.container, bg='black', borderwidth=0, highlightthickness=0)
            label.grid(row=row, column=col, sticky='nsew')
            label.grid_remove()  # Hide initially
            self.corner_labels[i] = label

        # Create no signal label (covers entire window)
        self.no_signal_label = tk.Label(self.container, bg='black', borderwidth=0, highlightthickness=0)
        self.no_signal_label.grid(row=0, column=0, rowspan=2, columnspan=2, sticky='nsew')

        # Load GIFs
        self._load_gifs()

    def _load_gifs(self):
        """Load all GIF assets."""
        gif_files = [
            "corner1.gif",
            "corner2.gif",
            "corner3.gif",
            "corner4.gif"
        ]

        for i, filename in enumerate(gif_files):
            path = os.path.join(self.assets_dir, filename)
            if self.corner_labels[i]:
                gif = AnimatedGIF(self.corner_labels[i], path)
                gif.load()
                self.corner_gifs[i] = gif

        # Load no signal GIF (will be started in _initial_display)
        no_signal_path = os.path.join(self.assets_dir, "no_signal.gif")
        if self.no_signal_label:
            self.no_signal_gif = AnimatedGIF(self.no_signal_label, no_signal_path)
            self.no_signal_gif.load()

    def _setup_ir(self):
        """Setup IR receiver."""
        self.ir_receiver.set_callback(self._on_ir_code)

        # Try to connect (will fail gracefully if no device)
        if self.ir_receiver.connect():
            self.ir_receiver.start()

    def _setup_media(self):
        """Setup media controller."""
        self.media_controller.init()
        self.media_controller.load_track("track.mp3")
        self.media_controller.load_noise("noise.mp3")

        # Start with noise playing
        self.media_controller.play_noise()

    def _on_resize(self, event):
        """Handle window resize."""
        if event.widget != self.root:
            return

        # Calculate cell size
        width = event.width // 2
        height = event.height // 2

        if width < 10 or height < 10:
            return

        # Resize GIFs
        for gif in self.corner_gifs:
            if gif:
                gif.resize((width, height))

        if self.no_signal_gif:
            self.no_signal_gif.resize((event.width, event.height))

    def _on_ir_code(self, code: int):
        """Handle received IR code."""
        print(f"Received IR code: 0x{code:08X}")

        # Schedule UI update on main thread
        self.root.after(0, lambda: self._process_ir_code(code))

    def _process_ir_code(self, code: int):
        """Process IR code on main thread."""
        cmd = (code >> 8) & 0xFF
        # Accept either full 32-bit code match or just the NEC command byte match.
        # This makes the app robust if address differs across remotes.
        if code == IRReceiver.CODE_GIF1_TOGGLE or cmd == IRReceiver.CMD_GIF1_TOGGLE:
            self._toggle_corner(0)
        elif code == IRReceiver.CODE_GIF2_TOGGLE or cmd == IRReceiver.CMD_GIF2_TOGGLE:
            self._toggle_corner(1)
        elif code == IRReceiver.CODE_GIF3_TOGGLE or cmd == IRReceiver.CMD_GIF3_TOGGLE:
            self._toggle_corner(2)
        elif code == IRReceiver.CODE_GIF4_TOGGLE or cmd == IRReceiver.CMD_GIF4_TOGGLE:
            self._toggle_corner(3)
        elif code == IRReceiver.CODE_PLAY_PAUSE or cmd == IRReceiver.CMD_PLAY_PAUSE:
            self.media_controller.toggle_track()
        elif code == IRReceiver.CODE_RESET_TRACK or cmd == IRReceiver.CMD_RESET_TRACK:
            self.media_controller.reset_track()

    def _toggle_corner(self, index: int):
        """Toggle visibility of a corner GIF."""
        self.corner_visible[index] = not self.corner_visible[index]
        self._update_display()

    def _update_display(self):
        """Update the display based on corner visibility."""
        any_visible = any(self.corner_visible)

        if any_visible:
            # Hide no signal
            if self.no_signal_gif:
                self.no_signal_gif.stop()
            if self.no_signal_label:
                self.no_signal_label.grid_remove()

            # Show/hide corner GIFs
            for i in range(4):
                if self.corner_visible[i]:
                    if self.corner_labels[i]:
                        self.corner_labels[i].grid()
                    if self.corner_gifs[i]:
                        self.corner_gifs[i].play()
                else:
                    if self.corner_gifs[i]:
                        self.corner_gifs[i].stop()
                    if self.corner_labels[i]:
                        self.corner_labels[i].grid_remove()
        else:
            # Hide all corners
            for i in range(4):
                if self.corner_gifs[i]:
                    self.corner_gifs[i].stop()
                if self.corner_labels[i]:
                    self.corner_labels[i].grid_remove()

            # Show no signal
            if self.no_signal_label:
                self.no_signal_label.grid()
            if self.no_signal_gif:
                self.no_signal_gif.play()

    def _bind_keyboard_shortcuts(self):
        """Bind keyboard shortcuts for testing without IR remote."""
        self.root.bind('1', lambda e: self._toggle_corner(0))
        self.root.bind('2', lambda e: self._toggle_corner(1))
        self.root.bind('3', lambda e: self._toggle_corner(2))
        self.root.bind('4', lambda e: self._toggle_corner(3))
        self.root.bind('<space>', lambda e: self.media_controller.toggle_track())
        self.root.bind('r', lambda e: self.media_controller.reset_track())
        self.root.bind('<Escape>', lambda e: self.root.quit())

        print("\nKeyboard shortcuts (for testing):")
        print("  1-4: Toggle corner GIFs")
        print("  Space: Play/Pause track")
        print("  R: Reset track")
        print("  Esc: Quit")

    def _initial_display(self):
        """Initialize display after window is ready."""
        # Get current window size
        self.root.update_idletasks()
        width = self.root.winfo_width()
        height = self.root.winfo_height()

        # Resize and start no_signal GIF
        if self.no_signal_gif and width > 0 and height > 0:
            self.no_signal_gif.resize((width, height))
            self.no_signal_gif.play()

        # Resize corner GIFs (but don't play them yet)
        cell_width = width // 2
        cell_height = height // 2
        if cell_width > 10 and cell_height > 10:
            for gif in self.corner_gifs:
                if gif:
                    gif.load((cell_width, cell_height))

    def run(self):
        """Run the application."""
        print("\nMedia Organizer started")
        print(f"Assets directory: {self.assets_dir}")

        try:
            self.root.mainloop()
        finally:
            self.cleanup()

    def cleanup(self):
        """Clean up resources."""
        self.ir_receiver.stop()
        self.ir_receiver.disconnect()
        self.media_controller.cleanup()
        print("Cleanup complete")


def main():
    """Main entry point."""
    # Check for --mock flag to use mock IR receiver
    use_mock = '--mock' in sys.argv or '-m' in sys.argv

    # Check for --port flag to specify serial port
    port = None
    for i, arg in enumerate(sys.argv):
        if arg in ('--port', '-p') and i + 1 < len(sys.argv):
            port = sys.argv[i + 1]

    app = MediaOrganizer(use_mock_ir=use_mock)

    if port and hasattr(app.ir_receiver, 'connect'):
        app.ir_receiver.disconnect()
        app.ir_receiver.connect(port)
        app.ir_receiver.start()

    app.run()


if __name__ == "__main__":
    main()

