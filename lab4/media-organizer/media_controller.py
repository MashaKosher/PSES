"""
Media Controller Module
=======================
Handles audio playback using pygame mixer.
Manages main track and background noise/interference sound.
"""

import os
import pygame
from typing import Optional


class MediaController:
    """Controls audio playback: main track and background noise."""

    def __init__(self, assets_dir: str = "assets"):
        self.assets_dir = assets_dir
        self.track_path: Optional[str] = None
        self.noise_path: Optional[str] = None

        self.track_playing = False
        self.track_paused = False
        self.initialized = False

        # Pygame mixer channels
        self.track_channel: Optional[pygame.mixer.Channel] = None
        self.noise_channel: Optional[pygame.mixer.Channel] = None
        self.track_sound: Optional[pygame.mixer.Sound] = None
        self.noise_sound: Optional[pygame.mixer.Sound] = None

    def init(self):
        """Initialize pygame mixer."""
        if self.initialized:
            return True

        try:
            pygame.mixer.init(frequency=44100, size=-16, channels=2, buffer=2048)
            pygame.mixer.set_num_channels(8)

            # Reserve channels
            self.track_channel = pygame.mixer.Channel(0)
            self.noise_channel = pygame.mixer.Channel(1)

            self.initialized = True
            print("Media controller initialized")
            return True
        except pygame.error as e:
            print(f"Failed to initialize mixer: {e}")
            return False

    def load_track(self, filename: str = "track.mp3"):
        """Load the main audio track."""
        self.track_path = os.path.join(self.assets_dir, filename)
        if os.path.exists(self.track_path):
            try:
                self.track_sound = pygame.mixer.Sound(self.track_path)
                print(f"Loaded track: {self.track_path}")
                return True
            except pygame.error as e:
                print(f"Failed to load track: {e}")
        else:
            print(f"Track file not found: {self.track_path}")
        return False

    def load_noise(self, filename: str = "noise.mp3"):
        """Load the background noise/interference sound."""
        self.noise_path = os.path.join(self.assets_dir, filename)
        if os.path.exists(self.noise_path):
            try:
                self.noise_sound = pygame.mixer.Sound(self.noise_path)
                print(f"Loaded noise: {self.noise_path}")
                return True
            except pygame.error as e:
                print(f"Failed to load noise: {e}")
        else:
            print(f"Noise file not found: {self.noise_path}")
        return False

    def play_noise(self):
        """Start playing background noise in loop."""
        if self.noise_sound and self.noise_channel:
            self.noise_channel.play(self.noise_sound, loops=-1)
            self.noise_channel.set_volume(0.3)  # Lower volume for background
            print("Noise started")

    def stop_noise(self):
        """Stop background noise."""
        if self.noise_channel:
            self.noise_channel.stop()
            print("Noise stopped")

    def fade_noise(self, fade_in: bool, duration_ms: int = 500):
        """Fade noise in or out."""
        if not self.noise_channel:
            return

        if fade_in:
            self.noise_channel.set_volume(0.3)
        else:
            self.noise_channel.set_volume(0.0)

    def play_track(self):
        """Start playing the main track."""
        if not self.track_sound or not self.track_channel:
            return False

        if self.track_paused:
            # Resume from pause
            self.track_channel.unpause()
            self.track_paused = False
        else:
            # Start fresh, loop forever
            self.track_channel.play(self.track_sound, loops=-1)

        self.track_playing = True
        self.stop_noise()  # Stop noise when track plays
        print("Track playing")
        return True

    def pause_track(self):
        """Pause the main track."""
        if self.track_channel and self.track_playing:
            self.track_channel.pause()
            self.track_paused = True
            self.track_playing = False
            self.play_noise()  # Resume noise when track paused
            print("Track paused")

    def toggle_track(self):
        """Toggle play/pause state of the track."""
        if self.track_playing:
            self.pause_track()
        else:
            self.play_track()

    def reset_track(self):
        """Reset track to beginning."""
        if self.track_channel:
            self.track_channel.stop()
            self.track_paused = False
            self.track_playing = False

        # Restart from beginning
        self.play_track()
        print("Track reset to start")

    def stop_track(self):
        """Stop the track completely."""
        if self.track_channel:
            self.track_channel.stop()
            self.track_playing = False
            self.track_paused = False
            self.play_noise()
            print("Track stopped")

    def set_track_volume(self, volume: float):
        """Set track volume (0.0 to 1.0)."""
        if self.track_channel:
            self.track_channel.set_volume(max(0.0, min(1.0, volume)))

    def set_noise_volume(self, volume: float):
        """Set noise volume (0.0 to 1.0)."""
        if self.noise_channel:
            self.noise_channel.set_volume(max(0.0, min(1.0, volume)))

    def cleanup(self):
        """Clean up pygame mixer."""
        if self.initialized:
            pygame.mixer.quit()
            self.initialized = False
            print("Media controller cleaned up")

