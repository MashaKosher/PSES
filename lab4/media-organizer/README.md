# Media Organizer

IR Remote controlled desktop application for displaying GIFs and playing audio.

## Features

- 4 corner GIF displays controlled by IR remote
- "No signal" screen when all corners are off
- MP3 audio playback with play/pause/reset
- Background static noise when track is not playing

## Installation

```bash
cd media-organizer
pip install -r requirements.txt
```

## Assets Setup

Place the following files in the `assets/` folder:

| File | Description |
|------|-------------|
| `corner1.gif` | Top-left corner GIF |
| `corner2.gif` | Top-right corner GIF |
| `corner3.gif` | Bottom-left corner GIF |
| `corner4.gif` | Bottom-right corner GIF |
| `no_signal.gif` | "No programs" screen GIF |
| `track.mp3` | Main audio track |
| `noise.mp3` | Background interference sound |

## Usage

### With IR Remote (STM32 connected)

```bash
python main.py
# or specify port
python main.py --port COM3
python main.py --port /dev/ttyUSB0
```

### Testing without hardware

```bash
python main.py --mock
```

## IR Remote Control Mapping

| Raw Code | Action |
|----------|--------|
| `0x20DF8877` | Toggle GIF 1 (top-left) |
| `0x20DF48B7` | Toggle GIF 2 (top-right) |
| `0x20DFC837` | Toggle GIF 3 (bottom-left) |
| `0x20DF28D7` | Toggle GIF 4 (bottom-right) |
| `0x20DFA857` | Play/Pause track |
| `0x20DF6897` | Reset track to start |

## Keyboard Shortcuts

For testing without IR remote:

| Key | Action |
|-----|--------|
| `1-4` | Toggle corner GIFs |
| `Space` | Play/Pause track |
| `R` | Reset track |
| `Esc` | Quit |

## Architecture

- `main.py` - Main application with tkinter GUI
- `ir_receiver.py` - Serial communication with STM32
- `media_controller.py` - Audio playback using pygame

