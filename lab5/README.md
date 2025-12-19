# Lab5 (RTOS): IR Remote Control Decoder

STM32F103RBT6-based IR remote control decoder that captures NEC protocol signals and sends them to a Python application for monitoring — **same functionality as lab4**, but **implemented using RTOS**.

## Hardware Configuration

- **MCU**: STM32F103RBTx
- **IR Receiver**: Connected to **PB10** (EXTI interrupt on rising/falling edges)
- **UART**: USART2 at 115200 baud
  - TX: PA2
  - RX: PA3

## RTOS Design

- **ISR (EXTI PB10)**: captures NEC edge timings into `ir_buffer`. When a full frame is received it sets `ir_data_ready` and **signals the RTOS task**.
- **`IRTask`**: blocks waiting for a signal; when awakened it decodes `ir_buffer` and sends `CODE:...` / `REPEAT` via UART (DMA), same as lab4.

Important note for RTOS + interrupts:
- The EXTI IRQ priority is set to a **lower urgency** value (numerically higher, e.g. 6) so calling RTOS APIs from ISR is safe.

## Project Structure

```
decoder/
├── Core/
│   ├── Inc/
│   │   ├── main.h              # Pin definitions
│   │   ├── stm32f1xx_hal_conf.h # HAL configuration
│   │   └── stm32f1xx_it.h      # Interrupt handlers header
│   └── Src/
│       ├── main.c              # RTOS app + IR decoder task
│       ├── stm32f1xx_hal_msp.c # MSP initialization
│       ├── stm32f1xx_it.c      # Interrupt handlers
│       └── system_stm32f1xx.c  # System initialization
├── Drivers/                     # Symlink to parent project's drivers
├── python/
│   ├── ir_monitor.py           # Python monitoring application
│   └── requirements.txt        # Python dependencies
├── decoder.ioc                  # STM32CubeMX configuration
└── README.md
```

## Enabling RTOS (CubeMX / Keil)

This repo contains the **application-side** RTOS integration (task + ISR signaling). To actually build/run with RTOS you need to enable an RTOS middleware:

1. Open `decoder.ioc` in STM32CubeMX.
2. Enable **FreeRTOS** (recommended) or another RTOS middleware supported by your toolchain.
3. Regenerate the project, keeping user code blocks.

## Protocol

### NEC IR Protocol

The decoder captures NEC protocol signals:
- **Start Pulse**: ~9ms burst + ~4.5ms space
- **Logic 1**: ~562.5µs burst + ~1.6875ms space
- **Logic 0**: ~562.5µs burst + ~562.5µs space
- **Repeat**: ~9ms burst + ~2.25ms space

### UART Output Format

```
CODE:0xXXXXXXXX ADDR:0xXX CMD:0xXX
```

Where:
- `CODE`: Full 32-bit NEC code (address + inverted address + command + inverted command)
- `ADDR`: 8-bit device address
- `CMD`: 8-bit command code

Repeat signals are sent as:
```
REPEAT
```

## Building

1. Open in Keil MDK-ARM or regenerate project files using STM32CubeMX (see RTOS note above)
2. Build and flash to your STM32F103RB board

## Python Monitor

### Installation

```bash
cd python
pip install -r requirements.txt
```

### Usage

```bash
# Auto-detect port
python ir_monitor.py

# Specify port
python ir_monitor.py /dev/ttyUSB0 115200

# Windows
python ir_monitor.py COM3 115200
```

### Output Example

```
[12:34:56.789] 📡 IR Code Received:
    Raw Code:  0x00FF6897
    Address:   0x00 (0)
    Command:   0x68 (104)
    Button:    0
------------------------------------------------------------
[12:34:57.123] ↺ REPEAT
```

## Maintainers

- [Your Team Name]

## License

See LICENSE file in root directory.

