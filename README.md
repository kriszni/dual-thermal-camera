# Dual Thermal Camera Recorder

A two-camera thermal imaging rig built from a **FLIR Lepton 3.x** and a **Melexis MLX90640**, each driven by its own Seeed XIAO ESP32-S3. Both boards stream raw frames over USB serial to a Python app that shows the two feeds live side by side and records them together to video and per-pixel CSV.

I built this to capture synchronized footage from a high-resolution radiometric sensor (Lepton, 160×120) and a low-cost thermopile array (MLX90640, 32×24), so the two can be compared frame by frame.

## How it works

```
 FLIR Lepton 3.x ──SPI/I²C──> XIAO ESP32-S3 ──USB──┐
                                                    ├──> dual_thermal_recorder.py ──> live view
 MLX90640 ────────I²C──────> XIAO ESP32-S3 ──USB──┘                                └─> .mp4 + .csv
```

Each board runs a small firmware that does one job: read a frame from its sensor and write it to serial in a simple binary packet. The Python side runs one reader thread per camera, parses the packets, colorizes the frames, and handles recording.

### Serial protocols

| Camera   | Packet layout | Size |
|----------|---------------|------|
| Lepton   | `DE AD BE EF` header + 160×120 big-endian `uint16` raw values | 38,404 bytes |
| MLX90640 | `AA 55 A5 5A` magic + `uint32` frame ID + 768 `float32` temps (°C) + `uint16` checksum | 3,082 bytes |

The MLX packet carries a checksum (sum of payload bytes, mod 2¹⁶), so corrupted frames are dropped and the reader resyncs on the next magic word. The MLX firmware also accepts `S` and `P` over serial to start and pause streaming.

## Features

- **Live dual view** with frame count and bytes received drawn on each window
- **Synchronized recording**: one key starts and stops both cameras at once
- **Video + raw data output**: an `.mp4` of the colorized feed and a `.csv` with a timestamp and every pixel value for each frame
- **Measured frame rate**: the saved video uses the actual capture rate, not a fixed guess
- **MLX90640 image cleanup**
  - Flat-field calibration: point at a uniform surface, press `c`, and 30 frames are averaged into a per-pixel offset map that removes fixed-pattern noise
  - Temporal smoothing (exponential moving average) plus a light Gaussian blur
  - Percentile-based contrast stretch with a minimum 3 °C span, so a flat scene doesn't turn into amplified noise
- **Protocol auto-detection** for the MLX port: works with this repo's binary firmware or a plain CSV-over-serial firmware
- **Built-in diagnostics**: when a feed isn't working, the window says why: silent port, wrong firmware flashed to the wrong board, text instead of frames, or checksum failures

## Hardware

| Part | Notes |
|------|-------|
| 2× Seeed XIAO ESP32-S3 | one per camera |
| FLIR Lepton 3.x on a breakout board | SPI for video (VoSPI), I²C for control |
| MLX90640 breakout | I²C, run at 800 kHz |

**Wiring**

| Signal | Lepton board | MLX board |
|--------|--------------|-----------|
| SDA | D4 | D4 |
| SCL | D5 | D5 |
| SPI CS | D7 | — |
| SPI SCK | D8 | — |
| SPI MISO | D9 | — |

## Repository layout

```
├── src/
│   ├── lepton.cpp              # Lepton firmware: VoSPI frame capture -> serial
│   └── mlx.cpp                 # MLX90640 firmware: temperature frames -> serial
├── dual_thermal_recorder.py    # live viewer, recorder, diagnostics
├── platformio.ini              # build envs: `lepton` and `mlx`
└── requirements.txt
```

## Getting started

### 1. Flash the firmware

Uses [PlatformIO](https://platformio.org/). Set `upload_port` in `platformio.ini` to match your boards, then:

```bash
pio run -e lepton -t upload
```

```bash
pio run -e mlx -t upload
```

### 2. Run the recorder

```bash
python3 -m venv venv && source venv/bin/activate
```

```bash
pip install -r requirements.txt
```

```bash
python dual_thermal_recorder.py
```

On startup the script prints every serial port it can see. If they don't match `MLX_PORT` and `LEP_PORT` at the top of the script, edit those two lines.

### Controls

| Key | Action |
|-----|--------|
| `r` | Start / stop recording both cameras |
| `c` | Flat-field calibrate the MLX90640 (aim at a blank wall and hold still) |
| `q` / `Esc` | Quit (saves any recording in progress) |

Recordings go to `~/Downloads` as `mlx_MMDDYYYY.mp4/.csv` and `lep_MMDDYYYY.mp4/.csv`. If a file with today's date already exists, `_2`, `_3` and so on are added.

## Things I ran into

- **Telling the boards apart.** The two XIAOs are identical and their USB port names can swap. The Lepton reader keeps a rolling buffer of what it receives, so when it finds MLX magic bytes it can tell you that board is running the wrong firmware.
- **Fixed-pattern noise on the MLX90640.** At 32×24 the raw image is dominated by per-pixel offsets. A one-shot flat-field calibration plus light temporal smoothing made the output usable.
- **Partial frames from the Lepton.** VoSPI drops sync often, so the firmware only sends a frame once the library has assembled a complete one, and the host resyncs on the header.

## Built with

- [ducky64/arduino-lepton](https://github.com/ducky64/arduino-lepton) for the Lepton driver
- [Adafruit MLX90640](https://github.com/adafruit/Adafruit_MLX90640) library
- Python: NumPy, OpenCV, pySerial
