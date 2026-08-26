# IMU Sensor-Fusion Node

An ESP32 + MPU6050 streaming real accelerometer/gyroscope data over a framed, CRC-checked USB serial protocol into a C++ host app that fuses the two sensors with a Kalman filter and times its own pipeline the way a low-latency trading engine times itself.

## The two halves, and why they need their own protocol

The ESP32 samples the MPU6050 at a fixed 200Hz and writes a 36-byte, CRC-checked, magic-marker-framed packet per sample straight to USB serial. A UART link is just a raw byte stream with no built-in message boundaries, so this project designs its own framing and a resync algorithm that recovers cleanly from a dropped or corrupted byte.

The C++ host app reads that byte stream (`FrameParser`, unit-tested with synthetic and deliberately-corrupted data), fuses the accelerometer and gyroscope readings into one stable tilt estimate (a 2-state Kalman filter that also identifies the gyro's own hidden bias, beating a simpler complementary filter by roughly 3x on this project's test data), and reports nanosecond latency percentiles for the parse-to-fused-output pipeline it controls.

## What's verified for real, and what needs a bench

The Python twin of the wire codec and fusion math (`sim/imu_sim`) and the C++ host app (`host/`) are both fully tested without any hardware attached: the host app was verified via Docker + Ubuntu's g++13/CMake 3.28, since no native C++ compiler was available in the environment this was built in, including cross-compiling the Windows-specific serial code with mingw-w64 to confirm it at least compiles against real Windows headers. The ESP32 firmware compiles clean via PlatformIO; whether a real MPU6050 streams the expected frames once wired up still needs a physical board, which is what `wiring/WIRING.md`'s staged bring-up order is for.

## Run it

```powershell
# Python side, no hardware, no C++ compiler needed
cd sim
pip install -r requirements.txt
python -m pytest -q                             # 25 passed

# C++ host app, needs a C++20 compiler + CMake 3.20+
cd ../host
cmake -B build -DCMAKE_BUILD_TYPE=Release .
cmake --build build -j
ctest --test-dir build --output-on-failure       # 20 passed

# Firmware, needs PlatformIO (install in its own venv)
cd ../firmware/imu_node
pio run
pio run --target upload && pio device monitor
```

Run it live once the firmware is flashed:

```powershell
cd host/build
./imu_host --port COM5 --csv-out session.csv
```

Or replay a synthetic session with zero hardware attached:

```python
import sys; sys.path.insert(0, "../sim")
from imu_sim.synth import generate_motion, rows_to_frames, frames_to_bytes
data = frames_to_bytes(rows_to_frames(generate_motion(duration_s=10.0, seed=42)))
open("session.bin", "wb").write(data)
```

```powershell
./imu_host --replay path\to\session.bin --csv-out out.csv
```

## What this demonstrates

Designing a real wire protocol for an unreliable byte stream instead of consuming a ready-made one, sensor fusion and Kalman filtering with the actual math derived and tested rather than a black-box library call, and a low-latency measurement discipline (percentile latency reporting) applied to physical sensor data.

## Repo layout

```
imu-sensor-fusion-node/
├── firmware/imu_node/   the real PlatformIO ESP32 project
├── host/                 the real C++20/CMake/GoogleTest host app
├── sim/imu_sim/          Python twin of the wire codec + fusion math
├── webapp/               replay/analysis dashboard
├── wiring/WIRING.md      BOM, pin table, staged bring-up order
└── hosting/               CI (firmware + C++ + Python) and dashboard hosting
```

## Requirements

Python 3.10+ for `sim` and most of the tooling. A C++20 compiler and CMake 3.20+ for the host app. [PlatformIO](https://platformio.org/) (in its own virtual environment) for the firmware. For a real build: an ESP32 DevKit, an MPU6050 breakout, and a USB cable, roughly $15.

## Hosting

See `hosting/HOSTING_GUIDE.md`: a CI gate covering all three languages (firmware compile check, C++ tests, Python tests), and a replay dashboard on Streamlit Cloud.
