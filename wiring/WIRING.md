# Wiring — IMU Sensor-Fusion Node

## Bill of materials (~$15)

| Part | Notes |
|---|---|
| ESP32 DevKit (ESP32-WROOM-32) | Reused from earlier projects on this roadmap if you already have one |
| MPU6050 breakout, I2C version | 6-axis: 3-axis accelerometer + 3-axis gyroscope, one chip. GY-521 is the common breakout board name |
| USB cable (data-capable, not charge-only) | This IS the "UART/USB link" in the project brief — the ESP32's USB-to-serial chip carries the framed data straight to the host, no separate wireless link needed |
| Breadboard + jumper wires | Just 4 wires needed (power + I2C) |

## Pin table

| Signal | ESP32 GPIO | Goes to |
|---|---|---|
| I2C SDA | GPIO21 | MPU6050 SDA |
| I2C SCL | GPIO22 | MPU6050 SCL |
| 3.3V | 3V3 | MPU6050 VCC |
| GND | GND | MPU6050 GND |

That's the entire physical build — the interesting engineering here is the
protocol and the software on both ends, not the wiring.

## Bring-up order

1. **Power + I2C only.** Wire SDA/SCL/VCC/GND. Flash
   `labs/01_i2c_bus_scanner_mpu6050` and confirm one address shows up —
   `0x68` (the common default) or `0x69` if the breakout's `AD0` pin is
   tied high.
2. **Raw sensor read.** Flash `labs/02_mpu6050_raw_read_only` and confirm
   plausible accel/gyro numbers print over serial — accel roughly summing
   to ~1g total magnitude at rest, gyro near zero at rest (small non-zero
   values from bias/noise are normal, large ones are not).
3. **Raw framed output.** Flash `labs/03_uart_raw_frame_transmit` and
   confirm you can see structured binary frames arriving (not readable
   text — that's expected, it's a binary protocol; a serial terminal will
   show garbage characters, which is the correct symptom, not a bug).
4. **The full firmware.** Only once 1-3 all work, flash
   `firmware/imu_node`. Find the ESP32's COM port in Windows Device
   Manager (Ports (COM & LPT)) before running the host app against it.
5. **The real host app.** Build `host/` (see its own section in
   `../README.md`) and run it with `--port COMx` pointed at the ESP32's
   actual port. You should see a live stream of fused pitch angles that
   track the device's real physical tilt as you move it by hand.

## What "done" looks like on the bench

Hold the board flat, then slowly tip it forward and back by hand. Watch
the host app's printed pitch angle track your motion in real time, settle
back near zero when you set it back down, and — check the printed
`resync bytes discarded` count at the end — stay at zero (or very close
to it) over a normal few-minute session on a short, clean USB cable. A
climbing resync count during a live run is itself a useful diagnostic:
it usually means USB cable/port quality, not a bug in the code.
