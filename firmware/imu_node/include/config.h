#pragma once

// I2C wiring -- see ../../wiring/WIRING.md for the pin table.
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

// Sample rate: 200Hz, matching build_from_scratch/sim/imu_sim/synth.py's
// default so the same synthetic-motion analysis in the notebooks applies
// directly to real captured data too.
#define SAMPLE_RATE_HZ 200
#define SAMPLE_PERIOD_US (1000000UL / SAMPLE_RATE_HZ)

// High baud rate matters here: at 200Hz and 36 bytes/frame, the link needs
// to sustain 7200 bytes/sec minimum with real headroom, or frames start
// queuing up in the UART's TX buffer and lag behind real time.
#define SERIAL_BAUD 921600
