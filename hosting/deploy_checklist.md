# Before you share either link

- [ ] `build_from_scratch/firmware/imu_node/.pio/` and
      `build_from_scratch/host/build/` are **not** in the repo.
- [ ] The GitHub Actions badge is green — all three jobs (firmware
      compile, host app build+test, Python tests) passing.
- [ ] You have a real, recorded `--csv-out` session from an actual bench
      run, and you've uploaded it to the live dashboard yourself at least
      once — not just relied on the synthetic generator.
- [ ] You have a real photo or short video of the breadboard build (just
      the ESP32 + MPU6050 + four wires — this is a small build, but it
      still has to be a REAL one, not a Wokwi screenshot).
- [ ] You can state, out loud without notes: why UART needs framing that
      UDP didn't, what the resync algorithm does on a bad byte, and what
      the Kalman filter's bias state actually buys you over the
      complementary filter.
- [ ] You know which specific piece of this project was verified only by
      cross-compiling against Windows headers (not run) — the Windows
      `SerialPort` backend — and can say so plainly if asked, rather than
      implying everything was tested identically.
