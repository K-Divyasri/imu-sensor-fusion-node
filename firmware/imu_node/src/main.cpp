// IMU Sensor-Fusion Node -- firmware entry point.
//
// Samples the MPU6050 at a fixed 200Hz cadence and writes one framed,
// CRC-checked WireFrame per sample out over USB-serial to the host C++
// app. No FreeRTOS here -- unlike Project 13's climate monitor, this
// project's differentiator is the wire protocol design (UART framing +
// resync), the fusion math, and cross-language latency measurement, not
// task concurrency. A single fixed-rate loop is the right amount of
// firmware for what this project is actually teaching.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "config.h"
#include "frame.h"

Adafruit_MPU6050 mpu;
uint32_t g_seq = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("\n[imu_node] booting...");

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  if (!mpu.begin()) {
    Serial.println("[imu_node] MPU6050 not found -- check wiring/address. Halting.");
    Serial.println("[imu_node] Run labs/01_i2c_bus_scanner_mpu6050 to confirm the address (0x68 or 0x69).");
    while (true) { delay(1000); }
  }

  // A modest range is plenty for the gentle hand-motion this project is
  // meant to demonstrate -- see knowledge/01 for what these ranges trade off.
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  Serial.println("[imu_node] MPU6050 ready. Streaming frames...");
}

void sendOneFrame() {
  sensors_event_t accelEvent, gyroEvent, tempEvent;
  mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

  WireFrame frame;
  frame.seq = g_seq++;
  frame.device_us = micros();
  // Adafruit's unified sensor interface returns SI units (m/s^2, rad/s) --
  // convert to g and deg/s to match the wire format's documented units.
  frame.accel_x = accelEvent.acceleration.x / 9.80665f;
  frame.accel_y = accelEvent.acceleration.y / 9.80665f;
  frame.accel_z = accelEvent.acceleration.z / 9.80665f;
  frame.gyro_x = gyroEvent.gyro.x * (180.0f / PI);
  frame.gyro_y = gyroEvent.gyro.y * (180.0f / PI);
  frame.gyro_z = gyroEvent.gyro.z * (180.0f / PI);

  encodeFrame(frame);  // fills in magic + crc
  Serial.write(reinterpret_cast<const uint8_t *>(&frame), sizeof(frame));
}

void loop() {
  // Non-blocking fixed-rate scheduler: compare against a running
  // "next sample due" timestamp instead of delay()-ing, so the sample
  // period stays accurate regardless of how long sendOneFrame() itself
  // takes. The signed-subtraction comparison correctly handles micros()
  // wrapping around back to 0 after ~71 minutes.
  static uint32_t next_sample_us = micros();
  uint32_t now = micros();
  if ((int32_t)(now - next_sample_us) >= 0) {
    next_sample_us += SAMPLE_PERIOD_US;
    sendOneFrame();
  }
}
