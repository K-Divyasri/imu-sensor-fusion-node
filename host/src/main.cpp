// IMU Sensor-Fusion Node -- C++ host app entry point.
//
// Two modes, same pipeline either way:
//   --replay <frames.bin>   reads a captured/synthetic session from disk --
//                           runs the exact same FrameParser -> fusion ->
//                           LatencyTracker pipeline with zero hardware
//                           involved, which is how this file is verified
//                           in this project's own CI/Docker environment.
//   --port COM5             reads live from a real serial port -- the
//                           actual point of this project, verify this
//                           mode yourself with a real ESP32 plugged in
//                           (see ../wiring/WIRING.md).
//
// Deliberately mirrors the existing C++ order-book engine's shape: a
// receiver hands raw bytes/structs to a processing pipeline, a
// LatencyTracker times the interesting part of that pipeline and reports
// percentiles at the end -- same discipline, applied to physical sensor
// data instead of market data.

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "core/frame.hpp"
#include "core/types.hpp"
#include "fusion/fusion.hpp"
#include "network/frame_parser.hpp"
#include "network/serial_port.hpp"
#include "utils/latency_tracker.hpp"

using namespace imu;

namespace {

struct Args {
  std::string replayPath;
  std::string portName;
  uint32_t baud = 921600;
  std::string csvOutPath;
  bool useKalman = true;
};

Args parseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    if (a == "--replay" && i + 1 < argc) {
      args.replayPath = argv[++i];
    } else if (a == "--port" && i + 1 < argc) {
      args.portName = argv[++i];
    } else if (a == "--baud" && i + 1 < argc) {
      args.baud = static_cast<uint32_t>(std::stoul(argv[++i]));
    } else if (a == "--csv-out" && i + 1 < argc) {
      args.csvOutPath = argv[++i];
    } else if (a == "--complementary") {
      args.useKalman = false;
    }
  }
  return args;
}

void printUsage(const char* prog) {
  std::cerr << "Usage:\n"
            << "  " << prog << " --replay <frames.bin> [--csv-out out.csv] [--complementary]\n"
            << "  " << prog << " --port COM5 [--baud 921600] [--csv-out out.csv] [--complementary]\n";
}

void printLatencyStats(const char* name, const LatencyStats& s) {
  std::cout << name << ": count=" << s.count
            << " min=" << s.min_ns << "ns"
            << " p50=" << s.p50_ns << "ns"
            << " p95=" << s.p95_ns << "ns"
            << " p99=" << s.p99_ns << "ns"
            << " p999=" << s.p999_ns << "ns"
            << " max=" << s.max_ns << "ns\n";
}

}  // namespace

int main(int argc, char** argv) {
  Args args = parseArgs(argc, argv);
  if (args.replayPath.empty() && args.portName.empty()) {
    printUsage(argv[0]);
    return 1;
  }

  FrameParser parser;
  ComplementaryFilter compFilter;
  KalmanFilter1D kalmanFilter;
  LatencyTracker processingLatency;    // parse-to-fused-output, host clock only
  LatencyTracker deviceIntervalJitter;  // the ESP32's OWN reported sample spacing

  std::ofstream csv;
  if (!args.csvOutPath.empty()) {
    csv.open(args.csvOutPath);
    csv << "seq,device_us,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,fused_pitch_deg\n";
  }

  uint32_t lastDeviceUs = 0;
  bool haveLastDeviceUs = false;
  uint64_t frameCount = 0;

  // Fixed nominal dt, matching firmware/imu_node/include/config.h's
  // SAMPLE_RATE_HZ, rather than a per-frame measured dt from device_us
  // deltas -- see knowledge/05_latency_and_jitter_measurement.md for why
  // that distinction matters (a measured dt would silently absorb real
  // sample-rate jitter into the fusion math instead of surfacing it as a
  // number worth looking at).
  const double dt = 1.0 / 200.0;

  auto processBuffer = [&](const uint8_t* data, size_t length) {
    auto frames = parser.feed(data, length);
    for (const auto& frame : frames) {
      const uint64_t startNs = now_ns();

      const double accelAngle = accelToPitchDeg(frame.accel_y, frame.accel_z, frame.accel_x);
      const double fused = args.useKalman ? kalmanFilter.update(accelAngle, frame.gyro_y, dt)
                                           : compFilter.update(accelAngle, frame.gyro_y, dt);

      processingLatency.record(now_ns() - startNs);

      if (haveLastDeviceUs) {
        // Unsigned subtraction wraps correctly even across a device_us
        // rollover (~71 minutes of uptime) -- modular arithmetic gives the
        // right small positive interval either way.
        const uint32_t intervalUs = frame.device_us - lastDeviceUs;
        deviceIntervalJitter.record(static_cast<uint64_t>(intervalUs) * 1000);  // us -> ns
      }
      lastDeviceUs = frame.device_us;
      haveLastDeviceUs = true;

      if (csv.is_open()) {
        csv << frame.seq << ',' << frame.device_us << ','
            << frame.accel_x << ',' << frame.accel_y << ',' << frame.accel_z << ','
            << frame.gyro_x << ',' << frame.gyro_y << ',' << frame.gyro_z << ','
            << fused << '\n';
      }

      ++frameCount;
      if (frameCount % 200 == 0) {  // roughly once a second at 200Hz
        std::cout << "seq=" << frame.seq << " pitch=" << fused << " deg\n";
      }
    }
  };

  if (!args.replayPath.empty()) {
    std::ifstream in(args.replayPath, std::ios::binary);
    if (!in) {
      std::cerr << "could not open replay file: " << args.replayPath << "\n";
      return 1;
    }
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::cout << "replaying " << data.size() << " bytes from " << args.replayPath << "\n";
    processBuffer(data.data(), data.size());
  } else {
    SerialPort port;
    if (!port.open(args.portName, args.baud)) {
      std::cerr << "could not open serial port: " << args.portName << "\n";
      return 1;
    }
    std::cout << "connected to " << args.portName << " at " << args.baud << " baud\n";
    uint8_t buf[4096];
    while (true) {
      int n = port.read(buf, sizeof(buf));
      if (n > 0) {
        processBuffer(buf, static_cast<size_t>(n));
      } else if (n < 0) {
        std::cerr << "serial read error\n";
        break;
      }
      // n == 0 is just a read timeout with nothing new -- loop and try again.
    }
  }

  std::cout << "\n--- summary (" << frameCount << " frames) ---\n";
  printLatencyStats("processing latency (parse+fuse, host clock)", processingLatency.computeStats());
  printLatencyStats("device-reported sample interval", deviceIntervalJitter.computeStats());
  std::cout << "resync bytes discarded: " << parser.resyncByteCount() << "\n";

  return 0;
}
