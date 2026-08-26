#pragma once
#include <cstdint>
#include <vector>
#include <algorithm>
#include <numeric>

// Same technique as the existing C++ order-book engine's LatencyTracker
// (see knowledge/05_latency_and_jitter_measurement.md) -- record raw
// nanosecond samples into a vector, then sort a copy and read off
// percentiles by index. No external stats library, on purpose: the whole
// point is that percentile math this simple doesn't need one.
//
// Independently reimplemented here (and again in
// ../../../sim/imu_sim/latency_tracker.py, Python) rather than shared
// code or a linked library -- matching this repo's house convention of
// technique-reuse over cross-project imports.

namespace imu {

struct LatencyStats {
  size_t count = 0;
  uint64_t min_ns = 0;
  uint64_t max_ns = 0;
  double mean_ns = 0.0;
  uint64_t p50_ns = 0;
  uint64_t p95_ns = 0;
  uint64_t p99_ns = 0;
  uint64_t p999_ns = 0;
};

class LatencyTracker {
 public:
  void record(uint64_t nanoseconds) { samples_.push_back(nanoseconds); }

  LatencyStats computeStats() const {
    LatencyStats stats;
    if (samples_.empty()) return stats;

    std::vector<uint64_t> ordered = samples_;
    std::sort(ordered.begin(), ordered.end());
    const size_t n = ordered.size();

    auto percentile = [&](double p) -> uint64_t {
      size_t index = std::min(n - 1, static_cast<size_t>(static_cast<double>(n) * p));
      return ordered[index];
    };

    stats.count = n;
    stats.min_ns = ordered.front();
    stats.max_ns = ordered.back();
    stats.mean_ns = static_cast<double>(std::accumulate(ordered.begin(), ordered.end(), uint64_t{0})) /
                    static_cast<double>(n);
    stats.p50_ns = percentile(0.50);
    stats.p95_ns = percentile(0.95);
    stats.p99_ns = percentile(0.99);
    stats.p999_ns = percentile(0.999);
    return stats;
  }

  void clear() { samples_.clear(); }
  size_t sampleCount() const { return samples_.size(); }

 private:
  std::vector<uint64_t> samples_;
};

}  // namespace imu
