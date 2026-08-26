#include <gtest/gtest.h>
#include "utils/latency_tracker.hpp"

using namespace imu;

TEST(LatencyTracker, EmptyTrackerReturnsZeroedStats) {
  LatencyTracker tracker;
  auto stats = tracker.computeStats();
  EXPECT_EQ(stats.count, 0u);
  EXPECT_EQ(stats.min_ns, 0u);
  EXPECT_EQ(stats.max_ns, 0u);
}

TEST(LatencyTracker, StatsOnKnownSamples) {
  LatencyTracker tracker;
  for (uint64_t ns : {100, 200, 300, 400, 500, 600, 700, 800, 900, 1000}) {
    tracker.record(ns);
  }
  auto stats = tracker.computeStats();
  EXPECT_EQ(stats.count, 10u);
  EXPECT_EQ(stats.min_ns, 100u);
  EXPECT_EQ(stats.max_ns, 1000u);
  EXPECT_DOUBLE_EQ(stats.mean_ns, 550.0);
  EXPECT_EQ(stats.p50_ns, 600u);  // index int(10*0.5)=5 -> sorted[5] == 600
}

TEST(LatencyTracker, PercentilesTrackASkewedTail) {
  LatencyTracker tracker;
  for (int i = 0; i < 999; ++i) tracker.record(100);
  tracker.record(50000);  // one extreme outlier
  auto stats = tracker.computeStats();
  EXPECT_EQ(stats.p50_ns, 100u);
  EXPECT_EQ(stats.p999_ns, 50000u);  // the tail shows up exactly where it should, nowhere else
}
