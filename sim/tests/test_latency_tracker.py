import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from imu_sim.latency_tracker import LatencyTracker


def test_empty_tracker_returns_zeroed_stats():
    stats = LatencyTracker().compute_stats()
    assert stats.count == 0
    assert stats.min_ns == 0
    assert stats.max_ns == 0


def test_stats_on_known_samples():
    tracker = LatencyTracker()
    for ns in [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000]:
        tracker.record(ns)
    stats = tracker.compute_stats()
    assert stats.count == 10
    assert stats.min_ns == 100
    assert stats.max_ns == 1000
    assert stats.mean_ns == 550.0
    assert stats.p50_ns == 600  # index int(10*0.5)=5 -> sorted[5] == 600


def test_percentiles_track_a_skewed_tail():
    tracker = LatencyTracker()
    for _ in range(999):
        tracker.record(100)
    tracker.record(50_000)  # one extreme outlier
    stats = tracker.compute_stats()
    assert stats.p50_ns == 100
    assert stats.p999_ns == 50_000  # the tail shows up exactly where it should, nowhere else
