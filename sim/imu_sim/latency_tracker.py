"""
Same technique as the existing C++ order-book engine's LatencyTracker
(see knowledge/05_latency_and_jitter_measurement.md for the full story of
where this pattern comes from) -- record raw nanosecond samples, then
compute count/min/max/mean plus p50/p95/p99/p999 percentiles by sorting
and indexing, no external stats library. Reimplemented here in Python
(and again, independently, in
../../host/include/utils/latency_tracker.hpp in C++) rather than shared
code, matching this whole repo's house convention of technique-reuse
over cross-project imports.
"""

from dataclasses import dataclass
from typing import List


@dataclass
class LatencyStats:
    count: int
    min_ns: int
    max_ns: int
    mean_ns: float
    p50_ns: int
    p95_ns: int
    p99_ns: int
    p999_ns: int


class LatencyTracker:
    def __init__(self):
        self._samples: List[int] = []

    def record(self, nanoseconds: int) -> None:
        self._samples.append(nanoseconds)

    def compute_stats(self) -> LatencyStats:
        if not self._samples:
            return LatencyStats(0, 0, 0, 0.0, 0, 0, 0, 0)
        ordered = sorted(self._samples)
        n = len(ordered)

        def percentile(p: float) -> int:
            index = min(n - 1, int(n * p))
            return ordered[index]

        return LatencyStats(
            count=n,
            min_ns=ordered[0],
            max_ns=ordered[-1],
            mean_ns=sum(ordered) / n,
            p50_ns=percentile(0.50),
            p95_ns=percentile(0.95),
            p99_ns=percentile(0.99),
            p999_ns=percentile(0.999),
        )
