#!/usr/bin/env python3
"""
multimodal_sensor_synchronizer.py
=================================
High-Precision Multimodal Sensor & Telemetry ApproximateTime Synchronizer.
Role: Member 3 - ROS 2 & System Integration Engineer.

Responsibilities:
 1. Match asynchronous message streams from Member 4 (SLAM Odometry @ 30Hz),
    Member 1 (AI Vision-Language Detections @ 10Hz), and Member 2 (IMU @ 250Hz).
 2. Bounded maximum queue latency window (slop = 0.015s = 15ms).
 3. Output unified, timestamp-aligned spatial perception tuples.
"""

import time
import math
from typing import Dict, Any, List, Optional, Tuple


class ApproximateTimeSyncBuffer:
    """Mathematical ApproximateTime synchronizer with sliding queue window."""

    def __init__(self, slop_seconds: float = 0.015, queue_size: int = 30):
        self.slop = slop_seconds
        self.queue_size = queue_size
        self.odom_queue: List[Tuple[float, Dict[str, Any]]] = []
        self.detection_queue: List[Tuple[float, Dict[str, Any]]] = []
        self.synchronized_tuples_count = 0
        self.sync_latencies_ms: List[float] = []

    def add_odometry(self, timestamp: float, data: Dict[str, Any]):
        self.odom_queue.append((timestamp, data))
        if len(self.odom_queue) > self.queue_size:
            self.odom_queue.pop(0)

    def add_detection(self, timestamp: float, data: Dict[str, Any]):
        self.detection_queue.append((timestamp, data))
        if len(self.detection_queue) > self.queue_size:
            self.detection_queue.pop(0)

    def synchronize_next(self) -> Optional[Dict[str, Any]]:
        """Finds closest temporal match between SLAM odometry and AI detection."""
        if not self.odom_queue or not self.detection_queue:
            return None

        best_match = None
        min_dt = float('inf')
        matched_odom_idx = -1
        matched_det_idx = -1

        for o_idx, (t_odom, d_odom) in enumerate(self.odom_queue):
            for d_idx, (t_det, d_det) in enumerate(self.detection_queue):
                dt = abs(t_odom - t_det)
                if dt <= self.slop and dt < min_dt:
                    min_dt = dt
                    matched_odom_idx = o_idx
                    matched_det_idx = d_idx
                    best_match = {
                        "sync_timestamp": (t_odom + t_det) / 2.0,
                        "time_skew_ms": round(dt * 1000.0, 2),
                        "odometry": d_odom,
                        "detection": d_det
                    }

        if best_match:
            # Pop matched and older messages
            self.odom_queue = self.odom_queue[matched_odom_idx + 1:]
            self.detection_queue = self.detection_queue[matched_det_idx + 1:]
            self.synchronized_tuples_count += 1
            self.sync_latencies_ms.append(best_match["time_skew_ms"])
            if len(self.sync_latencies_ms) > 100:
                self.sync_latencies_ms.pop(0)
            return best_match

        return None


def main():
    print("[Member 3 Sync] Testing Multimodal ApproximateTime Synchronizer...")
    sync = ApproximateTimeSyncBuffer(slop_seconds=0.015)
    now = time.time()

    sync.add_odometry(now, {"pos": [1.0, 2.0, 2.5]})
    sync.add_detection(now + 0.003, {"class": "medical_crate", "confidence": 0.95})

    matched = sync.synchronize_next()
    print(f"Matched Perception Tuple: {matched}")


if __name__ == '__main__':
    main()
