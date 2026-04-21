from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import csv
import os
from typing import Dict, List, Optional


@dataclass(frozen=True)
class PathPerformanceSample:
    timestamp_utc: str
    distance_m: float
    estimated_time_s: float
    planning_time_ms: float
    waypoint_count: int


class PathPerformanceTracker:
    def __init__(self, csv_path: str = '') -> None:
        self._history: List[PathPerformanceSample] = []
        self._csv_path = csv_path.strip()

        if self._csv_path:
            os.makedirs(os.path.dirname(self._csv_path), exist_ok=True)
            if not os.path.exists(self._csv_path):
                with open(self._csv_path, 'w', newline='', encoding='utf-8') as csv_file:
                    writer = csv.writer(csv_file)
                    writer.writerow(
                        [
                            'timestamp_utc',
                            'distance_m',
                            'estimated_time_s',
                            'planning_time_ms',
                            'waypoint_count',
                            'previous_distance_m',
                            'previous_time_s',
                            'distance_improvement_pct',
                            'time_improvement_pct',
                        ]
                    )

    @staticmethod
    def now_iso_utc() -> str:
        return datetime.now(timezone.utc).isoformat()

    def record(self, sample: PathPerformanceSample) -> Dict[str, Optional[float]]:
        previous = self._history[-1] if self._history else None

        distance_improvement_pct = self._improvement_pct(
            previous.distance_m if previous is not None else None,
            sample.distance_m,
        )
        time_improvement_pct = self._improvement_pct(
            previous.estimated_time_s if previous is not None else None,
            sample.estimated_time_s,
        )

        self._history.append(sample)

        if self._csv_path:
            with open(self._csv_path, 'a', newline='', encoding='utf-8') as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(
                    [
                        sample.timestamp_utc,
                        f'{sample.distance_m:.6f}',
                        f'{sample.estimated_time_s:.6f}',
                        f'{sample.planning_time_ms:.3f}',
                        sample.waypoint_count,
                        '' if previous is None else f'{previous.distance_m:.6f}',
                        '' if previous is None else f'{previous.estimated_time_s:.6f}',
                        '' if distance_improvement_pct is None else f'{distance_improvement_pct:.4f}',
                        '' if time_improvement_pct is None else f'{time_improvement_pct:.4f}',
                    ]
                )

        return {
            'previous_distance_m': None if previous is None else previous.distance_m,
            'current_distance_m': sample.distance_m,
            'distance_improvement_pct': distance_improvement_pct,
            'previous_time_s': None if previous is None else previous.estimated_time_s,
            'current_time_s': sample.estimated_time_s,
            'time_improvement_pct': time_improvement_pct,
        }

    @staticmethod
    def _improvement_pct(previous: Optional[float], current: float) -> Optional[float]:
        if previous is None or previous <= 1e-9:
            return None
        return ((previous - current) / previous) * 100.0

    @staticmethod
    def format_summary(
        comparison: Dict[str, Optional[float]],
        planning_time_ms: float,
        expanded_nodes: int,
    ) -> str:
        previous_distance = comparison['previous_distance_m']
        current_distance = comparison['current_distance_m']
        distance_improvement = comparison['distance_improvement_pct']

        previous_time = comparison['previous_time_s']
        current_time = comparison['current_time_s']
        time_improvement = comparison['time_improvement_pct']

        if previous_distance is None or previous_time is None:
            return (
                f'Baseline plan | Distance: {current_distance:.2f}m | '
                f'Estimated Time: {current_time:.2f}s | '
                f'Planning Time: {planning_time_ms:.2f}ms | '
                f'Expanded Nodes: {expanded_nodes}'
            )

        return (
            f'Previous distance: {previous_distance:.2f}m | '
            f'Current distance: {current_distance:.2f}m | '
            f'Distance improvement: {distance_improvement:.2f}% | '
            f'Previous time: {previous_time:.2f}s | '
            f'Current time: {current_time:.2f}s | '
            f'Time improvement: {time_improvement:.2f}% | '
            f'Planning Time: {planning_time_ms:.2f}ms | '
            f'Expanded Nodes: {expanded_nodes}'
        )
