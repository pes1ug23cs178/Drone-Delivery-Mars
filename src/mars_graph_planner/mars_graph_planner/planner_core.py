from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Dict, List, Optional, Tuple

GridIndex = Tuple[int, int, int]
Waypoint3D = Tuple[float, float, float]


@dataclass(frozen=True)
class PlanningBounds3D:
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float


@dataclass(frozen=True)
class PlannerConfig:
    drone_speed: float
    graph_resolution_xy: float
    graph_resolution_z: float
    heuristic_weight: float
    distance_weight: float
    time_weight: float
    allow_diagonal: bool


@dataclass(frozen=True)
class PlannedPath3D:
    waypoints: List[Waypoint3D]
    total_distance: float
    total_estimated_time: float
    expanded_nodes: int


class GraphAStar3D:
    def __init__(self, config: PlannerConfig) -> None:
        self.config = config

    def plan(
        self,
        start: Waypoint3D,
        goal: Waypoint3D,
        bounds: PlanningBounds3D,
    ) -> Optional[PlannedPath3D]:
        if self.config.graph_resolution_xy <= 0.0 or self.config.graph_resolution_z <= 0.0:
            return None

        x_cells = max(1, int(math.ceil((bounds.max_x - bounds.min_x) / self.config.graph_resolution_xy)))
        y_cells = max(1, int(math.ceil((bounds.max_y - bounds.min_y) / self.config.graph_resolution_xy)))
        z_cells = max(1, int(math.ceil((bounds.max_z - bounds.min_z) / self.config.graph_resolution_z)))

        start_idx = self._world_to_grid(start, bounds, x_cells, y_cells, z_cells)
        goal_idx = self._world_to_grid(goal, bounds, x_cells, y_cells, z_cells)

        if start_idx == goal_idx:
            distance = self._distance_between_points(start, goal)
            est_time = distance / max(self.config.drone_speed, 1e-6)
            return PlannedPath3D(
                waypoints=[start, goal],
                total_distance=distance,
                total_estimated_time=est_time,
                expanded_nodes=1,
            )

        open_heap: List[Tuple[float, int, GridIndex]] = []
        tie_break = 0

        g_cost: Dict[GridIndex, float] = {start_idx: 0.0}
        came_from: Dict[GridIndex, GridIndex] = {}
        closed_set = set()

        start_h = self._heuristic_cost(start_idx, goal_idx)
        heapq.heappush(open_heap, (start_h, tie_break, start_idx))

        expanded_nodes = 0
        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in closed_set:
                continue

            expanded_nodes += 1
            if current == goal_idx:
                waypoints = self._reconstruct_world_path(came_from, current, bounds)
                if not waypoints:
                    return None

                # Keep exact endpoints instead of snapped grid endpoints.
                waypoints[0] = start
                waypoints[-1] = goal

                total_distance = self.compute_total_distance(waypoints)
                total_time = total_distance / max(self.config.drone_speed, 1e-6)
                return PlannedPath3D(
                    waypoints=waypoints,
                    total_distance=total_distance,
                    total_estimated_time=total_time,
                    expanded_nodes=expanded_nodes,
                )

            closed_set.add(current)

            for neighbor in self._neighbors(current, x_cells, y_cells, z_cells):
                if neighbor in closed_set:
                    continue

                edge_distance = self._distance_between_indices(current, neighbor)
                edge_time = edge_distance / max(self.config.drone_speed, 1e-6)
                transition_cost = (
                    self.config.distance_weight * edge_distance
                    + self.config.time_weight * edge_time
                )

                tentative_g = g_cost[current] + transition_cost
                if tentative_g >= g_cost.get(neighbor, float('inf')):
                    continue

                came_from[neighbor] = current
                g_cost[neighbor] = tentative_g
                tie_break += 1
                f_cost = tentative_g + self._heuristic_cost(neighbor, goal_idx)
                heapq.heappush(open_heap, (f_cost, tie_break, neighbor))

        return None

    @staticmethod
    def compute_total_distance(waypoints: List[Waypoint3D]) -> float:
        if len(waypoints) < 2:
            return 0.0

        distance = 0.0
        for index in range(1, len(waypoints)):
            prev = waypoints[index - 1]
            curr = waypoints[index]
            dx = curr[0] - prev[0]
            dy = curr[1] - prev[1]
            dz = curr[2] - prev[2]
            distance += math.sqrt((dx * dx) + (dy * dy) + (dz * dz))
        return distance

    def _world_to_grid(
        self,
        point: Waypoint3D,
        bounds: PlanningBounds3D,
        x_cells: int,
        y_cells: int,
        z_cells: int,
    ) -> GridIndex:
        ix = int(round((point[0] - bounds.min_x) / self.config.graph_resolution_xy))
        iy = int(round((point[1] - bounds.min_y) / self.config.graph_resolution_xy))
        iz = int(round((point[2] - bounds.min_z) / self.config.graph_resolution_z))

        ix = min(max(ix, 0), x_cells)
        iy = min(max(iy, 0), y_cells)
        iz = min(max(iz, 0), z_cells)

        return (ix, iy, iz)

    def _grid_to_world(self, index: GridIndex, bounds: PlanningBounds3D) -> Waypoint3D:
        x = bounds.min_x + (index[0] * self.config.graph_resolution_xy)
        y = bounds.min_y + (index[1] * self.config.graph_resolution_xy)
        z = bounds.min_z + (index[2] * self.config.graph_resolution_z)
        return (x, y, z)

    def _neighbors(
        self,
        node: GridIndex,
        x_cells: int,
        y_cells: int,
        z_cells: int,
    ) -> List[GridIndex]:
        if self.config.allow_diagonal:
            offsets = [
                (dx, dy, dz)
                for dx in (-1, 0, 1)
                for dy in (-1, 0, 1)
                for dz in (-1, 0, 1)
                if not (dx == 0 and dy == 0 and dz == 0)
            ]
        else:
            offsets = [
                (-1, 0, 0),
                (1, 0, 0),
                (0, -1, 0),
                (0, 1, 0),
                (0, 0, -1),
                (0, 0, 1),
            ]

        neighbors: List[GridIndex] = []
        for dx, dy, dz in offsets:
            nx = node[0] + dx
            ny = node[1] + dy
            nz = node[2] + dz
            if nx < 0 or ny < 0 or nz < 0:
                continue
            if nx > x_cells or ny > y_cells or nz > z_cells:
                continue
            neighbors.append((nx, ny, nz))
        return neighbors

    def _distance_between_indices(self, a: GridIndex, b: GridIndex) -> float:
        dx = (b[0] - a[0]) * self.config.graph_resolution_xy
        dy = (b[1] - a[1]) * self.config.graph_resolution_xy
        dz = (b[2] - a[2]) * self.config.graph_resolution_z
        return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))

    def _distance_between_points(self, a: Waypoint3D, b: Waypoint3D) -> float:
        dx = b[0] - a[0]
        dy = b[1] - a[1]
        dz = b[2] - a[2]
        return math.sqrt((dx * dx) + (dy * dy) + (dz * dz))

    def _heuristic_cost(self, node: GridIndex, goal: GridIndex) -> float:
        distance = self._distance_between_indices(node, goal)
        time_cost = distance / max(self.config.drone_speed, 1e-6)
        return self.config.heuristic_weight * (
            (self.config.distance_weight * distance)
            + (self.config.time_weight * time_cost)
        )

    def _reconstruct_world_path(
        self,
        came_from: Dict[GridIndex, GridIndex],
        end: GridIndex,
        bounds: PlanningBounds3D,
    ) -> List[Waypoint3D]:
        path_indices = [end]
        current = end
        while current in came_from:
            current = came_from[current]
            path_indices.append(current)
        path_indices.reverse()

        return [self._grid_to_world(index, bounds) for index in path_indices]
