#!/usr/bin/env python3

from __future__ import annotations

import time
from typing import Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from mars_graph_planner.performance_tracker import PathPerformanceSample, PathPerformanceTracker
from mars_graph_planner.planner_core import GraphAStar3D, PlannedPath3D, PlannerConfig, PlanningBounds3D


Waypoint3D = Tuple[float, float, float]


class GraphPathPlannerNode(Node):
    def __init__(self) -> None:
        super().__init__('graph_path_planner')

        self.declare_parameter('drone_speed', 2.0)
        self.declare_parameter('graph_resolution', 2.0)
        self.declare_parameter('graph_resolution_z', 1.0)
        self.declare_parameter('heuristic_weight', 1.0)
        self.declare_parameter('distance_weight', 1.0)
        self.declare_parameter('time_weight', 1.0)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('graph_margin_xy', 10.0)
        self.declare_parameter('graph_margin_z', 3.0)
        self.declare_parameter('min_planning_z', 0.2)
        self.declare_parameter('max_planning_z', 12.0)
        self.declare_parameter('history_csv_path', '')

        self.drone_speed = float(self.get_parameter('drone_speed').value)
        self.graph_resolution = float(self.get_parameter('graph_resolution').value)
        self.graph_resolution_z = float(self.get_parameter('graph_resolution_z').value)
        self.heuristic_weight = float(self.get_parameter('heuristic_weight').value)
        self.distance_weight = float(self.get_parameter('distance_weight').value)
        self.time_weight = float(self.get_parameter('time_weight').value)
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.graph_margin_xy = float(self.get_parameter('graph_margin_xy').value)
        self.graph_margin_z = float(self.get_parameter('graph_margin_z').value)
        self.min_planning_z = float(self.get_parameter('min_planning_z').value)
        self.max_planning_z = float(self.get_parameter('max_planning_z').value)
        history_csv_path = str(self.get_parameter('history_csv_path').value)

        self.planner = GraphAStar3D(
            PlannerConfig(
                drone_speed=self.drone_speed,
                graph_resolution_xy=self.graph_resolution,
                graph_resolution_z=self.graph_resolution_z,
                heuristic_weight=self.heuristic_weight,
                distance_weight=self.distance_weight,
                time_weight=self.time_weight,
                allow_diagonal=self.allow_diagonal,
            )
        )
        self.performance_tracker = PathPerformanceTracker(csv_path=history_csv_path)

        self.current_position: Optional[Waypoint3D] = None

        self.odom_sub = self.create_subscription(Odometry, '/drone/odom', self.odom_callback, 20)
        self.goal_sub = self.create_subscription(PoseStamped, '/mars/goal_pose', self.goal_callback, 10)

        self.path_pub = self.create_publisher(Path, '/mars/planned_path', 10)
        self.performance_pub = self.create_publisher(String, '/mars/planner_performance', 10)

        self.get_logger().info(
            'Graph path planner is running. Waiting for goals on /mars/goal_pose and odometry on /drone/odom.'
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

    def goal_callback(self, msg: PoseStamped) -> None:
        if self.current_position is None:
            self.get_logger().warn('Ignoring planning request: odometry is not available yet.')
            return

        start = self.current_position
        goal = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        frame_id = msg.header.frame_id if msg.header.frame_id else 'world'

        bounds = self.build_planning_bounds(start, goal)

        planner_start = time.perf_counter()
        result = self.planner.plan(start, goal, bounds)
        planning_time_ms = (time.perf_counter() - planner_start) * 1000.0

        if result is None:
            failure_text = (
                f'Planner failed for goal ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}). '
                'Fallback to direct navigation is required.'
            )
            self.get_logger().warn(failure_text)
            self.publish_performance_text(failure_text)
            return

        self.publish_path(result, frame_id)
        self.publish_performance(result, planning_time_ms)

    def build_planning_bounds(self, start: Waypoint3D, goal: Waypoint3D) -> PlanningBounds3D:
        min_x = min(start[0], goal[0]) - self.graph_margin_xy
        max_x = max(start[0], goal[0]) + self.graph_margin_xy
        min_y = min(start[1], goal[1]) - self.graph_margin_xy
        max_y = max(start[1], goal[1]) + self.graph_margin_xy

        min_z = max(self.min_planning_z, min(start[2], goal[2]) - self.graph_margin_z)
        max_z = min(self.max_planning_z, max(start[2], goal[2]) + self.graph_margin_z)

        if max_z - min_z < self.graph_resolution_z:
            max_z = min_z + self.graph_resolution_z

        return PlanningBounds3D(
            min_x=min_x,
            max_x=max_x,
            min_y=min_y,
            max_y=max_y,
            min_z=min_z,
            max_z=max_z,
        )

    def publish_path(self, result: PlannedPath3D, frame_id: str) -> None:
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame_id

        for x_pos, y_pos, z_pos in result.waypoints:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = frame_id
            pose.pose.position.x = float(x_pos)
            pose.pose.position.y = float(y_pos)
            pose.pose.position.z = float(z_pos)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def publish_performance(self, result: PlannedPath3D, planning_time_ms: float) -> None:
        sample = PathPerformanceSample(
            timestamp_utc=PathPerformanceTracker.now_iso_utc(),
            distance_m=result.total_distance,
            estimated_time_s=result.total_estimated_time,
            planning_time_ms=planning_time_ms,
            waypoint_count=len(result.waypoints),
        )

        comparison = self.performance_tracker.record(sample)
        summary = PathPerformanceTracker.format_summary(
            comparison=comparison,
            planning_time_ms=planning_time_ms,
            expanded_nodes=result.expanded_nodes,
        )

        self.get_logger().info(summary)
        self.publish_performance_text(summary)

    def publish_performance_text(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.performance_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GraphPathPlannerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
