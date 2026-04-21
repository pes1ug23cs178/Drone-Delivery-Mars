#!/usr/bin/env python3

from enum import Enum
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class MissionState(str, Enum):
    IDLE = 'IDLE'
    TAKEOFF = 'TAKEOFF'
    FLY_TO_HOUSE = 'FLY_TO_HOUSE'
    DELIVER = 'DELIVER'
    FLY_TO_WAREHOUSE = 'FLY_TO_WAREHOUSE'
    LAND = 'LAND'


class MissionManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('mission_manager_node')

        self.use_graph_planner = bool(self.declare_parameter('use_graph_planner', False).value)
        self.graph_planner_timeout_sec = float(self.declare_parameter('graph_planner_timeout_sec', 2.0).value)
        self.graph_waypoint_tolerance = float(self.declare_parameter('graph_waypoint_tolerance', 0.4).value)
        self.graph_goal_topic = str(self.declare_parameter('graph_goal_topic', '/mars/goal_pose').value)
        self.planned_path_topic = str(self.declare_parameter('planned_path_topic', '/mars/planned_path').value)
        self.cmd_vel_body_frame = bool(self.declare_parameter('cmd_vel_body_frame', True).value)

        self.kp = 0.5
        self.max_velocity = 2.0
        self.arrival_threshold = 0.5
        self.target_altitude = 5.0
        # Warehouse is a raised platform; landing is on its roof surface.
        self.land_altitude = 2.6
        self.land_altitude_tolerance = 0.2
        self.control_period = 0.05

        self.warehouse_xy: Tuple[float, float] = (0.0, 0.0)
        self.house_positions: Dict[str, Tuple[float, float]] = {
            'House1': (10.0, 10.0),
            'House2': (10.0, -10.0),
            'House3': (-10.0, 10.0),
            'House4': (-10.0, -10.0),
            'House5': (20.0, 0.0),
            'House6': (-20.0, 0.0),
        }

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_orientation = (0.0, 0.0, 0.0, 1.0)
        self.has_odom = False

        self.state = MissionState.IDLE
        self.active_house: Optional[str] = None
        self.queued_house: Optional[str] = None
        self.delivery_start_time = None
        self.graph_waypoints: List[Tuple[float, float, float]] = []
        self.graph_waypoint_index = 0
        self.graph_waiting_for_plan = False
        self.graph_plan_requested_at = None
        self.graph_requested_house: Optional[str] = None

        self.drone_entity_name = 'mars_drone'
        self.pending_set_state_future = None

        self.cmd_vel_pub = self.create_publisher(Twist, '/drone/cmd_vel', 20)
        self.state_pub = self.create_publisher(String, '/mars/mission_state', 10)
        self.path_pub = self.create_publisher(Path, '/drone/path', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, self.graph_goal_topic, 10)

        self.target_sub = self.create_subscription(
            String,
            '/mars/delivery_target',
            self.target_callback,
            10,
        )
        self.planned_path_sub = self.create_subscription(
            Path,
            self.planned_path_topic,
            self.planned_path_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            20,
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.last_service_warning_time = self.get_clock().now()

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'world'
        self.last_path_publish_time = self.get_clock().now()

        self.last_control_time = self.get_clock().now()
        self.control_timer = self.create_timer(self.control_period, self.control_loop)

        self.publish_mission_state()
        self.get_logger().info('Mission manager started. Waiting for delivery targets on /mars/delivery_target.')
        self.get_logger().info(
            'Guidance algorithm: straight-line proportional vector control (P-controller with XY vector speed limit).'
        )
        if self.cmd_vel_body_frame:
            self.get_logger().info('Cmd frame: /drone/cmd_vel interpreted as body-frame (world velocity is rotated to body).')
        else:
            self.get_logger().info('Cmd frame: /drone/cmd_vel interpreted as world-frame (no velocity rotation applied).')
        if self.use_graph_planner:
            self.get_logger().info(
                f'Graph planner integration enabled. Goal topic: {self.graph_goal_topic}, '
                f'planned path topic: {self.planned_path_topic}'
            )
            self.get_logger().info(
                'Path planning algorithm: 3D graph A* (Euclidean heuristic with distance+time weighted cost).'
            )

    def clamp(self, value: float, limit: float) -> float:
        return max(min(value, limit), -limit)

    def publish_mission_state(self) -> None:
        state_msg = String()
        state_msg.data = self.state.value
        self.state_pub.publish(state_msg)

    def set_state(self, new_state: MissionState) -> None:
        if self.state != new_state:
            self.get_logger().info(f'State transition: {self.state.value} -> {new_state.value}')
            self.state = new_state
            self.publish_mission_state()

    def start_mission(self, house_name: str) -> None:
        self.active_house = house_name
        self.delivery_start_time = None
        self.clear_graph_plan()
        self.get_logger().info(f'Accepted mission to {house_name}.')
        self.set_state(MissionState.TAKEOFF)

    def target_callback(self, msg: String) -> None:
        house_name = msg.data.strip()

        if house_name not in self.house_positions:
            self.get_logger().warn(f'Unknown target "{house_name}" ignored.')
            return

        if self.state != MissionState.IDLE:
            self.queued_house = house_name
            self.get_logger().info(
                f'Queued mission to {house_name}; current state is {self.state.value}.'
            )
            return

        self.start_mission(house_name)

    def clear_graph_plan(self) -> None:
        self.graph_waypoints = []
        self.graph_waypoint_index = 0
        self.graph_waiting_for_plan = False
        self.graph_plan_requested_at = None
        self.graph_requested_house = None

    def request_graph_plan_for_active_house(self) -> None:
        if not self.use_graph_planner or self.active_house is None:
            return

        house_x, house_y = self.house_positions[self.active_house]

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'world'
        goal_msg.pose.position.x = float(house_x)
        goal_msg.pose.position.y = float(house_y)
        goal_msg.pose.position.z = float(self.target_altitude)
        goal_msg.pose.orientation.w = 1.0

        self.goal_pose_pub.publish(goal_msg)
        self.graph_waiting_for_plan = True
        self.graph_plan_requested_at = self.get_clock().now()
        self.graph_requested_house = self.active_house
        self.get_logger().info(f'Requested graph plan for {self.active_house}.')

    def arrived_waypoint(self, target_x: float, target_y: float, target_z: float) -> bool:
        xy_error = self.distance_xy(target_x, target_y)
        z_error = abs(target_z - self.current_z)
        return xy_error <= self.graph_waypoint_tolerance and z_error <= 0.45

    def planned_path_callback(self, msg: Path) -> None:
        if not self.use_graph_planner:
            return
        if self.active_house is None:
            return
        if not self.graph_waiting_for_plan:
            return
        if self.graph_requested_house != self.active_house:
            return
        if not msg.poses:
            self.get_logger().warn('Received empty graph path; keeping fallback behavior enabled.')
            return

        house_x, house_y = self.house_positions[self.active_house]
        end_pose = msg.poses[-1].pose.position
        goal_error_xy = ((end_pose.x - house_x) ** 2 + (end_pose.y - house_y) ** 2) ** 0.5
        if goal_error_xy > 1.5:
            self.get_logger().warn(
                f'Ignoring planned path with mismatched goal (xy error {goal_error_xy:.2f}m).'
            )
            return

        waypoints: List[Tuple[float, float, float]] = []
        for pose in msg.poses:
            waypoint = (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            )

            if waypoints:
                prev = waypoints[-1]
                step = (
                    (waypoint[0] - prev[0]) ** 2
                    + (waypoint[1] - prev[1]) ** 2
                    + (waypoint[2] - prev[2]) ** 2
                ) ** 0.5
                if step < 0.05:
                    continue

            waypoints.append(waypoint)

        if not waypoints:
            self.get_logger().warn('Received graph path had no usable waypoints.')
            return

        end_waypoint = waypoints[-1]
        end_error = (
            (end_waypoint[0] - house_x) ** 2
            + (end_waypoint[1] - house_y) ** 2
            + (end_waypoint[2] - self.target_altitude) ** 2
        ) ** 0.5
        if end_error > 0.25:
            waypoints.append((house_x, house_y, self.target_altitude))

        while waypoints and self.arrived_waypoint(*waypoints[0]):
            waypoints.pop(0)

        if not waypoints:
            self.get_logger().info('Received graph path but already near destination waypoint.')
            self.graph_waiting_for_plan = False
            self.graph_plan_requested_at = None
            return

        self.graph_waypoints = waypoints
        self.graph_waypoint_index = 0
        self.graph_waiting_for_plan = False
        self.graph_plan_requested_at = None
        self.get_logger().info(
            f'Accepted graph path for {self.active_house} with {len(waypoints)} waypoints.'
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_orientation = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.has_odom = True

        self.broadcast_tf(msg)
        self.update_path(msg)

    def broadcast_tf(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
            transform.header.stamp = self.get_clock().now().to_msg()

        transform.header.frame_id = 'world'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.current_x
        transform.transform.translation.y = self.current_y
        transform.transform.translation.z = self.current_z
        transform.transform.rotation.x = self.current_orientation[0]
        transform.transform.rotation.y = self.current_orientation[1]
        transform.transform.rotation.z = self.current_orientation[2]
        transform.transform.rotation.w = self.current_orientation[3]

        self.tf_broadcaster.sendTransform(transform)

    def update_path(self, msg: Odometry) -> None:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = msg.header.stamp
        pose_stamped.header.frame_id = 'world'
        pose_stamped.pose = msg.pose.pose

        self.path_msg.poses.append(pose_stamped)
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)

        now = self.get_clock().now()
        if (now - self.last_path_publish_time).nanoseconds > int(1e8):
            self.path_msg.header.stamp = now.to_msg()
            self.path_pub.publish(self.path_msg)
            self.last_path_publish_time = now

    def distance_xy(self, target_x: float, target_y: float) -> float:
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        return (dx * dx + dy * dy) ** 0.5

    def arrived_3d(self, target_x: float, target_y: float, target_z: float) -> bool:
        xy_error = self.distance_xy(target_x, target_y)
        z_error = abs(target_z - self.current_z)
        return xy_error <= self.arrival_threshold and z_error <= 0.35

    def current_yaw(self) -> float:
        qx, qy, qz, qw = self.current_orientation
        sin_yaw = 2.0 * ((qw * qz) + (qx * qy))
        cos_yaw = 1.0 - (2.0 * ((qy * qy) + (qz * qz)))
        return math.atan2(sin_yaw, cos_yaw)

    def compute_cmd_to_target(
        self,
        target_x: float,
        target_y: float,
        target_z: float,
        max_xy: Optional[float] = None,
        max_z: float = 1.0,
    ) -> Twist:
        xy_limit = self.max_velocity if max_xy is None else min(max_xy, self.max_velocity)
        z_limit = min(max_z, self.max_velocity)

        cmd = Twist()

        error_x = target_x - self.current_x
        error_y = target_y - self.current_y
        error_z = target_z - self.current_z

        # Preserve XY direction by limiting vector magnitude instead of clamping each axis independently.
        vel_world_x = self.kp * error_x
        vel_world_y = self.kp * error_y
        vel_world_norm = math.sqrt((vel_world_x * vel_world_x) + (vel_world_y * vel_world_y))

        if vel_world_norm > xy_limit and vel_world_norm > 1e-6:
            scale = xy_limit / vel_world_norm
            vel_world_x *= scale
            vel_world_y *= scale

        if self.cmd_vel_body_frame:
            yaw = self.current_yaw()
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            cmd.linear.x = (cos_yaw * vel_world_x) + (sin_yaw * vel_world_y)
            cmd.linear.y = (-sin_yaw * vel_world_x) + (cos_yaw * vel_world_y)
        else:
            cmd.linear.x = vel_world_x
            cmd.linear.y = vel_world_y

        cmd.linear.z = self.clamp(self.kp * error_z, z_limit)

        return cmd

    def _set_state_response_callback(self, future) -> None:
        try:
            response = future.result()
            if response is not None and not response.success:
                self.get_logger().warn(f'Gazebo set_entity_state failed: {response.status_message}')
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warn(f'Gazebo set_entity_state exception: {exc}')
        finally:
            self.pending_set_state_future = None

    def apply_vertical_motion(self, cmd: Twist, dt: float) -> None:
        if self.pending_set_state_future is not None and not self.pending_set_state_future.done():
            return

        if not self.set_state_client.wait_for_service(timeout_sec=0.0):
            now = self.get_clock().now()
            if (now - self.last_service_warning_time).nanoseconds > int(5e9):
                self.get_logger().warn('/gazebo/set_entity_state service is not available yet.')
                self.last_service_warning_time = now
            return

        target_z = self.current_z + (cmd.linear.z * dt)
        target_z = max(0.05, min(20.0, target_z))

        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.drone_entity_name
        request.state.reference_frame = 'world'
        request.state.pose.position.x = float(self.current_x)
        request.state.pose.position.y = float(self.current_y)
        request.state.pose.position.z = float(target_z)
        request.state.pose.orientation.x = float(self.current_orientation[0])
        request.state.pose.orientation.y = float(self.current_orientation[1])
        request.state.pose.orientation.z = float(self.current_orientation[2])
        request.state.pose.orientation.w = float(self.current_orientation[3])
        request.state.twist = Twist()
        request.state.twist.linear.z = float(cmd.linear.z)

        self.pending_set_state_future = self.set_state_client.call_async(request)
        self.pending_set_state_future.add_done_callback(self._set_state_response_callback)

    def control_loop(self) -> None:
        if not self.has_odom:
            return

        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = self.control_period
        self.last_control_time = now

        cmd = Twist()
        warehouse_x, warehouse_y = self.warehouse_xy

        if self.state == MissionState.IDLE:
            if self.queued_house is not None:
                queued_house = self.queued_house
                self.queued_house = None
                self.get_logger().info(f'Starting queued mission to {queued_house}.')
                self.start_mission(queued_house)
            cmd = Twist()

        elif self.state == MissionState.TAKEOFF:
            cmd = self.compute_cmd_to_target(
                warehouse_x,
                warehouse_y,
                self.target_altitude,
                max_xy=1.0,
                max_z=0.8,
            )
            if self.arrived_3d(warehouse_x, warehouse_y, self.target_altitude):
                if self.use_graph_planner and self.active_house is not None:
                    self.request_graph_plan_for_active_house()
                self.set_state(MissionState.FLY_TO_HOUSE)

        elif self.state == MissionState.FLY_TO_HOUSE:
            if self.active_house is None:
                self.clear_graph_plan()
                self.set_state(MissionState.IDLE)
            else:
                house_x, house_y = self.house_positions[self.active_house]

                if (
                    self.use_graph_planner
                    and self.graph_waypoint_index < len(self.graph_waypoints)
                ):
                    waypoint = self.graph_waypoints[self.graph_waypoint_index]
                    cmd = self.compute_cmd_to_target(
                        waypoint[0],
                        waypoint[1],
                        waypoint[2],
                        max_xy=self.max_velocity,
                        max_z=0.8,
                    )
                    if self.arrived_waypoint(*waypoint):
                        self.graph_waypoint_index += 1
                        if self.graph_waypoint_index >= len(self.graph_waypoints):
                            self.clear_graph_plan()
                            self.delivery_start_time = now
                            self.set_state(MissionState.DELIVER)
                else:
                    if self.use_graph_planner and self.graph_waiting_for_plan:
                        waited = 0.0
                        if self.graph_plan_requested_at is not None:
                            waited = (now - self.graph_plan_requested_at).nanoseconds / 1e9

                        if waited > self.graph_planner_timeout_sec:
                            self.graph_waiting_for_plan = False
                            self.graph_plan_requested_at = None
                            self.get_logger().warn(
                                'Graph planner timeout reached. Falling back to direct target navigation.'
                            )
                        else:
                            cmd = self.compute_cmd_to_target(
                                self.current_x,
                                self.current_y,
                                self.target_altitude,
                                max_xy=0.3,
                                max_z=0.8,
                            )

                    if not self.use_graph_planner or not self.graph_waiting_for_plan:
                        cmd = self.compute_cmd_to_target(
                            house_x,
                            house_y,
                            self.target_altitude,
                            max_xy=self.max_velocity,
                            max_z=0.8,
                        )
                        if self.arrived_3d(house_x, house_y, self.target_altitude):
                            self.delivery_start_time = now
                            self.set_state(MissionState.DELIVER)

        elif self.state == MissionState.DELIVER:
            if self.active_house is None:
                self.set_state(MissionState.IDLE)
            else:
                house_x, house_y = self.house_positions[self.active_house]
                cmd = self.compute_cmd_to_target(
                    house_x,
                    house_y,
                    self.target_altitude,
                    max_xy=0.7,
                    max_z=0.5,
                )

                if self.delivery_start_time is None:
                    self.delivery_start_time = now

                elapsed = (now - self.delivery_start_time).nanoseconds / 1e9
                if elapsed >= 3.0:
                    self.get_logger().info(f'Package Delivered to {self.active_house}')
                    self.set_state(MissionState.FLY_TO_WAREHOUSE)

        elif self.state == MissionState.FLY_TO_WAREHOUSE:
            cmd = self.compute_cmd_to_target(
                warehouse_x,
                warehouse_y,
                self.target_altitude,
                max_xy=self.max_velocity,
                max_z=0.8,
            )
            if self.arrived_3d(warehouse_x, warehouse_y, self.target_altitude):
                self.set_state(MissionState.LAND)

        elif self.state == MissionState.LAND:
            cmd = self.compute_cmd_to_target(
                warehouse_x,
                warehouse_y,
                self.land_altitude,
                max_xy=1.0,
                max_z=0.5,
            )
            if (
                self.distance_xy(warehouse_x, warehouse_y) <= 0.25
                and abs(self.current_z - self.land_altitude) <= self.land_altitude_tolerance
            ):
                cmd = Twist()
                completed_house = self.active_house
                self.active_house = None
                self.delivery_start_time = None
                self.clear_graph_plan()
                self.set_state(MissionState.IDLE)
                if completed_house is not None:
                    self.get_logger().info(f'Mission complete. Returned to warehouse from {completed_house}.')
                if self.queued_house is not None:
                    queued_house = self.queued_house
                    self.queued_house = None
                    self.get_logger().info(f'Starting queued mission to {queued_house}.')
                    self.start_mission(queued_house)

        else:
            self.get_logger().warn(f'Unknown state {self.state}. Resetting to IDLE.')
            self.set_state(MissionState.IDLE)
            cmd = Twist()

        self.cmd_vel_pub.publish(cmd)

        if self.state != MissionState.IDLE or abs(cmd.linear.z) > 1e-4:
            self.apply_vertical_motion(cmd, dt)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
