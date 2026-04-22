# Drone Delivery on Mars (ROS 2 Humble + Gazebo)

Complete source code: https://github.com/pes1ug23cs178/Drone-Delivery-Mars

Drive link for DEMO Video : https://drive.google.com/drive/folders/1VM6F9hEQgUdxeTzE2-x0lfkte3OGIJ1b

This project simulates autonomous drone parcel delivery on a Mars-themed map using ROS 2, Gazebo, RViz, a mission-state machine, and an optional 3D graph A* planner.

## 1) What this project includes

| Package | Purpose | Build Type |
|---|---|---|
| `mars_bringup` | End-to-end launch orchestration for Gazebo + robot + planner + mission manager + RViz | `ament_cmake` |
| `mars_world` | Mars world file and models used by Gazebo | `ament_cmake` |
| `mars_drone_description` | Drone URDF/Xacro and robot description | `ament_cmake` |
| `mars_mission_manager` | Mission state machine and drone guidance/control node | `ament_python` |
| `mars_graph_planner` | 3D graph A* planner and performance publishing | `ament_python` |
| `mars_gui` | PyQt5 operator GUI for selecting house delivery targets | `ament_python` |

## 2) Prerequisites

Tested with Linux and ROS 2 Humble.

### 2.1 OS and ROS

- Ubuntu 22.04 (recommended)
- ROS 2 Humble
- Gazebo Classic integration via `gazebo_ros`

Install the common dependency set:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-rviz2 \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pyqt5
```

Initialize rosdep (one-time on a fresh machine):

```bash
sudo rosdep init
rosdep update
```

## 3) Workspace setup

If you are cloning from scratch:

```bash
mkdir -p ~/mars_ws/src
cd ~/mars_ws
git clone https://github.com/pes1ug23cs178/Drone-Delivery-Mars.git .
```

Install ROS package dependencies via rosdep:

```bash
cd ~/mars_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build all packages:

```bash
cd ~/mars_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

Source the workspace overlay:

```bash
cd ~/mars_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 4) Quick start (complete demo with GUI and A*)

Use multiple terminals. Run each block in a separate terminal.

### Terminal 1: full simulation stack (Gazebo + RViz + mission manager + optional planner)

```bash
cd "/home/shihab/MARS PROJECT/mars_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mars_bringup mars_full.launch.py use_graph_planner:=true gui:=true server:=true
```

### Terminal 2: operator GUI (house selection)

```bash
cd "/home/shihab/MARS PROJECT/mars_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run mars_gui delivery_gui
```

### Terminal 3: planner performance monitor

```bash
cd "/home/shihab/MARS PROJECT/mars_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /mars/planner_performance
```

### Terminal 4: mission state monitor

```bash
cd "/home/shihab/MARS PROJECT/mars_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /mars/mission_state
```

Trigger missions by clicking a house in the GUI, or publish manually:

```bash
cd "/home/shihab/MARS PROJECT/mars_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /mars/delivery_target std_msgs/msg/String "{data: House1}"
```

Valid house targets are:

- `House1`
- `House2`
- `House3`
- `House4`
- `House5`
- `House6`

## 5) Launch command reference

### 5.1 Full system launch

```bash
ros2 launch mars_bringup mars_full.launch.py use_graph_planner:=true
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `use_graph_planner` | `false` | Enables `mars_graph_planner` node and planner-assisted routing |
| `gui` | `true` | Gazebo client UI toggle (inherited from Gazebo launch) |
| `server` | `true` | Gazebo server toggle (inherited from Gazebo launch) |

### 5.2 Full stack without A* planner

```bash
ros2 launch mars_bringup mars_full.launch.py use_graph_planner:=false
```

### 5.3 Headless mode (faster CI/test style run)

```bash
ros2 launch mars_bringup mars_full.launch.py use_graph_planner:=true gui:=false server:=true
```

### 5.4 Run individual nodes manually

Mission manager:

```bash
ros2 run mars_mission_manager mission_manager_node
```

Graph planner:

```bash
ros2 run mars_graph_planner graph_path_planner
```

GUI:

```bash
ros2 run mars_gui delivery_gui
```

## 6) Demo walkthrough (step-by-step)

1. Start Terminal 1 full launch with `use_graph_planner:=true`.
2. Wait for Gazebo to load and drone model to spawn.
3. Start the GUI in Terminal 2.
4. Click any house in the GUI map.
5. Observe `/mars/mission_state` transitions:
   `IDLE -> TAKEOFF -> FLY_TO_HOUSE -> DELIVER -> FLY_TO_WAREHOUSE -> LAND -> IDLE`
6. Observe planner output in Terminal 3 (`/mars/planner_performance`).
7. Repeat with different houses to compare routes and timings.

## 7) ROS interfaces

### 7.1 Core nodes

| Node | Package | Role |
|---|---|---|
| `/mission_manager_node` | `mars_mission_manager` | Mission sequencing, guidance, command publication |
| `/graph_path_planner` | `mars_graph_planner` | Computes path waypoints when A* is enabled |
| `/delivery_gui_node` | `mars_gui` | Operator UI, sends target house commands |
| `/robot_state_publisher` | `robot_state_publisher` | Publishes robot description transforms |

### 7.2 Main topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/mars/delivery_target` | `std_msgs/msg/String` | GUI/CLI -> Mission Manager | Target house command |
| `/mars/mission_state` | `std_msgs/msg/String` | Mission Manager -> Others | Current mission phase |
| `/drone/odom` | `nav_msgs/msg/Odometry` | Gazebo -> Planner/Manager/GUI | Drone localization input |
| `/drone/cmd_vel` | `geometry_msgs/msg/Twist` | Mission Manager -> Drone | Velocity command |
| `/mars/goal_pose` | `geometry_msgs/msg/PoseStamped` | Mission Manager -> Planner | Planner goal request |
| `/mars/planned_path` | `nav_msgs/msg/Path` | Planner -> Mission Manager | Planned route waypoints |
| `/mars/planner_performance` | `std_msgs/msg/String` | Planner -> Monitoring | Distance/time/planning summary |
| `/drone/path` | `nav_msgs/msg/Path` | Mission Manager -> Visualization | Flight trace history |

## 8) Mission manager behavior

- State machine phases:
  `IDLE`, `TAKEOFF`, `FLY_TO_HOUSE`, `DELIVER`, `FLY_TO_WAREHOUSE`, `LAND`
- Uses a proportional controller to compute velocity toward active target.
- Publishes path history (`/drone/path`) and mission state (`/mars/mission_state`).
- Supports queueing a new mission while one is in progress.
- If graph planning is enabled:
  mission manager requests plan on `/mars/goal_pose`, consumes `/mars/planned_path`, and falls back to direct target navigation on timeout.

## 9) Graph A* planner details

Planner class: `GraphAStar3D`

Key concepts:

- 3D grid graph generated inside dynamic bounds around start and goal.
- Optional diagonal expansion in 3D (`allow_diagonal`).
- Cost model combines distance and estimated time:

  `transition_cost = distance_weight * edge_distance + time_weight * edge_time`

- Heuristic uses weighted distance+time with `heuristic_weight`.
- Final path preserves exact start and goal endpoints.

### 9.1 Planner parameters (default values)

| Parameter | Default | Description |
|---|---|---|
| `drone_speed` | `2.0` | Speed used for time estimation |
| `graph_resolution` | `2.0` | XY grid resolution |
| `graph_resolution_z` | `1.0` | Z-axis grid resolution |
| `heuristic_weight` | `1.0` | A* heuristic multiplier |
| `distance_weight` | `1.0` | Distance cost multiplier |
| `time_weight` | `1.0` | Time cost multiplier |
| `allow_diagonal` | `true` | Enables 26-neighbor 3D expansion |
| `graph_margin_xy` | `10.0` | XY margin around start-goal bounds |
| `graph_margin_z` | `3.0` | Z margin around start-goal bounds |
| `min_planning_z` | `0.2` | Lower Z clamp for planning volume |
| `max_planning_z` | `12.0` | Upper Z clamp for planning volume |
| `history_csv_path` | `""` | Optional CSV output file path |

### 9.2 Mission manager graph-planning parameters

| Parameter | Default | Description |
|---|---|---|
| `use_graph_planner` | `false` | Enable graph planner integration |
| `graph_planner_timeout_sec` | `2.0` | Timeout before fallback to direct navigation |
| `graph_waypoint_tolerance` | `0.4` | Waypoint acceptance radius |
| `graph_goal_topic` | `/mars/goal_pose` | Planner request topic |
| `planned_path_topic` | `/mars/planned_path` | Planner response topic |
| `cmd_vel_body_frame` | `true` | Rotate world velocity into body frame |

## 10) Useful diagnostics

List nodes:

```bash
ros2 node list
```

List topics:

```bash
ros2 topic list
```

Inspect planned path:

```bash
ros2 topic echo /mars/planned_path
```

Inspect odometry:

```bash
ros2 topic echo /drone/odom
```

Check launch arguments:

```bash
ros2 launch mars_bringup mars_full.launch.py --show-args
```

## 11) Troubleshooting

### `Package not found` or executable not found

- Ensure each terminal has:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
```

- Rebuild if needed:

```bash
colcon build --symlink-install
```

### GUI fails to start

Install Qt runtime:

```bash
sudo apt install -y python3-pyqt5
```

### Planner output not appearing

- Confirm launch uses `use_graph_planner:=true`.
- Check planner node exists:

```bash
ros2 node list | grep graph_path_planner
```

### Gazebo models not visible

- `mars_bringup` sets `GAZEBO_MODEL_PATH` automatically.
- Relaunch from a clean terminal after sourcing ROS and workspace overlays.

## 12) Development notes

- Build only one package during iteration:

```bash
colcon build --packages-select mars_graph_planner --symlink-install
```

- Keep terminal overlays consistent after each rebuild.
- Prefer `--symlink-install` for faster Python package edits.

## 13) License

Apache-2.0 (as declared in package manifests).
