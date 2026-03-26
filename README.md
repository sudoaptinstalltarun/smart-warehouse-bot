# 🤖 Intelligent Warehouse Picking Robot
### Vision-Based Dynamic Task Optimization | ROS2 · OpenCV · Nav2 · Gazebo

**Team SudoSquad** — Arjun Sharma · Priya Nair · Rahul Menon · Sneha Patel  
Track: **Robotics + AI** | Hackathon 2025

---

## 📖 Overview

A fully autonomous warehouse robot that:
1. **Perceives** the environment via a calibrated RGB camera and ArUco marker detection (OpenCV)
2. **Decides** which task to execute next using a cost-based priority scheduler
3. **Navigates** optimally using the ROS2 Nav2 stack (A\* global planner + DWA local planner)
4. **Adapts** dynamically — re-plans in < 200 ms when higher-priority tasks arrive

### Key Innovation — Task Scheduler

```
score = 0.6 × distance + 0.2 × weight − 0.5 × priority
```

Lower score = higher execution priority. The scheduler re-evaluates the entire queue on every new detection.

---

## 🗂️ Package Structure

```
warehouse_robot_ws/
└── src/
    └── warehouse_robot/
        ├── CMakeLists.txt
        ├── package.xml
        ├── warehouse_robot/          # Python nodes
        │   ├── camera_node.py        # Camera relay / hardware capture
        │   ├── vision_node.py        # ArUco detection + temporal filter
        │   ├── task_scheduler.py     # ⭐ Core innovation – priority scheduling
        │   └── robot_controller.py   # Safety layer + e-stop + velocity clamping
        ├── launch/
        │   ├── warehouse_robot_launch.py  # Master launch (everything)
        │   ├── slam_launch.py             # Mapping only
        │   └── nav2_launch.py             # Navigation with pre-built map
        ├── urdf/
        │   └── warehouse_robot.urdf.xacro # Robot + all Gazebo plugins
        ├── worlds/
        │   └── warehouse.world            # Warehouse environment + shelves + markers
        ├── config/
        │   ├── nav2_params.yaml           # Full Nav2 configuration
        │   └── slam_toolbox_params.yaml   # SLAM Toolbox config
        ├── maps/
        │   └── marker_positions.json      # ArUco marker → world coordinate map
        └── rviz/
            └── warehouse_robot.rviz       # Pre-configured RViz2 layout
```

---

## ⚙️ Prerequisites

| Requirement | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Gazebo | Classic 11 |
| Python | ≥ 3.10 |
| OpenCV | ≥ 4.7 (with contrib) |

---

## 🚀 Installation

### 1. Clone the repository

```bash
mkdir -p ~/warehouse_robot_ws/src
cd ~/warehouse_robot_ws/src
git clone https://github.com/team-SudoSquad/warehouse_robot.git
```

### 2. Install ROS2 dependencies

```bash
cd ~/warehouse_robot_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Install Python dependencies

```bash
pip3 install opencv-contrib-python numpy transforms3d
```

### 4. Install additional ROS2 packages

```bash
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-tf2-tools \
  ros-humble-tf-transformations \
  ros-humble-teleop-twist-keyboard
```

### 5. Build

```bash
cd ~/warehouse_robot_ws
colcon build --symlink-install
source install/setup.bash
```

Add to `~/.bashrc`:
```bash
echo "source ~/warehouse_robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 🎮 Usage

### Option A — Full simulation (SLAM + Nav2 + all nodes)

```bash
ros2 launch warehouse_robot warehouse_robot_launch.py
```

Arguments:
```
use_sim_time:=true   # Use Gazebo clock (default: true)
slam:=true           # Run SLAM mapping (default: true)
nav2:=true           # Run Nav2 (default: true)
rviz:=true           # Open RViz2 (default: true)
```

### Option B — Map the warehouse first (manual teleoperation)

```bash
# Terminal 1: SLAM + Gazebo
ros2 launch warehouse_robot slam_launch.py

# Terminal 2: Drive around to map
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Terminal 3: Save the map when done
ros2 run nav2_map_server map_saver_cli -f ~/maps/warehouse_map
```

### Option C — Navigate with pre-built map

```bash
ros2 launch warehouse_robot nav2_launch.py \
  map:=~/maps/warehouse_map.yaml
```

---

## 🔌 ROS2 Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | camera_node → vision_node | Raw RGB frames (30 fps) |
| `/camera_info` | `sensor_msgs/CameraInfo` | camera_node → vision_node | Intrinsic parameters |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → Nav2, controller | 2D LiDAR scan |
| `/detected_markers` | `std_msgs/String` (JSON) | vision_node → task_scheduler | Confirmed ArUco detections |
| `/goal_pose` | `geometry_msgs/PoseStamped` | task_scheduler → Nav2 | Navigation goal |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 → robot | Safe velocity commands |
| `/task_status` | `std_msgs/String` (JSON) | task_scheduler | Queue status |
| `/emergency_stop` | `std_msgs/Bool` | robot_controller | E-stop signal |
| `/vision_debug` | `sensor_msgs/Image` | vision_node | Debug frame with overlays |
| `/odom` | `nav_msgs/Odometry` | Gazebo → Nav2 | Odometry |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Built map |

---

## 🏷️ ArUco Marker Configuration

| Marker ID | Task | Weight | Priority |
|---|---|---|---|
| 0 | Task A | 10 kg | HIGH (3) |
| 1 | Task B | 2 kg | LOW (1) |
| 2 | Task C | 7 kg | MEDIUM (2) |
| 3 | Task D | 5 kg | HIGH (3) — dynamic demo |

Dictionary: **DICT_6X6_250**  
Physical size: **200 × 200 mm** (matches `MARKER_SIZE = 0.20` in vision_node.py)

---

## 🧮 Demo Scenario

Three tasks detected simultaneously:

| Task | Weight | Priority | Distance | Score |
|---|---|---|---|---|
| A | 10 kg | HIGH (3) | 2.0 m | 0.6×2.0 + 0.2×10 − 0.5×3 = **1.70** |
| C | 7 kg | MEDIUM (2) | 1.8 m | 0.6×1.8 + 0.2×7 − 0.5×2 = **1.48** |
| B | 2 kg | LOW (1) | 1.2 m | 0.6×1.2 + 0.2×2 − 0.5×1 = **0.62** |

**Execution order: A → C → B**

Dynamic replanning: Task D (HIGH, 1.0 m) arrives while navigating to C  
→ Scheduler re-evaluates → D inserted before C  
→ New order: **A → D → C → B**

---

## 📊 Evaluation Metrics

| Metric | Target | Formula |
|---|---|---|
| Detection Accuracy | > 95% | TP / (TP + FP + FN) |
| Task Completion Time | < 60 s/task | avg(t_complete − t_assign) |
| Path Efficiency | > 85% | d_optimal / d_actual × 100 |
| Motion Smoothness | < 0.5 m/s² | avg(\|Δv/Δt\|) |

---

## 🔧 Configuration

### Scoring weights (`task_scheduler.py`)
```python
W_DISTANCE = 0.6   # Weight for distance term
W_WEIGHT   = 0.2   # Weight for item weight term
W_PRIORITY = 0.5   # Weight for priority term (subtracted)
```

### Safety parameters (`robot_controller.py`)
```python
stop_distance = 0.30   # m  — full stop
slow_distance = 0.60   # m  — 40% speed reduction
max_linear    = 0.26   # m/s
max_angular   = 0.80   # rad/s
```

### Vision parameters (`vision_node.py`)
```python
confirm_frames    = 3      # Frames before marker is confirmed
max_marker_dist   = 3.0    # Max detection range (m)
reproj_thresh     = 2.5    # Max reprojection error (px)
```

---

## 🛠️ Troubleshooting

| Issue | Fix |
|---|---|
| `spawn_entity.py: robot not found` | Wait 2-3 seconds for Gazebo to fully load |
| Nav2 not receiving goals | Check `/map` is being published: `ros2 topic hz /map` |
| ArUco not detected | Reduce `reproj_error_thresh` or check lighting / camera focus |
| `tf2 extrapolation error` | Increase `transform_tolerance` in `nav2_params.yaml` |
| Robot not moving | Check `/cmd_vel` is published: `ros2 topic echo /cmd_vel` |
| SLAM map not saving | Run `ros2 run nav2_map_server map_saver_cli -f ~/map` |

---

## 📝 License

Apache 2.0 — see [LICENSE](LICENSE)

---

*Vision that sees · Intelligence that decides · Precision that executes*
