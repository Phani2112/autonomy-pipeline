# autonomy_pipeline

**Track A** submission for the Automation Expert Intern evaluation task:
a minimal reproducible **Perception → local-mapping** pipeline for a
Jetson-class mobile robot using 2D LiDAR + wheel odometry.

The package ships with a synthetic sensor publisher, so you can clone,
build, and run it end-to-end in one terminal with **no hardware and no
rosbag**.

---

## What it does

```
 ┌──────────────────────┐   /scan (LaserScan)   ┌────────────────────────┐
 │ fake_sensor_publisher│──────────────────────▶│                        │
 │   (or real driver)   │   /odom (Odometry)    │  obstacle_grid_node    │
 │                      │──────────────────────▶│                        │
 └──────────────────────┘                       │  • Log-odds Bayesian   │
                                                │    grid update         │
                                                │  • Bresenham ray-trace │
                                                │    for free space      │
                                                │  • Range validation    │
                                                └────────┬───────────────┘
                                                         │
                   /local_costmap (nav_msgs/OccupancyGrid) ◀── RViz "Map"
                   /obstacles      (visualization_msgs/MarkerArray) ◀── RViz "MarkerArray"
```

A rolling 10 m × 10 m local occupancy grid is maintained around the
robot and republished at 5 Hz. Occupied cells within 0.6 m of the
robot are additionally emitted as red sphere markers — this is the
surface a downstream planner (e.g. Nav2) would consume.

---

## Prerequisites

| Requirement | Version tested |
|---|---|
| Ubuntu | 22.04 (Humble) or 24.04 (Jazzy) |
| ROS 2 | Humble or Jazzy |
| Python | 3.10+ |
| numpy | ≥ 1.20 |
| RViz2 | shipped with ROS 2 |

```bash
# ROS 2 Python deps (usually already installed)
sudo apt install ros-${ROS_DISTRO}-rclpy ros-${ROS_DISTRO}-rviz2 \
                 python3-numpy python3-colcon-common-extensions
```

---

## Build

```bash
# From the zip: unzip and `cd autonomy_ws`
cd autonomy_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select autonomy_pipeline
source install/setup.bash
```

`--symlink-install` is useful while iterating: Python source edits are
picked up without rebuilding.

---

## Run (synthetic data, recommended for review)

One command brings up synthetic sensors, the grid node, and RViz with a
pre-made layout:

```bash
ros2 launch autonomy_pipeline autonomy_pipeline.launch.py
```

Expected inside ~5 seconds:

* `/scan`, `/odom` being published.
* `/local_costmap` visible in RViz as a growing map (grey = unknown,
  black = occupied, white = free).
* `/obstacles` as red spheres near the robot when it drives past an
  obstacle.

### Useful launch arguments

| Arg | Default | Effect |
|---|---|---|
| `use_rviz:=false` | `true` | Headless mode — good for Jetson-over-SSH. |
| `use_fake:=false` | `true` | Disable the synthetic publisher (bring your own `/scan` + `/odom`). |
| `log_level:=debug` | `info` | Verbose logging. |
| `params_file:=/path/to/x.yaml` | bundled | Override parameter file. |

---

## Run with a real rosbag

Any rosbag that publishes `sensor_msgs/LaserScan` on `/scan` and
`nav_msgs/Odometry` on `/odom` will work:

```bash
ros2 launch autonomy_pipeline autonomy_pipeline.launch.py use_fake:=false
# in another terminal
ros2 bag play /path/to/bag --clock
```

If your topics are named differently, remap them:

```bash
ros2 launch autonomy_pipeline autonomy_pipeline.launch.py use_fake:=false \
    --ros-args -r /scan:=/your_scan_topic -r /odom:=/your_odom_topic
```

Public datasets that work out of the box:
* MIT Stata Center dataset (`scan`, `odom`)
* KTH longterm-office bags (`scan`, `odom`)
* Any TurtleBot3 Gazebo recording

---

## Verify it's working

```bash
ros2 topic list
# /local_costmap   /obstacles   /odom   /parameter_events   /rosout   /scan

ros2 topic hz /scan
# ~ 10.0 Hz
ros2 topic hz /local_costmap
# ~ 5.0 Hz

ros2 topic echo --once /local_costmap | head -n 20
# header, info.resolution: 0.05, info.width: 200, info.height: 200 ...
```

---

## Parameters

All knobs live in `src/autonomy_pipeline/config/params.yaml`. A few
noteworthy ones:

| Parameter | Default | Note |
|---|---|---|
| `grid.resolution_m` | 0.05 | 5 cm/cell. Lower → finer map, more memory. |
| `grid.size_m` | 10.0 | Window around the robot. 10 m × 10 m at 5 cm ≈ 40 kB. |
| `logodds.l_occ` | 0.85 | ≈ `ln(0.7/0.3)`. Higher converges to “occupied” faster. |
| `logodds.l_free` | -0.40 | ≈ `ln(0.4/0.6)`. Free-space is softer than occupied. |
| `scan.max_range_m` | 8.0 | Trim extra-long returns; saves ray-tracing cost. |

Override at runtime:

```bash
ros2 run autonomy_pipeline obstacle_grid_node --ros-args \
     -p grid.resolution_m:=0.10 -p publish.rate_hz:=2.0
```

---

## Project layout

```
autonomy_ws/
└── src/autonomy_pipeline/
    ├── autonomy_pipeline/
    │   ├── obstacle_grid_node.py     # main perception node
    │   └── fake_sensor_publisher.py  # synthetic /scan + /odom
    ├── launch/autonomy_pipeline.launch.py
    ├── config/params.yaml
    ├── rviz/autonomy.rviz
    ├── package.xml
    └── setup.py
```

---

## What I'd improve with more time

* TF2 integration (listen to `map → odom → base_link` instead of
  assuming `odom == map`).
* Replace the per-beam Python loop with a vectorised numpy raycaster
  (≈10× faster on a Jetson).
* Proper Nav2 costmap plugin wrapper so planners consume this directly.
* pytest-based unit tests for log-odds update, Bresenham, and quaternion
  helpers — stubbed out in `test/` but not included in this submission
  to stay within the time-box.
* Replay-driven CI: a tiny rosbag fixture + GH Actions run.

---

## Attribution

* Log-odds update: standard formulation from Thrun, Burgard, Fox,
  *Probabilistic Robotics*, Ch. 9.
* Bresenham line algorithm: classic (I wrote this one from scratch but
  it's textbook).
* Message types and launch/param idioms: ROS 2 design docs and the
  `rclpy` tutorials (nothing copied verbatim).

No third-party code was vendored into this repo.
