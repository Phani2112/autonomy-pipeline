# Track A — Perception to Local Mapping (ROS 2)

**Automation Expert Intern — 24-hour Evaluation Task — Technical Report**

---

## 1. Architecture

The submission is a single ROS 2 package, **`autonomy_pipeline`**, built as an `ament_python` project. It contains one production node (**`obstacle_grid_node`**) and one test-support node (**`fake_sensor_publisher`**). The production node consumes a 2D LiDAR and wheel odometry and produces a rolling local occupancy grid plus obstacle markers. The output is the contract a downstream planner such as Nav2 would consume.

Data flow (producer → topic → message type → rate → consumer):

| Producer | Topic | Message type | Rate | Consumer |
|---|---|---|---|---|
| LiDAR driver (or `fake_sensor_publisher`) | `/scan` | `sensor_msgs/LaserScan` | 10 Hz | `obstacle_grid_node` |
| Wheel-odometry source (or `fake_sensor_publisher`) | `/odom` | `nav_msgs/Odometry` | 50 Hz | `obstacle_grid_node` |
| `obstacle_grid_node` | `/local_costmap` | `nav_msgs/OccupancyGrid` | 5 Hz | Nav2 / RViz |
| `obstacle_grid_node` | `/obstacles` | `visualization_msgs/MarkerArray` | 5 Hz | RViz (debug) |

**Key design choices.**

- **Rolling grid.** The 10 m × 10 m window is re-centred on the robot every cycle, so memory is bounded and the node does not try to be a SLAM system.
- **Log-odds update.** Each cell stores a log-odds probability; free space is traced along every beam using Bresenham's line algorithm, and the endpoint is marked occupied. Values are clamped so no single return can lock a cell.
- **Sensor QoS.** The subscriber uses `BEST_EFFORT` to match real LiDAR drivers; the grid publisher uses `TRANSIENT_LOCAL` so RViz or Nav2 see the latest map even if they subscribe late.
- **TF intentionally omitted** in the prototype: `odom` is treated as the world frame so the node has no TF2 dependency and can be validated in isolation. A real deployment inserts the standard `map → odom → base_link` chain.

---

## 2. Assumptions and system constraints

| Dimension | Assumption / target |
|---|---|
| Compute | Jetson Orin Nano / Orin NX class (6–8 cores, 8 GB RAM) |
| Sensor rates | LiDAR 10 Hz, wheel odometry 50 Hz, IMU 100 Hz |
| Grid memory | 200 × 200 cells of `int8` = 40 kB; log-odds buffer 160 kB (`float32`) |
| Publish rate | `/local_costmap` 5 Hz; downstream planner consumes at ≤10 Hz |
| Environment | Constrained indoor, static plus slow dynamic obstacles |
| Localisation | Wheel odometry assumed good to ≤5 cm over a 30 s horizon |
| Network | On-board only; no ROS domain bridge required |
| Build | ROS 2 Humble or Jazzy; Python 3.10+; numpy only |

---

## 3. Top 5 failure modes and mitigations

| # | Failure mode | Mitigation |
|---|---|---|
| 1 | Odometry drift smears the grid over time. | Swap `/odom` for a fused odom+IMU estimate (EKF) or AMCL pose. Keep the grid body-centred and short-lived (rolling window, not a global map). |
| 2 | LiDAR glare, reflections, or dropouts create phantom obstacles. | Per-beam range validation (NaN, inf, below min, above max). Log-odds clamped to a fixed band so no single return locks a cell; asymmetric update (free slightly softer than occupied). |
| 3 | QoS mismatch between driver (`BEST_EFFORT`) and subscriber silently drops messages. | Subscriber uses `BEST_EFFORT` sensor QoS explicitly. Grid publisher uses `TRANSIENT_LOCAL` so late RViz / Nav2 subscribers still receive the latest map without a restart. |
| 4 | CPU pressure on Jetson causes publish lag. | Scan integration is decoupled from publishing (integration on callback, publish on a 5 Hz timer). Ray tracing uses integer Bresenham, not floating-point. `grid.size_m` and `grid.resolution_m` are parameters so the operator can trade fidelity for CPU. |
| 5 | Dynamic obstacles linger in the grid after they leave. | Asymmetric log-odds rates (free updates faster than occupied would be safer); longer term, add exponential decay of `l_occ` when a cell is repeatedly observed free. |

---

## 4. What I would validate first in a real lab

Validation is split into **correctness** (does the pipeline do what it claims?) and **deployment readiness** (does it hold up on a Jetson, over hours, with real sensors?). The six tests below are ordered by what I would run on day one.

| Test | Method | Pass criterion |
|---|---|---|
| Static-scene grid convergence | Park the robot. Record 30 s of real scans. Inspect the grid in RViz. | Walls form contiguous black cells; open floor is white; salt-and-pepper noise <2 % in known regions. |
| Ray-trace correctness (unit test) | Inject a single `LaserScan` with one return at (2.0, 0.0); step the integrator once; inspect the log-odds array. | Cells along the ray are marked free (p<0.4); endpoint is occupied (p>0.6); off-ray cells remain unknown (−1). |
| End-to-end loop latency | Measure scan-timestamp to `/local_costmap` publish timestamp on the target Jetson under nominal load. | p95 <150 ms; p99 <250 ms; zero dropped messages over a 10 min run. |
| QoS compatibility with a real driver | Replace `fake_sensor_publisher` with the `sllidar_ros2` driver (RPLIDAR A2/A3) and re-run. | Node continues publishing without QoS-mismatch warnings in `/rosout`. `rqt_graph` shows all connections live. |
| Odometry-drift robustness | Drive a 5 × 5 m square at 0.3 m/s; inject ±1 % rotational drift into `/odom` via a remap node. | Grid remains bounded; no runaway smearing within a 60 s window; obstacle markers stay associated with real walls in RViz. |
| Long-haul stability | Play back an 8 h recorded rosbag on the Jetson with memory tracing (`valgrind` / `tracemalloc`). | RSS growth <5 % over baseline; no segfaults; log-odds values remain inside the configured clamp band. |

---

## 5. Metrics we would watch in production

**Cost metrics.** CPU percent per core, resident memory, `/local_costmap` publish rate, scan-callback execution time (p50 / p95 / p99).

**Quality metrics.** Fraction of unknown cells inside the planner's danger radius, fraction of scan returns rejected as invalid (non-finite or out-of-range), log-odds saturation rate (cells hitting the clamp bounds per second).

**Safety metrics.** Number of frames where the planner's footprint overlaps an occupied cell but the robot did not stop or replan (target: 0).

---

## 6. What was used vs. what was written for this task

**Written from scratch for this submission:** `obstacle_grid_node`, `fake_sensor_publisher`, launch file, parameters YAML, README, RViz layout, and this technical report.

**Referenced (ideas, not code):** the log-odds occupancy-grid formulation comes from Thrun, Burgard, and Fox, *Probabilistic Robotics*, chapter 9. Bresenham's line algorithm is textbook. Message-type names and the `ament_python` package layout follow the standard ROS 2 `rclpy` tutorials. **No third-party code was vendored** into `src/`.

---

## 7. Reproduce in one command

```bash
cd autonomy_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select autonomy_pipeline
source install/setup.bash
ros2 launch autonomy_pipeline autonomy_pipeline.launch.py
```

Expected within 5 seconds: RViz shows a rolling grey/black/white local costmap, red obstacle markers near the robot, and the raw `/scan` returns.
