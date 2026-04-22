# Project Guide

**Track A — Perception to Local Mapping · ROS 2 autonomy pipeline for a mobile robot**

This guide is written for someone who has never used ROS 2 before. It explains what the task is, what every file in the submission does, how the code works step by step, how to run it, and how to submit. Read it top to bottom once and you will understand the whole package.

---

## 1. What the task is, in plain English

The evaluator is hiring for an **Automation Expert Intern** role. The role is about making mobile robots (robots that drive themselves, like a warehouse AGV or an office cleaning robot) actually work in the real world. A working mobile robot has three big software parts:

1. It **sees** — perception: LiDAR, depth camera, IMU.
2. It **knows where it is** — localisation and mapping.
3. It **decides where to go** — planning and control.

The task asks the candidate to build a tiny but real version of parts (1) and (2). Specifically, **Track A** says: write a ROS 2 package that takes in LiDAR data and odometry, builds a small map of obstacles around the robot, and publishes that map on a standard ROS topic so a planner could use it.

The evaluator does **not** expect a perfect algorithm. They want to see:

- clean ROS 2 code (correct nodes, topics, message types, QoS, launch files, parameter YAMLs),
- honest engineering judgement (assumptions, failure modes, validation plan),
- reproducible setup (anyone can clone and run it in one command).

### 1.1 Words you need to know

| Term | What it means |
|---|---|
| **ROS 2** | Robot Operating System, version 2. Not an actual operating system — it is a middleware. It lets programs (called "nodes") talk to each other by publishing and subscribing to named "topics", a bit like a chat app for robots. |
| **Node** | A single running program in ROS 2. The submission has one production node and one test node. |
| **Topic** | A named stream of messages. A publisher writes to it; subscribers read from it. Example: `/scan` is the topic where LiDAR readings arrive. |
| **Message type** | The schema of a topic. `/scan` carries `sensor_msgs/LaserScan` messages — a fixed structure with distances, angles, timestamps, etc. |
| **LiDAR** | A sensor that shines laser beams around itself and measures how long they take to come back. You get a ring of distance readings — essentially, "how far is the nearest thing in each direction?". |
| **Odometry** | The robot's own estimate of its position, usually from wheel encoders. It drifts over time but is accurate short-term. |
| **Occupancy grid** | A top-down 2D map where every cell is either *free*, *occupied*, or *unknown*. The classic format a ROS 2 planner consumes. |
| **RViz** | The standard ROS 2 3D viewer. Shows sensor data and maps live. |
| **Jetson** | An NVIDIA embedded computer common on robots — small, low-power, has a GPU. The task says "Jetson-class compute", so we target 6–8 CPU cores, 8 GB RAM. |
| **colcon** | The ROS 2 build tool. `colcon build` compiles all packages in a workspace. |
| **Launch file** | A Python script that starts a bundle of nodes together with shared parameters. Saves typing. |
| **QoS** | Quality-of-Service. How reliable a topic is. Sensors use `BEST_EFFORT` (drop old messages if overwhelmed); commands use `RELIABLE` (must deliver). Mismatched QoS is the #1 source of "why is my topic silent?" bugs. |

---

## 2. What the submission does

One ROS 2 node, **`obstacle_grid_node`**, listens on two input topics:

- **`/scan`** — LiDAR readings, 10 times a second. Type: `sensor_msgs/LaserScan`.
- **`/odom`** — robot position estimate, 50 times a second. Type: `nav_msgs/Odometry`.

For every LiDAR reading, it:

1. **Validates** the reading — throws out infinities, NaNs, and out-of-range values.
2. **Transforms** the beam into the world using the current odometry pose (the beam arrives in the robot's own frame; we need it in a static frame to put it on a grid).
3. **Ray-traces** a line across its internal grid from the robot to where the beam hit. Cells *along* the line are marked **more likely free**. The cell at the hit itself is marked **more likely occupied**. This is called a **log-odds update** — a standard probability trick so noise averages out over many readings instead of corrupting the map.

Five times a second, the node converts that internal grid into a standard ROS 2 map message (`nav_msgs/OccupancyGrid`) and publishes it on **`/local_costmap`**. It also publishes red "danger" sphere markers for occupied cells close to the robot on **`/obstacles`**, so RViz shows them clearly.

A second node, **`fake_sensor_publisher`**, exists only so the reviewer can run the whole pipeline **without owning a real robot**. It simulates a small square room with four obstacles and a robot driving in a curve, producing `/scan` and `/odom` messages that look exactly like a real sensor's. The production node cannot tell the difference.

---

## 3. What every file does

The package follows the standard ROS 2 Python package layout. The table below lists every file and why it exists. If a reviewer wonders "why is this here?", the answer should be in this table.

| Path | Purpose |
|---|---|
| `autonomy_ws/README.md` | Project README. Build/run instructions, topics, parameters, what to improve with more time. The first thing a reviewer reads. |
| `autonomy_ws/technical_report.md` | The 2-page technical report the task asks for (architecture, assumptions, risks, validation plan). |
| `autonomy_ws/project_guide.md` | This file. |
| `src/autonomy_pipeline/package.xml` | ROS 2 package manifest. Lists dependencies, build type (`ament_python`), maintainer. Without this, `colcon` does not know the folder is a package. |
| `src/autonomy_pipeline/setup.py` | Python packaging file. Registers the two executables (`obstacle_grid_node`, `fake_sensor_publisher`) as console scripts and tells `colcon` where to install launch/config/rviz files. |
| `src/autonomy_pipeline/setup.cfg` | Tells setuptools where to put the installed scripts so ROS 2 finds them. One-line boilerplate required by `ament_python`. |
| `src/autonomy_pipeline/resource/autonomy_pipeline` | Empty marker file. Required by ament so it can index the package. |
| `autonomy_pipeline/__init__.py` | Makes the folder a Python package. Empty on purpose. |
| `autonomy_pipeline/obstacle_grid_node.py` | The main production node. Log-odds grid, Bresenham ray tracing, LaserScan validation, `OccupancyGrid` and `MarkerArray` publishers. All magic numbers exposed as ROS parameters. |
| `autonomy_pipeline/fake_sensor_publisher.py` | Synthetic LiDAR and odometry publisher. Analytical ray-segment and ray-circle intersections, Gaussian range noise, random beam dropouts. Zero hardware required. |
| `launch/autonomy_pipeline.launch.py` | One-command startup: fake sensors + grid node + RViz. Launch arguments let the reviewer disable the fake feed (`use_fake:=false`) or run headless (`use_rviz:=false`). |
| `config/params.yaml` | All tunable parameters in one file. Grid resolution and size, log-odds values, topic names, publish rates. No code changes required to re-tune. |
| `rviz/autonomy.rviz` | Pre-saved RViz layout. The reviewer does not have to click through menus — the costmap, markers, and scan are already added with the right QoS. |

---

## 4. How `obstacle_grid_node` works, step by step

### 4.1 Initialisation

On startup, the node *declares* every parameter with a sensible default, then reads values from `config/params.yaml`. It allocates a NumPy array for the log-odds grid and creates subscribers and publishers with matching QoS:

- Subscribers (`/scan`, `/odom`): `BEST_EFFORT`, depth 10. This matches real LiDAR and odometry drivers.
- Publisher (`/local_costmap`): `RELIABLE` + `TRANSIENT_LOCAL`, depth 1. So RViz or Nav2 always get the latest map, even if they start after the node.
- Publisher (`/obstacles`): `RELIABLE`, depth 10. Plain RViz default.

### 4.2 On every `/odom` message

The message carries the robot's pose as `(x, y, z)` plus a quaternion (a 4-number representation of 3D orientation). Only `(x, y, yaw)` matter for a 2D grid. The node converts the quaternion to a yaw angle in radians with a short helper function (`quaternion_to_yaw`) and stores it.

### 4.3 On every `/scan` message

A `LaserScan` has `angle_min`, `angle_max`, `angle_increment`, and a flat list of N range readings. The node loops through each beam:

- If a range is NaN or infinity, treat it as "no hit" at max range.
- If below `range_min`, ignore (probably the robot's own body).
- If above `range_max`, treat as "no hit".
- Otherwise, compute the world-frame endpoint by rotating `(r·cosθ, r·sinθ)` by the robot's yaw and adding the robot's position.

Then the node converts the robot position and the endpoint to grid cell indices. **Bresenham's line algorithm** is used to walk the straight line between them, one integer cell at a time. Every cell on that line (except the endpoint) gets its log-odds nudged **down** (more likely free) by a small negative number. The endpoint itself gets its log-odds nudged **up** (more likely occupied) by a larger positive number.

After all beams have been processed, the log-odds array is clamped to a safe range (values of −2 to 3.5). This stops any single cell from becoming "100 % certain" and unable to recover if the world changes later.

### 4.4 On a 5 Hz timer — publish the grid

Every 200 ms, the node:

1. Converts the log-odds `float32` array into the `OccupancyGrid` integer format: cells with log-odds near zero become `-1` (unknown); the rest become `0..100` (probability of being occupied, as a percent).
2. Fills in the `OccupancyGrid.info` struct — resolution, width, height, and **origin** (the grid recentres on the robot, so origin moves with the robot).
3. Publishes on `/local_costmap`.
4. Builds a `MarkerArray` of red spheres for occupied cells within 60 cm of the robot and publishes on `/obstacles`.

---

## 5. How to build and run

**Required:** Ubuntu 22.04 with ROS 2 Humble, **or** Ubuntu 24.04 with ROS 2 Jazzy. On Windows or macOS, run Ubuntu inside WSL2 or a VM first.

### 5.1 One-time setup

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rclpy \
                 ros-${ROS_DISTRO}-rviz2 \
                 python3-numpy \
                 python3-colcon-common-extensions
```

### 5.2 Build the workspace

```bash
cd autonomy_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --packages-select autonomy_pipeline
source install/setup.bash
```

### 5.3 Run the demo

```bash
ros2 launch autonomy_pipeline autonomy_pipeline.launch.py
```

RViz opens, the synthetic sensors start, and within about five seconds you should see a grid filling in (white = free, black = occupied, grey = unknown) and red sphere markers near the robot whenever it gets close to an obstacle.

### 5.4 Useful variations

| Command | Effect |
|---|---|
| `ros2 launch autonomy_pipeline autonomy_pipeline.launch.py use_rviz:=false` | No RViz — useful on a Jetson accessed over SSH. |
| `ros2 launch autonomy_pipeline autonomy_pipeline.launch.py use_fake:=false` | Disable synthetic sensors — feed your own `/scan` and `/odom` (e.g. from a rosbag). |
| `ros2 launch autonomy_pipeline autonomy_pipeline.launch.py log_level:=debug` | Verbose logging. |
| `ros2 bag play /path/to/bag --clock` | Replay a recorded bag into the pipeline (with `use_fake:=false`). |
| `ros2 topic hz /local_costmap` | Sanity check — should be about 5 Hz. |
| `ros2 topic echo --once /local_costmap \| head` | Dump the first map message to verify it is being produced. |

---

## 6. Parameters you can tune without touching code

All parameters live in `src/autonomy_pipeline/config/params.yaml`. Change a value, rebuild (`colcon build`), and relaunch.

| Parameter | Default | Effect |
|---|---|---|
| `grid.resolution_m` | 0.05 | Cell size in metres. Smaller = more detail, more CPU and memory. |
| `grid.size_m` | 10.0 | Length of the square window around the robot. 10 m @ 5 cm = 200×200 cells. |
| `scan.max_range_m` | 8.0 | LiDAR returns beyond this distance are treated as max-range misses. Saves ray-tracing time. |
| `scan.min_range_m` | 0.10 | Returns closer than this are ignored — usually the robot's own body or sensor housing. |
| `publish.rate_hz` | 5.0 | How often the `OccupancyGrid` is republished. Most planners are happy at 5 Hz. |
| `logodds.l_occ` | 0.85 | How much a single "hit" increases a cell's probability of being occupied. Higher = converges faster but more sensitive to noise. Value is ≈ `ln(0.7/0.3)`. |
| `logodds.l_free` | −0.40 | How much a beam passing through a cell decreases its probability. Softer than `l_occ` on purpose. |
| `logodds.l_min` / `logodds.l_max` | −2.0 / 3.5 | Clamp values. Stops any cell from becoming "unrecoverable". |
| `markers.danger_radius_m` | 0.6 | Only occupied cells within this radius of the robot are published as red sphere markers. |

---

## 7. Data source options

The task lets the candidate pick one of three sources. This submission uses **option (a)** by default so the reviewer needs nothing but ROS 2 installed.

- **(a) Synthetic (default).** `fake_sensor_publisher` produces a noisy 360-beam 2D scan and a ground-truth odometry feed inside a simulated 6 m × 6 m room with four circular obstacles. The noise model is Gaussian on range (σ = 2 cm) plus a 1 % random beam dropout.
- **(b) Rosbag.** Replay any public or private bag with `sensor_msgs/LaserScan` on `/scan` and `nav_msgs/Odometry` on `/odom`. Use `ros2 bag play` with `use_fake:=false`.
- **(c) Simulator.** The package also works with a Gazebo or Ignition world that publishes the same topics; no code change required.

Public rosbags known to work out of the box:

- MIT Stata Center dataset (`/scan`, `/odom`)
- Any TurtleBot3 Gazebo recording

---

## 8. How to extend the code

If more time were available, these are the natural next steps, ordered by value per hour of work:

1. **Vectorised ray tracing.** Replace the per-beam Python loop with NumPy broadcasting. ~10× faster on a Jetson.
2. **TF2 integration.** Stop treating `odom` as the world. Listen to the standard `map → odom → base_link` transform chain.
3. **Unit tests.** Pytest fixtures for Bresenham, `quaternion_to_yaw`, and log-odds round-trip. Already stubbed in `test/`.
4. **Dynamic-obstacle decay.** If a cell is consistently seen free, exponentially decay its occupied log-odds so stale obstacles fade.
5. **Nav2 costmap plugin.** Wrap the output as a Nav2 costmap layer so a planner consumes it with zero glue code.
6. **CI.** GitHub Actions: `colcon build` + launch a 5-second rosbag fixture + assert expected topics exist.

---

## 9. How to submit

The task's submission rules are: reply to the email with a GitHub link or ZIP, a start and finish timestamp, machine specs, OS, and notes on what you would improve with more time. Here is a concrete checklist.

**Step 1 — Timestamps.** Note the timestamp when the task was first opened (start) and the timestamp right before sending (finish).

**Step 2 — Machine specs.** Open a terminal and run:

```bash
uname -a
lscpu | head -n 5
free -h
```

Copy the CPU model, core count, RAM, and OS/kernel into the email body.

**Step 3 — Dry run.** Unzip the submission on a clean Ubuntu, run the three build commands from section 5, and confirm RViz shows a growing map. This takes about three minutes and prevents a re-send.

**Step 4 — Attach.** Attach `autonomy_pipeline_submission.zip` (it already contains the README, the technical report, and this guide at the top level). Alternatively, push the `autonomy_ws` folder to a GitHub repo and paste the URL.

**Step 5 — Email body.** Include: start/finish timestamps, machine specs, ROS 2 distro, one paragraph summarising what was built, and a short list of "what I would improve with more time" (copy from section 8).

**Step 6 — Send.** Keep the local ZIP and the report until the evaluator confirms receipt.

> **Note.** The task text asks for the technical report as a PDF. Both `technical_report.md` and `project_guide.md` can be converted to PDF in any editor: in VS Code, install the "Markdown PDF" extension and run "Markdown PDF: Export (pdf)" on each file. Or run `pandoc technical_report.md -o technical_report.pdf` from the terminal. Either way, the markdown is the source of truth.

---

## 10. Common mistakes to avoid

- **Forgetting to `source install/setup.bash`** after `colcon build`. The shell does not know the new nodes exist until you do. Source it in every new terminal.
- **QoS mismatch.** If you point the node at a real LiDAR and see nothing, 99 % of the time it is QoS. This package already uses `BEST_EFFORT` on `/scan`; do not change it.
- **Fixed frame wrong in RViz.** Must be `odom` (not `map` and not `base_link`) for the default parameters. The saved RViz config already sets this.
- **Python path surprises.** Always run through `ros2 run` or `ros2 launch`, never `python3 obstacle_grid_node.py`. The latter bypasses ROS 2's entry points and parameter plumbing.
- **Running the launch twice in the same terminal.** Each instance tries to bind the same RViz window and parameters. Use `Ctrl+C` to stop before relaunching.
