# Maze Solver Robot using BUG2 + SLAM (ROS 2 + Gazebo)

## Project Overview

This project implements an autonomous maze-solving robot using the **BUG2 path planning algorithm** combined with a simple **SLAM (Simultaneous Localization and Mapping)** system in **ROS 2** and **Gazebo Simulation**.

The robot navigates through a maze environment, avoids obstacles using LiDAR sensor data, follows walls when blocked, and moves toward a predefined goal position. At the same time, it generates an occupancy grid map of the environment for visualization and analysis.

This project demonstrates concepts from:

* Robotics
* Path Planning
* Autonomous Navigation
* SLAM
* ROS 2
* Gazebo Simulation
* Sensor-based Obstacle Detection

---

## Features

* Autonomous maze navigation
* BUG2 obstacle avoidance algorithm
* Right-hand wall following
* Goal-directed navigation
* LiDAR-based obstacle detection
* Real-time occupancy grid map generation
* Robot pose tracking using odometry
* Path visualization in RViz
* Gazebo simulation environment

---

## Technologies Used

* Python
* ROS 2 (rclpy)
* Gazebo Simulator
* RViz2
* LiDAR Sensor
* Odometry
* Occupancy Grid Mapping
* MultiThreadedExecutor

---

## Project Structure

```text
maze_solver/
│
├── launch/
│   └── sim.launch.py
│
├── scripts/
│   ├── maze_solver.py
│   └── lidar_processor.py
│
├── models/
│   └── robot.sdf
│
├── worlds/
│   └── maze_world.sdf
│
├── package.xml
└── CMakeLists.txt
```

---

## Working Principle

## 1. GO TO GOAL Mode

The robot first tries to move directly toward the target goal location.

### Two Sub-steps:

### a) Rotation

The robot rotates in place until it aligns with the goal direction.

### b) Straight Drive

Once aligned, it moves straight toward the goal.

This avoids unnecessary curved motion and improves obstacle detection.

---

## 2. Obstacle Detection

Using LiDAR scan data:

* If no obstacle is detected → continue moving
* If obstacle detected ahead → switch to wall-following mode

Obstacle threshold is controlled using:

```python
OBSTACLE_DIST = 0.60
```

---

## 3. Wall Following Mode

The robot uses **Right-Hand Wall Following**.

It:

* maintains distance from the right wall
* adjusts turning dynamically
* avoids getting too close or too far

This helps the robot move around the obstacle safely.

---

## 4. M-Line Logic (BUG2)

The M-line is the straight line from:

```text
Start Point → Goal Point
```

The robot leaves wall-following only when:

* it reaches the M-line again
* it is closer to the goal than before
* it has followed the wall for a minimum safe distance

This prevents premature exits.

---

## 5. SLAM Mapping

The project also performs simple occupancy grid mapping.

### Process:

* Laser rays are projected
* Bresenham line algorithm marks free cells
* Hit points mark occupied cells
* Map is continuously published to `/map`

This creates a real-time 2D map of the maze.

---

## Main Files Explanation

## `maze_solver.py`

This is the core file.

### Contains:

### `SlamMapper`

Responsible for:

* map generation
* occupancy grid publishing
* robot pose estimation
* laser scan processing

### `Bug2Navigator`

Responsible for:

* goal navigation
* obstacle avoidance
* wall following
* path publishing
* state switching

### States Used

```python
ROTATING
DRIVING
FOLLOWING
DONE
```

---

## `lidar_processor.py`

This file processes raw LiDAR scan values.

It extracts minimum distances for:

* Front
* Right
* Left
* Back

and publishes them to:

```text
/obstacle_data
```

This simplifies obstacle awareness.

---

## `sim.launch.py`

This launch file starts the complete simulation.

### It launches:

* Gazebo simulation
* robot spawning
* ROS ↔ Gazebo bridge
* maze solver node
* optional RViz visualization

This is the main execution file.

---

## How to Run

## Step 1: Build Workspace

```bash
colcon build
```

---

## Step 2: Source Workspace

```bash
source install/setup.bash
```

---

## Step 3: Launch Simulation

```bash
ros2 launch maze_solver sim.launch.py
```

---

## Expected Output

You should see:

* Gazebo opens with maze world
* Robot spawns inside maze
* Robot starts autonomous navigation
* Occupancy grid map updates live
* Goal reached successfully
* Path visible in RViz (if enabled)

---

## Important Parameters

```python
GOAL_X = 3.0
GOAL_Y = -3.0
GOAL_RADIUS = 0.4

LINEAR_SPEED = 0.20
TURN_SPEED = 0.50

OBSTACLE_DIST = 0.60
WALL_FOLLOW_DIST = 0.50
```

These can be tuned based on maze complexity.

---

## Algorithm Used

## BUG2 Algorithm

BUG2 is a reactive path planning algorithm.

### Why BUG2?

Because it is:

* simple
* reliable
* memory efficient
* ideal for unknown environments
* easy to implement with LiDAR

It is widely used in maze solving and mobile robotics.

---

## Future Improvements

Possible enhancements:

* A* path optimization
* Dynamic obstacle handling
* Improved SLAM using GMapping
* Frontier-based exploration
* Camera integration
* Multi-goal navigation
* Real robot deployment

---

## Conclusion

This project successfully combines:

* autonomous navigation
* obstacle avoidance
* real-time mapping
* intelligent wall following

into a complete robotic maze-solving system.

It is a strong demonstration of practical robotics concepts using ROS 2 and simulation-based testing.

---

## Author

Developed as part of Robotics / Autonomous Navigation Project using ROS 2 and Gazebo.
