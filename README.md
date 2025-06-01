# rrt_star_planner

## RRT* 3D Path Planner for ROS

A ROS package implementing the RRT* (Rapidly-exploring Random Tree Star) algorithm for 3D path planning with point cloud-based obstacle avoidance.

## Overview

This package provides a 3D path planning solution using the RRT* algorithm, which finds near-optimal paths in 3D space while avoiding obstacles. The planner represents the robot as a sphere and uses point cloud data for collision checking.

### Key Features

- **3D RRT* Implementation**: Efficient path planning in 3D space
- **Point Cloud Collision Checking**: Uses KD-tree for fast collision detection
- **Path Optimization**: Includes rewiring for near-optimal paths and post-processing smoothing
- **Real-time Visualization**: Visualize the RRT tree and planned path in RViz
- **ROS Service Interface**: Easy integration with other ROS nodes
- **Configurable Parameters**: Adjust planning behavior via ROS parameters

## Installation

## Usage

### Running the Planner

1. Launch the RRT* planner node:

```bash
roslaunch rrt_star_planner rrt_star_planner.launch
```

### Service Interface

The planner provides a ROS service for path planning:

**Service**: `/plan_path`
**Type**: `rrt_star_planner/PlanPath`

**Request**:

```ros
geometry_msgs/PoseStamped goal_pose
```

**Response**:

```ros
barracuda_msgs/Waypoints waypoints
```

Example service call:

```bash
rosservice call /plan_path "goal_pose:
  header:
    frame_id: 'world'
  pose:
    position:
      x: 5.0
      y: 5.0
      z: 2.0
    orientation:
      w: 1.0"
```

### Topics

#### Subscribed Topics

- `/robot_pose` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
  - Current robot position

- `/obstacle_pointcloud` ([sensor_msgs/PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html))
  - Point cloud representing obstacles in the environment

#### Published Topics

- `/planned_path` ([barracuda_msgs/Waypoints](https://github.com/usc-robosub/barracuda_msgs/tree/main))
  - The planned path as a series of waypoints

- `/planned_path_viz` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
  - Visualization of the planned path for RViz

- `/rrt_tree` ([visualization_msgs/MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html))
  - Visualization of the RRT tree structure

## Parameters

Configure the planner behavior by setting these parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~max_step_size` | double | 0.5 | Maximum distance for each tree extension (meters) |
| `~goal_tolerance` | double | 0.3 | Distance threshold to consider goal reached (meters) |
| `~robot_radius` | double | 0.5 | Radius of robot sphere for collision checking (meters) |
| `~max_iterations` | int | 5000 | Maximum number of RRT* iterations |
| `~rewire_radius` | double | 1.0 | Radius for finding nearby nodes to rewire (meters) |
| `~goal_bias_probability` | double | 0.1 | Probability of sampling the goal directly (0-1) |
| `~x_min` | double | -10.0 | Minimum x coordinate for sampling (meters) |
| `~x_max` | double | 10.0 | Maximum x coordinate for sampling (meters) |
| `~y_min` | double | -10.0 | Minimum y coordinate for sampling (meters) |
| `~y_max` | double | 10.0 | Maximum y coordinate for sampling (meters) |
| `~z_min` | double | 0.0 | Minimum z coordinate for sampling (meters) |
| `~z_max` | double | 5.0 | Maximum z coordinate for sampling (meters) |
