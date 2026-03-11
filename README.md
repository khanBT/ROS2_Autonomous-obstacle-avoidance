# ROS2 Autonomous Obstacle Avoidance

This project implements an autonomous obstacle avoidance system using ROS2 and Python in a TurtleBot3 simulation environment.

The robot processes LiDAR data in real time and adjusts its movement to safely navigate around obstacles.

---

## Project Demo

![Gazebo Simulation](images/gazebo_simulation.png)

---

## Key Features

- Class-based ROS2 node architecture
- Real-time LiDAR (LaserScan) processing
- Sector-based direction tracking using robot yaw
- Autonomous obstacle avoidance decision logic
- Velocity control through `/cmd_vel`

---

## Technologies Used

- ROS2
- Python
- Gazebo
- LiDAR (LaserScan)
- Robot Navigation Algorithms

---

## System Architecture

```
LaserScan → Obstacle Detection → Direction Tracking → Velocity Command (/cmd_vel)
```

---

## Project Structure

```
ros2-autonomous-obstacle-avoidance
│
├── src
│   └── obstacle_avoider.py
│
├── images
│   ├── gazebo_simulation.png
│   └── lidar_visualization.png
│
├── videos
│   └── demo.mp4
│
└── README.md
```

---

## How It Works

1. The robot subscribes to `/scan` to receive LiDAR data.
2. The system calculates minimum obstacle distances.
3. The robot determines its direction using yaw orientation.
4. An avoidance decision is made based on obstacle position.
5. Velocity commands are published to `/cmd_vel`.

---

## Future Improvements

- SLAM integration
- Path planning algorithms
- Reinforcement learning navigation
