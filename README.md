# 🧠 maze_solver_bot

A ROS 2 package that enables a TurtleBot3 to autonomously solve a random maze using **Lidar**, **IMU**, and **Odometry** — built for the Maze Navigation Challenge.

---

## 🎯 Challenge Objective

- **Start Position:** `(-5.25, -5.25)`
- **Goal:** Reach the central black square (1x1, centered at the origin)
- **Constraints:** 
  - No top-down/vision sensors
  - Bot should not be stuck for more than 30 seconds
  - Best of two attempts is scored based on fewest steps

✅ This bot uses only **allowed sensors**:
- ✅ Lidar (`/scan`)
- ✅ Wheel Odometry (`/odom`)
- ✅ IMU (`/imu`)

---

## 🚀 Features

- 🧱 **Wall-following algorithm** using Lidar
- 🧭 **Odometry tracking** to detect movement
- 🌀 **IMU integration** (can be extended for orientation control)
- 🔁 **Recovery behavior** if the bot gets stuck
- ⚙️ Simple to launch and customize

---

## 🛠️ Installation

1. Clone inside your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   unzip maze_solver_bot.zip
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

---

## 🧪 Running the Bot

1. Launch the maze environment (example for TurtleBot3):
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. Launch the maze solver:
   ```bash
   ros2 launch maze_solver_bot wall_follower.launch.py
   ```

---

## 📦 Package Structure

```
maze_solver_bot/
├── maze_solver_bot/
│   └── wall_follower.py     # Main logic
├── launch/
│   └── wall_follower.launch.py
├── setup.py
├── package.xml
└── README.md
```

---

## 🧠 How It Works

- Uses Lidar to follow the left wall
- Monitors `/odom` to detect if the robot is stuck
- Triggers recovery spin if stuck for too long
- Smooth and safe movement through tight spaces

---

## 🛡️ License

MIT License

---

## ✨ Author

Made with ❤️ for the Maze Navigation Challenge.
