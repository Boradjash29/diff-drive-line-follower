# ROS 2 Line Following Robot 🤖🚗

A complete ROS 2 (Humble) simulation package for a differential drive robot designed to autonomously follow a complex path using computer vision. 

The robot is simulated in Gazebo Classic, utilizing OpenCV to process camera feeds, detect a line on the ground, and use a custom PID controller to smoothly navigate sharp turns, straightaways, and intricate track layouts.

---

## 🎯 Features

- **ROS 2 Humble Architecture**: Built entirely with modern ROS 2 paradigms (Python nodes, custom launch files).
- **Computer Vision Pipeline**: Real-time image processing via OpenCV (`cv_bridge`) to threshold, crop, and identify track centroids.
- **PID Control System**: Smooth, dynamic navigation that actively adjusts speed—slowing down during sharp 90-degree turns and speeding up on straight sections.
- **Line Recovery Logic**: If the robot briefly loses sight of the line, it remembers its last known error and rotates to reacquire the path instead of simply stopping.
- **Custom Gazebo Environment**: Includes a custom-built, highly complex `advanced.world` track with intricate sharp curves and varying angles to rigorously test the control algorithms.
- **URDF / Xacro Robot Model**: A configurable differential drive robot equipped with a downward-facing camera and a `skid_steer_drive` plugin for accurate physics simulation.

---

## 📁 Package Structure

```text
diff_line_following_robot/
├── launch/
│   └── main.launch.py        # Central launch file (Gazebo, RViz, Spawning, Vision, Control)
├── models/
│   └── advanced_track/       # Custom Gazebo models for the complex line track
├── scripts/
│   ├── controller_node.py    # PID navigation and recovery logic
│   └── line_detector_node.py # Image thresholding and centroid calculations
├── urdf/
│   └── robot.urdf.xacro      # Robot physical parameters and Gazebo plugins
├── worlds/
│   └── advanced.world        # The simulation environment loading the track
├── CMakeLists.txt
└── package.xml
```

---

## 🛠️ Prerequisites

Ensure you have the following installed on your Ubuntu system:
- **ROS 2 Humble**
- **Gazebo Classic 11** (`ros-humble-gazebo-ros-pkgs`)
- **OpenCV & cv_bridge** (`ros-humble-cv-bridge`)
- **Xacro** (`ros-humble-xacro`)

---

## 🚀 Installation & Build

1. **Clone the repository** into your ROS 2 workspace `src` directory:
   ```bash
   cd ~/your_ws/src
   git clone <your-repo-url> diff_line_following_robot
   ```

2. **Build the workspace**:
   ```bash
   cd ~/your_ws
   colcon build --packages-select diff_line_following_robot
   ```

3. **Source the setup file**:
   ```bash
   source install/setup.bash
   ```

---

## 🎮 Running the Simulation

To launch the entire stack (Gazebo world, robot spawn, camera processor, and PID controller):

```bash
ros2 launch diff_line_following_robot main.launch.py
```

The robot will spawn at the exact starting coordinate (`x=-6.5`, `y=0.0`) of the advanced track and immediately begin following the line!

---

## 🧠 How It Works

1. **Vision (`line_detector_node.py`)**: 
   - Subscribes to the raw camera feed (`/camera/image_raw`).
   - Crops the top portion of the image to focus only on the ground immediately ahead.
   - Converts the image to grayscale and applies an inverse binary threshold to isolate the black line.
   - Calculates the moments to find the centroid (center of mass) of the largest contour.
   - Publishes the horizontal offset (error) from the center of the camera frame to the topic `/line_error`.

2. **Control (`controller_node.py`)**:
   - Subscribes to `/line_error`.
   - Passes the error through a Proportional-Integral-Derivative (PID) algorithm to calculate the necessary angular velocity (steering).
   - Adjusts the linear velocity dynamically (e.g., slowing down when the error is large to negotiate sharp turns).
   - Publishes velocity commands to `/cmd_vel` to drive the robot.
   - **Failsafe**: If the camera loses the line, it uses the last known error direction to rotate in place until the line comes back into view.

---

## ⚙️ Tuning the PID

If you modify the track or the robot's physical properties, you may need to adjust the PID constants. Open `scripts/controller_node.py` and modify the following variables in the `__init__` function:

```python
self.kp = 0.015  # Proportional
self.ki = 0.000  # Integral
self.kd = 0.005  # Derivative
```

