# DrRoboto - Two-Link Robotic Arm Simulation

A ROS 2 educational project featuring a two-link robotic arm simulation with Gazebo physics and RViz visualization.

## 🤖 What is DrRoboto?

DrRoboto is a **ROS 2 (Robot Operating System)** educational project that simulates a two-link robotic arm. It includes:

- **Two-link articulated arm** with base, two rotating links, and end effector
- **Gazebo simulation** - 3D physics simulation environment  
- **Joint control** - Two revolute joints with position control
- **Predefined poses** - 4 different arm configurations
- **Automatic cycling** - Automatically cycles through poses every 3 seconds
- **Manual control** - Interactive joint control via GUI sliders
- **Real-time visualization** - RViz visualization

## 🐳 Running with Docker (Recommended)

### Prerequisites
- Docker and Docker Compose installed
- X11 forwarding enabled for GUI applications

### Quick Start
```bash
# Clone and navigate to the project
git clone <your-repo-url>
cd DrRoboto

# Allow X11 forwarding (run once)
xhost +local:docker

# Build and run with docker-compose (includes web interface)
docker-compose up --build

# Or run specific services:
docker-compose up drroboto-sim      # ROS 2 simulation only
docker-compose up drroboto-web      # Web interface only
docker-compose up drroboto-gazebo   # Gazebo physics simulation
```

### 🌐 Web Interface
The project now includes a modern web interface for controlling the robot:

- **Access**: http://localhost:5000
- **Features**: 
  - Interactive pose selection
  - Custom joint control with sliders
  - Real-time status monitoring
  - Wave gesture automation
  - Responsive design
- **Architecture**: Uses direct ROS 2 network communication (no Docker exec required)
- **Security**: No Docker socket access needed - much more secure

### Manual Docker Commands
```bash
# Build the Docker image
docker build -t drroboto .

# Run with manual control (RViz + Joint State Publisher GUI)
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume /home/$USER/.Xauthority:/home/ros/.Xauthority:rw \
  --network host \
  --privileged \
  drroboto

# Run Gazebo simulation
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume /home/$USER/.Xauthority:/home/ros/.Xauthority:rw \
  --network host \
  --privileged \
  drroboto \
  bash -c "source /home/ros/workspace/install/setup.bash && ros2 launch two_link_arm_sim arm_simulation.launch.py"
```

## 🚀 Running Natively (Without Docker)

### Prerequisites
- ROS 2 Humble installed
- Gazebo Garden installed
- Required ROS 2 packages

### Setup and Run
```bash
# Navigate to workspace
cd /home/dave/src/DrRoboto

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Run different simulation modes:

# 1. Manual control (RViz + Joint State Publisher GUI)
ros2 launch two_link_arm_sim display.launch.py

# 2. Automatic control (cycles through poses)
ros2 launch two_link_arm_sim auto_control.launch.py

# 3. Full Gazebo simulation (physics)
ros2 launch two_link_arm_sim arm_simulation.launch.py
```

## 🎮 How to Control the Robot

### Manual Control (display.launch.py)
- Use the **Joint State Publisher GUI** sliders to control joints in real-time
- See immediate visual feedback in RViz

### Programmatic Commands
```bash
# Send pose commands via ROS topics
ros2 topic pub /arm_pose_command std_msgs/String "data: 'home'"
ros2 topic pub /arm_pose_command std_msgs/String "data: 'pose1'"
ros2 topic pub /arm_pose_command std_msgs/String "data: 'pose2'"
ros2 topic pub /arm_pose_command std_msgs/String "data: 'pose3'"
```

### Monitor Robot Status
```bash
# Watch robot status messages
ros2 topic echo /arm_status

# Monitor joint states
ros2 topic echo /joint_states

# List all available topics
ros2 topic list
```

## 📁 Project Structure
```
DrRoboto/
├── docker-compose.yml        # Docker Compose setup
├── README.md                 # This file
├── control_robot.py          # Python control script (host)
├── control_robot_docker.py   # Python control script (Docker-based)
├── two_link_arm_sim/         # ROS 2 simulation package
│   ├── Dockerfile           # ROS 2 simulation Docker config
│   ├── src/                 # Python control nodes
│   ├── urdf/                # Robot description files
│   ├── launch/              # ROS 2 launch files
│   ├── worlds/              # Gazebo world files
│   └── config/              # Configuration files
└── webapp/                   # Flask web application
    ├── Dockerfile           # Web app Docker config
    ├── app.py               # Flask application
    ├── requirements.txt     # Python dependencies
    ├── templates/           # HTML templates
    └── static/              # CSS/JS assets
```

## 🎯 Available Poses
- **home**: [0°, 0°] - Straight up
- **pose1**: [45°, -45°] - Angled configuration  
- **pose2**: [-45°, 60°] - Different angle
- **pose3**: [90°, -30°] - Extended position

## 🛠 Technical Details
- **ROS 2 Humble** with Python nodes
- **Gazebo Garden** for physics simulation
- **URDF** (Unified Robot Description Format) for robot modeling
- **ros2_control** for joint control
- **RViz2** for visualization

## 🎓 Educational Value
This project teaches:
- Robot kinematics and dynamics
- Joint control and trajectory planning
- ROS 2 communication patterns
- Gazebo simulation concepts
- URDF robot modeling
- Docker containerization for robotics

## 🐛 Troubleshooting

### Docker GUI Issues
If GUI applications don't display:
```bash
# Allow X11 forwarding
xhost +local:docker

# Check DISPLAY variable
echo $DISPLAY

# Ensure X11 socket is mounted
ls -la /tmp/.X11-unix/
```

### ROS 2 Issues
```bash
# Check ROS 2 installation
ros2 --version

# Source the workspace
source install/setup.bash

# Check package installation
ros2 pkg list | grep two_link_arm_sim
```

## 📄 License
MIT License - See package.xml for details