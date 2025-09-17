# Use ROS 2 Humble as base image with Gazebo
FROM osrf/ros:humble-desktop-full

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/home/ros/workspace

# Install additional dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-wstool \
    git \
    wget \
    curl \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Garden (newer version for better compatibility)
RUN curl -sSL http://get.gazebosim.org | sh

# Install additional ROS 2 packages needed for the simulation
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-rviz2 \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR $WORKSPACE

# Copy the project files (now from the two_link_arm_sim directory)
COPY . src/two_link_arm_sim/

# Update rosdep and install dependencies (skip init as it's already done in base image)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select two_link_arm_sim"

# Source the workspace
RUN echo "source $WORKSPACE/install/setup.bash" >> /home/ros/.bashrc

# Set up environment for GUI applications (for RViz and Gazebo)
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Create a launch script for easy startup
RUN echo '#!/bin/bash\n\
echo "DrRoboto - Two Link Arm Simulation"\n\
echo "=================================="\n\
echo "Available launch options:"\n\
echo "1. Manual control: ros2 launch two_link_arm_sim display.launch.py"\n\
echo "2. Auto control: ros2 launch two_link_arm_sim auto_control.launch.py"\n\
echo "3. Gazebo simulation: ros2 launch two_link_arm_sim arm_simulation.launch.py"\n\
echo ""\n\
echo "Starting with manual control mode..."\n\
source /opt/ros/humble/setup.bash\n\
source $WORKSPACE/install/setup.bash\n\
ros2 launch two_link_arm_sim display.launch.py' > /home/ros/start_simulation.sh

RUN chmod +x /home/ros/start_simulation.sh

# Keep using root user for simplicity
# USER ros

# Expose ports for Gazebo and RViz
EXPOSE 11345 6080

# Default command
CMD ["/home/ros/start_simulation.sh"]
