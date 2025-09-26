#!/bin/bash

# This script will exit immediately if any command fails.
set -e

echo "--- Starting AeroSky OS Installation ---"

# --- Step 1: Update System and Install Core Tools ---
echo "[1/6] Updating system and installing core tools..."
sudo apt-get update && sudo apt-get upgrade -y
sudo apt-get install -y git python3-pip python3-venv curl software-properties-common colcon-common-extensions

# --- Step 2: Set up ROS 2 Repository ---
echo "[2/6] Setting up ROS 2 sources..."
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# --- Step 3: Install ROS 2 and Micro-ROS Agent ---
echo "[3/6] Installing ROS 2 Jazzy and Micro-ROS Agent..."
sudo apt-get update
sudo apt-get install -y ros-jazzy-desktop ros-jazzy-micro-ros-agent

# --- Step 4: Install PX4 Autopilot ---
echo "[4/6] Cloning and setting up PX4 Autopilot..."
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ~/PX4-Autopilot/Tools/setup/ubuntu.sh

# --- Step 5: Clone AeroSky OS Apps ---
echo "[5/6] Cloning AeroSky OS application workspace..."
mkdir -p ~/aerosky_ws/src
# !!! IMPORTANT: Replace this URL with your actual GitHub repository URL !!!
git clone https://github.com/Mr-McSizzle/resqnetOS/ ~/aerosky_ws/src

# --- Step 6: Build the AeroSky OS Apps ---
echo "[6/6] Building AeroSky OS apps with colcon..."
source /opt/ros/jazzy/setup.bash
cd ~/aerosky_ws
colcon build

echo ""
echo "--- AeroSky OS Installation Complete! ---"
echo "To get started, open a new terminal and run:"
echo "source ~/aerosky_ws/install/setup.bash"
