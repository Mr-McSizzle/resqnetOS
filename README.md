It seems your current view doesn't render Markdown formatting, causing you to see the raw code instead of formatted text.

Here is the same README content formatted as plain text for easier reading. When you save this as a README.md file on GitHub, it will display with the proper headings and bold text.

AEROSKY OS 🚁

THE OPEN-SOURCE OPERATING SYSTEM FOR DRONE SWARMS

AeroSky OS is a simulation-first, open-source platform for controlling autonomous drone swarms. Conceived as an "Android for the Sky," our mission is to democratize advanced drone technology for critical applications like emergency response, search and rescue, and logistics, with a special focus on tackling the unique environmental and connectivity challenges of India.

This repository contains the hackathon prototype, demonstrating the core OS, a sample mission application, and a proof-of-concept app marketplace.

== KEY FEATURES ==

Swarms on Demand: Built on the robust foundations of PX4 Autopilot and ROS 2, enabling complex multi-drone coordination.

Thriving App Ecosystem: A framework designed for a plug-and-play app ecosystem. Developers can contribute open-source mission apps or offer premium solutions through a marketplace.

India-Specific Design: Includes modules for integrating IMD weather data to navigate monsoon conditions and planned support for LoRa/SQLite for operations in low-connectivity rural areas.

Simulation-First: Develop, test, and validate complex swarm missions in the Gazebo Simulator without risking hardware.

Freemium "Sky" Standard: A free, open-source core (Apache 2.0) for the community, with a path to monetization through a premium app marketplace for enterprise users.

== TECH STACK ==

Operating System: Ubuntu 24.04 LTS

Core Middleware: ROS 2 Jazzy Jalisco

Flight Controller: PX4 Autopilot (SITL)

Simulator: Gazebo

App Development: Python 3

Marketplace PoC: Flask

== GETTING STARTED ==

Prerequisites:

A fresh installation of Ubuntu 24.04 LTS.

Git

Installation:
All dependencies and setup steps can be installed by running a single script.

COMMANDS:
# Clone this repository
git clone https://github.com/your-username/aerosky-os.git
cd aerosky-os

# Run the setup script to install all dependencies (ROS 2, PX4, etc.)
./setup.sh
== RUNNING THE HACKATHON DEMO ==

To demonstrate the full potential of AeroSky OS, follow this execution order precisely across four separate terminals.

TERMINAL 1: Launch the Simulator
This runs the drone and the virtual world.

COMMANDS:
# Navigate to the PX4 directory installed by the setup script
cd ~/PX4-Autopilot

# Launch the Gazebo simulator with a drone
make px4_sitl gz_x500
(Wait for the pxh> prompt to appear before proceeding.)

TERMINAL 2: Start the Communication Bridge
This connects the simulator to ROS 2.

COMMANDS:
# Source the ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Start the Micro-ROS agent
micrortps_agent -t UDP
TERMINAL 3: Verify Drone Readiness
This is a critical step to ensure the drone is ready to fly.

Wait 15-20 seconds after starting the bridge.

Go back to Terminal 1 and type the following at the pxh> prompt:

COMMAND:
ekf2 status

(Wait until the output shows global position: 1 and the health flags are true.)

Disable the GCS safety check for the simulation:

COMMANDS:
param set NAV_DLL_ACT 0
param save

TERMINAL 4: Run the Mission App
This runs the sample "takeoff and land" application.

COMMANDS:
# Navigate to your ROS 2 workspace
cd ~/aerosky_ws

# Source the workspace
source install/setup.bash

# Run the mission!
ros2 run autopilot_app mission_runner
You should now see the drone take off, hover, and land in the Gazebo window!

== THE APP ECOSYSTEM ==

AeroSky OS is designed to be extended through apps.

Community Apps: Developers can create their own mission scripts using our autopilot_app as a template. We encourage contributions for missions like search patterns, image capture, and more.

Premium Marketplace: Our proof-of-concept Flask web app demonstrates how enterprise users could purchase certified, high-performance apps for specialized tasks.

== ROADMAP ==

[ ] Hardware integration and testing on a physical drone.

[ ] Full implementation of IMD weather API for real-time storm avoidance.

[ ] Develop advanced swarm logic for formation flying.

[ ] Build out the full Flask marketplace with user authentication.

[ ] Integrate LoRa hardware for testing offline communication links.

== LICENSE ==

This project is licensed under the Apache 2.0 License. See the LICENSE file for details.
