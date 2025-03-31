# A* Path Planning on TurtleBot3 with FalconSim & ROS2

Welcome to **Project 03 - Phase 2** of ENPM661 Spring 2025. In this assignment, you will implement the A* algorithm for a differential-drive robot and simulate the results in a realistic warehouse environment using **FalconSim**.

## 🛠️ Requirements

| System Requirement | Minimum Spec |
|--------------------|--------------|
| OS                 | Ubuntu 22.04 |
| GPU                | Nvidia GeForce 2060 |
| RAM                | 16 GB        |

You also need:
- FalconSim 5.1 (Linux only)
- ROS2 Humble
- Colcon
- A Falcon Account

---

## 📦 Getting Started

### 1. Falcon Setup
- [Create a Falcon Account](https://falcon.duality.ai/auth/sign-up)
- [Download FalconSim Ubuntu 5.1](https://falcon.duality.ai/downloads)
```bash
cd ~/Downloads
sudo apt install ./FalconSim_Linux_Ubuntu22_v5.1.0216.deb
```

### 2. Install ROS2 Humble(If not done already)
Follow [this guide](https://docs.ros.org/en/humble/Installation.html)
Then install colcon:
```bash
sudo apt install python3-colcon-common-extensions
```
### 3. Run FalconSim Once
```bash
cd ~/duality/falconsim/
./Falcon.sh
```
- Login and save credentials when prompted, then close FalconSim.

### 4. Clone This Repo & Setup ROS2 Workspace
```bash
git clone https://github.com/your-username/AStarPlanningProject.git
cd AStarPlanningProject/ROS2/falcon_turtlebot3_project_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 5. Update Launch File Paths
Edit:
```bash
ros_falcon_astar.launch.py
```
Update:
-`cwd=` to your FalconSim install path
-`scenario=` to the full path of `AMRPathPlanning.usda`

