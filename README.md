# A* Path Planning on TurtleBot3 with FalconSim & ROS2

Welcome to **Project 03 - Phase 2** of ENPM661 Spring 2025. In this assignment, you will implement the A* algorithm for a differential-drive robot and simulate the results in a realistic warehouse environment using **FalconSim**.

## 📁 Project Structure

- `ROS2/`: Contains the ROS2 workspace with launch files and source code
- `Scenarios/AMRPathPlanning/`: FalconSim USD scenario and Python control script
- `Twins/`, `Work/`: Simulation-related data/configs
- `slides/`: PDF slides for classroom instruction
- `.gitignore`, `.gitattributes`: Standard project config

## 🛠️ Requirements (Local users)

| System Requirement | Minimum Spec |
|--------------------|--------------|
| OS                 | Ubuntu 22.04 |
| GPU                | Nvidia GeForce 3050 |
| RAM                | 16 GB        |

You also need:
- FalconSim 5.1 (Linux only)
- ROS2 Humble
- Colcon
- A Falcon Account

---

## 📦 Installation & Setup

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

### 6. Launch Simulation
```bash
ros2 launch astar_falcon_planner ros_falcon_astar.launch.py \
    start_position:=[0.35,1.2,0.0] \
    end_position:=[5.5,1.50,0.0] \
    robot_radius:=0.220 \
    clearance:=0.025 \
    delta_time:=1.0 \
    wheel_radius:=0.033 \
    wheel_distance:=0.287 \
    rpms:=[50.0,100.0]
```
## Learning Outcomes
- Understand A* path planning algorithm
- Gain hands-on experience with ROS2-based robotic control
- Integrate digital twin scenarios using FalconSim

**NOTE:** For questions or support regarding Falcon Setup or debugging, reach out to us on our [Community Discord]() 
