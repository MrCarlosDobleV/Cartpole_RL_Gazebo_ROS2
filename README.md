# CartPole ROS 2 + Gazebo Sim Project

This repository contains a **CartPole simulation integrated with ROS 2 and Gazebo Sim (Fortress)**.  
It is based on the official `ros_gz_project_template`, adapted for a **custom CartPole system** suitable for **control and reinforcement learning experiments**.

The project provides:
- A CartPole model defined in SDF
- A Gazebo world to simulate the system
- ROS 2 launch files and bridges
- A clean workspace structure following ROS 2 best practices

---

## Repository structure

```text
cartpole_project/
├── cartpole_application/     # ROS 2 application-level code (controllers, RL nodes, etc.)
├── cartpole_bringup/         # Launch files and bridge configurations
├── cartpole_description/     # CartPole model (SDF, model.config, hooks)
├── cartpole_gazebo/          # Gazebo systems, plugins, and worlds
├── LICENSE
├── README.md
└── template_workspace.yaml
```

---

## Package overview

- **cartpole_description**  
  Contains the CartPole SDF model and related assets.

- **cartpole_gazebo**  
  Contains Gazebo-specific code, systems, and the simulation world.

- **cartpole_bringup**  
  Contains launch files and ROS ↔ Gazebo bridge configuration.

- **cartpole_application**  
  Contains ROS 2 nodes for control, experimentation, or reinforcement learning.

---

## Requirements

This project is tested with:

- **ROS 2** compatible with Gazebo Fortress
- **Gazebo Sim Fortress**

Refer to the official compatibility table:  
https://gazebosim.org/docs/latest/ros_installation

If needed, explicitly set the Gazebo version:

```bash
export GZ_VERSION=fortress
```

---

## System dependencies

```bash
sudo apt install \
  python3-vcstool \
  python3-colcon-common-extensions \
  python3-rosdep \
  git
```

---

## Installation

### 1. Create a workspace and clone the repository

```bash
mkdir -p ~/cartpole_ws/src
cd ~/cartpole_ws/src
git clone <YOUR_REPOSITORY_URL>
```

---

### 2. Install ROS dependencies

```bash
cd ~/cartpole_ws
source /opt/ros/<ROS_DISTRO>/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -i -y
```

---

## Build

```bash
cd ~/cartpole_ws
source /opt/ros/<ROS_DISTRO>/setup.bash
colcon build
source install/setup.bash
```

---

## Usage

### Launch the CartPole simulation

```bash
ros2 launch cartpole_bringup cartpole.launch.py
```

This will:
- Start Gazebo Sim Fortress
- Load the CartPole world and model
- Start the ROS ↔ Gazebo bridge

---

## Controlling the CartPole (Gazebo CLI)

Apply a force to the cart (prismatic joint `cart_slide`):

```bash
ign topic -t /cart_force -m ignition.msgs.Double -p "data: 1"
```

Apply force in the opposite direction:

```bash
ign topic -t /cart_force -m ignition.msgs.Double -p "data: -1"
```



---

## Future work

Planned extensions include:
- Integration with `gazebo_ros2_control`
- Effort-based ROS controllers
- Reinforcement learning environments (Gym-style API)
- Automated evaluation and logging

---

## License

This project is licensed under the **Apache License 2.0**.  
See the `LICENSE` file for details.
