# GAKD Demo

This repository contains a demonstration of our recently published work:

> **A Genetic Approach to Gradient-Free Kinodynamic Planning in Uneven Terrains**  
> Otobong Jerome, Alexandr Klimchik, Alexander Maloletov, Geesara Kulathunga  
> *IEEE Robotics and Automation Letters (RA-L), Vol. 10, No. 6, pp. 5521-5528, 2025*  
> [DOI: 10.1109/LRA.2025.3560883](https://doi.org/10.1109/LRA.2025.3560883)

The demo showcases kinodynamic planning on mesh-based terrain representations using genetic heuristics for the **AgileX Scout Mini** robot.

## Overview

This package integrates components from several open-source projects to enable motion planning on uneven terrains.

- **Base Framework:**  
  Adapted from [mesh_navigation_tutorials](https://github.com/naturerobots/mesh_navigation_tutorials)

- **Robot Platform:**  
  [Scout ROS 2 Interface](https://github.com/westonrobot/scout_ros2)

- **Terrain Representation & Navigation Core:**  
  [Mesh Navigation](https://github.com/naturerobots/mesh_navigation)

- **Planning Framework:**  
  [Move Base Flex](https://github.com/naturerobots/move_base_flex)

## Dependencies

This demo is designed for **ROS 2 Humble** and requires the following:

- `ros2_control` and compatible robot controllers
- `ros2_topic_tools`
- Git Large File Storage (LFS)

## Setup Instructions

1. **Clone the Repository**

   ```bash
   mkdir -p ~/mesh_ws/src
   cd ~/mesh_ws/src
   git clone https://github.com/Stblacq/mesh_planner.git
   cd mesh_planner
   git lfs pull
   ```

2. **Install Dependencies**

   Ensure all required packages are sourced and installed. Install external dependencies (`scout_ros2`, `mesh_navigation`, `move_base_flex`) into the same workspace or overlayed ones.

3. **Build the Workspace**

   ```bash
   cd ~/mesh_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install
   ```

4. **Launch the Demo**

   ```bash
   source ~/mesh_ws/install/setup.bash
   ros2 launch terrain_nav terrain_nav_launch.py world_name:=desert
   ```

## Notes

### Collision Avoidance
This demo does **not** include collision avoidance. For collision-aware planning, refer to our recent [project](https://github.com/Stblacq/uneven_terrain.git) 

Refer to the [Mesh Navigation documentation](https://github.com/naturerobots/mesh_navigation) for details on setting up semantic layers and planners.

## Videos
[![Genetic Kinodynamic Planning Demo](https://img.youtube.com/vi/IYRplkzxtIA/0.jpg)](https://youtu.be/IYRplkzxtIA)


### 🎥 Gradient-Free Kinodynamic Planning

[![Gradient-Free Kinodynamic Planning](https://img.youtube.com/vi/KI9RNQK9xW4/0.jpg)](https://youtu.be/KI9RNQK9xW4)

## License

This project is shared for academic and demonstration purposes in conjunction with our publication. Refer to each included project's license for further details.

## Contributions

We welcome contributions and feedback. For questions, open an issue.

Kindly star ⭐ this project if it helps you.
