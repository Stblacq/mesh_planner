
# Setup Instructions

This guide will walk you through the setup process for the Meshnav ARIAC project using Docker.

## Prerequisites

Ensure you have Docker installed on your system. If not, you can install Docker from [here](https://docs.docker.com/get-docker/).

## Installation Steps

1. **Clone the meshnav_ariac Repository**

   Start by cloning the `meshnav_ariac` repository which contains the necessary Docker file and scripts.

   ```bash
   git clone https://github.com/Stblacq/meshnav_ariac
   ```

2. **Clone the mesh_planner Repository**

   Next, clone the `mesh_planner` repository into your development workspace directory.

   ```bash
   git clone https://github.com/Stblacq/mesh_planner ~/dev_workspace
   ```

3. **Start the Docker Container**

   Navigate to the `meshnav_ariac` directory and run the following scripts to start and enter the Docker container:

   ```bash
   cd meshnav_ariac
   ./ariac2019_docker.sh start
   ./ariac2019_docker.sh enter
   ```

4. **Environment Setup**

   Once inside the Docker container, set up the ROS environment by sourcing the necessary scripts:

   ```bash
   source /opt/ros/melodic/setup.bash
   source ~/workspace/devel/setup.bash
   ```

5. **Launch ROS Nodes**

   Launch the ROS nodes to initiate the simulation and navigation:

   ```bash
   roslaunch pluto_gazebo pluto_botanical_garden.launch
   roslaunch pluto_navigation botanical_garden_osnabrueck
   ```

## Usage

After completing the above steps, the ROS nodes should be running, and you can begin interacting with the simulations and navigation functionalities provided by the Meshnav ARIAC project.
