# hydra_ros

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

**ROS integration for the Hydra multi-robot system**. This ROS2 metapackage
provides launch files and configuration for the Hydra multi-robot system. See
this repository's branches for different variants of the multi robot system
configuration.

<p align="center">
  <img src=https://github.com/alexarbogast/za_ros/assets/46149643/02d6b3e5-c266-4b67-a2fe-4b5805c6f989 width=600/>
</p>


> [!NOTE]
> __`ros2`__ variant:
>
> The ampf branch contains the setup of the physical system at
> [ampf](https://ampf.research.gatech.edu/).

## Contents

- [Dependencies](#1)
- [Installation](#2)
- [Running the System](#3)

<a id='1'></a>

## Dependencies

When running the multi-robot system on the physical hardware, it is recommended
to build the Docker image and use the scripts provided in
[za_docker](https://github.com/alexarbogast/za_docker) (*on each robot
computer*). If you only plan to run the system in simualtion, this is not
required. 

<a id='2'></a>

## Installation

Create a ROS 2 workspace and clone this package into a `src` directory.

Import package dependencies:
```bash
sudo apt update
rosdep update
cd src
vcs import < hydra_ros/hydra.repos
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Build the packages:
```bash
cd <COLCON_WORKSPACE>
colcon build
```

<a id='3'></a>

## Running the System
### Visualizing the robot description 
To view the system in Rviz, run the following command after sourcing your bash
file:

```sh
ros2 launch hydra_bringup hydra_visualization.launch.py
```

### Running the System in Simulation
There are two forms of simulation for the system. The first uses the
`sim_hw_interface` from
[ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate).
The second uses the sim version of the `hal_hw_interface` from
[hal_ros_control](https://github.com/tormach/hal_ros_control). For more
information on the differences between these hardware interface see the README
in
[za_hardware](https://github.com/alexarbogast/za_ros/tree/hw_merge/za_hardware).

**Launch all hardware interfaces and controllers on the same computer using the `sim_hw_interface`**:

```sh
ros2 launch hydra_bringup hydra_system_sim.launch.py rviz:=true
```

From here you can run any custom controllers or start MoveIt! to begin planning. 

Alternatively, you can run the system in simulation using the
`hal_hw_interface`. The hydra_robot.launch file wraps the normal Za6 bringup in
a namespace. The launch file should be run in seperate Docker containers. This
works on seperate robot computers or all on the same machine. **Launch all
hardware interfaces (hal_hw_interface) in seperate docker containers and
visualize the system**:


```sh
# docker container 1
ros2 launch hydra_bringup hydra_robot.launch.py hardware:=hal arm_id:=rob1

# docker container 2
ros2 launch hydra_bringup hydra_robot.launch.py hardware:=hal arm_id:=rob2

# host computer
ros2 launch hydra_bringup hydra_visualization.launch.py
```

The "motors" can be activated by publishing the following ROS messages to the
hardware interface from the host computer:


```sh
ros2 topic pub /rob1/hal_io/state_cmd std_msgs/UInt32 "data: 2"
ros2 topic pub /rob2/hal_io/state_cmd std_msgs/UInt32 "data: 2"
```

### Running the System on Hardware

The only difference here is that the `sim:=false` argument should be passed to
the robot launch when running the hardware interface that will actually connect
to EtherCat. *Note: See [za_docker](https://github.com/alexarbogast/za_docker)
for instructions on running the containers in execution mode*

```sh
# docker container 1 on robot 1 computer
ros2 launch hydra_bringup hydra_robot.launch.py hardware:=hal sim:=false arm_id:=rob1

# docker container 2 on robot 2 computer
ros2 launch hydra_bringup hydra_robot.launch.py hardware:=hal sim:=false arm_id:=rob2

# host computer
ros2 launch hydra_bringup hydra_visualization.launch.py
```
After engaging the motors with the commands listed above, you can begin planning
with MoveIt!.

### Planning in MoveIt
MoveIt! has limited functionality for coordinated planning in multi-robot
systems. However, it still works for point-to-point collision-free motion. You
can use MoveIt! to plan for the Hydra system using any of the runtime
configurations listed above.

```sh
# host computer
ros2 launch hydra_bringup moveit_planning.launch.py
```
