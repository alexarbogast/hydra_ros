# hydra_ros
**ROS integration for the Hydra multi-robot system**. This ROS metapackage
provides launch files and configuration for the Hydra multi-robot system. See
this repository's branches for different variants of the multi robot system
configuration.

<p align="center">
  <img src=https://github.com/alexarbogast/za_ros/assets/46149643/ee7f7bb1-abc0-44a2-aa61-de2652a10314 width=600/>
</p>

> [!NOTE]
> __`robot_positioner`__ variant:
>
> The robot_positioner branch contains an experimental setup using the third
> robot as the positioner. Coordinated motion control for this system
> configuration is still a work in progress.

## Contents

- [Dependencies](#1)
- [Installation](#2)
- [Running the System](#3)

<a id='1'></a>

## Dependencies
The `za_ros` metapackage is required for `hydra_ros`. Follow the instructions at
[za_ros](https://github.com/alexarbogast/za_ros) to install the package and its
dependencies. Make sure to install
[ros_control_boilerplate](https://github.com/PickNikRobotics/ros_control_boilerplate)
and [ros_controllers](https://github.com/ros-controls/ros_controllers). 

When running the multi-robot system on the physical hardware, it is recommended
to build the Docker image and use the scripts provided in
[za_docker](https://github.com/alexarbogast/za_docker) (*on each robot
computer*). If you only plan to run the system in simualtion, this is not
required. 

<a id='2'></a>

## Installation

Clone the repository to your local workspace. If you have not previously cloned the 
[hydra_description](https://github.com/alexarbogast/hydra_description) package, you will need to clone recursively. 

```shell script
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone --recurse-submodules https://github.com/alexarbogast/hydra_ros.git
```

Build your workspace
```shell script
catkin build
```

<a id='3'></a>

## Running the System
### Visualizing the robot description 
To view the system in Rviz, run the following command after sourcing your bash
file:
```shell
roslaunch hydra_bringup hydra_visualization.launch
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
```shell
roslaunch hydra_bringup hydra_system_sim.launch rviz:=true
``` 

From here you can run any custom controllers or start MoveIt! to begin planning. 

Alternatively, you can run the system in simulation using the
`hal_hw_interface`. The hydra_robot.launch file wraps the normal Za6 bringup in
a namespace. The launch file should be run in seperate Docker containers. This
works on seperate robot computers or all on the same machine. **Launch all
hardware interfaces (hal_hw_interface) in seperate docker containers and
visualize the system**:
```shell
# docker container 1
roslaunch hydra_bringup hydra_robot.launch hardware:=hal arm_id:=rob1

# docker container 2
roslaunch hydra_bringup hydra_robot.launch hardware:=hal arm_id:=rob2

# docker container 3
roslaunch hydra_bringup hydra_robot.launch hardware:=hal arm_id:=rob3

# host computer
roslaunch hydra_bringup hydra_visualization.launch 
```
The "motors" can be activated by publishing the following ros messages to the
hardware interface from the host computer:
```shell
rostopic pub /rob1/hal_io/state_cmd std_msgs/UInt32 "data: 2"
rostopic pub /rob2/hal_io/state_cmd std_msgs/UInt32 "data: 2"
rostopic pub /rob3/hal_io/state_cmd std_msgs/UInt32 "data: 2"
``` 

### Running the System on Hardware

The only difference here is that the `sim:=false` argument should be passed to
the robot launch when running the hardware interface that will actually connect
to EtherCat. *Note: See [za_docker](https://github.com/alexarbogast/za_docker) 
for instructions on running the containers in execution mode*

```shell
# docker container 1 on robot 1 computer
roslaunch hydra_bringup hydra_robot.launch hardware:=hal sim:=false arm_id:=rob1

# docker container 2 on robot 2 computer
roslaunch hydra_bringup hydra_robot.launch hardware:=hal sim:=false arm_id:=rob2

# docker container 3 on robot 3 computer
roslaunch hydra_bringup hydra_robot.launch hardware:=hal sim:=false arm_id:=rob3

# host computer
roslaunch hydra_bringup hydra_visualization.launch 
```
After engaging the motors with the commands listed above, you can begin planning
with MoveIt!.

### Planning in MoveIt
MoveIt! has limited functionality for coordinated planning in multi-robot
systems. However, it still works for point-to-point collision-free motion. You
can use MoveIt! to plan for the Hydra system using any of the runtime
configurations listed above.

```shell
# host computer
roslaunch hydra_bringup moveit_planning.launch
```
