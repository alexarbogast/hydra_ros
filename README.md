# hydra_ros
**ROS integration for the Hydra multi-robot system**

## Contents

- [Installation](#1)
- [running the demos](#2)
<br>
<a id='1'></a>

## Installation

Clone the repository to your local workspace. If you do not already have the 
[za_description](https://github.com/alexarbogast/za_description/tree/e57f65c3f8eb0be88e7739a8b5162b4b3b875b15) package, you will need to clone recursively. 

```shell script
mkdir catkin_ws/src && cd catkin_ws/src
git clone --recurse-submodules https://github.com/alexarbogast/za_ros.git
git clone https://github.com/alexarbogast/hydra_ros.git
```

Build your workspace
```shell script
catkin build
```
<br>
<a id='2'></a>

## Running the Demos
The *hydra_gazebo* package implements a combined hardware interface for the gazebo simulation. When launching the gazebo simualtion, the combined hardware interface wraps the individual interfaces for each robot. 

Launch the Hydra gazebo simulation with the following command
```shell script
roslaunch hydra_gazebo hydra.launch
```

If you would like to launch controllers with the robot, the *controller* parameter can be used to select the desired ros_controls controller.
```shell script
roslaunch hydra_gazebo hydra.launch controller:="CONTROLLER1_NAME CONTROLLER2_NAME ..."
```
A few controller have been implemented in za_controllers. The available controllers can be found in [sim_controllers.yaml](https://github.com/alexarbogast/za_ros/blob/master/za_gazebo/config/sim_controllers.yaml). More information about the available controllers refer to [za_controller detailed description](https://github.com/alexarbogast/za_ros/tree/master/za_controllers#readme).

To actually execute a trajectory, you will need a node that feeds the controllers a setpoint. An example for executing linear trajectories with 4th order smoothing is provided at [kintrol](https://github.com/alexarbogast/kintrol.git). The kintrol package implements an online trajectory generator using [ruckig](https://github.com/pantor/ruckig).
