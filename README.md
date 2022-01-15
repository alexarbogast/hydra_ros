# `za_description` package

Contains URDF files and meshes for the ZA robot.

http://wiki.ros.org/xacro
http://wiki.ros.org/urdf
http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html

## `za_robot` package and MoveIt! Setup Assistant

- Run the demo
  - `roslaunch za_moveit_config demo.launch`
  - This doesn't depend on any robot hardware, and is useful to check
    URDFs and planning configuration
- Re-run the `moveit_setup_assistant`
  - `roslaunch za_moveit_config setup_assistant.launch`
  - To ease future regeneration, note changes in the below section
    "`za_moveit_config` package creation from scratch"


## Xacro operations

- Convert xacro to URDF
  `rosrun xacro xacro -o /tmp/za.urdf \
      $(rospack find za_description)/urdf/za.xacro`

- Verify URDF
  - `check_urdf /tmp/za.urdf`

- Visualize URDF structure
  - `urdf_to_graphiz /tmp/za.urdf; evince za.pdf`

- Convert to Collada format
  - `rosrun collada_urdf urdf_to_collada /tmp/za.urdf /tmp/za.dae`
  - Note:  OpenRAVE no longer available in ROS Noetic:
    https://index.ros.org/p/openrave/

## `za_moveit_config` package creation from scratch

See the [MoveIt! Setup Assistant tutorial][msa_tut]

- Build workspace to pick up `za_description` package
- Verify URDFs (see below)
- Run `roslaunch moveit_setup_assistant setup_assistant.launch`
- "Start" tab:
  - Select "Create New Moveit Configuration Package"
  - Load the `src/tormach/za_description/urdf/za.xacro` URDF model
  - Click "Load Files"
- "Self-Collisions" tab:  Adds `disable_collisions` elements to SRDF
  - Click "Generate Collision Matrix"
  - Optionally, disable additional collisions
    - Click "show enabled pairs"
    - Disable additional collisions:
      - base -> 2, 2 -> 4, 4 -> 6 can't collide with joint limits in
        effect (coarse collision models?)
- "Virtual Joints" tab:  Adds `virtual_joint` to SRDF (Skip)
- "Planning Groups" tab:  Adds `group` element to SRDF
  - Add `manipulator` group:  Click "Add Group"
    - "Group Name" "manipulator"
    - "Kinematics Solver" "kdl_kinematics_plugin/KDLKinematicsPlugin"
    - "Group Default Planner" "RRT"
    - Click "Add Kin. Chain"
      - Base Link:  "base_link"
      - Tip Link:  "tool0"
      - Click "Save"
- "Robot Poses" tab:  Adds `group_state` element to SRDF
  - Click "Add Pose"
    - "Pose Name" "all-zeros"
    - Leave other setting default
    - Click "Save"
- "End Effectors" tab:  (Skip)
- "Passive Joints" tab:  (Skip)
- "ROS Control" tab:  Configures `ros_controllers.yaml` file
  - Click "Add Controller"
    - "Controller Name" "position_trajectory_controller"
    - "Controller Type" "position_controllers/JointTrajectoryController"
    - Click "Add Individual Joints"
      - Select "joint_[1-6]"
      - Click ">" to add
      - Click "Save"
- "Simulation" tab:  (Skip)
- "3D Perception" tab:  (Skip)
- "Author information" tab:
  - Enter name and email
- "Configuration Files" tab:
  - "Configuration Package Save Path"
    "[...]/src/tormach/za_moveit_config" (Create directory first)
  - Click "Generate Package"
    - Click "Ok" at "Incomplete" warning
  - Click "Exit Setup Assistant"

After `catkin build && devel/setup.bash`, test:
```
roslaunch za_moveit_config demo.launch
```



[msa_tut]: http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/setup_assistant/setup_assistant_tutorial.html

## Verify URDFs

```
for URDF in \
    za.xacro \
    za_on_stand.xacro \
    robotiq_hand_e.xacro \
    za_hand_e.xacro \
    za_on_stand_robotiq.xacro \
    ; do

    echo -e "\n\n************** $URDF *****************"
    rosrun xacro xacro \
        `rospack find za_description`/urdf/${URDF} -o /tmp/test.urdf
    check_urdf /tmp/test.urdf

done
```
