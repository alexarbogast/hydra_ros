# `hydra_description` package

Contains URDF files and meshes for the hydra multi-robot additive system.

http://wiki.ros.org/xacro
http://wiki.ros.org/urdf
http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/urdf_srdf/urdf_srdf_tutorial.html

## Xacro operations

- Convert xacro to URDF
  `rosrun xacro xacro -o /tmp/za.urdf \
      $(rospack find hydra_description)/urdf/hydra.xacro`

- Verify URDF
  - `check_urdf /tmp/hydra.urdf`

- Visualize URDF structure
  - `urdf_to_graphiz /tmp/hydra.urdf; evince hydra.pdf`

- Convert to Collada format
  - `rosrun collada_urdf urdf_to_collada /tmp/za.urdf /tmp/za.dae`
  - Note:  OpenRAVE no longer available in ROS Noetic:
    https://index.ros.org/p/openrave/