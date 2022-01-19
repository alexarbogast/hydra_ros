search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=hydra.srdf
robot_name_in_srdf=hydra
moveit_config_pkg=hydra_moveit_config
robot_name=hydra
planning_group_name=rob3_planning_group
ikfast_plugin_pkg=hydra_rob3_planning_group_ikfast_plugin
base_link_name=rob3_base
eef_link_name=rob3_typhoon_extruder
ikfast_output_path=/home/alex/Desktop/hydra_rob3_planning_group_ikfast_plugin/src/hydra_rob3_planning_group_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
