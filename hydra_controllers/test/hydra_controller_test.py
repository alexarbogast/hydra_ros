#!/usr/bin/env python

import rospy
import actionlib
import unittest
import numpy as np
from copy import deepcopy

from za_msgs.msg import ZaState
from cartesian_control_msgs.msg import *
from hydra_controllers.srv import SwitchCoordination


def make_rectangle(center, x, y, z):
    xmin = center[0] - x / 2
    xmax = center[0] + x / 2
    ymin = center[1] - y / 2
    ymax = center[1] + y / 2
    zmin = center[2] - z / 2
    zmax = center[2] + z / 2

    # x: (f)ront, (b)ack
    # y: (l)eft, (r)ight
    # z: (t)op, (d)own
    flu = np.array([xmin, ymax, zmax])
    fru = np.array([xmin, ymin, zmax])
    frd = np.array([xmin, ymin, zmin])
    fld = np.array([xmin, ymax, zmin])
    bld = np.array([xmax, ymax, zmin])
    blu = np.array([xmax, ymax, zmax])
    bru = np.array([xmax, ymin, zmax])
    brd = np.array([xmax, ymin, zmin])

    return (flu, fru, frd, fld, bld, blu, bru, brd)

def make_cube(center, width):
    return make_rectangle(center, width, width, width)

# ============== example toolpaths ===============
def manipulability_example():
    x = 0.9
    y = 0.9
    z = 0.5

    floor_offset = 0.4
    center = np.array([0, 0, z / 2 + floor_offset])

    flu, fru, frd, fld, bld, blu, bru, brd = make_rectangle(center, x, y, z)
    path = [blu, flu, fld, bld, blu, bru, brd, frd, fru, bru, brd, bld]

    return path

def stars():
    z = 0.00238
    path1 = [np.array([-0.3236, -0.2351, z]), np.array([-0.1528, 0.0, z]), np.array([-0.3236, 0.2351, z]), np.array([-0.0472, 0.1453, z]), np.array([0.1236, 0.380, z]), np.array([0.1236, 0.09, z]), np.array([0.4, 0.0, z]), np.array([0.1236, -0.09, z]),  np.array([0.1236, -0.380, z]), np.array([-0.0472, -0.1453, z]), np.array([-0.3236, -0.2351, z])]
    path2 = [np.array([0.4, 0.0, z]), np.array([0.1236, -0.09, z]),  np.array([0.1236, -0.380, z]), np.array([-0.0472, -0.1453, z]), np.array([-0.3236, -0.2351, z]), np.array([-0.1528, 0.0, z]), np.array([-0.3236, 0.2351, z]), np.array([-0.0472, 0.1453, z]), np.array([0.1236, 0.380, z]), np.array([0.1236, 0.09, z]), np.array([0.4, 0.0, z])]
    path3 = [np.array([-0.3236, 0.2351, z]), np.array([-0.0472, 0.1453, z]), np.array([0.1236, 0.380, z]), np.array([0.1236, 0.09, z]), np.array([0.4, 0.0, z]), np.array([0.1236, -0.09, z]),  np.array([0.1236, -0.380, z]), np.array([-0.0472, -0.1453, z]), np.array([-0.3236, -0.2351, z]), np.array([-0.1528, 0.0, z]), np.array([-0.3236, 0.2351, z])]
    return (path1, path2, path3)

def all_edges(center, width):
    flu, fru, frd, fld, bld, blu, bru, brd = make_cube(center, width)

    path = [fru, flu, blu, bru, fru, frd, fru, flu,
            fld, bld, blu, flu, fld, frd, brd, bru,
            blu, bld, brd, bru]

    return path
    
def set_pose(position, orientation, point: CartesianTrajectoryPoint):
    point.pose.position.x = position[0]
    point.pose.position.y = position[1]
    point.pose.position.z = position[2]
    point.pose.orientation.w = orientation[0]
    point.pose.orientation.x = orientation[1]
    point.pose.orientation.y = orientation[2]
    point.pose.orientation.z = orientation[3]


class HydraControllerTest(unittest.TestCase):
    def setUp(self):
        # initialize ros
        rospy.init_node('hydra_controller_test')

        self.client1 = actionlib.SimpleActionClient('/hydra_controller/rob1/'
                                                    'follow_cartesian_trajectory',
                                                     TrajectoryExecutionAction)
        self.client2 = actionlib.SimpleActionClient('/hydra_controller/rob2/'
                                                    'follow_cartesian_trajectory',
                                                     TrajectoryExecutionAction)
        self.client3 = actionlib.SimpleActionClient('/hydra_controller/rob3/'
                                                    'follow_cartesian_trajectory',
                                                     TrajectoryExecutionAction)
        self.client1.wait_for_server();
        self.client2.wait_for_server();
        self.client3.wait_for_server();

        rospy.wait_for_service('/hydra_controller/switch_coordination')
        self.switch_coord_client = rospy.ServiceProxy('/hydra_controller/switch_coordination',
                                                        SwitchCoordination)

        self.orient = np.array([0, 1, 0, 0])
        
        self.v_fast = 500.0 / 1000.0;  # m/s
        self.a_fast = 1000.0 / 1000.0; # m/s^2
        self.j_fast = 3000.0 / 1000.0; # m/s^3
    
        self.v_slow = 100.0 / 1000.0;  # m/s
        self.a_slow = 400.0 / 1000.0;  # m/s^2
        self.j_slow = 1000.0 / 1000.0; # m/s^3
    
        # wait for initial transformations
        state1 = rospy.wait_for_message('rob1_state_controller/za_states', ZaState, 10)
        state2 = rospy.wait_for_message('rob2_state_controller/za_states', ZaState, 10)
        state3 = rospy.wait_for_message('rob3_state_controller/za_states', ZaState, 10)
        timeout_msg = "timed out waiting for za state"
        self.assertIsNotNone(state1, timeout_msg)
        self.assertIsNotNone(state2, timeout_msg)
        self.assertIsNotNone(state3, timeout_msg)

        init_pose1 = np.array(state1.O_T_EE).reshape(4, 4).T
        init_pose2 = np.array(state2.O_T_EE).reshape(4, 4).T
        init_pose3 = np.array(state3.O_T_EE).reshape(4, 4).T

        self.start_state_base1 = CartesianTrajectoryPoint()
        self.start_state_base2 = CartesianTrajectoryPoint()
        self.start_state_base3 = CartesianTrajectoryPoint()
        set_pose(init_pose1[:3, 3], self.orient, self.start_state_base1)
        set_pose(init_pose2[:3, 3], self.orient, self.start_state_base2)
        set_pose(init_pose3[:3, 3], self.orient, self.start_state_base3)

        self.start_state_pos1 = CartesianTrajectoryPoint()
        self.start_state_pos2 = CartesianTrajectoryPoint()
        self.start_state_pos3 = CartesianTrajectoryPoint()
        pos_start1 = np.array([-0.117879, -0.204162,  0.389365])
        pos_start2 = np.array([0.235749, 6.74803e-06, 0.389365])
        pos_start3 = np.array([-0.117883, 0.20416, 0.389365])
        set_pose(pos_start1, self.orient, self.start_state_pos1)
        set_pose(pos_start2, self.orient, self.start_state_pos2)
        set_pose(pos_start3, self.orient, self.start_state_pos3)

    #@unittest.skip("skipping test_switch_coordination")
    def test_switch_coordination(self):
        resp1 = self.switch_coord_client(arm_id="rob1", coordinated=True)
        resp2 = self.switch_coord_client(arm_id="rob2", coordinated=True)
        resp3 = self.switch_coord_client(arm_id="rob3", coordinated=True)
        self.assertTrue(resp1.success)
        self.assertTrue(resp2.success)
        self.assertTrue(resp3.success)

        resp1 = self.switch_coord_client(arm_id="rob1", coordinated=False)
        resp2 = self.switch_coord_client(arm_id="rob2", coordinated=False)
        resp3 = self.switch_coord_client(arm_id="rob3", coordinated=False)
        self.assertTrue(resp1.success)
        self.assertTrue(resp2.success)
        self.assertTrue(resp3.success)

    #@unittest.skip("skipping test_synchronized")    
    def test_synchronized(self):
        # switch to synchronized mode
        self.switch_coord_client(arm_id="rob1", coordinated=False)
        self.switch_coord_client(arm_id="rob2", coordinated=False)
        self.switch_coord_client(arm_id="rob3", coordinated=False)

        # create trajectories
        traj = CartesianTrajectory()

        path = manipulability_example()
        for segment in path:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj.points.append(pose)

        traj1 = deepcopy(traj)
        traj2 = deepcopy(traj)
        traj3 = deepcopy(traj)
#
        traj1.points.append(self.start_state_base1)
        traj2.points.append(self.start_state_base2)
        traj3.points.append(self.start_state_base3)

        goal = TrajectoryExecutionGoal()
        goal.limits.v_max = self.v_fast
        goal.limits.a_max = self.a_fast
        goal.limits.j_max = self.j_fast

        goal.trajectory = traj1
        self.client1.send_goal(goal)
        goal.trajectory = traj2
        self.client2.send_goal(goal)
        goal.trajectory = traj3
        self.client3.send_goal(goal)

        self.client1.wait_for_result()
        self.client2.wait_for_result()
        self.client3.wait_for_result()

    #@unittest.skip("skipping test_coordinated")    
    def test_coordinated(self):
        self.switch_coord_client(arm_id="rob1", coordinated=True)
        self.switch_coord_client(arm_id="rob2", coordinated=True)
        self.switch_coord_client(arm_id="rob3", coordinated=True)

        traj1 = CartesianTrajectory()
        traj2 = CartesianTrajectory()
        traj3 = CartesianTrajectory()

        path1, path2, path3 = stars()
        for segment in path1:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj1.points.append(pose)
        for segment in path2:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj2.points.append(pose)
        for segment in path3:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj3.points.append(pose)

        traj1.points.append(self.start_state_pos1)
        traj2.points.append(self.start_state_pos2)
        traj3.points.append(self.start_state_pos3)

        goal = TrajectoryExecutionGoal()
        goal.limits.v_max = self.v_slow
        goal.limits.a_max = self.a_slow
        goal.limits.j_max = self.j_slow

        goal.trajectory = traj1
        self.client1.send_goal(goal)
        goal.trajectory = traj2
        self.client2.send_goal(goal)
        goal.trajectory = traj3
        self.client3.send_goal(goal)

        self.client1.wait_for_result()
        self.client2.wait_for_result()
        self.client3.wait_for_result()    

    #@unittest.skip("skipping test_coordinated")    
    def test_multi_modal(self):
        self.switch_coord_client(arm_id="rob1", coordinated=False)
        self.switch_coord_client(arm_id="rob2", coordinated=True)
        self.switch_coord_client(arm_id="rob3", coordinated=True)

        traj1 = CartesianTrajectory()
        traj2 = CartesianTrajectory()
        traj3 = CartesianTrajectory()

        path1 = manipulability_example()
        _, path2, path3 = stars()
        for segment in path1:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj1.points.append(pose)
        for segment in path2[:6]:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj2.points.append(pose)
        for segment in path3:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj3.points.append(pose)

        traj1.points.append(self.start_state_base1)
        traj3.points.append(self.start_state_pos3)

        goal_fast = TrajectoryExecutionGoal()
        goal_fast.limits.v_max = self.v_fast
        goal_fast.limits.a_max = self.a_fast
        goal_fast.limits.j_max = self.j_fast

        goal_slow = TrajectoryExecutionGoal()
        goal_slow.limits.v_max = self.v_slow
        goal_slow.limits.a_max = self.a_slow
        goal_slow.limits.j_max = self.j_slow

        goal_fast.trajectory = traj1
        self.client1.send_goal(goal_fast)
        goal_slow.trajectory = traj2
        self.client2.send_goal(goal_slow)
        goal_slow.trajectory = traj3
        self.client3.send_goal(goal_slow)

        self.client2.wait_for_result()

        # send second path to robot 2
        self.switch_coord_client(arm_id="rob2", coordinated=False)

        traj2.points.clear()
        traj2.points.append(self.start_state_base2)
        center = np.array([self.start_state_base2.pose.position.x,
                           self.start_state_base2.pose.position.y,
                           self.start_state_base2.pose.position.z])
        path2 = all_edges(center, 0.3)
        for segment in path2:
            pose = CartesianTrajectoryPoint()
            set_pose(segment, self.orient, pose)
            traj2.points.append(pose)
        traj2.points.append(self.start_state_base2)

        goal_fast.trajectory = traj2
        self.client2.send_goal(goal_fast)
        self.client1.wait_for_result()
        self.client2.wait_for_result()
        self.client3.wait_for_result()
        self.switch_coord_client(arm_id="rob3", coordinated=False)


if __name__ == '__main__':
    import rostest
    rostest.rosrun("hydra_controllers", "hydra_controller_test", HydraControllerTest)