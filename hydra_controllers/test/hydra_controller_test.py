#!/usr/bin/env python

import rospy
import unittest
import quaternion
import numpy as np

from za_msgs.msg import PosVelSetpoint
from za_msgs.msg import ZaState

from hydra_controllers.srv import SwitchCoordination


def createCommand(position, orientation, velocity):
    command = PosVelSetpoint()
    command.pose.position.x = position[0]
    command.pose.position.y = position[1]
    command.pose.position.z = position[2]

    command.pose.orientation.w = 1
    command.pose.orientation.x = 0
    command.pose.orientation.y = 0
    command.pose.orientation.z = 0

    command.twist.linear.x = velocity[0]
    command.twist.linear.y = velocity[1]
    command.twist.linear.z = velocity[2]
    return command


class CircleTrajectory:
    def __init__(self, radius, start):
        self._radius = radius
        self._start  = start
        self._origin = np.array([start[0] + radius, start[1], start[2]])

    def __call__(self, t):
        position = np.array([-self._radius * np.cos(t * 2 * np.pi),
                            -self._radius * np.sin(t * 2 * np.pi),
                            0]) + self._origin
        
        velocity = 2 * np.pi * self._radius * np.array([np.sin(t * 2 * np.pi),
                                                       -np.cos(t * 2 * np.pi),
                                                        0])

        return position, velocity


class HydraControllerTest(unittest.TestCase):
    def setUp(self):
        # initialize ros
        rospy.init_node('hydra_controller_test')

        self.cmd_pub1 = rospy.Publisher("hydra_controller/rob1/command", PosVelSetpoint, queue_size=1)
        self.cmd_pub2 = rospy.Publisher("hydra_controller/rob2/command", PosVelSetpoint, queue_size=1)
        self.cmd_pub3 = rospy.Publisher("hydra_controller/rob3/command", PosVelSetpoint, queue_size=1)

        rospy.wait_for_service('/hydra_controller/switch_coordination')
        self.switch_coord_client = rospy.ServiceProxy('/hydra_controller/switch_coordination',
                                                        SwitchCoordination)

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
        self.switch_coord_client(arm_id="rob1", coordinated=False)
        self.switch_coord_client(arm_id="rob2", coordinated=False)
        self.switch_coord_client(arm_id="rob3", coordinated=False)

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
        orient1 = quaternion.from_rotation_matrix(init_pose1[:3, :3])
        orient2 = quaternion.from_rotation_matrix(init_pose2[:3, :3])
        orient3 = quaternion.from_rotation_matrix(init_pose3[:3, :3])

        traj1 = CircleTrajectory(0.075, init_pose1[:3, 3])
        traj2 = CircleTrajectory(0.075, init_pose2[:3, 3])
        traj3 = CircleTrajectory(0.075, init_pose3[:3, 3])

        t_final = 30 # sec
        period = 5 # sec

        ti = rospy.Time.now()
        rate = rospy.Rate(250) # 250 Hz
        scaling = 1 / period
        while True:
            elapsed = (rospy.Time.now() - ti).to_sec()
            if t_final < elapsed:
                return True 
            
            pos1, vel1 = traj1(elapsed * scaling)
            pos2, vel2 = traj2(elapsed * scaling)
            pos3, vel3 = traj3(elapsed * scaling)
            cmd1 = createCommand(pos1, orient1, vel1)
            cmd2 = createCommand(pos2, orient2, vel2)
            cmd3 = createCommand(pos3, orient3, vel3)

            self.cmd_pub1.publish(cmd1)
            self.cmd_pub2.publish(cmd2)
            self.cmd_pub3.publish(cmd3)
            rate.sleep()

    #@unittest.skip("skipping test_coordinated")    
    def test_coordinated(self):
        self.switch_coord_client(arm_id="rob1", coordinated=True)
        self.switch_coord_client(arm_id="rob2", coordinated=True)
        self.switch_coord_client(arm_id="rob3", coordinated=True)

        start1 = np.array([-0.117879, -0.204162,  0.389365])
        start2 = np.array([0.235749, 6.74803e-06, 0.389365])
        start3 = np.array([-0.117883, 0.20416, 0.389365])
        orient = np.array([0, 1, 0, 0])

        traj1 = CircleTrajectory(0.075, start1)
        traj2 = CircleTrajectory(0.075, start2)
        traj3 = CircleTrajectory(0.075, start3)

        t_final = 30 # sec
        period = 5 # sec

        ti = rospy.Time.now()
        rate = rospy.Rate(250) # 250 Hz
        scaling = 1 / period
        while True:
            elapsed = (rospy.Time.now() - ti).to_sec()
            if t_final < elapsed:
                return True 
            
            pos1, vel1 = traj1(elapsed * scaling)
            pos2, vel2 = traj2(elapsed * scaling)
            pos3, vel3 = traj3(elapsed * scaling)
            cmd1 = createCommand(pos1, orient, vel1)
            cmd2 = createCommand(pos2, orient, vel2)
            cmd3 = createCommand(pos3, orient, vel3)

            self.cmd_pub1.publish(cmd1)
            self.cmd_pub2.publish(cmd2)
            self.cmd_pub3.publish(cmd3)
            rate.sleep()
    

if __name__ == '__main__':
    import rostest
    rostest.rosrun("hydra_controllers", "hydra_controller_test", HydraControllerTest)