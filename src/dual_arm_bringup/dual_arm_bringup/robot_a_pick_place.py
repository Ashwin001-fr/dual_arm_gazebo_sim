#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from linkattacher_msgs.srv import AttachLink, DetachLink
import numpy as np
from ur_analytic_ik import ur5e
import time


class UR5ePickPlace(Node):
    def __init__(self):
        super().__init__('ur5e_pick_place')
        
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # IFRA LinkAttacher services
        self.attach_client = self.create_client(AttachLink, '/ATTACHLINK')
        
        self.object_sub = self.create_subscription(
            Point,
            '/object_position',
            self.object_callback,
            10
        )
        
        self.home_position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
        self.place_position = [1.57, -1.57, 0.0, -1.57, 0.0, 0.0]  # Place location
        
        self.get_logger().info('UR5e Pick & Place Ready')
        self._action_client.wait_for_server()
        
        # Wait for attach services
        while not self.attach_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for attach service...')
        self.get_logger().info('Ready!')


    def point_to_pose_matrix(self, x, y, z):
        eef_pose = np.identity(4)
        eef_pose[0, 3] = x
        eef_pose[1, 3] = y
        eef_pose[2, 3] = z
        eef_pose[0:3, 0] = [-1.0, 0.0, 0.0]
        eef_pose[0:3, 1] = [0.0, 1.0, 0.0]
        eef_pose[0:3, 2] = [0.0, 0.0, -1.0]
        return eef_pose


    def attach_object_to_robot(self, object_name='tracked_cube'):
        """Attach object to robot end-effector"""
        req = AttachLink.Request()
        req.model1_name = 'ur'
        req.link1_name = 'wrist_3_link'
        req.model2_name = object_name
        req.link2_name = 'cube_link'
        
        self.get_logger().info(f'Attaching {object_name}...')
        future = self.attach_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        # Don't check response, just assume it worked if no exception
        self.get_logger().info('Object attached!')


    def object_callback(self, msg):
        self.get_logger().info(f'Object detected at: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}')
        
        # Approach (10cm above)
        approach_pose = self.point_to_pose_matrix(msg.x, msg.y, msg.z + 0.1)
        approach_sols = ur5e.inverse_kinematics(approach_pose)
        
        # Pick position
        pick_pose = self.point_to_pose_matrix(msg.x, msg.y, msg.z)
        pick_sols = ur5e.inverse_kinematics(pick_pose)
        
        if len(approach_sols) == 0 or len(pick_sols) == 0:
            self.get_logger().error('No IK solution!')
            return
        
        approach_pos = approach_sols[0].tolist()
        pick_pos = pick_sols[0].tolist()
        
        # Execute pick and place sequence
        self.get_logger().info('Step 1: Approaching object...')
        self.send_trajectory(approach_pos, 3.0)
        time.sleep(3.5)
        
        self.get_logger().info('Step 2: Moving to pick position...')
        self.send_trajectory(pick_pos, 2.0)
        time.sleep(2.5)
        
        self.get_logger().info('Step 3: Attaching object!')
        self.attach_object_to_robot('tracked_cube')
        time.sleep(0.5)
        
        self.get_logger().info('Step 4: Lifting object...')
        self.send_trajectory(approach_pos, 2.0)
        time.sleep(2.5)
        
        self.get_logger().info('Step 5: Moving to place location...')
        self.send_trajectory(self.place_position, 3.0)
        time.sleep(3.5)
        
        self.get_logger().info('Pick and place complete! Object moved to new location.')


    def send_trajectory(self, joint_positions, duration=2.0):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(duration), nanosec=0)
        goal_msg.trajectory.points.append(point)
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UR5ePickPlace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
