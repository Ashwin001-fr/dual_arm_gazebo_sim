#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
from ur_analytic_ik import ur5e
import time


class MotionPatternGenerator(Node):
    def __init__(self):
        super().__init__('motion_pattern_generator')
        
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )
        
        # Motion parameters (SLOWER!)
        self.center = np.array([-0.32, -0.44, 0.3])
        self.radius = 0.23  # Smaller radius
        self.frequency = 0.08  # Slower frequency (was 0.5)
        self.dt = 0.5  # 2 Hz control rate (was 0.1)
        self.trajectory_duration = 0.6  # Give robot time to reach each point
        
        self.get_logger().info('Motion Pattern Generator Ready')
        self._action_client.wait_for_server()
        
        self.create_timer(2.0, self.start_motion)
    
    
    def start_motion(self):
        self.get_logger().info('Starting motion pattern...')
        
        # Execute circular motion for 15 seconds
        self.execute_circle(duration=15.0)
        
        time.sleep(2.0)
        
        # Execute Lissajous (∞) motion for 15 seconds
        self.execute_lissajous(duration=15.0)
    
    
    def execute_circle(self, duration=15.0):
        """Execute circular trajectory in YZ plane with sinusoidal X"""
        self.get_logger().info('Executing CIRCULAR motion...')
        
        t = 0
        while t < duration:
            theta = 2 * np.pi * self.frequency * t
            
            # Position
            x = self.center[0] + 0.03 * np.sin(2 * np.pi * 0.2 * t)
            y = self.center[1] + self.radius * np.cos(theta)
            z = self.center[2] + self.radius * np.sin(theta)
            
            success = self.move_to_position(x, y, z)
            if not success:
                self.get_logger().warn('Skipping unreachable position')
            
            time.sleep(self.dt)
            t += self.dt
        
        self.get_logger().info('Circle complete!')
    
    
    def execute_lissajous(self, duration=15.0):
        """Execute Lissajous (∞) curve in YZ plane with sinusoidal X"""
        self.get_logger().info('Executing LISSAJOUS (∞) motion...')
        
        t = 0
        while t < duration:
            theta = 2 * np.pi * self.frequency * t
            
            # Position
            x = self.center[0] + 0.03 * np.sin(2 * np.pi * 0.2 * t)
            y = self.center[1] + self.radius * np.sin(theta)
            z = self.center[2] + self.radius * np.sin(2 * theta) / 2
            
            success = self.move_to_position(x, y, z)
            if not success:
                self.get_logger().warn('Skipping unreachable position')
            
            time.sleep(self.dt)
            t += self.dt
        
        self.get_logger().info('Lissajous complete!')
    
    
    def move_to_position(self, x, y, z):
        """Move robot to Cartesian position using IK"""
        eef_pose = np.identity(4)
        eef_pose[0, 3] = x
        eef_pose[1, 3] = y
        eef_pose[2, 3] = z
        eef_pose[0:3, 0] = [-1.0, 0.0, 0.0]
        eef_pose[0:3, 1] = [0.0, 1.0, 0.0]
        eef_pose[0:3, 2] = [0.0, 0.0, -1.0]
        
        solutions = ur5e.inverse_kinematics(eef_pose)
        
        if len(solutions) == 0:
            return False
        
        joint_positions = solutions[0].tolist()
        self.send_trajectory(joint_positions)
        return True
    
    
    def send_trajectory(self, joint_positions):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * 6  # Stop at each point
        point.time_from_start = Duration(sec=0, nanosec=int(self.trajectory_duration * 1e9))
        goal_msg.trajectory.points.append(point)
        
        # Set goal tolerances
        goal_msg.goal_tolerance = []
        goal_msg.path_tolerance = []
        
        self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotionPatternGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
