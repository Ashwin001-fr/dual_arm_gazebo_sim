#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point


class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')

        self.declare_parameter('object_name', 'tracked_cube')
        self.object_name = self.get_parameter('object_name').value
        self.object_published = False

        self.robot_base_x = 0.587792
        self.robot_base_y = -0.760304
        self.robot_base_z = 1.05

        self.model_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        self.position_pub = self.create_publisher(
            Point,
            '/object_position',
            10
        )

        self.get_logger().info("Object Tracker ready")

    def model_states_callback(self, msg):
        if self.object_published:
            return
        
        if self.object_name in msg.name:
            idx = msg.name.index(self.object_name)
            pose = msg.pose[idx]
            
            wx = pose.position.x
            wy = pose.position.y
            wz = pose.position.z
            
            self.get_logger().info(f"World: x={wx:.3f}, y={wy:.3f}, z={wz:.3f}")
            
            # Calculate deltas from robot base
            dx = wx - self.robot_base_x
            dy = wy - self.robot_base_y
            dz = wz - self.robot_base_z
            
            # Transform to robot frame (empirical from your data)
            robot_frame_position = Point()
            robot_frame_position.x = dy * -0.333  # World Y → Robot X (inverted, scaled)
            robot_frame_position.y = dx * 2.34    # World X → Robot Y (scaled)
            robot_frame_position.z = -dz * 0.001  # World Z → Robot Z (inverted, scaled)
            
            self.get_logger().info(
                f"Robot: x={robot_frame_position.x:.3f}, "
                f"y={robot_frame_position.y:.3f}, z={robot_frame_position.z:.3f}"
            )
            
            self.position_pub.publish(robot_frame_position)
            self.object_published = True


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
