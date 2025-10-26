#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity

class CubeSpawner(Node):
    def __init__(self):
        super().__init__('cube_spawner')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        while not self.spawn_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        
        self.get_logger().info('Spawn service ready!')
    
    def spawn_cube(self, x, y, z):
        """Spawn a simple colored cube"""
        
        sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="tracked_cube">
    <static>false</static>
    <link name="cube_link">
      <pose>0.5 -0.5 1.0 0 0 0 </pose>
      
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.00004167</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.00004167</iyy>
          <iyz>0.0</iyz>
          <izz>0.00004167</izz>
        </inertia>
      </inertial>
      
      <visual name="visual">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.5 0.5 0.5 1</specular>
        </material>
      </visual>
      
      <collision name="collision">
        <geometry>
          <box>
            <size>0.05 0.05 0.05</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>"""
        
        req = SpawnEntity.Request()
        req.name = 'tracked_cube'
        req.xml = sdf
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        req.initial_pose.orientation.w = 1.0
        
        self.get_logger().info(f'Spawning cube at ({x}, {y}, {z})')
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info(f'✓ Cube spawned: {future.result().status_message}')
            return True
        else:
            self.get_logger().error('✗ Spawn failed')
            return False


def main(args=None):
    rclpy.init(args=args)
    spawner = CubeSpawner()
    
    # Spawn in UR5e's reachable workspace
    spawner.spawn_cube(x=0.4, y=0.2, z=0.825)
    
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
