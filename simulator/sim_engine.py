# simulator/sim_engine.py
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from simulator.maze_layouts import active_env

class VirtualMouse:
    def __init__(self, x: float, y: float, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta 
        

        self.v = 0.0 
        self.omega = 0.0 #rad/sec
        

        self.radius = 0.25 
        self.max_sensor_range = 5.0 

    def step(self, dt: float):
        """
        Updates the mouse position using Kinematics
        and checks for wall collisions.
        """
        # 1. normalize between -pi and pi
        self.theta += self.omega * dt
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
        
        # 2. Compute proposed next position
        new_x = self.x + self.v * math.cos(self.theta) * dt
        new_y = self.y + self.v * math.sin(self.theta) * dt
        
        # 3. Collision handling: Check points along its bounding circle
        if not self._check_collision(new_x, new_y):
            self.x = new_x
            self.y = new_y
        else:
            # Penalize by stopping forward motion on impact
            self.v = 0.0

    def _check_collision(self, nx: float, ny: float) -> bool:
        """Helper to check if the mouse boundary intersects a wall cell."""
        # Check 4 cardinal points around the circle perimeter
        for angle in [0, math.pi/2, math.pi, 3*math.pi/2]:
            cx = nx + self.radius * math.cos(angle)
            cy = ny + self.radius * math.sin(angle)
            if active_env.is_wall(cx, cy):
                return True
        return False

    def cast_ray(self, relative_angle: float) -> float:
        """
        Simulates a distance sensor (like a Time-of-Flight or LiDAR ray) 
        by step-marching a ray forward until it hits a wall matrix block.
        """
        # Absolute angle of the sensor ray in the global frame
        ray_angle = self.theta + relative_angle
        
        step_size = 0.05
        distance = 0.0
        
        while distance < self.max_sensor_range:
            distance += step_size
            # Calculate checking coordinate along the ray vector
            rx = self.x + distance * math.cos(ray_angle)
            ry = self.y + distance * math.sin(ray_angle)
            
            if active_env.is_wall(rx, ry):
                return distance
                
        return self.max_sensor_range

    def get_sensor_readings(self):
        """
        Returns distance values for Left (+90 deg), Front (0 deg), and Right (-90 deg) sensors.
        """
        return {
            "left": self.cast_ray(math.pi / 2),
            "front": self.cast_ray(0.0),
            "right": self.cast_ray(-math.pi / 2)
        }


class MicromouseSimNode(Node):
    def __init__(self):
        super().__init__('micromouse_sim_node')
        
        # 1. Initialize our physical mouse state at the training start position
        self.mouse = VirtualMouse(active_env.start_pos[0], active_env.start_pos[1])
        
        # 2. ROS Subscriber: Listen for movement commands from the student's solver
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/mouse/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # 3. ROS Publisher: Broadcast simulated distance sensors to the student
        self.scan_pub = self.create_publisher(LaserScan, '/mouse/scan', 10)
        
        # 4. Simulation Loop Timer: Run physics updates and publish data at 20Hz (dt = 0.05s)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.sim_loop_callback)
        
        self.get_logger().info("ROS 2 Micromouse Simulator Node initialized successfully.")

    def cmd_vel_callback(self, msg: Twist):
        """Catches the steering velocities sent by the student's algorithm node."""
        self.mouse.v = msg.linear.x
        self.mouse.omega = msg.angular.z

    def sim_loop_callback(self):
        """The main heart-beat loop of the simulator."""
        # Step 1: Advance physics and handle collisions
        self.mouse.step(self.dt)
        
        # Step 2: Get the fresh raycasted sensor data
        readings = self.mouse.get_sensor_readings()
        
        # Step 3: Package it into a standard ROS LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'mouse_link'
        
        # We map our 3 discrete rays (Left, Front, Right) into the ranges array
        # Freshers will read: msg.ranges[0] (Left), msg.ranges[1] (Front), msg.ranges[2] (Right)
        scan_msg.ranges = [readings['left'], readings['front'], readings['right']]
        
        # Publish the sensor data so the student's code can see the walls
        self.scan_pub.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MicromouseSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()