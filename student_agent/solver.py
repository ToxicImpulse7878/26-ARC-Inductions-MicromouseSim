"""
Write your own solver in the scan_callback function
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random

# ==========================================
# These four parameters MUST add up to exactly 30!
# ==========================================
TOP_SPEED = 5
ACCELARATION = 5
TURN_SPEED = 10
SENSOR_RANGE = 10

class StudentSolver(Node):
    def __init__(self):
        super().__init__('student_solver')
        
        # subscriber to read sensor values (L,F,R)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/mouse/scan',
            self.scan_callback,
            10
        )
        
        # publisher to send movement commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mouse/cmd_vel',
            10
        )
        
        self.get_logger().info("Student Solver Node initialized successfully.")
        self.get_logger().info(f"Stats -> Speed: {TOP_SPEED}, Accel: {ACCELARATION}, Turn: {TURN_SPEED}, Range: {SENSOR_RANGE}")
        self.prev_left = 0.0


    def scan_callback(self, msg):
        """
        This function runs every time a new sensor reading is received (at 20 Hz).
        msg.ranges contains the distances:
        msg.ranges[0] -> Left ray distance
        msg.ranges[1] -> Front ray distance
        msg.ranges[2] -> Right ray distance
        """

        d_left = msg.ranges[0]
        d_front = msg.ranges[1]
        d_right = msg.ranges[2]
        dL = d_left - self.prev_left     
        

        print(f"L={d_left:.2f} F={d_front:.2f} R={d_right:.2f} dL={dL:.2f}")

        cmd = Twist()
        
        #-------- DEMO LOGIC, REMOVE THIS AND WRITE YOUR OWN ---------
        # 1. Front is blocked -> Pivot strictly in place (do not move forward!)
        # Increased threshold to 0.65 so it has room to spin without its 0.15 radius clipping the front wall
        
        if d_front < 0.65:
            cmd.linear.x = 0.0
            cmd.angular.z = -1.5
            
        # 2. Left side is open -> Curve around the corner
        elif d_left > 0.8:
            cmd.linear.x = 0.3
            cmd.angular.z = 1.2   # Turn left
            
        # 3. Wall hugging -> P-Controller
        else:
            cmd.linear.x = 0.5
            
            # The cell is 1.0 units wide. Perfect center is 0.5.
            target_distance = 0.5 
            error = d_left - target_distance
            
            # Multiply error by a gain to steer back to the center
            cmd.angular.z = error * 3.0
        #-----------------------------------------------------------------
        self.prev_left = d_left   
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = StudentSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

