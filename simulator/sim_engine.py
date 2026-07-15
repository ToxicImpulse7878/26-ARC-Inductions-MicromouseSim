import os
import sys
import time
import importlib.util
import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def validate_and_load_constraints():
    """
    Enforce the 30 point logic and fetch it from the solver file
    """
    solver_path = os.path.join(os.path.dirname(__file__), '../student_agent/solver.py')
    
    spec = importlib.util.spec_from_file_location("solver_module", solver_path)
    solver_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(solver_module)
    
    top_speed = getattr(solver_module, 'TOP_SPEED', 0)
    acceleration = getattr(solver_module, 'ACCELARATION', 0)
    turn_speed = getattr(solver_module, 'TURN_SPEED', 0)
    sensor_range = getattr(solver_module, 'SENSOR_RANGE', 0)
    
    total_points = top_speed + acceleration + turn_speed + sensor_range
    
    if total_points > 30:
        raise ValueError(f" CONSTRAINT VIOLATION: Total allocated points ({total_points}) exceeds budget of 30!")
    if total_points == 0:
        raise ValueError(" CONFIGURATION ERROR: Point values cannot be evaluated at zero.")
        
    print(f"Constraint Verification Succeeded! (Used: {total_points}/30)")
    
    return {
        "max_speed": top_speed * 0.2,          
        "accel_rate": acceleration * 0.1,      
        "max_turn_rate": turn_speed * 0.15,    
        "max_sensor_range": sensor_range * 0.4 
    }

class DummyMouse:
    """
    A temporary placeholder until the UI work is done. This gives the ROS node something to interact with so it doesn't crash.
    """
    def __init__(self, config):
        self.config = config
        
    def set_targets(self, linear, angular):
        # printing just to verify ros recieving cmds
        print(f"[DummyMouse] Received target velocity -> Linear: {linear:.2f}, Angular: {angular:.2f}")

    def calculate_ui_raycasts(self):
        # fake sensor data for now
        return [2.0, 2.0, 2.0] 

class SimNode(Node):
    def __init__(self, physics_mouse_reference):
        super().__init__('micromouse_sim_node')
        self.mouse = physics_mouse_reference 
        
        self.scan_pub = self.create_publisher(LaserScan, '/mouse/scan', 10)
        self.cmd_sub = self.create_subscription(Twist, '/mouse/cmd_vel', self.cmd_callback, 10)
        
        self.timer = self.create_timer(1.0 / 20.0, self.publish_sensor_data)
        self.last_cmd_time = time.time()
        
    def cmd_callback(self, msg):
        self.last_cmd_time = time.time()
        
        max_s = self.mouse.config["max_speed"]
        max_t = self.mouse.config["max_turn_rate"]
        
        target_linear = max(-max_s, min(max_s, msg.linear.x))
        target_angular = max(-max_t, min(max_t, msg.angular.z))
        
        self.mouse.set_targets(target_linear, target_angular)

    def publish_sensor_data(self):
        if time.time() - self.last_cmd_time > 0.5:
            self.mouse.set_targets(0.0, 0.0)
            
        ranges = self.mouse.calculate_ui_raycasts() 
        
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'mouse_link'
        scan.ranges = ranges
        
        self.scan_pub.publish(scan)

def main():
    # if constraint fail, then crash with log
    try:
        config = validate_and_load_constraints()
    except ValueError as e:
        print(e)
        sys.exit(1)

    # init ROS and dummy mouse
    rclpy.init()
    dummy_mouse = DummyMouse(config)
    sim_node = SimNode(dummy_mouse)

    # init pygame
    os.environ.setdefault("SDL_VIDEO_CENTERED", "1")
    pygame.init()
    screen = pygame.display.set_mode((800, 800))
    pygame.display.set_caption("Micromouse Simulator")
    clock = pygame.time.Clock()

    print("Simulation Engine is running...")

    # main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # --- PYGAME RENDERING ---
        screen.fill((18, 18, 22))
        pygame.draw.rect(screen, (220, 20, 60), pygame.Rect(300, 300, 200, 200))
        pygame.display.flip()

        # --- ROS 2 SPIN  ---
        # this allows ROS to process messages without freezing Pygame
        rclpy.spin_once(sim_node, timeout_sec=0.0)

        # cap at 60 FPS
        clock.tick(60)

    # cleanup
    sim_node.destroy_node()
    rclpy.shutdown()
    pygame.quit()
    sys.exit(0)

if __name__ == '__main__':
    main()