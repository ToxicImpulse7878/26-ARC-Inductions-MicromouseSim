import os
import sys
import time
import importlib.util
import pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
from maze_layouts import MAZE_GRID, MAZE_SIZE, NUM_CELLS, GOAL_CELLS, START_POS

WINDOW_W         = 800
WINDOW_H         = 680
MAZE_MARGIN_PX   = 20
MAZE_DRAW_SIZE   = WINDOW_H - 2 * MAZE_MARGIN_PX   # 640 = 16 × 40 exactly
PX_PER_CELL      = MAZE_DRAW_SIZE / NUM_CELLS        # 40.0 px per cell
MAZE_MARGIN_X    = (WINDOW_W - MAZE_DRAW_SIZE) // 2  # 80 px — centres maze horizontally

COLOR_BG         = (18,  18,  22)
COLOR_FLOOR      = (34,  36,  42)
COLOR_WALL       = (220, 20,  60)
COLOR_GOAL       = (40,  200, 90)

def world_to_screen(wx: float, wy: float):
    """
    World space  : origin bottom-left, +Y upward
    Screen space : origin top-left,    +Y downward
    """
    px = MAZE_MARGIN_X  + wx * PX_PER_CELL
    py = MAZE_MARGIN_PX + (NUM_CELLS - wy) * PX_PER_CELL
    return (px, py)

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
    
    if top_speed < 0 or acceleration < 0 or turn_speed < 0 or sensor_range < 0:
        raise ValueError(" CONSTRAINT VIOLATION: Nice try! Point allocations cannot be negative.")

    if total_points != 30:
        raise ValueError(f" CONSTRAINT VIOLATION: Total allocated points ({total_points}) must equal exactly 30!")
            
    print(f"Constraint Verification Succeeded! (Used: {total_points}/30)")
    
    return {
        "max_speed": top_speed * 0.2,
        "accel_rate": acceleration * 0.1,
        "max_turn_rate": turn_speed * 0.15,
        "max_sensor_range": sensor_range * 0.4,
        "pts_speed": top_speed,
        "pts_accel": acceleration,
        "pts_turn": turn_speed,
        "pts_sensor": sensor_range,
    }

class VirtualMouse:
    RADIUS = 0.15  # collision radius in world units

    def __init__(self, config):
        self.config = config
        self.x, self.y = START_POS
        self.heading   = math.pi / 2  # facing north
        
        # Current speeds
        self.v_linear  = 0.0
        self.v_angular = 0.0
        
        # Commanded targets
        self.target_linear = 0.0
        self.target_angular = 0.0

    def set_targets(self, linear: float, angular: float):
        self.target_linear = linear
        self.target_angular = angular

    def update(self, dt: float):
        # --- Apply Acceleration Limit ---
        accel = self.config["accel_rate"]
        
        if self.v_linear < self.target_linear:
            self.v_linear = min(self.target_linear, self.v_linear + accel * dt)
        elif self.v_linear > self.target_linear:
            self.v_linear = max(self.target_linear, self.v_linear - accel * dt)
            
        # Turn speed is usually instant for micromice, but we cap its max rate
        self.v_angular = self.target_angular
        
        # --- Apply Physics ---
        self.heading += self.v_angular * dt
        dx = self.v_linear * math.cos(self.heading) * dt
        dy = self.v_linear * math.sin(self.heading) * dt
        
        nx, ny = self.x + dx, self.y + dy
        if not self._collides(nx, ny):
            self.x, self.y = nx, ny

    # ------------------------------------------------------------------
    # Wall queries — work in WALL-SPACE (integer world coords), not
    # grid-index space.  Walls live at integer wx / wy values (0-16).
    # ------------------------------------------------------------------

    def _h_wall(self, ky: int, col: int) -> bool:
        """Is there a horizontal wall at world y = ky in column col?"""
        col  = max(0, min(NUM_CELLS - 1, col))
        grow = ky * 2                         # draw_maze uses wy = grow/2 → grow = ky*2
        gcol = col * 2 + 1                   # grid col centre for this column
        if not (0 <= grow < MAZE_SIZE):
            return True
        return bool(MAZE_GRID[grow, gcol])

    def _v_wall(self, kx: int, row_top: int) -> bool:
        """Is there a vertical wall at world x = kx in row row_top (0 = top)?"""
        row_top = max(0, min(NUM_CELLS - 1, row_top))
        grow = row_top * 2 + 1               # grid row centre for this row
        gcol = kx * 2                        # grid col for this wall seam
        if not (0 <= gcol < MAZE_SIZE):
            return True
        return bool(MAZE_GRID[grow, gcol])

    def _collides(self, wx: float, wy: float) -> bool:
        """
        True if a circle of RADIUS centred at (wx, wy) overlaps any wall.
        Walls are thin lines at integer wx / wy values; we check distance
        to each integer coordinate within reach of the radius.
        """
        r       = self.RADIUS
        col     = max(0, min(NUM_CELLS - 1, int(wx)))
        row_top = max(0, min(NUM_CELLS - 1, int(wy)))

        for ky in range(int(math.floor(wy - r)), int(math.ceil(wy + r)) + 1):
            if abs(wy - ky) < r and self._h_wall(ky, col):
                return True

        for kx in range(int(math.floor(wx - r)), int(math.ceil(wx + r)) + 1):
            if abs(wx - kx) < r and self._v_wall(kx, row_top):
                return True

        return False

    def calculate_ui_raycasts(self) -> list:
        max_r = self.config["max_sensor_range"]
        return [
            self._cast_ray(self.heading + math.pi / 2, max_r),  # left
            self._cast_ray(self.heading,                max_r),  # front
            self._cast_ray(self.heading - math.pi / 2, max_r),  # right
        ]

    def _cast_ray(self, angle: float, max_range: float) -> float:
        """DDA raycast: steps to each cell-boundary crossing, returns exact hit distance."""
        EPS   = 1e-9
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        x0, y0 = self.x, self.y

        if abs(sin_a) > EPS:
            ky0  = math.floor(y0 + EPS) + 1 if sin_a > 0 else math.ceil(y0 - EPS) - 1
            t_h  = (ky0 - y0) / sin_a
            dt_h = 1.0 / abs(sin_a)
        else:
            t_h = dt_h = float('inf')

        if abs(cos_a) > EPS:
            kx0  = math.floor(x0 + EPS) + 1 if cos_a > 0 else math.ceil(x0 - EPS) - 1
            t_v  = (kx0 - x0) / cos_a
            dt_v = 1.0 / abs(cos_a)
        else:
            t_v = dt_v = float('inf')

        while min(t_h, t_v) < max_range:
            if t_h <= t_v:
                ky  = round(y0 + sin_a * t_h)
                col = max(0, min(NUM_CELLS - 1, int(x0 + cos_a * t_h)))
                if self._h_wall(ky, col):
                    return t_h
                t_h += dt_h
            else:
                kx      = round(x0 + cos_a * t_v)
                row_top = max(0, min(NUM_CELLS - 1, int(y0 + sin_a * t_v)))
                if self._v_wall(kx, row_top):
                    return t_v
                t_v += dt_v

        return max_range

class SimNode(Node):
    def __init__(self, physics_mouse_reference):
        super().__init__('micromouse_sim_node')
        self.mouse = physics_mouse_reference

        self.scan_pub = self.create_publisher(LaserScan, '/mouse/scan', 10)
        self.vel_pub  = self.create_publisher(Twist,     '/mouse/vel',  10)
        self.cmd_sub  = self.create_subscription(Twist, '/mouse/cmd_vel', self.cmd_callback, 10)

        self.timer = self.create_timer(1.0 / 20.0, self.publish_sensor_data)
        self.last_cmd_time = 0.0  # epoch → keyboard works immediately at startup

    def cmd_callback(self, msg):
        self.last_cmd_time = time.time()

        max_s = self.mouse.config["max_speed"]
        max_t = self.mouse.config["max_turn_rate"]

        target_linear  = max(-max_s, min(max_s, msg.linear.x))
        target_angular = max(-max_t, min(max_t, msg.angular.z))

        self.mouse.set_targets(target_linear, target_angular)

    def publish_sensor_data(self):
        # Only auto-stop if a ROS command was previously received and has timed out.
        # last_cmd_time == 0.0 means only keyboard is in use — don't interfere.
        if self.last_cmd_time > 0 and time.time() - self.last_cmd_time > 0.5:
            self.mouse.set_targets(0.0, 0.0)

        # laser scan — left / front / right distances
        ranges = self.mouse.calculate_ui_raycasts()
        scan = LaserScan()
        scan.header.stamp    = self.get_clock().now().to_msg()
        scan.header.frame_id = 'mouse_link'
        scan.ranges          = ranges
        self.scan_pub.publish(scan)

        # current velocity — solver can subscribe to /mouse/vel to read this back
        vel = Twist()
        vel.linear.x  = self.mouse.v_linear
        vel.angular.z = self.mouse.v_angular
        self.vel_pub.publish(vel)


def draw_maze(surface: pygame.Surface):
    surface.fill(COLOR_BG)

    # Floor background
    floor_tl = world_to_screen(0, NUM_CELLS)
    pygame.draw.rect(surface, COLOR_FLOOR,
                     pygame.Rect(floor_tl[0], floor_tl[1],
                                 MAZE_DRAW_SIZE, MAZE_DRAW_SIZE))

    # Goal zone
    goal_rows = [r for r, c in GOAL_CELLS]
    goal_cols = [c for r, c in GOAL_CELLS]
    gx0, gx1 = min(goal_cols), max(goal_cols) + 1
    gy0, gy1 = min(goal_rows), max(goal_rows) + 1
    goal_tl   = world_to_screen(gx0, gy1)
    goal_surf = pygame.Surface(
        (int((gx1 - gx0) * PX_PER_CELL), int((gy1 - gy0) * PX_PER_CELL)),
        pygame.SRCALPHA
    )
    goal_surf.fill((*COLOR_GOAL, 110))
    surface.blit(goal_surf, goal_tl)

    # Wall drawing
    # The 33x33 grid has three kinds of solid cells:
    #   (even row, odd col)  -> horizontal wall segment spanning one cell width
    #   (odd row,  even col) -> vertical wall segment spanning one cell height
    #   (even row, even col) -> corner peg, tiny dot
    

    WALL_T = 3   # wall thickness in pixels

    cell_px = PX_PER_CELL  # pixels per one full cell unit

    for grow in range(MAZE_SIZE):
        for gcol in range(MAZE_SIZE):
            if MAZE_GRID[grow, gcol] != 1:
                continue

            # Center of this grid cell in world space
            wx = gcol / 2.0
            wy = grow / 2.0
            cx, cy = world_to_screen(wx, wy)
            cx, cy = int(cx), int(cy)

            if grow % 2 == 0 and gcol % 2 == 1:
                # Horizontal wall segment
                half = int(cell_px // 2)
                pygame.draw.rect(surface, COLOR_WALL,
                    pygame.Rect(cx - half, cy - WALL_T // 2,
                                int(cell_px), WALL_T))

            elif grow % 2 == 1 and gcol % 2 == 0:
                # Vertical wall segment
                half = int(cell_px // 2)
                pygame.draw.rect(surface, COLOR_WALL,
                    pygame.Rect(cx - WALL_T // 2, cy - half,
                                WALL_T, int(cell_px)))

            elif grow % 2 == 0 and gcol % 2 == 0:
                # Corner peg
                pygame.draw.rect(surface, COLOR_WALL,
                    pygame.Rect(cx - WALL_T // 2, cy - WALL_T // 2,
                                WALL_T, WALL_T))


def draw_mouse(surface: pygame.Surface, mouse: VirtualMouse):
    sx, sy = world_to_screen(mouse.x, mouse.y)
    sx, sy = int(sx), int(sy)
    r_px = max(4, int(VirtualMouse.RADIUS * PX_PER_CELL))

    pygame.draw.circle(surface, (255, 200, 0), (sx, sy), r_px)

    # Heading arrow — screen +y is down so negate sin
    hx = int(sx + math.cos(mouse.heading) * r_px * 1.8)
    hy = int(sy - math.sin(mouse.heading) * r_px * 1.8)
    pygame.draw.line(surface, (255, 255, 255), (sx, sy), (hx, hy), 2)


def draw_rays(surface: pygame.Surface, mouse: VirtualMouse):
    max_r = mouse.config["max_sensor_range"]
    angles = [
        mouse.heading + math.pi / 2,  # left
        mouse.heading,                 # front
        mouse.heading - math.pi / 2,  # right
    ]
    sx, sy = world_to_screen(mouse.x, mouse.y)
    for angle in angles:
        dist = mouse._cast_ray(angle, max_r)
        ex = mouse.x + math.cos(angle) * dist
        ey = mouse.y + math.sin(angle) * dist
        esx, esy = world_to_screen(ex, ey)
        pygame.draw.line(surface, (80, 180, 255), (int(sx), int(sy)), (int(esx), int(esy)), 1)
        pygame.draw.circle(surface, (255, 80, 80), (int(esx), int(esy)), 3)


def draw_hud(surface: pygame.Surface, config: dict, elapsed_sec: float):
    font = pygame.font.SysFont("monospace", 13, bold=True)
    pts = [config["pts_speed"], config["pts_accel"], config["pts_turn"], config["pts_sensor"]]
    total = sum(pts)
    mins = int(elapsed_sec) // 60
    secs = elapsed_sec % 60
    lines = [
        (f"SPD {pts[0]:2d}", f"ACC {pts[1]:2d}"),
        (f"TRN {pts[2]:2d}", f"SNS {pts[3]:2d}"),
        (f"TOT {total:2d}/30", f"{mins:02d}:{secs:05.2f}"),
    ]
    pw, ph = 148, 60
    px = MAZE_MARGIN_X + MAZE_DRAW_SIZE - pw - 4
    py = MAZE_MARGIN_PX + 4
    panel = pygame.Surface((pw, ph), pygame.SRCALPHA)
    panel.fill((10, 10, 20, 190))
    surface.blit(panel, (px, py))
    y = py + 7
    for left, right in lines:
        surface.blit(font.render(left,  True, (190, 210, 255)), (px + 6, y))
        surface.blit(font.render(right, True, (190, 210, 255)), (px + pw // 2 + 4, y))
        y += 17


def draw_solved(surface: pygame.Surface, elapsed_sec: float):
    overlay = pygame.Surface((WINDOW_W, WINDOW_H), pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 160))
    surface.blit(overlay, (0, 0))
    font_big = pygame.font.SysFont("monospace", 52, bold=True)
    font_sm  = pygame.font.SysFont("monospace", 24)
    mins = int(elapsed_sec) // 60
    secs = elapsed_sec % 60
    t1 = font_big.render("SOLVED!", True, (60, 230, 110))
    t2 = font_sm.render(f"Time: {mins:02d}:{secs:05.2f}", True, (200, 255, 210))
    t3 = font_sm.render("Press R to restart", True, (160, 160, 180))
    cx, cy = WINDOW_W // 2, WINDOW_H // 2
    surface.blit(t1, t1.get_rect(center=(cx, cy - 40)))
    surface.blit(t2, t2.get_rect(center=(cx, cy + 20)))
    surface.blit(t3, t3.get_rect(center=(cx, cy + 55)))


def main():
    # if constraint fail, then crash with log
    try:
        config = validate_and_load_constraints()
    except ValueError as e:
        print(e)
        sys.exit(1)

    # init ROS and virtual mouse
    rclpy.init()
    virtual_mouse = VirtualMouse(config)
    sim_node = SimNode(virtual_mouse)

    # init pygame
    os.environ.setdefault("SDL_VIDEO_CENTERED", "1")
    pygame.init()
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    pygame.display.set_caption("Micromouse Simulator")
    clock = pygame.time.Clock()

    print("Simulation Engine is running...")

    max_s = config["max_speed"]
    max_t = config["max_turn_rate"]

    start_time  = time.time()
    solved      = False
    solved_time = 0.0

    # main loop
    running = True
    while running:
        dt = clock.tick(60) / 1000.0  # seconds since last frame
        dt = min(dt, 0.1) # Never simulate a step larger than 100ms

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    # --- Live-reload constraints on reset ---
                    try:
                        new_config = validate_and_load_constraints()
                        virtual_mouse.config = new_config
                        print("Constraints successfully reloaded!")
                    except Exception as e:
                        print(f"FAILED TO RELOAD CONSTRAINTS (Check your solver.py for errors): {e}")
                        print("Keeping previous constraints.")
                    
                    # Reset physics
                    virtual_mouse.x, virtual_mouse.y = START_POS
                    virtual_mouse.heading = math.pi / 2
                    virtual_mouse.set_targets(0.0, 0.0)
                    start_time  = time.time()
                    solved      = False
                    solved_time = 0.0

        if not solved:
            # keyboard fallback — only active when ROS solver isn't sending commands
            if time.time() - sim_node.last_cmd_time > 0.5:
                keys = pygame.key.get_pressed()
                kl = ka = 0.0
                if keys[pygame.K_w] or keys[pygame.K_UP]:    kl =  max_s
                if keys[pygame.K_s] or keys[pygame.K_DOWN]:  kl = -max_s
                if keys[pygame.K_a] or keys[pygame.K_LEFT]:  ka =  max_t
                if keys[pygame.K_d] or keys[pygame.K_RIGHT]: ka = -max_t
                virtual_mouse.set_targets(kl, ka)

            # --- PHYSICS ---
            virtual_mouse.update(dt)

            # --- GOAL DETECTION ---
            if 7.0 <= virtual_mouse.x <= 9.0 and 7.0 <= virtual_mouse.y <= 9.0:
                solved      = True
                solved_time = time.time() - start_time
                virtual_mouse.set_targets(0.0, 0.0)

        # --- ROS 2 SPIN ---
        rclpy.spin_once(sim_node, timeout_sec=0.0)

        # --- RENDERING ---
        elapsed = solved_time if solved else time.time() - start_time
        draw_maze(screen)
        draw_mouse(screen, virtual_mouse)
        draw_rays(screen, virtual_mouse)
        draw_hud(screen, config, elapsed)
        if solved:
            draw_solved(screen, solved_time)
        pygame.display.flip()

    # cleanup
    sim_node.destroy_node()
    rclpy.shutdown()
    pygame.quit()
    sys.exit(0)

if __name__ == '__main__':
    main()