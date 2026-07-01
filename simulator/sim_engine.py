#!/usr/bin/env python3
"""
sim_engine.py — Core Micromouse simulation engine.

Combines a Pygame physics/rendering loop with a native ROS 2 node so that
a *separate* student process (student_agent/solver.py) can drive the mouse
purely over ROS 2 topics, with zero coupling to Pygame internals.

=========================================================================
COORDINATE SPACE GUARDRAIL — read this before touching any math below
=========================================================================
There are THREE coordinate systems in play and they all disagree with
each other. Every conversion point is centralized in the functions below
so there is exactly one place to fix bugs.

1. MAZE GRID SPACE (numpy array indices, from maze_layouts.MAZE_GRID)
       grid[row, col]   row increases DOWNWARD in the array
                         col increases RIGHTWARD in the array
       This is just data — never rendered or simulated directly.

2. WORLD SPACE (continuous Cartesian, used for all physics/kinematics)
       (x, y) in "cell units" (1.0 = one micromouse cell ~ 18cm IRL)
       +X is to the right (same sense as grid col)
       +Y is "up the maze" / forward from the start, i.e. increasing Y
          corresponds to increasing maze ROW. This matches how a robot's
          heading angle (theta) is conventionally defined: theta=0 along
          +X, theta=+90 deg along +Y, counter-clockwise positive — the
          standard math/robotics convention (and ROS REP-103).
       Grid <-> World relationship:
           world_x = grid_col / 2.0          (grid col 1 -> world x 0.5)
           world_y = grid_row / 2.0
         (Recall grid space is 2x finer resolution than cell space: cell
         (r,c) sits at grid (2r+1, 2c+1), i.e. world (c+0.5, r+0.5).)

3. SCREEN SPACE (Pygame pixel coordinates)
       (px, py) where (0,0) is the TOP-LEFT corner and +Y is DOWNWARD.
       This is the *opposite* vertical sense from world space, so every
       single draw call must flip Y. We do this in world_to_screen() and
       NOWHERE else — if you're tempted to flip Y inline somewhere else,
       stop and route through that function instead.

THE GOLDEN RULE: physics and kinematics ALWAYS operate in world space.
Only the rendering layer (draw_*  functions) ever touches screen space,
and only via world_to_screen(). The maze grid is only ever read through
maze_value_at() / is_wall_at(), never indexed directly elsewhere.
=========================================================================
"""

import math
import sys
import time
import threading

import numpy as np
import pygame

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from maze_layouts import (
    MAZE_GRID,
    MAZE_SIZE,
    NUM_CELLS,
    START_POS,
    GOAL_CELLS,
)

# ---------------------------------------------------------------------------
# Rendering / window configuration
# ---------------------------------------------------------------------------
WINDOW_SIZE = 800            # matches the 800x800 Xvfb virtual screen
MAZE_MARGIN_PX = 20           # padding around the maze drawing
MAZE_DRAW_SIZE_PX = WINDOW_SIZE - 2 * MAZE_MARGIN_PX
PX_PER_CELL = MAZE_DRAW_SIZE_PX / NUM_CELLS   # pixels per 1.0 world unit
TARGET_FPS = 60

COLOR_BG = (18, 18, 22)
COLOR_FLOOR = (34, 36, 42)
COLOR_WALL = (220, 20, 60)        # crimson red
COLOR_GOAL = (40, 200, 90, 110)   # translucent green (RGBA)
COLOR_RAY = (255, 215, 0)         # yellow
COLOR_MOUSE = (40, 130, 255)      # electric blue
COLOR_GRID_LINE = (50, 52, 60)
COLOR_TEXT = (230, 230, 235)

# ---------------------------------------------------------------------------
# Mouse / physics configuration
# ---------------------------------------------------------------------------
MOUSE_RADIUS_WORLD = 0.18         # collision circle radius, in cell-units
MAX_LINEAR_VEL = 2.0              # cell-units / sec
MAX_ANGULAR_VEL = 6.0             # rad / sec
SENSOR_MAX_RANGE = 4.0            # cell-units
SENSOR_ANGLES_DEG = [90.0, 0.0, -90.0]   # Left, Front, Right (relative to heading)
SENSOR_NAMES = ["left", "front", "right"]
RAYCAST_STEP = 0.02                # marching step size for raycasting, world units

CMD_VEL_TIMEOUT_SEC = 0.5          # safety: stop the mouse if no cmd_vel arrives in time


# ---------------------------------------------------------------------------
# Coordinate transforms — THE single source of truth (see module docstring)
# ---------------------------------------------------------------------------
def world_to_screen(wx: float, wy: float) -> tuple:
    """World space (cell-units, +Y up) -> Pygame pixel space (+Y down)."""
    px = MAZE_MARGIN_PX + wx * PX_PER_CELL
    py = MAZE_MARGIN_PX + (NUM_CELLS - wy) * PX_PER_CELL
    return (px, py)


def world_len_to_px(length: float) -> float:
    """Convert a world-space distance (no direction) to a pixel distance."""
    return length * PX_PER_CELL


def world_to_grid(wx: float, wy: float) -> tuple:
    """World space -> nearest maze grid index (row, col), clamped in-bounds."""
    gcol = int(round(wx * 2.0))
    grow = int(round(wy * 2.0))
    gcol = max(0, min(MAZE_SIZE - 1, gcol))
    grow = max(0, min(MAZE_SIZE - 1, grow))
    return (grow, gcol)


def is_wall_at(wx: float, wy: float) -> bool:
    """True if the given world-space point lies inside a solid wall cell."""
    if wx < 0 or wy < 0 or wx > NUM_CELLS or wy > NUM_CELLS:
        return True  # outside the maze bounds counts as solid
    grow, gcol = world_to_grid(wx, wy)
    return bool(MAZE_GRID[grow, gcol] == 1)


def world_to_cell(wx: float, wy: float) -> tuple:
    """World space -> the (row, col) micromouse CELL (not grid index) it's in."""
    col = int(math.floor(wx))
    row = int(math.floor(wy))
    col = max(0, min(NUM_CELLS - 1, col))
    row = max(0, min(NUM_CELLS - 1, row))
    return (row, col)


# ---------------------------------------------------------------------------
# VirtualMouse — differential-drive kinematics + collision + sensors
# ---------------------------------------------------------------------------
class VirtualMouse:
    """
    Simple differential-drive kinematic model operating entirely in WORLD
    SPACE. Commanded via (linear_vel, angular_vel) exactly like a ROS 2
    geometry_msgs/Twist (using only linear.x and angular.z, the standard
    2D mobile-robot convention).
    """

    def __init__(self, x: float, y: float, theta: float = math.pi / 2):
        self.x = x
        self.y = y
        self.theta = theta  # radians, 0 = +X axis, increases CCW
        self.linear_vel = 0.0    # commanded forward speed, cell-units/sec
        self.angular_vel = 0.0   # commanded turn rate, rad/sec
        self.radius = MOUSE_RADIUS_WORLD
        self.last_cmd_time = time.monotonic()
        self._lock = threading.Lock()

    def set_cmd(self, linear: float, angular: float) -> None:
        """Thread-safe setter — called from the ROS callback thread context."""
        with self._lock:
            self.linear_vel = max(-MAX_LINEAR_VEL, min(MAX_LINEAR_VEL, linear))
            self.angular_vel = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angular))
            self.last_cmd_time = time.monotonic()

    def _safety_check(self) -> tuple:
        """Zero out velocity if no fresh cmd_vel has arrived (stale-command safety stop)."""
        with self._lock:
            if time.monotonic() - self.last_cmd_time > CMD_VEL_TIMEOUT_SEC:
                return (0.0, 0.0)
            return (self.linear_vel, self.angular_vel)

    def step(self, dt: float) -> None:
        """
        Advance the kinematic state by dt seconds using a standard
        differential-drive unicycle model:
            x'     = x + v * cos(theta) * dt
            y'     = y + v * sin(theta) * dt
            theta' = theta + omega * dt
        Then resolves collisions against the maze via axis-separated
        circle-vs-wall-cell checks so the mouse slides along walls
        instead of getting stuck dead the instant it grazes one.
        """
        v, omega = self._safety_check()

        new_theta = self.theta + omega * dt
        new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))  # wrap to [-pi, pi]

        dx = v * math.cos(self.theta) * dt
        dy = v * math.sin(self.theta) * dt

        # Resolve X and Y movement independently so sliding along a wall
        # in one axis still permits movement along the other.
        candidate_x = self.x + dx
        if not self._circle_collides(candidate_x, self.y):
            self.x = candidate_x

        candidate_y = self.y + dy
        if not self._circle_collides(self.x, candidate_y):
            self.y = candidate_y

        self.theta = new_theta

    def _circle_collides(self, cx: float, cy: float) -> bool:
        """
        Checks the mouse's collision circle (centered at cx, cy) against
        nearby wall cells. Samples points around the circle's perimeter
        plus the center — cheap and robust enough for thin micromouse
        walls at this scale (walls are ~0.1 world units thick at this
        grid resolution).
        """
        if is_wall_at(cx, cy):
            return True
        samples = 8
        for i in range(samples):
            ang = (2 * math.pi / samples) * i
            sx = cx + self.radius * math.cos(ang)
            sy = cy + self.radius * math.sin(ang)
            if is_wall_at(sx, sy):
                return True
        return False

    def cast_ray(self, angle_offset_deg: float) -> float:
        """
        Marches a ray from the mouse's center, at heading + angle_offset,
        until it hits a wall or exceeds SENSOR_MAX_RANGE. Returns the
        distance in world (cell) units. This directly models a
        time-of-flight / IR distance sensor, the standard micromouse
        sensor suite.
        """
        ray_angle = self.theta + math.radians(angle_offset_deg)
        dxr = math.cos(ray_angle)
        dyr = math.sin(ray_angle)

        dist = 0.0
        while dist < SENSOR_MAX_RANGE:
            sx = self.x + dxr * dist
            sy = self.y + dyr * dist
            if is_wall_at(sx, sy):
                return max(0.0, dist - RAYCAST_STEP)
            dist += RAYCAST_STEP
        return SENSOR_MAX_RANGE

    def sensor_readings(self) -> dict:
        """Returns {'left': d, 'front': d, 'right': d} in world (cell) units."""
        return {
            name: self.cast_ray(angle)
            for name, angle in zip(SENSOR_NAMES, SENSOR_ANGLES_DEG)
        }

    def reached_goal(self) -> bool:
        row, col = world_to_cell(self.x, self.y)
        return (row, col) in GOAL_CELLS


# ---------------------------------------------------------------------------
# MicromouseSimNode — the ROS 2 side of the bridge
# ---------------------------------------------------------------------------
class MicromouseSimNode(Node):
    """
    Native ROS 2 node embedded directly in the simulation process.

    Subscribes : /mouse/cmd_vel  (geometry_msgs/Twist)   <- student's solver
    Publishes  : /mouse/scan     (sensor_msgs/LaserScan) -> student's solver

    The LaserScan message is repurposed (rather than introducing a custom
    msg type, which would require building a separate ROS package) to
    carry exactly 3 ranges — left, front, right — at fixed angular
    offsets. This keeps the student dependency surface to core ROS 2
    message types only, with no custom .msg compilation step required.
    """

    SCAN_FRAME_ID = "mouse_base_link"

    def __init__(self, mouse: VirtualMouse):
        super().__init__("micromouse_sim_node")
        self.mouse = mouse

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/mouse/cmd_vel", self._on_cmd_vel, 10
        )
        self.scan_pub = self.create_publisher(LaserScan, "/mouse/scan", 10)

        # Publish sensor data on a fixed timer, independent of Pygame's
        # frame rate, so students get a steady, predictable scan rate.
        self.scan_publish_hz = 20.0
        self.scan_timer = self.create_timer(1.0 / self.scan_publish_hz, self._publish_scan)

        self.get_logger().info(
            "MicromouseSimNode ready. Listening on /mouse/cmd_vel, "
            "publishing /mouse/scan."
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.mouse.set_cmd(msg.linear.x, msg.angular.z)

    def _publish_scan(self) -> None:
        readings = self.mouse.sensor_readings()
        # Pack left/front/right into a 3-element LaserScan, with angles
        # matching SENSOR_ANGLES_DEG order: [left(+90deg), front(0), right(-90deg)].
        # angle_min/max/increment describe that 3-sample sweep from +90 to -90.
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.SCAN_FRAME_ID
        msg.angle_min = math.radians(SENSOR_ANGLES_DEG[0])   # +90 deg (left)
        msg.angle_max = math.radians(SENSOR_ANGLES_DEG[-1])  # -90 deg (right)
        msg.angle_increment = math.radians(
            (SENSOR_ANGLES_DEG[-1] - SENSOR_ANGLES_DEG[0]) / (len(SENSOR_ANGLES_DEG) - 1)
        )
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / self.scan_publish_hz
        msg.range_min = 0.0
        msg.range_max = SENSOR_MAX_RANGE
        msg.ranges = [
            readings["left"],
            readings["front"],
            readings["right"],
        ]
        msg.intensities = []
        self.scan_pub.publish(msg)


# ---------------------------------------------------------------------------
# Rendering
# ---------------------------------------------------------------------------
def draw_maze(surface: pygame.Surface) -> None:
    """
    Draws the maze grid: floor tiles, crimson walls, and the translucent
    green goal zone. Walls are drawn as filled rects matching each solid
    grid cell's footprint in world space, mapped through world_to_screen.
    """
    surface.fill(COLOR_BG)

    cell_px = PX_PER_CELL / 2.0  # half-cell == one grid-index step in pixels

    # Floor (drawn under everything, full maze extent)
    floor_tl = world_to_screen(0, NUM_CELLS)
    floor_rect = pygame.Rect(
        floor_tl[0], floor_tl[1], MAZE_DRAW_SIZE_PX, MAZE_DRAW_SIZE_PX
    )
    pygame.draw.rect(surface, COLOR_FLOOR, floor_rect)

    # Goal zone highlight (translucent green) — drawn before walls so wall
    # pegs/edges still render crisply on top of it.
    goal_rows = [r for r, c in GOAL_CELLS]
    goal_cols = [c for r, c in GOAL_CELLS]
    gx0, gx1 = min(goal_cols), max(goal_cols) + 1
    gy0, gy1 = min(goal_rows), max(goal_rows) + 1
    goal_tl = world_to_screen(gx0, gy1)
    goal_w = world_len_to_px(gx1 - gx0)
    goal_h = world_len_to_px(gy1 - gy0)
    goal_surf = pygame.Surface((goal_w, goal_h), pygame.SRCALPHA)
    goal_surf.fill(COLOR_GOAL)
    surface.blit(goal_surf, goal_tl)

    # Walls: iterate every grid cell; each solid grid cell occupies one
    # half-cell-pitch footprint in world space.
    for grow in range(MAZE_SIZE):
        for gcol in range(MAZE_SIZE):
            if MAZE_GRID[grow, gcol] != 1:
                continue
            wx = gcol / 2.0
            wy = grow / 2.0
            # Each grid step is half a world unit; draw a square footprint
            # of that size centered appropriately so adjacent wall segments
            # tile seamlessly with no gaps.
            top_left_world = (wx - 0.25, wy + 0.25)
            px, py = world_to_screen(*top_left_world)
            size_px = cell_px / 1.0 + 1  # +1 to avoid hairline seams between tiles
            pygame.draw.rect(
                surface, COLOR_WALL, pygame.Rect(px, py, size_px, size_px)
            )


def draw_sensors(surface: pygame.Surface, mouse: VirtualMouse, readings: dict) -> None:
    """Draws the three yellow raycast distance-sensor streams."""
    origin_px = world_to_screen(mouse.x, mouse.y)
    for name, angle_offset in zip(SENSOR_NAMES, SENSOR_ANGLES_DEG):
        dist = readings[name]
        ray_angle = mouse.theta + math.radians(angle_offset)
        end_wx = mouse.x + math.cos(ray_angle) * dist
        end_wy = mouse.y + math.sin(ray_angle) * dist
        end_px = world_to_screen(end_wx, end_wy)
        pygame.draw.line(surface, COLOR_RAY, origin_px, end_px, 2)
        pygame.draw.circle(surface, COLOR_RAY, (int(end_px[0]), int(end_px[1])), 3)


def draw_mouse(surface: pygame.Surface, mouse: VirtualMouse) -> None:
    """
    Draws the mouse as an electric-blue triangle pointing along theta,
    so heading is visible at a glance — this is the standard way to
    render a differential-drive robot's orientation in 2D sims.
    """
    r = mouse.radius
    # Triangle points: nose (forward), and two rear corners, all defined
    # in the mouse's local frame then rotated by theta and translated.
    local_pts = [
        (r * 1.4, 0.0),       # nose
        (-r * 0.9, r * 0.8),  # rear-left
        (-r * 0.9, -r * 0.8),  # rear-right
    ]
    world_pts = []
    for lx, ly in local_pts:
        rx = lx * math.cos(mouse.theta) - ly * math.sin(mouse.theta)
        ry = lx * math.sin(mouse.theta) + ly * math.cos(mouse.theta)
        world_pts.append((mouse.x + rx, mouse.y + ry))
    screen_pts = [world_to_screen(wx, wy) for wx, wy in world_pts]
    pygame.draw.polygon(surface, COLOR_MOUSE, screen_pts)
    pygame.draw.polygon(surface, (255, 255, 255), screen_pts, 1)


def draw_hud(surface: pygame.Surface, font: pygame.font.Font, mouse: VirtualMouse, readings: dict, goal_reached: bool) -> None:
    """Small heads-up readout: sensor distances + goal status."""
    lines = [
        f"pos=({mouse.x:.2f}, {mouse.y:.2f})  theta={math.degrees(mouse.theta):.1f} deg",
        f"L={readings['left']:.2f}  F={readings['front']:.2f}  R={readings['right']:.2f}",
    ]
    if goal_reached:
        lines.append("GOAL REACHED!")
    for i, line in enumerate(lines):
        text_surf = font.render(line, True, COLOR_TEXT)
        surface.blit(text_surf, (8, 8 + i * 18))


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main() -> int:
    # --- Pygame / display setup ---------------------------------------
    import os
    os.environ.setdefault("SDL_VIDEO_CENTERED", "1")

    pygame.init()
    pygame.display.set_caption("Micromouse Simulator")
    screen = pygame.display.set_mode((WINDOW_SIZE, WINDOW_SIZE))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 14)

    # --- ROS 2 setup -----------------------------------------------------
    rclpy.init(args=None)
    mouse = VirtualMouse(x=START_POS[0], y=START_POS[1], theta=math.pi / 2)
    node = MicromouseSimNode(mouse)

    print("[sim_engine] Simulation started. Window centered on virtual display.")
    print(f"[sim_engine] Mouse start position: world={START_POS}")

    running = True
    try:
        while running:
            dt = clock.tick(TARGET_FPS) / 1000.0
            dt = min(dt, 0.1)  # clamp to avoid huge steps after a stall/breakpoint

            # --- Event handling ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        # Manual reset hotkey, handy during development/demos.
                        mouse.x, mouse.y = START_POS
                        mouse.theta = math.pi / 2
                        mouse.set_cmd(0.0, 0.0)

            # --- Non-blocking ROS 2 spin, interleaved into the frame loop ---
            rclpy.spin_once(node, timeout_sec=0)

            # --- Physics step ---
            mouse.step(dt)

            # --- Render ---
            draw_maze(screen)
            readings = mouse.sensor_readings()
            draw_sensors(screen, mouse, readings)
            draw_mouse(screen, mouse)
            goal_reached = mouse.reached_goal()
            draw_hud(screen, font, mouse, readings, goal_reached)
            pygame.display.flip()

    except KeyboardInterrupt:
        pass
    finally:
        print("[sim_engine] Shutting down...")
        node.destroy_node()
        rclpy.shutdown()
        pygame.quit()

    return 0


if __name__ == "__main__":
    sys.exit(main())
