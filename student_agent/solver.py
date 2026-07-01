#!/usr/bin/env python3
"""
student_agent/solver.py — Micromouse Student Agent Template
============================================================
This is YOUR file. Fork it, break it, rewrite it.

HOW TO RUN (inside the container, in a second terminal):
    docker exec -it micromouse_simulator bash
    python3 student_agent/solver.py

HOW IT WORKS
------------
The simulation engine (simulator/sim_engine.py) runs as a ROS 2 node
that owns the physics and the Pygame window. This script is a completely
separate ROS 2 node that talks to it over two topics:

    /mouse/scan      (sensor_msgs/LaserScan)   <- you READ from this
    /mouse/cmd_vel   (geometry_msgs/Twist)      <- you WRITE to this

The LaserScan message carries exactly 3 distance readings (in cell-units,
where 1.0 ≈ one maze cell ≈ 18 cm on a real micromouse):

    msg.ranges[0]  = left sensor distance   (ray at +90° from heading)
    msg.ranges[1]  = front sensor distance  (ray at   0° from heading)
    msg.ranges[2]  = right sensor distance  (ray at -90° from heading)

    msg.range_max  = maximum measurable range (4.0 cell-units)

The Twist message uses only two fields (standard 2D mobile robot):
    msg.linear.x   = forward speed  (cell-units / second, + = forward)
    msg.angular.z  = turn rate      (radians / second, + = turn left / CCW)

BUILT-IN BASELINE: LEFT-HAND WALL FOLLOWER
-------------------------------------------
The default implementation below runs a simple reactive wall-follower that
keeps the left wall in contact whenever possible and turns away from
obstacles in front. It is intentionally simple so you can trace every
decision the robot makes just by reading the callback.

Your challenge: replace this with something smarter.
  - Flood-fill solver?
  - Right-hand follower?
  - A* with the sensor map you build up over multiple runs?
  - Pure-pursuit trajectory follower?

All of these are valid upgrades. Just keep publishing to /mouse/cmd_vel
and subscribing to /mouse/scan.
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# ---------------------------------------------------------------------------
# Tunable parameters — feel free to adjust these
# ---------------------------------------------------------------------------

# How fast to drive straight when the path is clear (cell-units / sec)
DRIVE_SPEED = 0.8

# How fast to spin in-place when turning (radians / sec)
# Positive = counter-clockwise = turn left
TURN_SPEED_FAST = 1.8   # full right-angle turn in-place
TURN_SPEED_SLOW = 0.6   # gentle heading correction

# Distance thresholds (in cell-units, same scale as msg.ranges)
FRONT_CLEAR = 0.55      # front must exceed this to keep driving forward
SIDE_TARGET = 0.35      # desired gap to the left wall
SIDE_TOLERANCE = 0.08   # ± band around SIDE_TARGET before we correct

# Proportional gain for the wall-follow correction turn
WALL_FOLLOW_KP = 1.2


class WallFollowerNode(Node):
    """
    A simple reactive left-hand wall-follower for the micromouse maze.

    ALGORITHM OVERVIEW
    ------------------
    Each time a new scan arrives the callback classifies the situation into
    one of four states and emits an appropriate (linear, angular) command:

    State 1 — BLOCKED: front wall too close
        → spin right (clockwise, negative angular.z) in-place.
          The right spin preference means we naturally hug the LEFT wall
          when navigating corridors: we spin right to face a new corridor,
          drive into it, and keep doing so.

    State 2 — FREE_TURN: front is clear AND left wall is gone (open junction)
        → turn left to try to hug the left wall again. This is the
          "left-hand rule": at every open junction, prefer the left turn.

    State 3 — WALL_FOLLOW: front is clear AND left wall is present but offset
        → drive forward while applying a proportional correction to the
          heading to maintain SIDE_TARGET distance from the left wall.

    State 4 — (fallback) STRAIGHT: all else — just drive forward.

    This is enough to solve any simply-connected maze (one with no loops)
    when you enter from the outside. It does NOT guarantee finding the
    optimal path in a maze with loops — that needs flood-fill or A*.
    """

    def __init__(self):
        super().__init__("wall_follower_solver")

        self.cmd_pub = self.create_publisher(Twist, "/mouse/cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/mouse/scan", self._on_scan, 10
        )

        # Track whether we've ever seen the left wall so we don't
        # immediately free-turn before the first sensor reading.
        self._left_wall_seen = False

        self.get_logger().info(
            "WallFollowerNode ready.\n"
            f"  DRIVE_SPEED    = {DRIVE_SPEED} cell-units/s\n"
            f"  TURN_SPEED_FAST= {TURN_SPEED_FAST} rad/s\n"
            f"  FRONT_CLEAR    = {FRONT_CLEAR} cell-units\n"
            f"  SIDE_TARGET    = {SIDE_TARGET} cell-units\n"
            "Waiting for /mouse/scan messages..."
        )

    # ------------------------------------------------------------------
    # Main control callback
    # ------------------------------------------------------------------
    def _on_scan(self, msg: LaserScan) -> None:
        """Called at ~20 Hz by the sim engine's scan publisher."""
        d_left  = msg.ranges[0]   # left sensor
        d_front = msg.ranges[1]   # front sensor
        d_right = msg.ranges[2]   # right sensor

        # Log sensors at a low rate so the terminal isn't flooded.
        # Remove the modulo guard if you want every scan logged for debugging.
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
        if self._log_counter % 20 == 0:
            self.get_logger().info(
                f"L={d_left:.3f}  F={d_front:.3f}  R={d_right:.3f}"
            )

        linear_x = 0.0
        angular_z = 0.0

        # --- Determine left wall presence ---
        left_wall_present = d_left < (msg.range_max * 0.85)
        if left_wall_present:
            self._left_wall_seen = True

        # === STATE 1: BLOCKED — something directly ahead ===
        if d_front < FRONT_CLEAR:
            # Prefer right spin so left-hand rule drives us left at junctions.
            # If left is also blocked (corner), spin right faster.
            if d_left < FRONT_CLEAR and d_right > FRONT_CLEAR:
                # Dead-end with left blocked → turn right
                angular_z = -TURN_SPEED_FAST
            elif d_right < FRONT_CLEAR and d_left > FRONT_CLEAR:
                # Dead-end with right blocked → turn left
                angular_z = +TURN_SPEED_FAST
            else:
                # General frontal block: spin right (left-hand rule)
                angular_z = -TURN_SPEED_FAST
            linear_x = 0.0

        # === STATE 2: FREE_TURN — front clear, left wall gone (open junction) ===
        elif not left_wall_present and self._left_wall_seen:
            # Left-hand rule: turn left to follow the vanished wall around
            # the corner into the opening on our left.
            # We do a gentle forward arc rather than a spin so the mouse
            # doesn't overshoot the junction and miss the left passage.
            linear_x  = DRIVE_SPEED * 0.4
            angular_z = TURN_SPEED_SLOW * 2.0

        # === STATE 3: WALL_FOLLOW — front clear, left wall present ===
        elif left_wall_present:
            # Proportional controller: if we're too close to the left wall,
            # steer right (negative z); if too far, steer left (positive z).
            wall_error = SIDE_TARGET - d_left   # +err = too far, –err = too close
            if abs(wall_error) < SIDE_TOLERANCE:
                angular_z = 0.0
            else:
                angular_z = WALL_FOLLOW_KP * wall_error
                # Clamp correction to avoid spinning in place while following
                angular_z = max(-TURN_SPEED_SLOW, min(TURN_SPEED_SLOW, angular_z))
            linear_x = DRIVE_SPEED

        # === STATE 4: STRAIGHT OPEN — no walls visible anywhere ===
        else:
            linear_x  = DRIVE_SPEED
            angular_z = 0.0

        self._publish_cmd(linear_x, angular_z)

    # ------------------------------------------------------------------
    def _publish_cmd(self, linear_x: float, angular_z: float) -> None:
        """Package and send a Twist command to the simulator."""
        msg = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_pub.publish(msg)


# ---------------------------------------------------------------------------
# ---- MODIFY BELOW THIS LINE TO IMPLEMENT YOUR OWN ALGORITHM ----
#
# Tips for a better solver:
#
# 1. BUILD A MAP — keep a 16x16 boolean grid and mark cells as visited
#    when the mouse enters them. Derive wall presence from sensor readings
#    at cell centers (when d_front < ~0.6, there's a wall ahead of you).
#
# 2. FLOOD-FILL — once you have a map, compute the Manhattan-distance
#    flood-fill from every cell to the goal and always move toward the
#    cell with the smallest flood value.
#
# 3. ODOMETRY — use a second subscriber to track position without needing
#    ground-truth from the sim. Integrate cmd_vel over time:
#       x += v*cos(theta)*dt,  y += v*sin(theta)*dt,  theta += omega*dt
#
# 4. STATE MACHINE — replace the if/elif chain above with an explicit
#    state machine class (EXPLORE, RETURN, SPEED_RUN states) so the logic
#    stays readable as it grows.
# ---------------------------------------------------------------------------


def main() -> None:
    rclpy.init(args=None)
    node = WallFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Solver shutting down.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
