# Micromouse Simulator — ARC Induction Task

A fully containerised, browser-accessible Micromouse simulation framework built on **ROS 2 Humble** and **Pygame**, streamed to any browser via **noVNC** — no native ROS installation, no X11 forwarding, no host-side display setup required.

---

## Quick Start (3 commands)

```bash
git clone <this-repo>
cd micromouse_sim
docker-compose up --build
```

Open **http://localhost:8080/vnc.html** in any browser. You will see the live Pygame simulation window running inside a sandboxed Linux desktop.

> **Apple Silicon / Raspberry Pi users:** prefix with `DOCKER_DEFAULT_PLATFORM=linux/arm64 docker-compose up --build`

---

## Project Structure

```
micromouse_sim/
├── Dockerfile              # ROS 2 Humble + Pygame + Xvfb/noVNC image
├── docker-compose.yml      # Single-service compose with bind mount + port 8080
├── entrypoint.sh           # Boots virtual display stack, then launches sim
│
├── simulator/
│   ├── maze_layouts.py     # 33×33 NumPy maze matrix (encodes 16×16 cell maze)
│   └── sim_engine.py       # Core engine: Pygame loop + ROS 2 node + physics
│
└── student_agent/
    └── solver.py           # YOUR file: wall-follower baseline you modify
```

---

## Architecture Deep-Dive

### Why Docker + noVNC?

A ROS 2 + Pygame stack normally requires:
- A working ROS 2 Humble installation on the host
- A connected display (or `$DISPLAY` forwarding, which breaks on Windows/macOS)
- Matching Python/library versions

This framework eliminates all of that. The container runs a **headless virtual framebuffer** (Xvfb) that Pygame draws into. `x11vnc` captures that framebuffer and streams it over VNC. `noVNC` proxies the VNC stream into an HTML5 canvas served over HTTP. You only need Docker and a browser.

```
Host browser (any OS)
        │  HTTP :8080
        ▼
   [noVNC HTML5 client]  (inside container)
        │  WebSocket → TCP proxy (websockify)
        ▼
   [x11vnc]              (inside container, VNC :5900)
        │  reads
        ▼
   [Xvfb :1]             (virtual 800×800×16 framebuffer)
        │  DISPLAY=:1
        ▼
   [Pygame window]        (inside sim_engine.py)
```

### The ROS 2 Control Bridge

The simulation engine (`sim_engine.py`) runs **one ROS 2 node** (`micromouse_sim_node`) inside the Pygame process. Your solver (`solver.py`) runs as a **completely separate process** with its own node. They communicate only over ROS 2 topics — the standard pattern for real robot development.

```
┌──────────────────────────────────┐     ┌───────────────────────────┐
│         sim_engine.py            │     │      solver.py            │
│                                  │     │                           │
│  VirtualMouse (physics)          │     │  WallFollowerNode         │
│       │                          │     │       │                   │
│  MicromouseSimNode               │     │       │                   │
│    publishes /mouse/scan ────────┼────►│  scan subscriber          │
│    subscribes /mouse/cmd_vel ◄───┼─────│  cmd_vel publisher        │
└──────────────────────────────────┘     └───────────────────────────┘
           (one process, foreground)          (separate terminal / process)
```

### Coordinate Systems

Three systems in play — read `simulator/sim_engine.py`'s module docstring for the full explanation. Summary:

| System | Origin | +Y direction | Used for |
|--------|--------|-------------|----------|
| **Grid space** | top-left of array | downward (numpy default) | maze data only |
| **World space** | bottom-left of maze | upward (+Y = forward) | all physics/kinematics |
| **Screen space** | top-left pixel | downward (Pygame default) | rendering only |

All conversions happen in `world_to_screen()` and `world_to_grid()` — nowhere else.

---

## Running Your Solver

While the simulator is running (`docker-compose up`), open a **second terminal** and exec into the same container:

```bash
docker exec -it micromouse_simulator bash
python3 student_agent/solver.py
```

You should immediately see the blue triangular mouse start moving in the browser window.

To reset the mouse to the start position, press **`R`** in the noVNC window (click the canvas first to give it keyboard focus).

---

## The Sensor/Actuator Interface

### Reading sensors — `/mouse/scan` (sensor_msgs/LaserScan)

Published at **20 Hz**. Only `ranges` matters:

```python
d_left  = msg.ranges[0]   # left ray,  +90° from heading
d_front = msg.ranges[1]   # front ray,  0° from heading
d_right = msg.ranges[2]   # right ray, -90° from heading

# All in "cell-units" (1.0 ≈ one maze cell ≈ 18 cm IRL)
# Maximum measurable range: msg.range_max = 4.0
```

### Sending commands — `/mouse/cmd_vel` (geometry_msgs/Twist)

```python
from geometry_msgs.msg import Twist

cmd = Twist()
cmd.linear.x  = 0.8   # forward speed, cell-units/sec (+forward, -reverse)
cmd.angular.z = 1.0   # turn rate, rad/sec (+left/CCW, -right/CW)
publisher.publish(cmd)
```

If no command arrives for **0.5 seconds**, the mouse automatically stops (safety timeout).

---

## The Maze

- **16 × 16 cell** competition layout, encoded as a **33 × 33** grid (`simulator/maze_layouts.py`)
- Start position: cell **(0, 0)** — bottom-left corner, world coordinate `(1.5, 1.5)`
- Goal: central **2 × 2** cell block (cells (7,7), (7,8), (8,7), (8,8) in 0-indexed row/col)
- All 256 cells are reachable from the start (verified by BFS during build)
- Shortest path to goal: **32 moves** — not trivial

### Maze encoding key

```
1  = solid wall / column peg    (drawn as crimson red)
0  = open driving corridor      (drawn as dark floor)
```

---

## Upgrading the Solver

The built-in `WallFollowerNode` is a reactive left-hand follower. Here are progressively harder challenges:

### Level 1 — Parameter Tuning (change the constants at the top of `solver.py`)
Adjust `DRIVE_SPEED`, `SIDE_TARGET`, `FRONT_CLEAR` and observe how the mouse behaviour changes.

### Level 2 — Right-Hand Follower
Swap the spin preference from right to left in `STATE 1` and the free-turn direction in `STATE 2`. Does it still solve the maze? Why might one hand-rule fail in mazes the other doesn't?

### Level 3 — Build a Map
Track which cells have been visited. At each cell center (when `d_front > 0.6`, you're not at a wall), record which directions have open passages. Print the accumulated map to the terminal.

### Level 4 — Flood-Fill Solver
Implement the classic micromouse flood-fill: compute the Manhattan-distance from every cell to the goal, always move to the adjacent cell with the smallest value, and update the map (and recompute distances) when you discover a wall that wasn't predicted.

### Level 5 — Speed Run
After a mapping pass (any algorithm), compute the shortest known path and replay it as fast as possible using velocity ramps (accelerate, cruise, brake).

---

## Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `DISPLAY` | `:1` | X11 display that Pygame and Xvfb share |
| `PYTHONPATH` | `/workspace` | lets `simulator/` and `student_agent/` import each other |
| `ROS_DOMAIN_ID` | `42` | isolates ROS 2 DDS traffic from other students on the same LAN |
| `SDL_VIDEODRIVER` | `x11` | forces Pygame to use X11 (not framebuffer or Wayland) |

---

## Keyboard Shortcuts (click the noVNC canvas first)

| Key | Action |
|-----|--------|
| `R` | Reset mouse to start position |
| `Esc` | Quit the simulator (container will restart per `restart: unless-stopped`) |

---

## Troubleshooting

**Black screen in browser:**
Wait 5–10 seconds after `docker-compose up` for all background services to start. If it stays black, check `docker logs micromouse_simulator` for startup errors.

**"Cannot connect to display :1":**
Xvfb didn't start cleanly. Run `docker-compose down && docker-compose up` to restart cleanly.

**Solver can't connect / no motion:**
Make sure `ROS_DOMAIN_ID=42` is set in the terminal where you run `solver.py`:
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
python3 student_agent/solver.py
```

**Mouse stutters or teleports:**
The sim is CPU-bound on a slow host. Reduce `TARGET_FPS` in `sim_engine.py` from 60 to 30.

**ARM64 / Apple Silicon build fails:**
```bash
DOCKER_DEFAULT_PLATFORM=linux/arm64 docker-compose up --build
```

---

## File Edit Workflow (Live Reload)

The project folder is bind-mounted into the container at `/workspace`. This means:
- Edit `student_agent/solver.py` in any editor on your host machine
- The changes are immediately visible inside the container
- Just restart the solver process inside the container — no image rebuild needed

Only changes to `Dockerfile`, `entrypoint.sh`, or system-level dependencies require a rebuild.

---

*Built for the Automation & Robotics Club (ARC) freshmen induction, BITS Pilani.*
