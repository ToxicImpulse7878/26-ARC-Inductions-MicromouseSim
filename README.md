# Micromouse Simulator — ARC Induction Task

A  containerised, browser-accessible Micromouse simulation framework built on **ROS 2 Humble** and **Pygame**, streamed to any browser via **noVNC**.

## Progress as of now:
Step 1  ← we are here
        sim_engine.py draws a red square
        confirms: Xvfb → x11vnc → noVNC → Pygame all work
        so basically, all the pipelines or connections are done.

Step 2
        add maze_layouts.py rendering into sim_engine.py
        confirms: coordinate system, wall drawing, goal zone
        this is the coolest step, I think we should hardcode the thing first and then keep it as a backup and then look into backtracking maze generation and all that..      

Step 3
        add VirtualMouse into sim_engine.py
        (position, heading, movement, collision)
        confirms: physics loop, keyboard-driven testing

Step 4
        add SimNode into sim_engine.py
        (the ROS node, scan publisher, cmd_vel subscriber)
        confirms: rclpy.spin_once() inside pygame loop works

Step 5
        solver.py subscribes to /mouse/scan
        publishes to /mouse/cmd_vel
        confirms: full end-to-end ROS bridge works

Step 6
        replace hardcoded maze with .maz loader
        + recursive backtracker fallback
---
## install
whatever, you can figure out how to clone and make the container... the dockerfie is in the repo only... it should have a native ros2 humble installed. and all python things like pygame and all the novnc things already installed..              <br>

```bash
git clone "whatever-the-link-is"
cd micromouse_sim
docker-compose up --build
```

Open **http://localhost:8080/vnc.html** in any browser. You will see the live Pygame simulation of a box hopefully

> **Apple Silicon / Raspberry Pi users:** prefix with `DOCKER_DEFAULT_PLATFORM=linux/arm64 docker-compose up --build`

---

Read This AI jargon about why x11 forwarding is bad and novnc is good.(read it if u want)

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

---
## Running Your Solver

While the simulator is running (`docker-compose up`), open a **second terminal** and exec into the same container:

```bash
docker exec -it micromouse_simulator bash
python3 student_agent/solver.py
```
the mouse should reflect the movement instructions given by the solver

---

## Plan for the ROS topics: 

### Sensors:  `/mouse/scan` (sensor_msgs/LaserScan)

Published at **20 Hz**. Only `ranges` matters:

```python
d_left  = msg.ranges[0]   # left ray,  
d_front = msg.ranges[1]   # front ray,  
d_right = msg.ranges[2]   # right ray, 


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
## Environment Variables

| Variable | Default | Purpose |
|----------|---------|---------|
| `DISPLAY` | `:1` | X11 display that Pygame and Xvfb share |
| `PYTHONPATH` | `/workspace` | lets `simulator/` and `student_agent/` import each other |
| `ROS_DOMAIN_ID` | `42` | isolates ROS 2 DDS traffic from other students on the same LAN |
| `SDL_VIDEODRIVER` | `x11` | forces Pygame to use X11 (not framebuffer or Wayland) |

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


