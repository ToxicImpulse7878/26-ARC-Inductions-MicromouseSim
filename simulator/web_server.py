# simulator/web_server.py
import asyncio
import threading
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
import rclpy

from simulator.sim_engine import MicromouseSimNode
from simulator.maze_layouts import active_env

app = FastAPI()

# Global reference to share data between the ROS thread and the WebSocket loop
shared_state = {
    "x": active_env.start_pos[0],
    "y": active_env.start_pos[1],
    "theta": 0.0,
    "sensor_left": 0.0,
    "sensor_front": 0.0,
    "sensor_right": 0.0
}

def run_ros_thread():
    """Background target that keeps the ROS 2 loop spinning."""
    rclpy.init()
    node = MicromouseSimNode()
    
    # We intercept the simulation loop step to copy data to our shared state
    # before publishing to the network
    base_callback = node.sim_loop_callback
    
    def wrapped_callback():
        base_callback()
        # Update the shared dictionary with the latest physics calculations
        shared_state["x"] = node.mouse.x
        shared_state["y"] = node.mouse.y
        shared_state["theta"] = node.mouse.theta
        
        readings = node.mouse.get_sensor_readings()
        shared_state["sensor_left"] = readings["left"]
        shared_state["sensor_front"] = readings["front"]
        shared_state["sensor_right"] = readings["right"]

    # Override the timer callback with our thread-safe state updater
    node.timer.cancel()
    node.timer = node.create_timer(node.dt, wrapped_callback)

    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"ROS Spin Exception: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

@app.on_event("startup")
def startup_event():
    """Spins up the ROS 2 processing loop in a separate native thread."""
    threading.Thread(target=run_ros_thread, daemon=True).start()

@app.get("/")
async def get():
    """Serves the frontend visualization dashboard to the browser."""
    with open("simulator/templates/index.html", "r") as f:
        html_content = f.read()
    return HTMLResponse(content=html_content)

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Streams real-time mouse telemetry data to the connected HTML canvas."""
    await websocket.accept()
    try:
        while True:
            # Package the shared state data along with the static maze layout matrix
            payload = {
                "mouse": {
                    "x": shared_state["x"],
                    "y": shared_state["y"],
                    "theta": shared_state["theta"],
                    "sensors": [
                        shared_state["sensor_left"],
                        shared_state["sensor_front"],
                        shared_state["sensor_right"]
                    ]
                },
                "maze": active_env.matrix.tolist(),
                "goal": list(active_env.goal_pos)
            }
            await websocket.send_json(payload)
            # Sleep for 50ms to match our 20Hz simulation refresh rate
            await asyncio.sleep(0.05)
    except WebSocketDisconnect:
        print("Browser disconnected from telemetry stream.")