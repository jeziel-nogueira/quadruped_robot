import os
import json
import time
import logging
import threading
import asyncio
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles

logger = logging.getLogger(__name__)

CONFIG_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "config.json"))

# Disabling default access logs from uvicorn to prevent spamming console
uvicorn_logger = logging.getLogger("uvicorn.access")
uvicorn_logger.disabled = True

class TelemetryServer:
    def __init__(self, host="0.0.0.0", port=8000):
        self.host = host
        self.port = port
        
        # Thread-safe state storage
        self.lock = threading.Lock()
        self.telemetry_data = {}
        
        # Queues for inputs/configurations to be read by the main robot loop
        self.command_queue = {
            "gait_enabled": True,
            "steering": 0.0,
        }
        self.config_updates = {}
        
        # Active WebSockets clients
        self.active_connections = set()
        
        self.loop = None
        
        # Hardware reference (set externally by main.py after hardware init)
        self.hardware = None
        
        # Create FastAPI app
        self.app = FastAPI(title="Quadruped Telemetry Server")
        self.setup_routes()

    def setup_routes(self):
        # WebSocket route for real-time telemetry and controls
        @self.app.websocket("/ws")
        async def websocket_endpoint(websocket: WebSocket):
            await websocket.accept()
            self.active_connections.add(websocket)
            logger.info(f"[Telemetry] Client connected. Active clients: {len(self.active_connections)}")
            try:
                while True:
                    # Listen for incoming command JSONs from the frontend
                    data = await websocket.receive_text()
                    try:
                        msg = json.loads(data)
                        self.handle_incoming_message(msg)
                    except json.JSONDecodeError:
                        logger.error(f"[Telemetry] Invalid JSON received: {data}")
            except WebSocketDisconnect:
                self.active_connections.discard(websocket)
                logger.info(f"[Telemetry] Client disconnected. Active clients: {len(self.active_connections)}")
            except Exception as e:
                logger.error(f"[Telemetry] WebSocket error: {e}")
                self.active_connections.discard(websocket)

        # ── Servo Calibration REST API ──────────────────────────────────

        @self.app.get("/api/servos")
        async def get_servo_configs():
            """Return the current servo calibration configs."""
            try:
                with open(CONFIG_PATH, "r") as f:
                    data = json.load(f)
                return JSONResponse(content=data.get("servos", []))
            except Exception as e:
                return JSONResponse(content={"error": str(e)}, status_code=500)

        @self.app.post("/api/servo/move")
        async def move_servo(body: dict):
            """Move a single servo. Body: { index: 0-11, angle_deg: 0-180 }"""
            idx = body.get("index", 0)
            angle = body.get("angle_deg", 90)
            if self.hardware:
                try:
                    self.hardware.servo.set_angle_deg(idx, angle)
                    return JSONResponse(content={"ok": True, "index": idx, "angle_deg": angle})
                except Exception as e:
                    return JSONResponse(content={"error": str(e)}, status_code=500)
            return JSONResponse(content={"error": "Hardware not connected"}, status_code=503)

        @self.app.post("/api/servo/center-all")
        async def center_all_servos():
            """Move all servos to center (90°)."""
            if self.hardware:
                try:
                    self.hardware.servo.center_all()
                    return JSONResponse(content={"ok": True})
                except Exception as e:
                    return JSONResponse(content={"error": str(e)}, status_code=500)
            return JSONResponse(content={"error": "Hardware not connected"}, status_code=503)

        @self.app.post("/api/servo/detach-all")
        async def detach_all_servos():
            """Detach (PWM off) all servos."""
            if self.hardware:
                try:
                    self.hardware.servo.detach_all()
                    return JSONResponse(content={"ok": True})
                except Exception as e:
                    return JSONResponse(content={"error": str(e)}, status_code=500)
            return JSONResponse(content={"error": "Hardware not connected"}, status_code=503)

        @self.app.post("/api/servo/sweep")
        async def sweep_servo(body: dict):
            """Sweep a single servo 0°→180°→0°. Body: { index: 0-11 }"""
            idx = body.get("index", 0)
            if self.hardware:
                try:
                    driver = self.hardware.servo
                    for deg in range(0, 181, 5):
                        driver.set_angle_deg(idx, deg)
                        time.sleep(0.015)
                    for deg in range(180, -1, -5):
                        driver.set_angle_deg(idx, deg)
                        time.sleep(0.015)
                    driver.set_angle_deg(idx, 90)
                    return JSONResponse(content={"ok": True, "index": idx})
                except Exception as e:
                    return JSONResponse(content={"error": str(e)}, status_code=500)
            return JSONResponse(content={"error": "Hardware not connected"}, status_code=503)

        @self.app.post("/api/servo/stand")
        async def stand_pose():
            """Move all servos to a standing pose."""
            if self.hardware:
                try:
                    driver = self.hardware.servo
                    for i, cfg in enumerate(driver.servo_configs):
                        if "hip_yaw" in cfg.name:
                            driver.set_angle_deg(i, 90)
                        elif "hip_pitch" in cfg.name:
                            driver.set_angle_deg(i, 45)
                        elif "knee" in cfg.name:
                            driver.set_angle_deg(i, 135)
                        time.sleep(0.05)
                    return JSONResponse(content={"ok": True})
                except Exception as e:
                    return JSONResponse(content={"error": str(e)}, status_code=500)
            return JSONResponse(content={"error": "Hardware not connected"}, status_code=503)

        @self.app.post("/api/servo/update-config")
        async def update_servo_config(body: dict):
            """Update a single servo's calibration. Body: { index, tick_min, tick_center, tick_max, inverted, trim_deg, limit_min_deg, limit_max_deg }"""
            idx = body.get("index", 0)
            if self.hardware and 0 <= idx < len(self.hardware.servo.servo_configs):
                cfg = self.hardware.servo.servo_configs[idx]
                if "tick_min" in body: cfg.tick_min = int(body["tick_min"])
                if "tick_center" in body: cfg.tick_center = int(body["tick_center"])
                if "tick_max" in body: cfg.tick_max = int(body["tick_max"])
                if "inverted" in body: cfg.inverted = bool(body["inverted"])
                if "trim_deg" in body: cfg.trim_deg = float(body["trim_deg"])
                if "limit_min_deg" in body: cfg.limit_min_deg = float(body["limit_min_deg"])
                if "limit_max_deg" in body: cfg.limit_max_deg = float(body["limit_max_deg"])
                return JSONResponse(content={"ok": True, "servo": cfg.to_dict()})
            return JSONResponse(content={"error": "Invalid index or hardware not connected"}, status_code=400)

        @self.app.post("/api/servo/save")
        async def save_servo_config():
            """Persist current servo calibration to config.json."""
            if self.hardware:
                try:
                    with open(CONFIG_PATH, "r") as f:
                        data = json.load(f)
                    data["servos"] = self.hardware.servo.get_configs_as_dicts()
                    with open(CONFIG_PATH, "w") as f:
                        json.dump(data, f, indent=2)
                    return JSONResponse(content={"ok": True})
                except Exception as e:
                    return JSONResponse(content={"error": str(e)}, status_code=500)
            return JSONResponse(content={"error": "Hardware not connected"}, status_code=503)

        # Mount static frontend dashboard files (MUST be last — catch-all)
        web_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "web"))
        if os.path.exists(web_dir):
            self.app.mount("/", StaticFiles(directory=web_dir, html=True), name="web")
            logger.info(f"[Telemetry] Serving web static dashboard from: {web_dir}")
        else:
            logger.warning(f"[Telemetry] Web static dashboard directory not found at: {web_dir}")

    def handle_incoming_message(self, msg):
        """Processes control parameters and configuration updates sent by frontend dashboard."""
        msg_type = msg.get("type")
        payload = msg.get("payload", {})
        
        with self.lock:
            if msg_type == "command":
                # Control commands: start/stop gait, steering joystick value
                if "gait_enabled" in payload:
                    self.command_queue["gait_enabled"] = bool(payload["gait_enabled"])
                if "steering" in payload:
                    self.command_queue["steering"] = float(payload["steering"])
                    
            elif msg_type == "config":
                # Updates to CPG mathematical parameters or gains
                self.config_updates.update(payload)
                logger.info(f"[Telemetry] Received config updates: {payload}")

    def start(self):
        """Starts the server in a separate background thread so it doesn't block the main control loop."""
        self.thread = threading.Thread(target=self._run_server, daemon=True)
        self.thread.start()
        logger.info(f"[Telemetry] Web telemetry server thread started on http://{self.host}:{self.port}")

    def _run_server(self):
        # Force asyncio event loop creation for the thread
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        config = uvicorn.Config(
            app=self.app, 
            host=self.host, 
            port=self.port, 
            log_level="info", 
            loop="asyncio"
        )
        server = uvicorn.Server(config)
        self.loop.run_until_complete(server.serve())

    def update_telemetry(self, data):
        """
        Thread-safe method called by the main robot loop to update telemetry stats.
        Triggers an asynchronous broadcast to all WebSocket clients.
        """
        with self.lock:
            self.telemetry_data = data
            
        # Get active event loop or run inside loop context
        if self.active_connections and self.loop and self.loop.is_running():
            asyncio.run_coroutine_threadsafe(self._broadcast_telemetry(), self.loop)

    async def _broadcast_telemetry(self):
        """Helper to send the current telemetry_data to all WebSocket connections."""
        with self.lock:
            message = json.dumps({
                "type": "telemetry",
                "payload": self.telemetry_data
            })
            
        # Build copy to prevent concurrent modification errors
        clients = list(self.active_connections)
        for client in clients:
            try:
                await client.send_text(message)
            except Exception as e:
                logger.debug(f"[Telemetry] Failed to send message to client: {e}")
            except Exception:
                # Connection might have died
                pass

    def get_commands(self):
        """Called by the main robot loop to get outstanding steering/gait commands."""
        with self.lock:
            return dict(self.command_queue)

    def get_config_updates(self):
        """Called by the main robot loop to read and clear parameter updates from the UI."""
        with self.lock:
            updates = dict(self.config_updates)
            self.config_updates.clear() # Clear queue after reading
            return updates
