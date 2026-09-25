"""
NEXUS control room: FastAPI server for the live fleet view and layout editor.

    python -m nexus_gui            # http://127.0.0.1:8000

The simulation runs server-side in a background task at real time x speed and
streams state over /ws. Layouts live in layouts/*.json (see nexus/layout.py).
"""

from __future__ import annotations

import asyncio
import json
import re
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from nexus.grid_simulation import GridSimulation
from nexus.layout import Layout, LayoutError, STATION_TYPES, load_layout, normalize, save_layout

ROOT = Path(__file__).resolve().parent.parent
LAYOUT_DIR = ROOT / "layouts"
STATIC_DIR = Path(__file__).resolve().parent / "static"

TICK_SECONDS = 0.05
MAX_STEPS_PER_TICK = 40
SPEEDS = (0.25, 0.5, 1.0, 2.0, 4.0, 8.0)
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")


def layout_path(name: str) -> Path:
    if not NAME_RE.match(name):
        raise HTTPException(400, "Layout names may only contain letters, digits, '-' and '_'")
    return LAYOUT_DIR / f"{name}.json"


def list_layouts() -> List[Dict[str, Any]]:
    items = []
    for path in sorted(LAYOUT_DIR.glob("*.json")):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        items.append({
            "name": path.stem,
            "title": data.get("name", path.stem),
            "width": data.get("width"),
            "height": data.get("height"),
            "robots": len(data.get("robots", [])),
            "stations": len(data.get("stations", [])),
        })
    return items


class Controller:
    """Owns the running simulation and the connected viewers."""

    def __init__(self):
        self.clients: Set[WebSocket] = set()
        self.running = False
        self.speed = 1.0
        self.layout_name: Optional[str] = None
        self.sim: Optional[GridSimulation] = None
        self.sent_events = 0
        self.lock = asyncio.Lock()

    # -- layout ----------------------------------------------------------

    def load(self, name: str):
        layout = load_layout(layout_path(name))
        self.layout_name = name
        self.sim = GridSimulation(layout)
        self.running = False
        self.sent_events = 0

    def layout_message(self) -> Dict[str, Any]:
        layout = self.sim.layout
        return {
            "type": "layout",
            "name": self.layout_name,
            "layout": layout.to_dict(),
            "issues": layout.validate(),
        }

    def state_message(self, include_all_events: bool = False) -> Dict[str, Any]:
        events = self.sim.events if include_all_events else self.sim.events[self.sent_events:]
        return {
            "type": "state",
            "running": self.running,
            "speed": self.speed,
            "state": self.sim.snapshot(),
            "events": events[-120:],
            "reset_events": include_all_events,
        }

    # -- broadcast -------------------------------------------------------

    async def send(self, ws: WebSocket, message: Dict[str, Any]):
        try:
            await ws.send_text(json.dumps(message))
        except Exception:
            self.clients.discard(ws)

    async def broadcast(self, message: Dict[str, Any]):
        for ws in list(self.clients):
            await self.send(ws, message)

    async def broadcast_state(self, include_all_events: bool = False):
        message = self.state_message(include_all_events)
        self.sent_events = len(self.sim.events)
        await self.broadcast(message)

    # -- commands --------------------------------------------------------

    async def handle(self, command: Dict[str, Any]):
        cmd = command.get("cmd")
        async with self.lock:
            if cmd == "play":
                if self.sim.status == "COMPLETED":
                    self.sim.reset()
                    self.sent_events = 0
                    await self.broadcast_state(include_all_events=True)
                self.running = self.sim.status != "INVALID"
            elif cmd == "pause":
                self.running = False
            elif cmd == "step":
                self.running = False
                self.sim.step()
            elif cmd == "reset":
                self.running = False
                self.sim.reset()
                self.sent_events = 0
                await self.broadcast_state(include_all_events=True)
                return
            elif cmd == "speed":
                value = float(command.get("value", 1.0))
                self.speed = min(SPEEDS, key=lambda s: abs(s - value))
            elif cmd == "load":
                self.load(str(command.get("name")))
                await self.broadcast(self.layout_message())
                await self.broadcast_state(include_all_events=True)
                return
            await self.broadcast_state()

    # -- loop ------------------------------------------------------------

    async def run(self):
        loop = asyncio.get_running_loop()
        last = loop.time()
        backlog = 0.0
        while True:
            await asyncio.sleep(TICK_SECONDS)
            now = loop.time()
            elapsed, last = now - last, now
            if not self.running or self.sim is None:
                backlog = 0.0
                continue
            async with self.lock:
                backlog += elapsed * self.speed
                steps = 0
                while backlog >= self.sim.time_step and steps < MAX_STEPS_PER_TICK:
                    self.sim.step()
                    backlog -= self.sim.time_step
                    steps += 1
                    if self.sim.status in {"COMPLETED", "INVALID"}:
                        self.running = False
                        backlog = 0.0
                        break
                if steps == MAX_STEPS_PER_TICK:
                    backlog = 0.0  # can't keep up: drop time rather than spiral
                if steps:
                    await self.broadcast_state()


controller = Controller()


@asynccontextmanager
async def lifespan(_app: FastAPI):
    available = list_layouts()
    if available:
        preferred = next((l["name"] for l in available if l["name"] == "fulfillment_center"), available[0]["name"])
        controller.load(preferred)
    task = asyncio.create_task(controller.run())
    yield
    task.cancel()


app = FastAPI(title="NEXUS Control Room", lifespan=lifespan)


# ----------------------------------------------------------------------
# REST
# ----------------------------------------------------------------------

@app.get("/api/layouts")
def api_layouts():
    return {"layouts": list_layouts(), "active": controller.layout_name, "station_types": list(STATION_TYPES)}


@app.get("/api/layouts/{name}")
def api_layout(name: str):
    path = layout_path(name)
    if not path.exists():
        raise HTTPException(404, f"Layout {name} not found")
    layout = load_layout(path)
    return {"name": name, "layout": layout.to_dict(), "issues": layout.validate()}


@app.put("/api/layouts/{name}")
def api_save_layout(name: str, payload: Dict[str, Any]):
    try:
        layout = Layout(payload)
    except (LayoutError, KeyError, TypeError, ValueError) as error:
        raise HTTPException(422, str(error))
    save_layout(layout, layout_path(name))
    return {"name": name, "issues": layout.validate()}


@app.post("/api/validate")
def api_validate(payload: Dict[str, Any]):
    try:
        layout = Layout(payload)
    except (LayoutError, KeyError, TypeError, ValueError) as error:
        return {"issues": [{"severity": "error", "message": str(error), "target": ""}]}
    return {"issues": layout.validate(), "layout": layout.to_dict()}


# ----------------------------------------------------------------------
# Live stream
# ----------------------------------------------------------------------

@app.websocket("/ws")
async def websocket(ws: WebSocket):
    await ws.accept()
    controller.clients.add(ws)
    if controller.sim is not None:
        await controller.send(ws, controller.layout_message())
        await controller.send(ws, controller.state_message(include_all_events=True))
    try:
        while True:
            message = await ws.receive_text()
            try:
                command = json.loads(message)
            except json.JSONDecodeError:
                continue
            try:
                await controller.handle(command)
            except HTTPException as error:
                await controller.send(ws, {"type": "error", "message": error.detail})
            except Exception as error:  # surface, don't kill the socket
                await controller.send(ws, {"type": "error", "message": str(error)})
    except WebSocketDisconnect:
        pass
    finally:
        controller.clients.discard(ws)


# ----------------------------------------------------------------------
# Frontend
# ----------------------------------------------------------------------

@app.get("/")
def index():
    return FileResponse(STATIC_DIR / "index.html")


app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
