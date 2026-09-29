"""
NEXUS control room: FastAPI server for the live fleet view and layout editor.

    python -m nexus_gui            # http://127.0.0.1:8000

The simulation runs server-side in a background task at real time x speed and
streams state over /ws. Layouts live in layouts/*.json (see nexus/layout.py).
"""

from __future__ import annotations

import asyncio
import json
import os
import re
import threading
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles

from nexus import benchmark
from nexus.catalog import catalog
from nexus.grid_simulation import GridSimulation
from nexus.report import build_report, missions_csv, robots_csv
from nexus.layout import Layout, LayoutError, STATION_TYPES, STRATEGIES, load_layout, normalize, save_layout

ROOT = Path(__file__).resolve().parent.parent
LAYOUT_DIR = ROOT / "layouts"
BENCHMARK_FILE = ROOT / "data" / "benchmark" / "latest.json"
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
        # Coordination strategy chosen in the GUI (None: the layout's own).
        self.strategy: Optional[str] = None
        # A benchmark scenario replayed live: (scenario, seed, tasks).
        self.watch: Optional[Dict[str, Any]] = None

    # -- layout ----------------------------------------------------------

    def _with_strategy(self, layout_dict: Dict[str, Any]) -> Layout:
        if self.strategy:
            layout_dict = {**layout_dict, "simulation": {**layout_dict.get("simulation", {}), "strategy": self.strategy}}
        return Layout(layout_dict)

    def load(self, name: str):
        layout = load_layout(layout_path(name))
        self.layout_name = name
        self.watch = None
        self.sim = GridSimulation(self._with_strategy(layout.to_dict()))
        self.running = False
        self.sent_events = 0

    def load_benchmark(self, scenario: str, seed: int = 1):
        """Replay one benchmark run live: same layout and orders as the benchmark."""
        if scenario not in benchmark.SCENARIOS:
            raise ValueError(f"Unknown benchmark scenario '{scenario}'")
        layout_dict, tasks = benchmark.build_watch(scenario, self.strategy or "nexus", seed)
        self.layout_name = f"benchmark-{scenario}"
        self.watch = {"scenario": scenario, "seed": seed, "tasks": tasks, "done": False,
                      "title": benchmark.SCENARIOS[scenario]["title"]}
        self.sim = GridSimulation(Layout(layout_dict))
        for pickup, dropoff in tasks:
            self.sim.fleet.create(pickup, dropoff, "normal", now=0.0)
        self.sim._event("system", f"Benchmark replay: {len(tasks) or len(self.sim.agents)} "
                        f"{'orders' if tasks else 'robot trips'}, seed {seed}, "
                        f"strategy {'NEXUS' if self.sim.strategy == 'nexus' else 'stop-and-wait'}")
        self.running = False
        self.sent_events = 0

    def reload(self):
        """Start the current layout (or benchmark replay) over."""
        if self.watch:
            self.load_benchmark(self.watch["scenario"], self.watch["seed"])
        else:
            self.sim.reset()
            self.sent_events = 0

    def check_watch_done(self):
        """Announce when a benchmark replay has delivered every order."""
        w = self.watch
        if not w or w["done"] or not w["tasks"]:
            return
        if self.sim.fleet.counters["completed"] >= len(w["tasks"]):
            w["done"] = True
            self.running = False
            self.sim._event("system", f"All {len(w['tasks'])} orders delivered in {self.sim.time:.1f} s "
                            f"({'NEXUS' if self.sim.strategy == 'nexus' else 'stop-and-wait'})")

    def layout_message(self) -> Dict[str, Any]:
        layout = self.sim.layout
        return {
            "type": "layout",
            "name": self.layout_name,
            "benchmark": None if not self.watch else {k: self.watch[k] for k in ("scenario", "seed", "title")},
            "layout": layout.to_dict(),
            "issues": layout.validate(),
        }

    def state_message(self, include_all_events: bool = False) -> Dict[str, Any]:
        # Events carry a sequence number; the simulation trims its log, so
        # "new since last broadcast" must not be a list position.
        events = self.sim.events if include_all_events else [e for e in self.sim.events if e["seq"] > self.sent_events]
        return {
            "type": "state",
            "running": self.running,
            "speed": self.speed,
            "strategy": self.sim.strategy,
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
        self.sent_events = self.sim.event_seq
        await self.broadcast(message)

    # -- commands --------------------------------------------------------

    async def handle(self, command: Dict[str, Any]):
        cmd = command.get("cmd")
        async with self.lock:
            if cmd == "play":
                if self.sim.status == "COMPLETED" or (self.watch and self.watch["done"]):
                    self.reload()
                    await self.broadcast_state(include_all_events=True)
                self.running = self.sim.status != "INVALID"
            elif cmd == "pause":
                self.running = False
            elif cmd == "step":
                self.running = False
                self.sim.step()
                self.check_watch_done()
            elif cmd == "reset":
                self.running = False
                self.reload()
                await self.broadcast_state(include_all_events=True)
                return
            elif cmd == "strategy":
                value = str(command.get("value"))
                if value not in STRATEGIES:
                    raise ValueError(f"Unknown strategy '{value}'")
                self.strategy = value
                if self.watch:
                    self.load_benchmark(self.watch["scenario"], self.watch["seed"])
                else:
                    self.load(self.layout_name)
                await self.broadcast(self.layout_message())
                await self.broadcast_state(include_all_events=True)
                return
            elif cmd == "benchmark.watch":
                if command.get("strategy") in STRATEGIES:
                    self.strategy = command["strategy"]
                self.load_benchmark(str(command.get("scenario")), int(command.get("seed", 1)))
                await self.broadcast(self.layout_message())
                await self.broadcast_state(include_all_events=True)
                return
            elif cmd == "speed":
                value = float(command.get("value", 1.0))
                self.speed = min(SPEEDS, key=lambda s: abs(s - value))
            elif cmd == "estop":
                self.sim.set_estop(bool(command.get("value")))
            elif cmd == "robot.hold":
                self.sim.set_hold(str(command.get("id")), bool(command.get("value")))
            elif cmd == "robot.charge":
                self.sim.fleet.charge_now(str(command.get("id")))
            elif cmd == "alert.ack":
                self.sim.alerts.acknowledge(command.get("id"))
            elif cmd == "mission.create":
                self.sim.fleet.create(str(command.get("pickup")), str(command.get("dropoff")),
                                      str(command.get("priority", "normal")))
            elif cmd == "mission.cancel":
                self.sim.fleet.cancel(str(command.get("id")))
            elif cmd == "fleet.generator":
                self.sim.fleet.cfg["generator"] = bool(command.get("value"))
                self.sim._event("system", f"Order generator {'on' if self.sim.fleet.cfg['generator'] else 'off'}")
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
                    self.check_watch_done()
                    if not self.running or self.sim.status in {"COMPLETED", "INVALID"}:
                        self.running = False
                        backlog = 0.0
                        break
                if steps == MAX_STEPS_PER_TICK:
                    backlog = 0.0  # can't keep up: drop time rather than spiral
                if steps:
                    await self.broadcast_state()


controller = Controller()


class BenchmarkJob:
    """One benchmark run at a time, in a background thread (it uses its own
    worker processes, so the live view keeps running)."""

    def __init__(self):
        self.running = False
        self.done = 0
        self.total = 0
        self.mode = None
        self.started = None
        self.error: Optional[str] = None

    def status(self) -> Dict[str, Any]:
        return {"running": self.running, "done": self.done, "total": self.total, "mode": self.mode,
                "elapsed": round(time.time() - self.started, 1) if self.started else None, "error": self.error}

    def start(self, mode: str):
        if self.running:
            raise HTTPException(409, "A benchmark is already running")
        quick = mode == "quick"
        names = [n for n in benchmark.SCENARIOS if not (quick and benchmark.SCENARIOS[n].get("heavy"))]
        self.running, self.done, self.total, self.mode, self.error = True, 0, 0, mode, None
        self.started = time.time()

        def progress(done, total):
            self.done, self.total = done, total

        def work():
            try:
                jobs = max(1, (os.cpu_count() or 2) - 1)
                result = benchmark.run_suite(names, seeds=1 if quick else 3, jobs=jobs, progress=progress)
                result["mode"] = mode
                BENCHMARK_FILE.parent.mkdir(parents=True, exist_ok=True)
                BENCHMARK_FILE.write_text(json.dumps(result, indent=2))
            except Exception as error:  # surfaced on the Benchmark page
                self.error = str(error)
            finally:
                self.running = False

        threading.Thread(target=work, daemon=True).start()


bench_job = BenchmarkJob()


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


@app.middleware("http")
async def revalidate_frontend(request, call_next):
    # Without an explicit policy browsers may reuse a cached page/script
    # after an update, so the GUI shows stale features. "no-cache" makes
    # the browser revalidate every time (unchanged files answer 304).
    response = await call_next(request)
    if request.url.path == "/" or request.url.path.startswith("/static/"):
        response.headers["Cache-Control"] = "no-cache"
    return response


# ----------------------------------------------------------------------
# REST
# ----------------------------------------------------------------------

@app.get("/api/benchmark")
def api_benchmark():
    """Latest saved results, the scenario catalogue and the job status."""
    result = None
    if BENCHMARK_FILE.exists():
        try:
            result = json.loads(BENCHMARK_FILE.read_text(encoding="utf-8"))
            if not result.get("generated_at"):  # files written by older versions
                result["generated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(BENCHMARK_FILE.stat().st_mtime))
        except (OSError, json.JSONDecodeError):
            result = None
    scenarios = [{"name": n, "title": s["title"], "heavy": bool(s.get("heavy")), "seeded": s["seeded"]}
                 for n, s in benchmark.SCENARIOS.items()]
    return {"result": result, "scenarios": scenarios, "job": bench_job.status()}


@app.post("/api/benchmark/run")
def api_benchmark_run(payload: Dict[str, Any]):
    mode = payload.get("mode", "quick")
    if mode not in ("quick", "full"):
        raise HTTPException(400, "mode must be 'quick' or 'full'")
    bench_job.start(mode)
    return bench_job.status()


@app.get("/api/layouts")
def api_layouts():
    return {"layouts": list_layouts(), "active": controller.layout_name, "station_types": list(STATION_TYPES)}


@app.get("/api/catalog")
def api_catalog():
    """Object types (racks, equipment, areas, safety zones) and station types."""
    return {**catalog(), "station_types": list(STATION_TYPES)}


@app.post("/api/estop")
async def api_estop(payload: Dict[str, Any]):
    """Engage ({"engaged": true}) or release ({"engaged": false}) the fleet-wide E-stop."""
    if controller.sim is None:
        raise HTTPException(409, "No layout loaded")
    async with controller.lock:
        controller.sim.set_estop(bool(payload.get("engaged")))
    await controller.broadcast_state()
    return {"estop": controller.sim.estop}


@app.get("/api/alerts")
def api_alerts():
    if controller.sim is None:
        return {"active": [], "recent": [], "counts": {}, "totals": {}}
    return controller.sim.alerts.snapshot()


@app.post("/api/alerts/ack")
async def api_ack(payload: Dict[str, Any]):
    """Acknowledge one alert ({"id": "A-0003"}) or all active alerts ({})."""
    if controller.sim is None:
        raise HTTPException(409, "No layout loaded")
    async with controller.lock:
        try:
            count = controller.sim.alerts.acknowledge(payload.get("id"))
        except ValueError as error:
            raise HTTPException(404, str(error))
    await controller.broadcast_state()
    return {"acknowledged": count}


@app.get("/api/report")
def api_report():
    if controller.sim is None:
        raise HTTPException(409, "No layout loaded")
    return build_report(controller.sim)


def _csv_response(text: str, name: str) -> PlainTextResponse:
    stamp = f"{controller.layout_name or 'layout'}_{int(controller.sim.world.time)}s"
    return PlainTextResponse(text, media_type="text/csv",
                             headers={"Content-Disposition": f'attachment; filename="nexus_{name}_{stamp}.csv"'})


@app.get("/api/report/missions.csv")
def api_missions_csv():
    if controller.sim is None:
        raise HTTPException(409, "No layout loaded")
    return _csv_response(missions_csv(controller.sim), "missions")


@app.get("/api/report/robots.csv")
def api_robots_csv():
    if controller.sim is None:
        raise HTTPException(409, "No layout loaded")
    return _csv_response(robots_csv(controller.sim), "robots")


@app.post("/api/missions")
async def api_create_mission(payload: Dict[str, Any]):
    """Create a transport order in the running simulation (e.g. from a WMS/MES).
    Body: {"pickup": station id, "dropoff": station id, "priority": "high|normal|low"}"""
    if controller.sim is None:
        raise HTTPException(409, "No layout loaded")
    async with controller.lock:
        try:
            mission = controller.sim.fleet.create(str(payload.get("pickup")), str(payload.get("dropoff")),
                                                  str(payload.get("priority", "normal")), name=payload.get("name"))
        except ValueError as error:
            raise HTTPException(422, str(error))
    await controller.broadcast_state()
    return mission.to_dict()


@app.delete("/api/missions/{mission_id}")
async def api_cancel_mission(mission_id: str):
    if controller.sim is None:
        raise HTTPException(409, "No layout loaded")
    async with controller.lock:
        try:
            mission = controller.sim.fleet.cancel(mission_id)
        except ValueError as error:
            raise HTTPException(422, str(error))
    await controller.broadcast_state()
    return mission.to_dict()


@app.get("/api/missions/all")
def api_missions_all():
    """Every order of this run plus per-flow statistics (Missions page)."""
    if controller.sim is None or not controller.sim.fleet.active:
        return {"missions": [], "flows": [], "generator": False, "time": 0.0}
    fleet = controller.sim.fleet
    return {"missions": fleet.all_missions(), "flows": fleet.flow_stats(),
            "generator": bool(fleet.cfg.get("generator")), "time": controller.sim.world.time}


@app.get("/api/missions")
def api_missions():
    if controller.sim is None or not controller.sim.fleet.active:
        return {"queued": [], "active": [], "recent": [], "metrics": {}}
    return controller.sim.fleet.snapshot()


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
        return {"issues": [{"severity": "error", "message": str(error), "target": "", "cells": []}], "routes": {}}
    issues, routes = layout.analyse()
    return {"issues": issues, "routes": routes, "layout": layout.to_dict()}


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
