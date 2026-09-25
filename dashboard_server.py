from __future__ import annotations

import json
import threading
import time
from typing import Optional

from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from pydantic import BaseModel

from nexus.persistence import save_config
from nexus.simulation_manager import SimulationManager


app = FastAPI(title="NEXUS Fleet Dashboard", version="0.1.0")
manager = SimulationManager()
stream_lock = threading.Lock()
stream_stop = threading.Event()
stream_thread: Optional[threading.Thread] = None


class ConfigPayload(BaseModel):
    config: dict


class SavePayload(BaseModel):
    path: str = "data/simulations/demo.json"


def ensure_stream():
    global stream_thread
    if stream_thread is not None and stream_thread.is_alive():
        return

    stream_stop.clear()

    def _loop():
        while not stream_stop.is_set():
            with stream_lock:
                if manager.status == "RUNNING":
                    manager.step()
            time.sleep(1.0)

    stream_thread = threading.Thread(target=_loop, daemon=True)
    stream_thread.start()


@app.get("/", response_class=HTMLResponse)
def dashboard_page() -> HTMLResponse:
    default_state = {
        "simulation_time": 0.0,
        "status": "READY",
        "warehouse": {
            "name": "Warehouse A",
            "width": 30,
            "height": 20,
            "obstacles": [
                {"type": "rect", "x": 4, "y": 5, "w": 7, "h": 3},
                {"type": "rect", "x": 15, "y": 5, "w": 7, "h": 3},
                {"type": "rect", "x": 4, "y": 12, "w": 7, "h": 3},
                {"type": "rect", "x": 15, "y": 12, "w": 7, "h": 3},
            ],
        },
        "robots": [
            {"id": "R1", "position": [2.0, 10.0], "start": [2.0, 10.0], "goal": [28.0, 10.0], "intent": "MOVING", "task_id": "T1", "planned_path": [[2.0, 10.0], [28.0, 10.0]], "velocity": [0.0, 0.0], "battery": 100.0, "heading": 0.0, "network": {"peers": {}}},
            {"id": "R4", "position": [28.0, 10.0], "start": [28.0, 10.0], "goal": [2.0, 10.0], "intent": "MOVING", "task_id": "T4", "planned_path": [[28.0, 10.0], [2.0, 10.0]], "velocity": [0.0, 0.0], "battery": 100.0, "heading": 3.14159, "network": {"peers": {}}},
        ],
        "tasks": [
            {"id": "T1", "assigned_robot": "R1", "status": "PENDING"},
            {"id": "T4", "assigned_robot": "R4", "status": "PENDING"},
        ],
        "events": ["[T=0.0s] SIMULATION READY"],
        "metrics": {"robots": 2, "active_tasks": 2, "completed_tasks": 0, "conflicts_detected": 0, "safety_violations": 0, "robots_moving": 2, "robots_waiting": 0, "robots_rerouting": 0, "throughput": 0.0, "simulation_time": 0.0},
    }
    default_json = json.dumps(default_state)
    config_json = json.dumps(manager.config)
    default_config_text = json.dumps(manager.config, indent=2)

    return HTMLResponse(
        """
        <!doctype html>
        <html lang="en">
        <head>
            <meta charset="utf-8" />
            <meta name="viewport" content="width=device-width, initial-scale=1" />
            <title>NEXUS Dashboard</title>
            <style>
                :root {{
                    --bg: #071426;
                    --panel: #0e1d2f;
                    --panel-alt: #14273d;
                    --text: #e6edf7;
                    --muted: #9eb5d0;
                    --accent: #4fc3f7;
                    --accent-2: #4ade80;
                    --warn: #fbbf24;
                    --danger: #fb7185;
                    --border: rgba(148, 163, 184, 0.26);
                }}
                * {{ box-sizing: border-box; }}
                html, body {{ margin: 0; height: 100%; }}
                body {{
                    font-family: Arial, Helvetica, sans-serif;
                    background: linear-gradient(180deg, #071426 0%, #0b1728 100%);
                    color: var(--text);
                }}
                .container {{ max-width: 1460px; margin: 0 auto; padding: 18px 22px 30px; }}
                .topbar {{ display: flex; justify-content: space-between; align-items: center; padding: 8px 0 18px; border-bottom: 1px solid var(--border); }}
                .brand {{ font-size: 22px; letter-spacing: 0.08em; font-weight: 700; text-transform: uppercase; }}
                .status-pill {{ padding: 7px 14px; border-radius: 999px; background: rgba(79,195,247,0.12); border: 1px solid rgba(79,195,247,0.28); color: var(--accent); font-size: 12px; letter-spacing: 0.08em; text-transform: uppercase; }}
                .layout {{ display: grid; grid-template-columns: 220px minmax(0, 1fr) 360px; gap: 18px; margin-top: 18px; }}
                .sidebar, .main-panel, .side-panel {{ background: rgba(14, 29, 47, 0.92); border: 1px solid var(--border); border-radius: 12px; }}
                .sidebar {{ padding: 14px 12px; }}
                .nav {{ display: grid; gap: 8px; margin-top: 10px; }}
                .nav button {{ width: 100%; background: rgba(148,163,184,0.04); border: 1px solid var(--border); color: var(--text); border-radius: 8px; padding: 10px 12px; text-align: left; cursor: default; }}
                .main-panel {{ padding: 16px; }}
                .toolbar {{ display: flex; flex-wrap: wrap; gap: 8px; margin-bottom: 12px; }}
                .toolbar button, .side-panel button {{ border: 1px solid var(--border); background: rgba(15,23,42,0.7); color: var(--text); border-radius: 8px; padding: 8px 12px; cursor: pointer; }}
                .stats {{ display: grid; grid-template-columns: repeat(3, minmax(110px, 1fr)); gap: 10px; margin-bottom: 12px; }}
                .card {{ background: rgba(20,39,61,0.9); border: 1px solid var(--border); border-radius: 10px; padding: 12px 10px; min-height: 76px; }}
                .label {{ color: var(--muted); font-size: 11px; letter-spacing: 0.12em; text-transform: uppercase; }}
                .value {{ margin-top: 8px; font-size: 26px; font-weight: 700; }}
                .warehouse-box {{ border: 1px solid var(--border); border-radius: 12px; overflow: hidden; height: 420px; background: linear-gradient(180deg, rgba(9,18,31,1) 0%, rgba(11,24,38,1) 100%); }}
                .warehouse-box svg {{ width: 100%; height: 100%; display: block; }}
                .task-list {{ margin-top: 12px; display: grid; gap: 8px; }}
                .task-item {{ display: flex; justify-content: space-between; align-items: center; background: rgba(20,39,61,0.9); border: 1px solid var(--border); border-radius: 8px; padding: 9px 12px; }}
                .status-tag {{ border-radius: 999px; padding: 4px 8px; font-size: 11px; text-transform: uppercase; }}
                .status-pending {{ background: rgba(251,191,36,0.12); color: var(--warn); }}
                .status-active {{ background: rgba(79,195,247,0.12); color: var(--accent); }}
                .status-completed {{ background: rgba(74,222,128,0.12); color: var(--accent-2); }}
                .side-panel {{ padding: 16px; }}
                .side-panel h3 {{ margin: 0 0 10px; font-size: 18px; }}
                textarea {{ width: 100%; min-height: 260px; border-radius: 10px; border: 1px solid var(--border); background: rgba(2,6,23,0.86); color: var(--text); padding: 12px; resize: vertical; font-family: Consolas, monospace; font-size: 12px; }}
                .mini-row {{ display: flex; gap: 8px; flex-wrap: wrap; margin-top: 8px; }}
                .metric-list {{ display: grid; gap: 8px; margin-top: 8px; }}
                .metric-item {{ display: flex; justify-content: space-between; align-items: center; background: rgba(20,39,61,0.9); border: 1px solid var(--border); border-radius: 8px; padding: 8px 10px; }}
                .robot-grid {{ display: grid; gap: 8px; margin-top: 14px; }}
                .robot-card {{ background: rgba(20,39,61,0.9); border: 1px solid var(--border); border-radius: 10px; padding: 10px 12px; }}
                .robot-header {{ display: flex; justify-content: space-between; align-items: center; font-weight: 700; margin-bottom: 8px; }}
                .robot-meta {{ font-size: 12px; color: var(--muted); display: grid; gap: 3px; }}
                .network-block {{ margin-top: 8px; padding-top: 8px; border-top: 1px solid var(--border); font-size: 12px; color: var(--muted); }}
                pre {{ margin: 0; white-space: pre-wrap; word-wrap: break-word; font-family: Consolas, monospace; font-size: 12px; line-height: 1.5; color: var(--text); }}
            </style>
        </head>
        <body>
            <script>
                window.__INITIAL_STATE__ = __DEFAULT_STATE__;
                window.__INITIAL_CONFIG__ = __DEFAULT_CONFIG__;
            </script>

            <div class="container">
                <div class="topbar">
                    <div class="brand">NEXUS</div>
                    <div class="status-pill" id="statusPill">READY</div>
                </div>

                <div class="layout">
                    <aside class="sidebar">
                        <h3>Navigation</h3>
                        <div class="nav">
                            <button>Overview</button>
                            <button>Warehouse</button>
                            <button>Robots</button>
                            <button>Tasks</button>
                            <button>Simulation</button>
                            <button>Analytics</button>
                        </div>
                    </aside>

                    <main class="main-panel">
                        <div class="toolbar">
                            <button id="startBtn">Start</button>
                            <button id="stepBtn">Step</button>
                            <button id="pauseBtn">Pause</button>
                            <button id="resetBtn">Reset</button>
                            <button id="saveBtn">Save</button>
                        </div>

                        <div class="stats" id="statsGrid">
                            <div class="card"><div class="label">Robots</div><div class="value">2</div></div>
                            <div class="card"><div class="label">Active Tasks</div><div class="value">2</div></div>
                            <div class="card"><div class="label">Completed</div><div class="value">0</div></div>
                        </div>

                        <div class="warehouse-box" id="warehouseBox">
                            <svg viewBox="0 0 30 20" preserveAspectRatio="xMidYMid meet">
                                <rect x="0" y="0" width="30" height="20" fill="#0b1220" stroke="#475569" stroke-width="0.12" />
                                <rect x="4" y="5" width="7" height="3" fill="#2b3f57" stroke="#9eb5d0" stroke-width="0.12"/>
                                <rect x="15" y="5" width="7" height="3" fill="#2b3f57" stroke="#9eb5d0" stroke-width="0.12"/>
                                <rect x="4" y="12" width="7" height="3" fill="#2b3f57" stroke="#9eb5d0" stroke-width="0.12"/>
                                <rect x="15" y="12" width="7" height="3" fill="#2b3f57" stroke="#9eb5d0" stroke-width="0.12"/>
                                <polyline points="2,10 28,10" fill="none" stroke="#4fc3f7" stroke-width="0.18" stroke-dasharray="0.35 0.2" opacity="0.8" />
                                <circle cx="2" cy="10" r="0.28" fill="#4ade80" stroke="#e5e7eb" stroke-width="0.08" />
                                <circle cx="28" cy="10" r="0.28" fill="#f87171" stroke="#e5e7eb" stroke-width="0.08" />
                                <circle cx="2" cy="10" r="0.42" fill="#4fc3f7" stroke="#e2e8f0" stroke-width="0.08" />
                                <circle cx="28" cy="10" r="0.42" fill="#4fc3f7" stroke="#e2e8f0" stroke-width="0.08" />
                                <text x="2.5" y="9.5" fill="#e2e8f0" font-size="0.6" font-family="Arial">R1</text>
                                <text x="28.5" y="9.5" fill="#e2e8f0" font-size="0.6" font-family="Arial">R4</text>
                            </svg>
                        </div>

                        <div class="task-list" id="taskList">
                            <div class="task-item"><span><strong>T1</strong> · Robot R1</span><span class="status-tag status-pending">PENDING</span></div>
                            <div class="task-item"><span><strong>T4</strong> · Robot R4</span><span class="status-tag status-pending">PENDING</span></div>
                        </div>

                        <div class="robot-grid" id="robotGrid">
                            <div class="robot-card">
                                <div class="robot-header"><span>R1</span><span class="status-tag status-active">MOVING</span></div>
                                <div class="robot-meta">
                                    <div>Pos: (2.00, 10.00)</div>
                                    <div>Goal: (28.00, 10.00)</div>
                                    <div>Task: T1</div>
                                </div>
                            </div>
                            <div class="robot-card">
                                <div class="robot-header"><span>R4</span><span class="status-tag status-active">MOVING</span></div>
                                <div class="robot-meta">
                                    <div>Pos: (28.00, 10.00)</div>
                                    <div>Goal: (2.00, 10.00)</div>
                                    <div>Task: T4</div>
                                </div>
                            </div>
                        </div>
                    </main>

                    <aside class="side-panel">
                        <h3>Simulation Config</h3>
                        <textarea id="configEditor">__DEFAULT_CONFIG_TEXT__</textarea>
                        <div class="mini-row">
                            <button id="loadConfigBtn">Load Config</button>
                            <button id="resetDefaultBtn">Reset Default</button>
                        </div>

                        <h3 style="margin-top: 18px;">Live Metrics</h3>
                        <div class="metric-list" id="metricList">
                            <div class="metric-item"><span>Simulation Time</span><strong>0.0 s</strong></div>
                            <div class="metric-item"><span>Robots Moving</span><strong>2</strong></div>
                            <div class="metric-item"><span>Robots Waiting</span><strong>0</strong></div>
                            <div class="metric-item"><span>Robots Rerouting</span><strong>0</strong></div>
                        </div>

                        <h3 style="margin-top: 18px;">Event Feed</h3>
                        <pre id="eventLog">[T=0.0s] SIMULATION READY</pre>
                    </aside>
                </div>
            </div>

            <script>
                const initialState = window.__INITIAL_STATE__ || {};
                const initialConfig = window.__INITIAL_CONFIG__ || {};
                const statusPill = document.getElementById('statusPill');
                const statsGrid = document.getElementById('statsGrid');
                const warehouseBox = document.getElementById('warehouseBox');
                const taskList = document.getElementById('taskList');
                const robotGrid = document.getElementById('robotGrid');
                const metricList = document.getElementById('metricList');
                const eventLog = document.getElementById('eventLog');
                const configEditor = document.getElementById('configEditor');

                function fmt(value, digits = 2) {
                    const n = Number(value || 0);
                    return Number.isFinite(n) ? n.toFixed(digits) : '0.00';
                }

                function renderWarehouse(data) {
                    const warehouse = data.warehouse || { width: 30, height: 20, obstacles: [] };
                    const robots = data.robots || [];
                    const width = Number(warehouse.width || 30);
                    const height = Number(warehouse.height || 20);
                    const obstacles = warehouse.obstacles || [];
                    const obstacleMarkup = obstacles.map((obstacle) => {
                        if (obstacle.type === 'rect') {
                            const x = Number(obstacle.x || 0);
                            const y = Number(obstacle.y || 0);
                            const w = Number(obstacle.w || 0);
                            const h = Number(obstacle.h || 0);
                            return `<rect x="${x}" y="${y}" width="${w}" height="${h}" fill="#2b3f57" stroke="#9eb5d0" stroke-width="0.12" />`;
                        }
                        return '';
                    }).join('');
                    const pathMarkup = robots.map((robot) => {
                        const points = (robot.planned_path || []).map((p) => `${p[0]},${p[1]}`).join(' ');
                        if (!points) return '';
                        const color = robot.intent === 'STOPPED' ? '#fbbf24' : robot.intent === 'REROUTING' ? '#c084fc' : '#4fc3f7';
                        return `<polyline points="${points}" fill="none" stroke="${color}" stroke-width="0.18" stroke-dasharray="0.35 0.2" opacity="0.8" />`;
                    }).join('');
                    const robotMarkup = robots.map((robot) => {
                        const pos = robot.position || [0, 0];
                        const color = robot.intent === 'STOPPED' ? '#fbbf24' : robot.intent === 'REROUTING' ? '#c084fc' : '#4fc3f7';
                        return `
                            <g>
                                <circle cx="${pos[0]}" cy="${pos[1]}" r="0.42" fill="${color}" stroke="#e2e8f0" stroke-width="0.08" />
                                <text x="${pos[0] + 0.5}" y="${pos[1] - 0.6}" fill="#e2e8f0" font-size="0.6" font-family="Arial">${robot.id}</text>
                            </g>
                        `;
                    }).join('');
                    warehouseBox.innerHTML = `
                        <svg viewBox="0 0 ${width} ${height}" preserveAspectRatio="xMidYMid meet">
                            <rect x="0" y="0" width="${width}" height="${height}" fill="#0b1220" stroke="#475569" stroke-width="0.12" />
                            ${obstacleMarkup}
                            ${pathMarkup}
                            ${robotMarkup}
                        </svg>
                    `;
                }

                function renderStats(data) {
                    const metrics = data.metrics || {};
                    statsGrid.innerHTML = [
                        ['Robots', metrics.robots || 0],
                        ['Active Tasks', metrics.active_tasks || 0],
                        ['Completed', metrics.completed_tasks || 0]
                    ].map(([label, value]) => `
                        <div class="card"><div class="label">${label}</div><div class="value">${value}</div></div>
                    `).join('');
                    statusPill.textContent = (data.status || 'READY').toUpperCase();
                }

                function renderTasks(data) {
                    const tasks = data.tasks || [];
                    taskList.innerHTML = tasks.map((task) => `
                        <div class="task-item"><span><strong>${task.id}</strong> · Robot ${task.assigned_robot || 'Unassigned'}</span><span class="status-tag ${task.status === 'COMPLETED' ? 'status-completed' : 'status-pending'}">${(task.status || 'PENDING').toUpperCase()}</span></div>
                    `).join('');
                }

                function renderRobots(data) {
                    const robots = data.robots || [];
                    robotGrid.innerHTML = robots.map((robot) => `
                        <div class="robot-card">
                            <div class="robot-header"><span>${robot.id}</span><span class="status-tag ${robot.intent === 'STOPPED' ? 'status-pending' : 'status-active'}">${robot.intent || 'IDLE'}</span></div>
                            <div class="robot-meta">
                                <div>Pos: (${fmt(robot.position[0], 2)}, ${fmt(robot.position[1], 2)})</div>
                                <div>Goal: (${fmt((robot.goal || [0, 0])[0], 2)}, ${fmt((robot.goal || [0, 0])[1], 2)})</div>
                                <div>Task: ${robot.task_id || '—'}</div>
                                <div>Battery: ${fmt(robot.battery, 1)}%</div>
                            </div>
                        </div>
                    `).join('');
                }

                function renderMetrics(data) {
                    const metrics = data.metrics || {};
                    metricList.innerHTML = [
                        ['Simulation Time', `${fmt(data.simulation_time, 1)} s`],
                        ['Robots Moving', metrics.robots_moving || 0],
                        ['Robots Waiting', metrics.robots_waiting || 0],
                        ['Robots Rerouting', metrics.robots_rerouting || 0],
                    ].map(([label, value]) => `
                        <div class="metric-item"><span>${label}</span><strong>${value}</strong></div>
                    `).join('');
                }

                function renderEvents(data) {
                    const events = Array.isArray(data.events) ? data.events : ['[T=0.0s] SIMULATION READY'];
                    eventLog.textContent = events.length ? events.join('\n') : '[T=0.0s] SIMULATION READY';
                }

                function hydrate(data) {
                    if (!data || !data.warehouse) return;
                    renderWarehouse(data);
                    renderStats(data);
                    renderTasks(data);
                    renderRobots(data);
                    renderMetrics(data);
                    renderEvents(data);
                    configEditor.value = JSON.stringify(initialConfig, null, 2);
                }

                async function tick() {
                    try {
                        const response = await fetch('/api/state');
                        const data = await response.json();
                        hydrate(data);
                    } catch (error) {
                        hydrate(initialState);
                    }
                }

                document.getElementById('startBtn').addEventListener('click', async () => {
                    try { await fetch('/api/start', { method: 'POST' }); } catch (error) {}
                    tick();
                });
                document.getElementById('stepBtn').addEventListener('click', async () => {
                    try { await fetch('/api/step', { method: 'POST' }); } catch (error) {}
                    tick();
                });
                document.getElementById('pauseBtn').addEventListener('click', async () => {
                    try { await fetch('/api/pause', { method: 'POST' }); } catch (error) {}
                    tick();
                });
                document.getElementById('resetBtn').addEventListener('click', async () => {
                    try { await fetch('/api/reset', { method: 'POST' }); } catch (error) {}
                    tick();
                });
                document.getElementById('saveBtn').addEventListener('click', async () => {
                    await fetch('/api/save', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ path: 'data/simulations/demo.json' })});
                });
                document.getElementById('loadConfigBtn').addEventListener('click', async () => {
                    try {
                        const payload = JSON.parse(configEditor.value || '{}');
                        await fetch('/api/config', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ config: payload })});
                    } catch (error) {}
                    tick();
                });
                document.getElementById('resetDefaultBtn').addEventListener('click', async () => {
                    try { await fetch('/api/load-default', { method: 'POST' }); } catch (error) {}
                    tick();
                });

                hydrate(initialState);
                configEditor.value = JSON.stringify(initialConfig, null, 2);
                setInterval(tick, 1000);
            </script>
        </body>
        </html>
        """.replace("__DEFAULT_STATE__", default_json).replace("__DEFAULT_CONFIG__", config_json).replace("__DEFAULT_CONFIG_TEXT__", default_config_text)
    )


@app.get("/api/state")
def get_state():
    return manager.snapshot()


@app.get("/api/config")
def get_config():
    return manager.config


@app.post("/api/reset")
def reset_simulation():
    with stream_lock:
        stream_stop.set()
        manager.reset()
    return manager.snapshot()


@app.post("/api/step")
def step_simulation():
    with stream_lock:
        return manager.step()


@app.post("/api/pause")
def pause_simulation():
    with stream_lock:
        stream_stop.set()
        manager.status = "PAUSED"
    return manager.snapshot()


@app.post("/api/start")
def start_simulation():
    with stream_lock:
        manager.status = "RUNNING"
        ensure_stream()
    return manager.snapshot()


@app.post("/api/config")
def set_config(payload: ConfigPayload):
    with stream_lock:
        stream_stop.set()
        manager.config = payload.config
        manager.reset()
    return manager.snapshot()


@app.post("/api/load-default")
def load_default_config():
    with stream_lock:
        stream_stop.set()
        manager.config = SimulationManager.create_default_config()
        manager.reset()
    return manager.snapshot()


@app.post("/api/save")
def save_simulation(payload: SavePayload):
    path = payload.path
    save_config(manager.config, path)
    return {"status": "saved", "path": path}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("dashboard_server:app", host="0.0.0.0", port=8000, reload=False)
