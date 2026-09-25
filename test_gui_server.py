"""
Smoke test for the NEXUS control room server (nexus_gui).

Checks the REST API, that the page and scripts are served, and that the
live WebSocket accepts play / step / speed / reset / load commands.
No browser or running server needed.

Run:  python test_gui_server.py
"""

import json

from fastapi.testclient import TestClient

from nexus_gui.server import app


def check(condition, message):
    print(f"[{'PASS' if condition else 'FAIL'}] {message}")
    return 0 if condition else 1


def main():
    print("=== NEXUS control room server ===")
    failures = 0

    with TestClient(app) as client:
        failures += check(client.get("/").status_code == 200, "GET / serves the page")
        for asset in ("/static/app.css", "/static/js/main.js", "/static/js/map.js"):
            failures += check(client.get(asset).status_code == 200, f"GET {asset}")

        layouts = client.get("/api/layouts").json()
        names = [item["name"] for item in layouts["layouts"]]
        failures += check(len(names) >= 1, f"layouts listed: {names}")

        for name in names:
            body = client.get(f"/api/layouts/{name}").json()
            errors = [i for i in body["issues"] if i["severity"] == "error"]
            failures += check(not errors, f"layout {name} validates ({len(body['layout']['robots'])} robots)")

        bad = client.post("/api/validate", json={"width": 3, "height": 2, "rows": ["+++", "+++"],
                                                  "robots": [{"id": "R1", "start": [0, 0], "goal": "NOPE"}]}).json()
        failures += check(any(i["severity"] == "error" for i in bad["issues"]), "validation reports an unknown goal station")
        failures += check(client.get("/api/layouts/..%2Fsecret").status_code in (400, 404), "layout names are sanitised")

        with client.websocket_connect("/ws") as ws:
            first = json.loads(ws.receive_text())
            second = json.loads(ws.receive_text())
            failures += check(first["type"] == "layout" and second["type"] == "state", "WebSocket sends layout then state")

            ws.send_text(json.dumps({"cmd": "step"}))
            state = json.loads(ws.receive_text())
            failures += check(state["state"]["time"] > 0 and not state["running"], "step advances the simulation")

            ws.send_text(json.dumps({"cmd": "speed", "value": 4}))
            state = json.loads(ws.receive_text())
            failures += check(state["speed"] == 4, "speed changes to 4x")

            ws.send_text(json.dumps({"cmd": "reset"}))
            state = json.loads(ws.receive_text())
            failures += check(state["state"]["time"] == 0 and state["reset_events"], "reset returns to t = 0")

            target = names[-1]
            ws.send_text(json.dumps({"cmd": "load", "name": target}))
            loaded = json.loads(ws.receive_text())
            failures += check(loaded["type"] == "layout" and loaded["name"] == target, f"load switches layout to {target}")
            json.loads(ws.receive_text())

            ws.send_text(json.dumps({"cmd": "load", "name": "does-not-exist"}))
            error = json.loads(ws.receive_text())
            failures += check(error["type"] == "error", "loading a missing layout reports an error")

    print("ALL PASS" if failures == 0 else f"{failures} check(s) FAILED")
    raise SystemExit(1 if failures else 0)


if __name__ == "__main__":
    main()
