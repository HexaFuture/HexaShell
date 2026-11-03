"""URAI's HTTP calls.  A motion is a draft, then a preview (the plan), then its execution; executing
polls the state of that one execution till it ends and never sends the motion again."""

from __future__ import annotations

import time

import httpx


class UraiClient:
    def __init__(self, url: str, token: str | None = None, *, log):
        headers = {} if token is None else {"Authorization": f"Bearer {token}"}
        self.http = httpx.Client(base_url=url, headers=headers, timeout=None, trust_env=False)
        self.log = log

    def request(self, method: str, path: str, body=None):
        response = self.http.request(method, path, json=body)
        response.raise_for_status()
        return response.json()

    def model(self) -> dict:
        """The robot: its name, world frame, TCP, bases, table height, grippers and cameras."""
        return self.request("GET", "/api/harness/model")

    def observe(self, camera: str = "head") -> dict:
        """One camera's RGB-D picture (npy, base64), its K and pose, the arms and their capsules."""
        return self.request("POST", "/api/harness/observe", {"camera": camera})

    def state(self) -> dict:
        return self.request("GET", "/api/state")

    def plan(self, observation_id: str, arms: dict, *, planning_only: bool = False) -> dict:
        """The plan of a motion of each arm (``arms``: arm -> what it does), checked against the picture
        ``observation_id``.  A plan URAI refuses raises an HTTPStatusError 409; ``detail`` says why."""
        draft = self.request("PUT", "/api/draft", {
            "observation_id": observation_id, "completion": "hold", "arms": arms})
        return self.request("POST", "/api/preview?trajectory=full", {
            "expected_revision": draft["revision"], "planning_only": planning_only})

    def execute(self, plan: dict) -> dict:
        """Run a plan to its end; the state then."""
        self.request("POST", "/api/execute", {"preview_id": plan["id"]})
        status = self.state()
        while status["execution"]["state"] in ("running", "cancelling"):
            time.sleep(0.02)
            status = self.state()
        self.log({"kind": "urai_execution", "preview_id": plan["id"], "execution": status["execution"]})
        return status

    def close(self) -> None:
        self.http.close()
