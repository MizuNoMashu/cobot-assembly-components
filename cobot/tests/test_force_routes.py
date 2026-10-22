import numpy as np
import pytest

flask = pytest.importorskip("flask")


@pytest.fixture
def client(monkeypatch):
    from app.routes import robot as robot_routes

    started = []

    class FakeManager:
        is_connected = True
        is_busy = False
        robot = type("R", (), {
            "get_state": lambda self: type("S", (), {"q": [0, -0.78, 0, -2.36, 0, 1.57, -2.5]})(),
            "motion": type("M", (), {
                "run_force_task": lambda self, params, progress_callback=None: True,
                "run_screw_passes": lambda self, params, regrasp, iterations, progress_callback=None: True,
            })(),
        })()

        def run_async(self, func, *args, **kwargs):
            started.append((func, args, kwargs))
            return True

    monkeypatch.setattr(robot_routes, "manager", FakeManager())
    app = flask.Flask(__name__)
    app.register_blueprint(robot_routes.robot_bp)
    client = app.test_client()
    client.started = started
    return client


def test_apply_accepts_and_builds_params(client):
    r = client.post("/api/motion-force/apply", json={"force": 10, "duration": 3})
    assert r.status_code == 202 and r.json["operation"] == "force_hold"
    params = client.started[0][1][0]
    assert params.mode == "hold" and params.hold_time == 3 and params.force == 10.0


def test_invalid_parameters_are_400_not_background_errors(client):
    assert client.post("/api/motion-force/apply", json={"force": 500}).status_code == 400
    assert client.post("/api/motion-force/apply", json={}).status_code == 400
    assert client.post("/api/motion-force/insert", json={"force": 10}).status_code == 400
    assert client.post("/api/motion-force/apply", json={"force": 10, "bogus": 1}).status_code == 400
    assert client.post("/api/motion-force/apply", json={"force": "abc"}).status_code == 400
    assert client.started == []


def test_screw_rejects_rotation_beyond_wrist_range_immediately(client):
    r = client.post("/api/motion-force/screw", json={"force": 10, "angle": 6.0})
    assert r.status_code == 400 and "giunto 7" in r.json["error"]
    assert client.started == []
    ok = client.post("/api/motion-force/screw", json={"force": 10, "angle": 1.0, "max_torque": 2})
    assert ok.status_code == 202


def test_screw_iterations_start_multi_pass_operation(client):
    body = {"force": 10, "angle": 4.0, "iterations": 3, "regrasp_mode": "gripper",
            "grasp_width_mm": 8, "open_width_mm": 20}
    r = client.post("/api/motion-force/screw", json=body)
    assert r.status_code == 202 and r.json["iterations"] == 3 and r.json["regrasp_mode"] == "gripper"
    func, args, _ = client.started[0]
    params, regrasp, iterations = args
    assert iterations == 3 and params.angle == 4.0 and regrasp.grasp_width_mm == 8


def test_screw_iterations_invalid_is_400(client):
    base = {"force": 10, "angle": 4.0, "iterations": 2}
    assert client.post("/api/motion-force/screw", json={**base, "regrasp_mode": "gripper"}).status_code == 400
    assert client.post("/api/motion-force/screw", json={**base, "iterations": 0, "regrasp_mode": "none"}).status_code == 400
    assert client.post("/api/motion-force/screw", json={**base, "iterations": 1.5, "regrasp_mode": "none"}).status_code == 400
    assert client.post("/api/motion-force/screw", json={**base, "regrasp_mode": "none", "angle": 6.2}).status_code == 400
    assert client.started == []
