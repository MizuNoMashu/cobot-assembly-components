import math

import numpy as np
import pytest

from franka_controller.force_control import (
    ForceTaskAbort,
    ForceTaskController,
    ForceTaskParams,
    Phase,
    as_jacobian,
    make_params,
    precheck_wrist_range,
    rotation_about_axis,
    rotation_vector,
)
from franka_controller.kinematics import FR3_Q_MAX, forward_kinematics_and_jacobian

READY = np.array([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4])
INERTIA = np.array([3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 0.3])


class SimPlant:
    """Robot a 7 giunti con gravità compensata e un ambiente semplice.

    `resist(depth, speed)` restituisce la forza di reazione (N, ≥ 0) lungo -a
    e `resist_torque(angle)` la coppia di reazione (Nm) attorno ad a.
    """

    def __init__(self, resist, resist_torque=None):
        self.q = READY.copy()
        self.dq = np.zeros(7)
        self.tau_prev = np.zeros(7)
        flange, _ = forward_kinematics_and_jacobian(self.q)
        self.x0 = flange[0, :3, 3].copy()
        self.R0 = flange[0, :3, :3].copy()
        self.a = self.R0 @ np.array([0.0, 0.0, 1.0])
        self.resist = resist
        self.resist_torque = resist_torque or (lambda angle: 0.0)
        self.t = 0.0
        self.log = []

    def pose(self):
        flange, J = forward_kinematics_and_jacobian(self.q)
        return flange[0, :3, 3], flange[0, :3, :3], J[0]

    def run(self, controller, max_time=30.0, dt=0.001):
        while self.t < max_time:
            x, R, J = self.pose()
            depth = float((x - self.x0) @ self.a)
            speed = float((J @ self.dq)[:3] @ self.a)
            angle = float(rotation_vector(R @ self.R0.T) @ self.a)
            f_env = -self.resist(depth, speed) * self.a
            m_env = -self.resist_torque(angle) * self.a
            ext = np.concatenate([f_env, m_env])
            out = controller.step(self.t, dt, self.q, self.dq, x, R, J, np.zeros(7), ext, self.tau_prev)
            self.tau_prev = out.tau
            ddq = (out.tau + J.T @ ext - 1.0 * self.dq) / INERTIA
            self.dq = self.dq + ddq * dt
            self.q = self.q + self.dq * dt
            self.t += dt
            self.log.append((self.t, out.info))
            if out.finished:
                return True
        return False


def _controller(plant, params):
    x, R, _ = plant.pose()
    return ForceTaskController(params, plant.q, x, R)


def wall(position=0.02, k=8000.0, c=30.0):
    return lambda depth, speed: max(k * (depth - position) + c * max(speed, 0.0), 0.0) if depth > position else 0.0


def test_geometry_helpers():
    R = rotation_about_axis(np.array([0, 0, 1.0]), 0.7)
    assert rotation_vector(R) == pytest.approx([0, 0, 0.7], abs=1e-9)
    J = np.arange(42.0).reshape(6, 7)
    assert as_jacobian(J.flatten(order="F")) == pytest.approx(J)
    assert as_jacobian(J) == pytest.approx(J)


def test_params_validation():
    with pytest.raises(ValueError):
        make_params("hold", force=100.0)
    with pytest.raises(ValueError):
        make_params("insert", force=10.0)  # manca target_depth
    with pytest.raises(ValueError):
        make_params("screw", force=10.0, angle=0.0)
    with pytest.raises(ValueError):
        make_params("hold", force=10.0, foo=1)
    thr = make_params("hold", force=20.0).collision_thresholds()
    assert thr["upper_force_thresholds"][0] > 20.0


def test_wrist_range_precheck():
    params = make_params("screw", force=10.0, angle=6.0)
    with pytest.raises(ValueError, match="giunto 7"):
        precheck_wrist_range(params, READY)
    precheck_wrist_range(make_params("screw", force=10.0, angle=-1.5), READY)


def test_hold_reaches_setpoint_force_against_wall():
    plant = SimPlant(wall())
    params = make_params("hold", force=10.0, hold_time=2.0, approach_speed=0.01)
    assert plant.run(_controller(plant, params), max_time=40.0)
    # finale: ha toccato, ha tenuto ~10 N e rilasciato
    held = [i["force_measured"] for t, i in plant.log if i["phase"] == "run"]
    assert held and abs(np.mean(held[-500:]) - 10.0) < 1.5
    assert plant.log[-1][1]["force_measured"] < 2.0
    assert max(i["force_measured"] for _, i in plant.log) < 18.0
    assert np.linalg.norm((plant.pose()[0] - plant.x0) - ((plant.pose()[0] - plant.x0) @ plant.a) * plant.a) < 5e-3


def test_no_contact_fails_gracefully():
    plant = SimPlant(lambda depth, speed: 0.0)
    params = make_params("hold", force=10.0, max_approach_distance=0.01)
    controller = _controller(plant, params)
    assert plant.run(controller, max_time=30.0)
    assert controller.success is False and "nessun contatto" in controller.reason


def test_insert_reaches_target_depth():
    # inserimento: attrito + molla fino a 15 mm, poi fondo corsa rigido
    def peg(depth, speed):
        if depth <= 0.0:
            return 0.0
        if depth < 0.015:
            return 3.0 + 400.0 * depth
        return 9.0 + 20000.0 * (depth - 0.015)

    plant = SimPlant(peg)
    params = make_params("insert", force=15.0, target_depth=0.012, approach_speed=0.0, max_time=20.0)
    controller = _controller(plant, params)
    assert plant.run(controller, max_time=30.0)
    assert controller.success is True, controller.reason
    assert controller.last_depth > 0.0119 or controller.result()["depth_m"] > 0.0115


def test_insert_jam_is_detected():
    # ostacolo rigido a 4 mm: l'inserimento si blocca e il task termina in ordine
    def jam(depth, speed):
        return 0.0 if depth <= 0.004 else 30000.0 * (depth - 0.004)

    plant = SimPlant(jam)
    params = make_params("insert", force=10.0, target_depth=0.02, approach_speed=0.0, stall_time=1.0)
    controller = _controller(plant, params)
    assert plant.run(controller, max_time=40.0)
    assert controller.success is False and "bloccato" in controller.reason


def test_over_force_aborts():
    plant = SimPlant(lambda depth, speed: 200.0 if depth > -1 else 0.0)
    params = make_params("hold", force=10.0, approach_speed=0.0)
    with pytest.raises(ForceTaskAbort):
        plant.run(_controller(plant, params), max_time=10.0)


def test_screw_rotates_with_axial_force():
    plant = SimPlant(lambda depth, speed: 0.0 if depth < 0.0 else 8000.0 * depth)
    # forza assiale = la superficie sostiene la spinta (il tool si ferma a depth≈0)
    params = make_params("screw", force=8.0, angle=1.0, angular_speed=0.4, approach_speed=0.0, max_torque=3.0)
    controller = _controller(plant, params)
    assert plant.run(controller, max_time=40.0)
    assert controller.success is True, controller.reason
    assert controller.result()["angle_rad"] == pytest.approx(1.0, abs=0.12)


def test_screw_stops_at_max_torque():
    # la vite si blocca a 0.5 rad con coppia di reazione crescente
    def torque(angle):
        return 0.0 if angle < 0.5 else 400.0 * (angle - 0.5)

    plant = SimPlant(lambda depth, speed: 8000.0 * max(depth, 0.0), torque)
    params = make_params("screw", force=8.0, angle=2.0, angular_speed=0.4, approach_speed=0.0,
                         max_torque=1.5, stop_at_torque=True)
    controller = _controller(plant, params)
    assert plant.run(controller, max_time=60.0)
    assert controller.success is True and "coppia" in controller.reason
    assert controller.result()["angle_rad"] < 1.0

    plant = SimPlant(lambda depth, speed: 8000.0 * max(depth, 0.0), torque)
    params = make_params("screw", force=8.0, angle=2.0, angular_speed=0.4, approach_speed=0.0, max_torque=1.5)
    controller = _controller(plant, params)
    assert plant.run(controller, max_time=60.0)
    assert controller.success is False


# ---------------------------------------------------------------------------
# Integrazione MotionController con un robot finto in torque control
# ---------------------------------------------------------------------------


class _Period:
    def __init__(self, dt):
        self._dt = dt

    def to_sec(self):
        return self._dt


class _FakeTorqueRobot:
    """Fa da FrankaRobot e da pylibfranka.Robot, usando SimPlant come fisica."""

    def __init__(self, plant):
        from franka_controller.robot import RobotMode

        self.plant = plant
        self.mode = RobotMode.READY
        self.robot = self
        self.locked = None
        self.collision_calls = []
        self.stopped = False
        self.last_tau = np.zeros(7)
        self.stop_requested = False

    # -- FrankaRobot
    def _ensure_can_command(self):
        pass

    def check_stop(self):
        pass

    def get_state(self):
        return self._state()

    def set_collision_behavior(self, **kwargs):
        self.collision_calls.append(kwargs)

    def _enter_error_locked(self, operation, exception, recoverable):
        self.locked = (operation, exception)
        return type("E", (), {"message": str(exception)})()

    # -- pylibfranka.Robot
    def load_model(self):
        outer = self

        class Model:
            def zero_jacobian(self, state):
                return outer.plant.pose()[2].flatten(order="F").tolist()

            def coriolis(self, state):
                return [0.0] * 7

        return Model()

    def start_torque_control(self):
        return self

    def stop(self):
        self.stopped = True

    def readOnce(self):
        return self._state(), _Period(0.001)

    def writeOnce(self, command):
        p = self.plant
        x, R, J = p.pose()
        depth = float((x - p.x0) @ p.a)
        speed = float((J @ p.dq)[:3] @ p.a)
        ext = np.concatenate([-p.resist(depth, speed) * p.a, np.zeros(3)])
        tau = np.array(command.tau_J)
        p.dq = p.dq + (tau + J.T @ ext - p.dq) / INERTIA * 0.001
        p.q = p.q + p.dq * 0.001
        self.last_tau = tau

    def _state(self):
        p = self.plant
        x, R, J = p.pose()
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, x
        depth = float((x - p.x0) @ p.a)
        speed = float((J @ p.dq)[:3] @ p.a)
        ext = np.concatenate([-p.resist(depth, speed) * p.a, np.zeros(3)])
        return type("S", (), {
            "q": p.q.tolist(), "dq": p.dq.tolist(),
            "O_T_EE": T.flatten(order="F").tolist(),
            "O_F_ext_hat_K": ext.tolist(),
            "tau_J_d": self.last_tau.tolist(),
            "cartesian_collision": [0.0] * 6, "cartesian_contact": [0.0] * 6,
        })()


@pytest.fixture
def force_motion(monkeypatch):
    from franka_controller import motion as motion_module

    class Torques:
        def __init__(self, tau):
            self.tau_J = tau
            self.motion_finished = False

    monkeypatch.setattr(motion_module.pylibfranka, "Torques", Torques, raising=False)
    monkeypatch.setattr(motion_module.pylibfranka, "ControlException", type("CE", (Exception,), {}), raising=False)

    def build(resist):
        fake = _FakeTorqueRobot(SimPlant(resist))
        return motion_module.MotionController(fake), fake

    return build


def test_apply_force_end_to_end_restores_collision_thresholds(force_motion):
    controller, fake = force_motion(wall())
    assert controller.apply_force(10.0, duration=1.0) is True
    assert fake.locked is None and fake.stopped is False
    # soglie alzate all'inizio, poi ripristinate ai default (chiamata senza argomenti)
    assert len(fake.collision_calls) == 2
    assert fake.collision_calls[0]["upper_force_thresholds"][0] > 20.0
    assert fake.collision_calls[1] == {"allow_locked": True}


def test_failed_goal_raises_without_locking_robot(force_motion):
    from franka_controller.force_control import ForceTaskFailed

    controller, fake = force_motion(lambda depth, speed: 0.0)
    with pytest.raises(ForceTaskFailed, match="nessun contatto"):
        controller.apply_force(10.0, duration=1.0, max_approach_distance=0.01)
    assert fake.locked is None
    assert fake.collision_calls[-1] == {"allow_locked": True}


def test_over_force_locks_robot_and_stops(force_motion):
    controller, fake = force_motion(lambda depth, speed: 300.0)
    assert controller.apply_force(10.0, duration=1.0, approach_speed=0.0) is False
    assert fake.locked is not None and fake.stopped is True


def test_wrist_range_rejected_before_any_torque(force_motion):
    controller, fake = force_motion(wall())
    with pytest.raises(ValueError, match="giunto 7"):
        controller.screw(6.0)
    assert fake.collision_calls == []


# ---------------------------------------------------------------------------
# Avvitamento a più passate (ripresa della vite)
# ---------------------------------------------------------------------------


class _FakeGripper:
    def __init__(self, log, fail_grasp=False):
        self.log, self.fail_grasp = log, fail_grasp

    def open(self, width, speed):
        self.log.append(("open", round(width, 3)))
        return True

    def grasp(self, width, force, speed):
        self.log.append(("grasp", round(width, 3), force))
        return not self.fail_grasp


def _passes_controller(monkeypatch, results, fail_grasp=False, q7=-2.5):
    from franka_controller import motion as motion_module
    from franka_controller.robot import RobotMode

    log = []
    fake = type("F", (), {})()
    fake.mode = RobotMode.READY
    fake._ensure_can_command = lambda: None
    fake.check_stop = lambda: None
    fake.get_state = lambda: type("S", (), {"q": [0, -0.78, 0, -2.36, 0, 1.57, q7]})()
    fake.gripper = _FakeGripper(log, fail_grasp)
    controller = motion_module.MotionController(fake)
    queue = list(results)

    def run_force_task(params, progress_callback=None):
        log.append(("pass", params.angle))
        result = queue.pop(0)
        if isinstance(result, Exception):
            raise result
        controller.last_force_result = result
        return True

    controller.run_force_task = run_force_task
    controller.move_to_joint_positions = lambda target, **kw: log.append(("rewind", round(target[6], 2))) or True
    controller.move_cartesian_relative = lambda dx, dy, dz, **kw: log.append(("retract", round(dz, 4))) or True
    return controller, log


def test_screw_passes_gripper_regrasp_sequence(monkeypatch):
    from franka_controller.force_control import make_params, make_regrasp

    ok = {"success": True, "torque_reached": False, "angle_rad": 4.0}
    controller, log = _passes_controller(monkeypatch, [ok, ok, ok])
    params = make_params("screw", force=10.0, angle=4.0, approach_speed=0.0)
    regrasp = make_regrasp(regrasp_mode="gripper", grasp_width_mm=8, open_width_mm=20)
    assert controller.run_screw_passes(params, regrasp, 3) is True
    kinds = [e[0] for e in log]
    assert kinds == ["pass", "open", "rewind", "grasp", "pass", "open", "rewind", "grasp", "pass"]
    assert log[1] == ("open", 0.02) and log[3][:2] == ("grasp", 0.008)
    assert log[2][1] < -2.5  # estremo opposto all'avvitamento positivo


def test_screw_passes_retract_mode_and_early_stop_on_torque(monkeypatch):
    from franka_controller.force_control import make_params, make_regrasp

    ok = {"success": True, "torque_reached": False, "angle_rad": 4.0}
    tight = {"success": True, "torque_reached": True, "angle_rad": 1.0}
    controller, log = _passes_controller(monkeypatch, [ok, tight, ok], q7=2.5)
    params = make_params("screw", force=10.0, angle=-4.0, approach_speed=0.01, stop_at_torque=True)
    regrasp = make_regrasp(regrasp_mode="retract", retract_distance=0.008)
    assert controller.run_screw_passes(params, regrasp, 3) is True
    assert [e[0] for e in log] == ["pass", "retract", "rewind", "pass"]  # la 3ª passata è saltata
    assert log[1] == ("retract", -0.008) and log[2][1] > 2.5  # angolo negativo → estremo positivo


def test_screw_passes_failures_are_labelled(monkeypatch):
    from franka_controller.force_control import ForceTaskFailed, make_params, make_regrasp

    params = make_params("screw", force=10.0, angle=4.0, approach_speed=0.0)
    regrasp = make_regrasp(regrasp_mode="gripper", grasp_width_mm=8)
    ok = {"success": True, "torque_reached": False, "angle_rad": 4.0}

    controller, _ = _passes_controller(monkeypatch, [ok], fail_grasp=True)
    with pytest.raises(ForceTaskFailed, match=r"passata 1/2.*presa della vite"):
        controller.run_screw_passes(params, regrasp, 2)

    controller, _ = _passes_controller(monkeypatch, [ok, ForceTaskFailed("bloccato")])
    with pytest.raises(ForceTaskFailed, match=r"passata 2/2: bloccato"):
        controller.run_screw_passes(params, regrasp, 2)


def test_screw_passes_validation():
    from franka_controller.force_control import make_params, make_regrasp, precheck_screw_passes

    q = READY.copy()
    gripper = make_regrasp(regrasp_mode="gripper", grasp_width_mm=8)
    with pytest.raises(ValueError):
        make_regrasp(regrasp_mode="gripper")  # manca grasp_width_mm
    with pytest.raises(ValueError):
        make_regrasp(regrasp_mode="gripper", grasp_width_mm=30, open_width_mm=20)
    with pytest.raises(ValueError, match="iterations"):
        precheck_screw_passes(make_params("screw", force=10.0, angle=1.0), gripper, 0, q)
    # una passata più larga dell'escursione non ci sta nemmeno dopo il riavvolgimento
    with pytest.raises(ValueError, match="escursione"):
        precheck_screw_passes(make_params("screw", force=10.0, angle=-5.9), gripper, 2, np.array([0, 0, 0, 0, 0, 0, 2.96]))
    with pytest.raises(ValueError, match="approach_speed"):
        precheck_screw_passes(make_params("screw", force=10.0, angle=1.0, approach_speed=0.0),
                              make_regrasp(regrasp_mode="retract"), 2, q)
    with pytest.raises(ValueError, match="asse"):
        precheck_screw_passes(make_params("screw", force=10.0, angle=1.0, axis_tool=(1, 0, 0)), gripper, 2, q)
    precheck_screw_passes(make_params("screw", force=10.0, angle=5.0), gripper, 4, np.array([0, 0, 0, 0, 0, 0, -2.9]))
