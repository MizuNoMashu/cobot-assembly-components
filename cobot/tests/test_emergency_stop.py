"""
Arresto software (POST /api/robot/stop).

Il robot è simulato: i loop di controllo girano davvero ma contro un finto
pylibfranka, e lo stop viene richiesto "da un altro thread" in mezzo al loop.
"""

import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

import franka_controller.gripper as gripper_module
import franka_controller.motion as motion_module
from franka_controller import FrankaRobot, RobotErrorType, RobotMode
from franka_controller.robot import EMERGENCY_STOP_MESSAGE, EmergencyStop

MAX_CYCLES = 300


# ---------------------------------------------------------------------------
# Finto pylibfranka
# ---------------------------------------------------------------------------


class FakeCommand:
    def __init__(self, values):
        self.values = list(values)
        self.motion_finished = False


class FakeLib:
    class ControlException(Exception):
        pass

    ControllerMode = SimpleNamespace(JointImpedance="ji", CartesianImpedance="ci")
    JointPositions = FakeCommand
    JointVelocities = FakeCommand
    CartesianPose = FakeCommand
    Torques = FakeCommand


class FakeState:
    def __init__(self):
        self.q = np.zeros(7)
        self.q_d = np.zeros(7)
        self.dq = np.zeros(7)
        self.tau_J_d = np.zeros(7)
        self.O_T_EE = np.eye(4).reshape(-1, order="F")
        self.O_T_EE_d = np.eye(4).reshape(-1, order="F")
        self.O_T_EE_c = np.eye(4).reshape(-1, order="F")
        self.O_F_ext_hat_K = np.zeros(6)
        self.K_F_ext_hat_K = np.zeros(6)
        self.cartesian_contact = np.zeros(6)
        self.cartesian_collision = np.zeros(6)
        self.control_command_success_rate = 1.0


class FakePeriod:
    def to_sec(self):
        return 0.001


class FakeControl:
    def __init__(self, hw):
        self.hw = hw
        self.cycles = 0
        self.written = []

    def readOnce(self):
        self.cycles += 1
        if self.cycles > MAX_CYCLES:
            pytest.fail("il loop non si è fermato dopo la richiesta di stop")
        if self.cycles == self.hw.stop_at:
            self.hw.franka.request_stop()  # come farebbe la route HTTP
        return FakeState(), FakePeriod()

    def writeOnce(self, command):
        self.written.append(command)


class FakeModel:
    def coriolis(self, state):
        return np.zeros(7)

    def zero_jacobian(self, state):
        return np.eye(6, 7)


class FakeHardware:
    """Fa le veci di pylibfranka.Robot."""

    def __init__(self):
        self.franka = None
        self.stop_at = 5
        self.controls = []
        self.stop_calls = 0
        self.collision_calls = []

    def _control(self, *_):
        control = FakeControl(self)
        self.controls.append(control)
        return control

    start_joint_position_control = _control
    start_cartesian_pose_control = _control
    start_joint_velocity_control = _control
    start_torque_control = _control

    def read_once(self):
        return FakeState()

    def load_model(self):
        return FakeModel()

    def set_collision_behavior(self, *thresholds):
        self.collision_calls.append(thresholds)

    def stop(self):
        self.stop_calls += 1

    def automatic_error_recovery(self):
        pass


class DummyGripper:
    def __init__(self, *_):
        self.calls = []

    def stop(self):
        self.calls.append("stop")


@pytest.fixture
def franka(monkeypatch):
    hw = FakeHardware()

    def fake_connect(self):
        self._robot = hw
        self.mode = RobotMode.READY

    monkeypatch.setattr(motion_module, "pylibfranka", FakeLib)
    monkeypatch.setattr(gripper_module, "GripperController", DummyGripper)
    monkeypatch.setattr(FrankaRobot, "_connect", fake_connect)
    robot = FrankaRobot(robot_ip="127.0.0.1")
    hw.franka = robot
    robot.hw = hw
    return robot


def assert_locked_by_stop(franka):
    assert franka.mode == RobotMode.ERROR_LOCKED
    assert franka.last_error.error_type == RobotErrorType.EMERGENCY_STOP
    assert franka.last_error.message == EMERGENCY_STOP_MESSAGE
    assert franka.last_error.recoverable is False


# ---------------------------------------------------------------------------
# Taglio immediato in ogni modalità di controllo
# ---------------------------------------------------------------------------


def test_joint_motion_stops_at_next_cycle_and_stays_locked(franka):
    ok = franka.motion.move_to_joint_positions([0.1] * 7, speed_factor=1.0)

    control = franka.hw.controls[0]
    assert ok is False
    assert control.cycles == franka.hw.stop_at  # fermo nello stesso ciclo
    assert control.written[-1].motion_finished is True
    assert control.written[-1].values == [0.0] * 7  # tiene la posizione misurata
    assert franka.hw.stop_calls == 0  # il controllo è già chiuso col comando finale
    assert_locked_by_stop(franka)


def test_commands_are_rejected_until_recovery(franka):
    franka.motion.move_to_joint_positions([0.1] * 7, speed_factor=1.0)

    with pytest.raises(RuntimeError, match="ERROR_LOCKED"):
        franka.motion.move_to_joint_positions([0.1] * 7, speed_factor=1.0)

    assert franka.automatic_error_recovery() is True
    assert franka.mode == RobotMode.READY
    assert franka.stop_requested is False


def test_fly_trajectory_stops(franka):
    ok = franka.motion.execute_trajectory_fly([[0.1] * 7], speed_factor=1.0, precheck=False)

    control = franka.hw.controls[0]
    assert ok is False
    assert control.cycles == franka.hw.stop_at
    assert control.written[-1].motion_finished is True
    assert_locked_by_stop(franka)


def test_cartesian_motion_stops_and_locks_instead_of_going_ready(franka):
    # una RuntimeError generica nel cartesiano riporterebbe a READY: lo stop no
    ok = franka.motion.move_to_cartesian_pose(0.3, 0.0, 0.4, speed_factor=1.0)

    control = franka.hw.controls[0]
    assert ok is False
    assert control.cycles == franka.hw.stop_at
    assert control.written[-1].motion_finished is True
    assert_locked_by_stop(franka)


def test_impedance_control_stops(franka):
    ok = franka.motion.impedance_control([0.1] * 7, duration=5.0)

    assert ok is False
    assert franka.hw.controls[0].cycles == franka.hw.stop_at
    assert franka.hw.stop_calls == 1  # torque control: chiusura con robot.stop()
    assert_locked_by_stop(franka)


def test_force_task_is_cut_immediately_and_thresholds_restored(franka):
    franka.hw.stop_at = 3

    ok = franka.motion.apply_force(10.0, duration=5.0)

    assert ok is False
    assert franka.hw.controls[0].cycles == 3
    assert franka.hw.stop_calls == 1
    # niente rilascio morbido: nessuna fase RELEASE, il controllo viene troncato
    assert franka.motion.last_force_result is None
    # soglie di collisione alzate per il task e poi ripristinate ai default
    assert len(franka.hw.collision_calls) == 2
    assert franka.hw.collision_calls[-1][0] == [10] * 7
    assert_locked_by_stop(franka)


def test_velocity_control_stops_and_zeroes_velocity(franka):
    ok = franka.motion.velocity_control(lambda state, dt: [0.0] * 7, duration=5.0)

    control = franka.hw.controls[0]
    assert ok is False
    assert control.cycles == franka.hw.stop_at
    assert control.written[-1].motion_finished is True
    assert control.written[-1].values == [0.0] * 7
    assert franka.mode == RobotMode.ERROR_LOCKED


def test_without_stop_the_loop_is_not_interrupted(franka):
    franka.hw.stop_at = None  # nessuno stop
    # impedenza: dopo molti cicli deve arrivare in fondo senza stop
    franka.hw.controls.clear()

    class Limited(FakeControl):
        def readOnce(self):
            if self.cycles >= 50:
                raise RuntimeError("fine del test")
            return super().readOnce()

    franka.hw._control = lambda *_: Limited(franka.hw)
    franka.hw.start_torque_control = franka.hw._control

    franka.motion.impedance_control([0.1] * 7, duration=5.0)

    assert not any(
        e.error_type == RobotErrorType.EMERGENCY_STOP for e in franka.errors
    )


def test_check_stop_locks_the_robot_between_steps(franka):
    franka.check_stop()  # nessuno stop: non fa nulla
    assert franka.mode == RobotMode.READY

    franka.request_stop()
    with pytest.raises(EmergencyStop):
        franka.check_stop()
    assert_locked_by_stop(franka)


# ---------------------------------------------------------------------------
# RobotManager: qualunque tipo di operazione, thread reali
# ---------------------------------------------------------------------------


@pytest.fixture
def manager(franka):
    from app.robot_manager import RobotManager

    RobotManager._instance = None
    mgr = RobotManager()
    mgr.robot = franka
    yield mgr
    RobotManager._instance = None


def wait_until(predicate, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_manager_stop_when_idle_does_nothing(manager, franka):
    result = manager.request_stop()

    assert result["stop_requested"] is False
    assert franka.stop_requested is False
    assert franka.mode == RobotMode.READY


def test_manager_stop_requires_connection(manager):
    manager.robot = None
    with pytest.raises(RuntimeError):
        manager.request_stop()


def test_manager_locks_robot_even_if_operation_ignores_the_flag(manager, franka):
    """Operazione che non legge mai il flag (es. homing): il blocco è comunque garantito."""
    running = threading.Event()
    release = threading.Event()

    def homing(progress_callback=None):
        running.set()
        release.wait(5)
        return True  # termina "con successo", ignorando lo stop

    assert manager.run_async(homing) is True
    assert running.wait(5)

    result = manager.request_stop()
    assert result == {
        "stop_requested": True,
        "operation": "homing",
        "message": result["message"],
    }
    assert manager.status_dict()["stop_requested"] is True
    assert franka.mode == RobotMode.READY  # finché l'operazione non finisce

    release.set()
    assert wait_until(lambda: not manager.is_busy)

    assert_locked_by_stop(franka)
    assert manager.status_dict()["last_operation"]["emergency_stop"] is True
    assert franka.gripper.calls == ["stop"]  # l'homing si interrompe anche sul gripper


def test_manager_marks_stopped_operation_as_failed(manager, franka):
    running = threading.Event()

    def long_motion(progress_callback=None):
        running.set()
        assert wait_until(lambda: franka.stop_requested)
        return False

    manager.run_async(long_motion)
    assert running.wait(5)
    manager.request_stop()
    assert wait_until(lambda: not manager.is_busy)

    last = manager.status_dict()["last_operation"]
    assert last["success"] is False
    assert last["error"] == EMERGENCY_STOP_MESSAGE
    assert_locked_by_stop(franka)


def test_stale_stop_does_not_affect_next_operation(manager, franka):
    franka.request_stop()  # residuo di uno stop precedente
    seen = []

    def next_op(progress_callback=None):
        seen.append(franka.stop_requested)
        return True

    manager.run_async(next_op)
    assert wait_until(lambda: not manager.is_busy)

    assert seen == [False]
    assert franka.mode == RobotMode.READY


# ---------------------------------------------------------------------------
# Route HTTP e pick-and-place
# ---------------------------------------------------------------------------

flask = pytest.importorskip("flask")


@pytest.fixture
def routes(monkeypatch):
    from app.routes import robot as robot_routes

    class FakeManager:
        is_connected = True
        is_busy = False
        _socketio = None
        robot = None
        started = []

        def request_stop(self):
            return {"stop_requested": True, "operation": "move_joints", "message": "ok"}

        def run_async(self, func, *args, **kwargs):
            self.started.append(func)
            return True

    fake = FakeManager()
    monkeypatch.setattr(robot_routes, "manager", fake)
    app = flask.Flask(__name__)
    app.register_blueprint(robot_routes.robot_bp)
    return SimpleNamespace(client=app.test_client(), manager=fake)


def test_stop_route_returns_manager_result(routes):
    r = routes.client.post("/api/robot/stop")

    assert r.status_code == 200
    assert r.json["stop_requested"] is True and r.json["operation"] == "move_joints"


def test_stop_route_when_not_connected(routes):
    routes.manager.is_connected = False

    assert routes.client.post("/api/robot/stop").status_code == 503


class PickPlaceRobot:
    def __init__(self, stop_during=None, fail_on=None):
        self.stop_requested = False
        self.calls = []
        self.recoveries = 0
        robot = self

        class Motion:
            def go_to_home(self, speed_factor=0.2, progress_callback=None):
                robot.calls.append("go_to_home")
                if stop_during == "go_to_home":
                    robot.stop_requested = True  # lo stop arriva durante il movimento
                return False if stop_during else True

            def move_to_joint_positions(self, *args, **kwargs):
                robot.calls.append("move")
                return True

        class Gripper:
            def open(self, **kwargs):
                robot.calls.append("gripper_open")
                if fail_on == "gripper_open":
                    raise RuntimeError("gripper guasto")

            def grasp(self, **kwargs):
                robot.calls.append("gripper_grasp")

            def release(self):
                robot.calls.append("gripper_release")

        self.motion, self.gripper = Motion(), Gripper()

    def check_stop(self):
        if self.stop_requested:
            raise EmergencyStop(EMERGENCY_STOP_MESSAGE)

    def automatic_error_recovery(self):
        self.recoveries += 1


def start_pick_and_place(routes, robot):
    routes.manager.robot = robot
    routes.manager.started.clear()
    assert routes.client.post("/api/motion/pick-and-place", json={}).status_code == 202
    return routes.manager.started[0]


def test_pick_and_place_does_not_continue_after_stop(routes):
    robot = PickPlaceRobot(stop_during="go_to_home")
    workflow = start_pick_and_place(routes, robot)

    assert workflow() is False

    # il gripper non deve aprirsi (potrebbe far cadere un pezzo) e niente recovery
    assert robot.calls == ["go_to_home"]
    assert robot.recoveries == 0


def test_pick_and_place_still_recovers_from_ordinary_errors(routes):
    robot = PickPlaceRobot(fail_on="gripper_open")
    workflow = start_pick_and_place(routes, robot)

    assert workflow() is False
    assert robot.recoveries == 1


def test_pick_and_place_completes_normally(routes):
    robot = PickPlaceRobot()
    workflow = start_pick_and_place(routes, robot)

    assert workflow() is True
    assert robot.recoveries == 0
