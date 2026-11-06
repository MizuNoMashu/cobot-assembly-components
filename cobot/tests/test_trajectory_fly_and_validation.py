import numpy as np
import pytest

from franka_controller.fly_path import (
    build_blended_path,
    plan_fly_trajectory,
    split_at_reversals,
)
from franka_controller.kinematics import (
    FR3_DQ_MAX,
    forward_kinematics_and_jacobian,
    jacobian_singular_values,
)
from franka_controller.trajectory_validation import (
    TrajectoryValidationError,
    ValidationLimits,
    validate_sampled_trajectory,
    validate_stop_and_go,
)

READY = np.array([0, -np.pi / 4, 0, -3 * np.pi / 4, 0, np.pi / 2, np.pi / 4])


def _line(delta, n):
    return [READY + delta * (i + 1) / n for i in range(n)]


def test_fk_matches_known_ready_pose():
    flange, _ = forward_kinematics_and_jacobian(READY)
    assert flange[0, :3, 3] == pytest.approx([0.307, 0.0, 0.59], abs=2e-3)


def test_sigma_min_small_near_stretched_elbow():
    stretched = np.array([0, 0, 0, -0.1518, 0, 1.5, 0])
    assert jacobian_singular_values(stretched)[0, -1] < 0.05
    assert jacobian_singular_values(READY)[0, -1] > 0.2


def test_stop_and_go_accepts_safe_trajectory():
    delta = np.array([0.2, 0.1, -0.1, 0.1, 0.0, 0.1, 0.0])
    report = validate_stop_and_go(READY, _line(delta, 5), [1.0] * 5)
    assert report["ok"] and report["min_sigma"] > 0.1


def test_stop_and_go_rejects_too_fast_segment():
    target = READY + np.array([1.5, 0, 0, 0, 0, 0, 0])
    with pytest.raises(TrajectoryValidationError, match="velocità"):
        validate_stop_and_go(READY, [target], [0.7])


def test_stop_and_go_rejects_joint_limit():
    target = READY.copy()
    target[3] = -0.1  # oltre q4_max = -0.1518
    with pytest.raises(TrajectoryValidationError, match="fuori da"):
        validate_stop_and_go(READY, [target], [10.0])


def test_stop_and_go_rejects_singularity_along_path_not_only_at_waypoints():
    # Entrambi i waypoint sono OK, ma il giunto 4 viene portato ad angoli
    # dove il braccio è quasi disteso a metà percorso: controlliamo i campioni.
    start = np.array([0, 0, 0, -1.0, 0, 1.5, 0])
    end = np.array([0, 0, 0, -0.2, 0, 1.5, 0])
    with pytest.raises(TrajectoryValidationError, match="singolarità"):
        validate_stop_and_go(start, [end], [10.0], ValidationLimits(min_sigma=0.06))


def test_precheck_timeout_is_fail_safe():
    with pytest.raises(TrajectoryValidationError, match="tempo massimo"):
        validate_stop_and_go(
            READY, _line(np.full(7, 0.01), 400), [1.0] * 400,
            ValidationLimits(timeout_sec=0.0, samples_per_segment=20),
        )


def test_blend_cuts_corner_and_stays_between_vertices():
    v = [np.zeros(7), np.eye(7)[0] * 1.0, np.eye(7)[0] * 1.0 + np.eye(7)[1] * 1.0]
    path = build_blended_path(v, blend_radius=0.2)
    assert path[0] == pytest.approx(v[0]) and path[-1] == pytest.approx(v[-1])
    # il vertice interno non viene toccato: distanza minima ~ 0.2/2*... > 0
    assert min(np.linalg.norm(path - v[1], axis=1)) > 0.02


def test_reversal_is_split_into_two_parts():
    a, b = np.zeros(7), np.eye(7)[0] * 0.5
    parts = split_at_reversals([a, b, a + 1e-3])
    assert len(parts) == 2


def test_fly_trajectory_is_continuous_and_respects_limits():
    waypoints = [READY + np.array([0.1 * i, 0.05 * np.sin(i), 0, 0.02 * i, 0, 0, 0]) for i in range(1, 9)]
    traj = plan_fly_trajectory(READY, waypoints, blend_radius=0.1, speed_factor=0.3)
    times, Q = traj.sample(dt=0.005)
    assert Q[0] == pytest.approx(READY, abs=1e-9)
    assert Q[-1] == pytest.approx(waypoints[-1], abs=1e-9)
    vel = np.abs(np.diff(Q, axis=0)) / np.diff(times)[:, None]
    assert np.all(vel / FR3_DQ_MAX < 0.8)
    # velocità non nulla nei punti intermedi: nessuno stop (solo agli estremi)
    speed = np.linalg.norm(vel, axis=1)
    assert speed[len(speed) // 3: 2 * len(speed) // 3].min() > 0.05
    assert speed[0] < speed.max() * 0.2 and speed[-1] < speed.max() * 0.2
    # la validazione sul campionato passa
    validate_sampled_trajectory(times[::4], Q[::4])


def test_fly_duration_is_stretched_not_shortened():
    waypoints = _line(np.array([2.0, 0, 0, 0, 0, 0, 0]), 4)
    traj = plan_fly_trajectory(READY, waypoints, requested_duration=0.5)
    assert traj.duration > 0.5
    assert traj.summary()["limited_by"][0] != "richiesta"


def test_fly_noop_when_already_there():
    assert plan_fly_trajectory(READY, [READY.copy()]).parts == []


# ---------------------------------------------------------------------------
# Integrazione MotionController con un robot finto (nessun hardware)
# ---------------------------------------------------------------------------


class _Period:
    def __init__(self, dt):
        self._dt = dt

    def to_sec(self):
        return self._dt


class _FakeControl:
    def __init__(self, q0, log):
        self.q = np.array(q0, dtype=float)
        self.log = log

    def readOnce(self):
        state = type("S", (), {})()
        state.q = self.q.tolist()
        state.dq = [0.0] * 7
        state.cartesian_contact = [0.0] * 6
        state.cartesian_collision = [0.0] * 6
        state.O_T_EE = np.eye(4).flatten(order="F").tolist()
        return state, _Period(0.001)

    def writeOnce(self, command):
        self.q = np.array(command.q)
        self.log.append((np.array(command.q), command.motion_finished))


class _FakeFranka:
    def __init__(self, q0):
        from franka_controller.robot import RobotMode

        self.mode = RobotMode.READY
        self.stop_requested = False
        self.q = np.array(q0, dtype=float)
        self.log = []
        self.locked = None
        control = _FakeControl(q0, self.log)
        self._control = control
        self.robot = type("R", (), {
            "start_joint_position_control": lambda _s, _m: control,
            "stop": lambda _s: None,
        })()

    def _ensure_can_command(self):
        pass

    def get_state(self):
        q = self._control.q
        return type("S", (), {"q": q.tolist(), "q_d": q.tolist()})()

    def _enter_error_locked(self, operation, exception, recoverable):
        self.locked = (operation, exception)
        return type("E", (), {"message": str(exception)})()


@pytest.fixture
def motion(monkeypatch):
    from franka_controller import motion as motion_module

    class JointPositions:
        def __init__(self, q):
            self.q = q
            self.motion_finished = False

    monkeypatch.setattr(motion_module.pylibfranka, "JointPositions", JointPositions, raising=False)
    monkeypatch.setattr(motion_module.pylibfranka, "ControlException", type("CE", (Exception,), {}), raising=False)
    franka = _FakeFranka(READY)
    return motion_module.MotionController(franka), franka


def test_execute_trajectory_fly_runs_end_to_end(motion):
    controller, franka = motion
    waypoints = [w.tolist() for w in _line(np.array([0.3, 0.1, 0, 0.1, 0, 0, 0]), 6)]
    assert controller.execute_trajectory_fly(waypoints, segment_duration=0.5, tolerance=0.01) is True
    q_last, finished = franka.log[-1]
    assert finished and q_last == pytest.approx(waypoints[-1], abs=1e-6)
    # una sola sessione di controllo, nessun comando con salti
    steps = np.abs(np.diff(np.array([q for q, _ in franka.log]), axis=0))
    assert steps.max() < 0.01
    assert franka.locked is None


def test_precheck_blocks_motion_before_any_command(motion):
    controller, franka = motion
    out_of_limits = READY.copy()
    out_of_limits[3] = -0.1
    too_fast = (READY + np.array([2.0, 0, 0, 0, 0, 0, 0])).tolist()
    with pytest.raises(Exception, match="pre-esecuzione"):
        controller.execute_trajectory_fly([out_of_limits.tolist()], segment_duration=5.0)
    with pytest.raises(Exception, match="pre-esecuzione"):
        controller.execute_trajectory([too_fast], segment_duration=0.2)
    assert franka.log == [] and franka.locked is None


def test_fly_slows_down_instead_of_rejecting_when_too_fast(motion):
    controller, franka = motion
    far = (READY + np.array([2.0, 0, 0, 0, 0, 0, 0])).tolist()
    assert controller.execute_trajectory_fly([far], segment_duration=0.2, tolerance=0.01)
    assert len(franka.log) > 1000  # ~1.5 s a 1 kHz invece dei 0.2 s richiesti
