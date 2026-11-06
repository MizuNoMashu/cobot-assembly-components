"""
Controller per il movimento del robot Franka usando pylibfranka.
"""

import traceback
import numpy as np
from typing import Any, List, Optional, Callable, Sequence, Tuple

import pylibfranka
from .robot import EMERGENCY_STOP_MESSAGE, EmergencyStop
from .utils import compute_minimum_jerk_trajectory
from .fly_path import plan_fly_trajectory
from .force_control import (
    ForceTaskController,
    ForceTaskFailed,
    ForceTaskParams,
    RegraspConfig,
    as_jacobian,
    make_params,
    precheck_screw_passes,
    precheck_wrist_range,
    wrist_rewind_target,
)
from .trajectory_validation import (
    ValidationLimits,
    validate_sampled_trajectory,
    validate_stop_and_go,
)




def _rpy_to_rotation_matrix(
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    """
    Converte RPY in una matrice di rotazione attiva.

    Convenzione:
        R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    Gli angoli sono espressi in radianti.
    """

    cr = np.cos(roll)
    sr = np.sin(roll)

    cp = np.cos(pitch)
    sp = np.sin(pitch)

    cy = np.cos(yaw)
    sy = np.sin(yaw)

    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ])

    Ry = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ])

    Rz = np.array([
        [cy, -sy, 0.0],
        [sy, cy, 0.0],
        [0.0, 0.0, 1.0],
    ])

    return Rz @ Ry @ Rx


def _rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    trace = np.trace(R)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    else:
        if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
    quat = np.array([w, x, y, z], dtype=float)
    return quat / np.linalg.norm(quat)


def _quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)

    if q.shape != (4,):
        raise ValueError(
            "Il quaternion deve avere shape (4,)."
        )

    norm_q = np.linalg.norm(q)

    if norm_q < 1e-12:
        raise ValueError(
            "Il quaternion ha norma nulla "
            "o troppo piccola."
        )

    w, x, y, z = q / norm_q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
    ], dtype=float)


def _slerp_quaternion(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:

    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)

    if q0.shape != (4,) or q1.shape != (4,):
        raise ValueError(
            "q0 e q1 devono avere shape (4,)."
        )

    if not 0.0 <= t <= 1.0:
        raise ValueError(
            "t deve appartenere all'intervallo [0, 1]."
        )

    norm_q0 = np.linalg.norm(q0)
    norm_q1 = np.linalg.norm(q1)

    if norm_q0 < 1e-12 or norm_q1 < 1e-12:
        raise ValueError(
            "I quaternion non possono avere "
            "norma nulla."
        )
    
    if norm_q0 < 1e-12 or norm_q1 < 1e-12:
        raise ValueError(
            "I quaternion non possono avere "
            "norma nulla."
        )

    q0 = q0 / norm_q0
    q1 = q1 / norm_q1

    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = np.clip(dot, -1.0, 1.0)

    # Per orientamenti quasi uguali si usa
    # un'interpolazione lineare normalizzata.
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)

    coefficient_0 = (
        np.sin((1.0 - t) * theta_0)
        / sin_theta_0
    )

    coefficient_1 = (
        np.sin(t * theta_0)
        / sin_theta_0
    )

    result = (
        coefficient_0 * q0
        + coefficient_1 * q1
    )

    return result / np.linalg.norm(result)

def _rotation_matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def _build_cartesian_pose(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> np.ndarray:
    pose = np.eye(4, dtype=float) #create a 4x4 identity matrix
    pose[:3, :3] = _rpy_to_rotation_matrix(roll, pitch, yaw)
    pose[:3, 3] = [x, y, z]
    return pose


def _pose_to_cartesian_command(pose: np.ndarray) -> pylibfranka.CartesianPose:
    return pylibfranka.CartesianPose(pose.reshape(-1, order="F").tolist())


class MotionController:
    def __init__(self, franka_robot):
        self.robot = franka_robot
        self.last_force_result: Optional[dict] = None

    # ------------------------------------------------------------
    # Internal utilities
    # ------------------------------------------------------------

    def _validate_joint_vector(self, values: List[float], name: str) -> None:
        if len(values) != 7:
            raise ValueError(f"{name} deve contenere 7 valori, ricevuti {len(values)}.")

    def _validate_speed_factor(self, speed_factor: float) -> None:
        if speed_factor <= 0.0 or speed_factor > 1.0:
            raise ValueError(
                f"speed_factor non valido: {speed_factor}. Deve essere nel range (0.0, 1.0]."
            )

    def _validate_duration(self, duration: float) -> None:
        if duration <= 0.0:
            raise ValueError(f"duration non valida: {duration}. Deve essere positiva.")

    # ------------------------------------------------------------
    # Joint position control
    # ------------------------------------------------------------

    def move_to_joint_positions(
        self,
        target_positions: List[float],
        speed_factor: float = 0.2,
        tolerance: float = 0.04,
        progress_callback: Optional[Callable[[dict], None]] = None,
        duration_override: Optional[float] = None,
    ) -> bool:
        """Muove i sette giunti verso una configurazione target."""

        self.robot._ensure_can_command()

        active_control = None
        control_finished = False

        try:
            self._validate_joint_vector(
                target_positions,
                "target_positions",
            )
            self._validate_speed_factor(speed_factor)

            if tolerance <= 0.0:
                raise ValueError("tolerance deve essere positiva.")
            if duration_override is not None and (
                not np.isfinite(duration_override) or duration_override <= 0.0
            ):
                raise ValueError("duration_override deve essere un numero finito positivo.")

            target = np.asarray(target_positions, dtype=float)

            initial_state = self.robot.get_state()
            q_start = np.asarray(initial_state.q_d, dtype=float)

            print(
                "[MOTION] Target:",
                np.round(target, 3).tolist(),
            )
            print(
                "[MOTION] Posizione iniziale:",
                np.round(q_start, 3).tolist(),
            )

            self.robot.mode = self.robot.mode.__class__.RUNNING

            active_control = (
                self.robot.robot.start_joint_position_control(
                    pylibfranka.ControllerMode.JointImpedance
                )
            )

            duration = duration_override or (5.0 / speed_factor)
            time_elapsed = 0.0
            iteration = 0

            while True:
                robot_state, period = active_control.readOnce()

                dt = period.to_sec()
                time_elapsed += dt
                iteration += 1

                q_measured = np.asarray(
                    robot_state.q,
                    dtype=float,
                )
                dq_measured = np.asarray(
                    robot_state.dq,
                    dtype=float,
                )

                cartesian_contact = np.asarray(
                    robot_state.cartesian_contact,
                    dtype=bool,
                )
                cartesian_collision = np.asarray(
                    robot_state.cartesian_collision,
                    dtype=bool,
                )

                if self.robot.stop_requested:
                    stop_command = pylibfranka.JointPositions(
                        q_measured.tolist()
                    )
                    stop_command.motion_finished = True
                    active_control.writeOnce(stop_command)
                    control_finished = True
                    raise EmergencyStop(EMERGENCY_STOP_MESSAGE)

                if (
                    np.any(cartesian_contact)
                    or np.any(cartesian_collision)
                ):
                    stop_command = pylibfranka.JointPositions(
                        q_measured.tolist()
                    )
                    stop_command.motion_finished = True
                    active_control.writeOnce(stop_command)
                    control_finished = True

                    if np.any(cartesian_collision):
                        raise RuntimeError(
                            "Collisione cartesiana rilevata."
                        )

                    raise RuntimeError(
                        "Contatto cartesiano rilevato."
                    )

                progress = min(
                    time_elapsed / duration,
                    1.0,
                )

                s = compute_minimum_jerk_trajectory(
                    progress
                )

                q_command = (
                    q_start
                    + s * (target - q_start)
                )

                command = pylibfranka.JointPositions(
                    q_command.tolist()
                )

                command.motion_finished = (
                    progress >= 1.0
                )

                active_control.writeOnce(command)

                # Telemetria limitata per non rallentare
                # eccessivamente il ciclo di controllo.
                if iteration % 100 == 0:
                    max_error = float(
                        np.max(
                            np.abs(
                                target - q_measured
                            )
                        )
                    )

                    print(
                        f"[MOTION] "
                        f"{progress * 100:5.1f}% | "
                        f"errore massimo: "
                        f"{max_error:.5f} rad"
                    )

                # Anche il callback viene limitato.
                if (
                    progress_callback is not None
                    and (
                        iteration % 50 == 0
                        or progress >= 1.0
                    )
                ):
                    tcp_position = np.asarray(
                        robot_state.O_T_EE,
                        dtype=float,
                    ).reshape((4, 4), order="F")[:3, 3]
                    progress_callback({
                        "type": "motion_progress",
                        "iteration": iteration,
                        "progress": round(
                            progress * 100.0,
                            2,
                        ),
                        "time_elapsed": round(
                            time_elapsed,
                            4,
                        ),
                        "delta_time": round(dt, 6),
                        "q_measured": (
                            np.round(q_measured, 6)
                            .tolist()
                        ),
                        "dq_measured": (
                            np.round(dq_measured, 6)
                            .tolist()
                        ),
                        "q_command": (
                            np.round(q_command, 6)
                            .tolist()
                        ),
                        "tcp_position": {
                            "x": float(tcp_position[0]),
                            "y": float(tcp_position[1]),
                            "z": float(tcp_position[2]),
                        },
                        "contact": (
                            cartesian_contact.tolist()
                        ),
                        "collision": (
                            cartesian_collision.tolist()
                        ),
                    })

                if progress >= 1.0:
                    control_finished = True
                    break

            final_state = self.robot.get_state()
            q_final = np.asarray(
                final_state.q,
                dtype=float,
            )

            final_error = float(
                np.max(
                    np.abs(target - q_final)
                )
            )

            print(
                "[MOTION] Posizione finale:",
                np.round(q_final, 4).tolist(),
            )
            print(
                f"[MOTION] Errore massimo finale: "
                f"{final_error:.6f} rad"
            )

            if final_error > tolerance:
                raise RuntimeError(
                    "Tolleranza non rispettata: "
                    f"errore={final_error:.6f} rad, "
                    f"tolleranza={tolerance:.6f} rad."
                )

            self.robot.mode = (
                self.robot.mode.__class__.READY
            )

            print("✓ Movimento completato")
            return True

        except pylibfranka.ControlException as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/move_to_joint_positions "
                    "(ControlException)"
                ),
                exception=exc,
                recoverable=False,
            )

            print(
                "✗ ControlException durante il movimento: "
                f"{error.message}"
            )
            traceback.print_exc()
            return False

        except RuntimeError as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/move_to_joint_positions "
                    "(RuntimeError)"
                ),
                exception=exc,
                recoverable=False,
            )

            print(
                "✗ Movimento interrotto: "
                f"{error.message}"
            )
            traceback.print_exc()
            return False

        except Exception as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/move_to_joint_positions "
                    "(Exception)"
                ),
                exception=exc,
                recoverable=True,
            )

            print(
                "✗ Errore durante il movimento: "
                f"{error.message}"
            )
            traceback.print_exc()
            return False

        finally:
            if (
                active_control is not None
                and not control_finished
            ):
                try:
                    self.robot.robot.stop()
                except Exception as cleanup_exc:
                    print(
                        "[MOTION] Arresto fallito: "
                        f"{cleanup_exc}"
                    )
    # ------------------------------------------------------------
    # Relative joint motion
    # ------------------------------------------------------------

    def move_relative(
        self,
        delta_positions: List[float],
        speed_factor: float = 0.2,
        progress_callback: Optional[Callable[[dict], None]] = None,
    ) -> bool:
        self.robot._ensure_can_command()

        try:
            self._validate_joint_vector(delta_positions, "delta_positions")
            self._validate_speed_factor(speed_factor)

            current = self.robot.get_current_joint_positions()
            target = current + np.array(delta_positions)

            print(f"Movimento relativo: Δ{np.round(delta_positions, 3).tolist()}")
            print(f"Posizione target: {np.round(target, 3).tolist()}")

            return self.move_to_joint_positions(
                target.tolist(), speed_factor, progress_callback=progress_callback
            )

        except Exception as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_relative",
                exception=e,
                recoverable=True,
            )
            print(f"✗ Errore durante movimento relativo: {error.message}")
            return False

        # ------------------------------------------------------------
    # Trajectory execution (es. traiettoria pianificata esternamente da MoveIt)
    # ------------------------------------------------------------

    def execute_trajectory(
        self,
        waypoints: List[List[float]],
        speed_factor: float = 0.2,
        tolerance: float = 0.04,
        progress_callback: Optional[Callable[[dict], None]] = None,
        segment_duration: Optional[float] = None,
        precheck: bool = True,
        precheck_limits: Optional[ValidationLimits] = None,
    ) -> bool:
        """
        Esegue in sequenza una lista di waypoint articolari (7 valori ciascuno),
        tipicamente i punti di una traiettoria pianificata da moveit_api.

        Ogni segmento riusa move_to_joint_positions, quindi eredita le stesse
        verifiche di sicurezza (collision/contact) e la stessa interpolazione
        minimum-jerk. Si interrompe al primo segmento fallito senza tentare i
        successivi. Si assume che l'ordine dei 7 valori corrisponda all'ordine
        articolare usato altrove in questo modulo (fr3_joint1..7).

        Con precheck=True (default), prima di muovere il robot l'intera
        traiettoria viene validata (limiti di giunto, velocità/accelerazioni,
        vicinanza a singolarità lungo il percorso). Se la validazione fallisce
        o supera il suo budget di tempo viene sollevata
        TrajectoryValidationError e il robot non si muove.
        """
        for waypoint in waypoints:
            self._validate_joint_vector(waypoint, "waypoint")
        self._validate_speed_factor(speed_factor)
        if tolerance <= 0.0:
            raise ValueError("tolerance deve essere positiva.")

        if segment_duration is not None and (
            not np.isfinite(segment_duration) or segment_duration <= 0.0
        ):
            raise ValueError("segment_duration deve essere un numero finito positivo.")

        if precheck:
            self.robot._ensure_can_command()
            q_now = np.asarray(self.robot.get_state().q_d, dtype=float)
            duration_each = segment_duration or (5.0 / speed_factor)
            report = validate_stop_and_go(
                q_now,
                waypoints,
                [duration_each] * len(waypoints),
                precheck_limits,
            )
            print(
                f"[TRAJECTORY] Pre-check OK in {report['elapsed_ms']} ms | "
                f"sigma_min={report['min_sigma']:.4f} | "
                f"vel max={report['max_velocity_ratio'] * 100:.0f}% | "
                f"acc max={report['max_acceleration_ratio'] * 100:.0f}%"
            )

        duration_note = (
            f", {segment_duration:.3f} s per waypoint"
            if segment_duration is not None
            else f", durata legacy {5.0 / speed_factor:.3f} s per waypoint"
        )
        print(f"[TRAJECTORY] Esecuzione di {len(waypoints)} waypoint{duration_note}")

        for index, waypoint in enumerate(waypoints):
            print(
                f"[TRAJECTORY] Waypoint {index + 1}/{len(waypoints)}: "
                f"{np.round(waypoint, 3).tolist()}"
            )
            success = self.move_to_joint_positions(
                waypoint,
                speed_factor=speed_factor,
                tolerance=tolerance,
                duration_override=segment_duration,
                progress_callback=progress_callback,
            )
            if not success:
                print(
                    f"[TRAJECTORY] ✗ Fallito al waypoint {index + 1}/{len(waypoints)}, "
                    "traiettoria interrotta"
                )
                return False

        print("[TRAJECTORY] ✓ Traiettoria completata")
        return True

    # ------------------------------------------------------------
    # Fly motion (fly-by / continuous path): nessuno stop nei punti intermedi
    # ------------------------------------------------------------

    def execute_trajectory_fly(
        self,
        waypoints: List[List[float]],
        speed_factor: float = 0.2,
        tolerance: float = 0.04,
        progress_callback: Optional[Callable[[dict], None]] = None,
        segment_duration: Optional[float] = None,
        blend_radius: float = 0.15,
        precheck: bool = True,
        precheck_limits: Optional[ValidationLimits] = None,
    ) -> bool:
        """
        Esegue i waypoint articolari come un unico percorso continuo (fly-by):
        il robot non si ferma nei punti intermedi ma li "vola" curvando entro
        una zona di raccordo di raggio `blend_radius` (rad, spazio dei giunti).
        Si ferma solo al punto finale (e dove il percorso si inverte quasi del
        tutto, perché lì non si può volare).

        A differenza di execute_trajectory (un movimento fermo→fermo per
        waypoint) usa UNA sola sessione di controllo in posizione e una legge
        temporale a velocità continua: rampa di accelerazione smooth, velocità
        di crociera, rampa di decelerazione.

        Tempi:
          - con segment_duration la durata totale richiesta è
            len(waypoints) * segment_duration (stessa durata complessiva del
            punto-punto, ma senza soste). Se i limiti di velocità/accelerazione
            dei giunti o la curvatura dei raccordi non la permettono, la durata
            viene ALLUNGATA, mai accorciata;
          - senza segment_duration speed_factor è la frazione del limite di
            velocità dei giunti usata come velocità di crociera.

        Con precheck=True (default) il percorso raccordato effettivo, campionato
        nel tempo, viene validato prima di muovere il robot (limiti di giunto,
        velocità, singolarità) e viene sollevata TrajectoryValidationError se
        non è sicuro o se il pre-check supera il suo budget di tempo.
        """
        if not waypoints:
            raise ValueError("waypoints non può essere vuoto.")
        for waypoint in waypoints:
            self._validate_joint_vector(waypoint, "waypoint")
        self._validate_speed_factor(speed_factor)
        if tolerance <= 0.0:
            raise ValueError("tolerance deve essere positiva.")
        if not np.isfinite(blend_radius) or blend_radius < 0.0:
            raise ValueError("blend_radius deve essere un numero finito >= 0.")
        if segment_duration is not None and (
            not np.isfinite(segment_duration) or segment_duration <= 0.0
        ):
            raise ValueError("segment_duration deve essere un numero finito positivo.")

        self.robot._ensure_can_command()

        limits = precheck_limits or ValidationLimits.from_env()
        q_start = np.asarray(self.robot.get_state().q_d, dtype=float)
        requested_duration = (
            segment_duration * len(waypoints) if segment_duration is not None else None
        )

        trajectory = plan_fly_trajectory(
            q_start,
            waypoints,
            blend_radius=blend_radius,
            speed_factor=speed_factor,
            requested_duration=requested_duration,
            limits=limits,
        )
        if not trajectory.parts:
            print("[FLY] Il robot è già nella posizione richiesta: nessun movimento.")
            return True

        summary = trajectory.summary()
        total_duration = trajectory.duration
        print(
            f"[FLY] Percorso: {len(waypoints)} waypoint, raccordo {blend_radius:.3f} rad, "
            f"durata {total_duration:.2f} s"
            + (f" (richiesta {requested_duration:.2f} s)" if requested_duration else "")
            + f", {summary['num_parts']} tratto/i, limite: {summary['limited_by']}"
        )

        if precheck:
            times, samples = trajectory.sample(dt=0.02)
            report = validate_sampled_trajectory(times, samples, limits)
            print(
                f"[FLY] Pre-check OK in {report['elapsed_ms']} ms | "
                f"sigma_min={report['min_sigma']:.4f} | "
                f"vel max={report['max_velocity_ratio'] * 100:.0f}%"
            )

        target = trajectory.end_point
        active_control = None
        control_finished = False

        try:
            self.robot.mode = self.robot.mode.__class__.RUNNING
            active_control = self.robot.robot.start_joint_position_control(
                pylibfranka.ControllerMode.JointImpedance
            )

            time_elapsed = 0.0
            iteration = 0

            while True:
                robot_state, period = active_control.readOnce()
                dt = period.to_sec()
                time_elapsed += dt
                iteration += 1

                q_measured = np.asarray(robot_state.q, dtype=float)
                cartesian_contact = np.asarray(robot_state.cartesian_contact, dtype=bool)
                cartesian_collision = np.asarray(robot_state.cartesian_collision, dtype=bool)

                if self.robot.stop_requested:
                    stop_command = pylibfranka.JointPositions(q_measured.tolist())
                    stop_command.motion_finished = True
                    active_control.writeOnce(stop_command)
                    control_finished = True
                    raise EmergencyStop(EMERGENCY_STOP_MESSAGE)

                if np.any(cartesian_contact) or np.any(cartesian_collision):
                    stop_command = pylibfranka.JointPositions(q_measured.tolist())
                    stop_command.motion_finished = True
                    active_control.writeOnce(stop_command)
                    control_finished = True
                    if np.any(cartesian_collision):
                        raise RuntimeError("Collisione cartesiana rilevata.")
                    raise RuntimeError("Contatto cartesiano rilevato.")

                finished = time_elapsed >= total_duration
                q_command = trajectory.q_at(min(time_elapsed, total_duration))

                command = pylibfranka.JointPositions(q_command.tolist())
                command.motion_finished = finished
                active_control.writeOnce(command)

                progress = min(time_elapsed / total_duration, 1.0)

                if iteration % 100 == 0:
                    max_error = float(np.max(np.abs(q_command - q_measured)))
                    print(
                        f"[FLY] {progress * 100:5.1f}% | "
                        f"errore max comando-misura: {max_error:.5f} rad"
                    )

                if progress_callback is not None and (iteration % 50 == 0 or finished):
                    tcp_position = np.asarray(
                        robot_state.O_T_EE, dtype=float
                    ).reshape((4, 4), order="F")[:3, 3]
                    progress_callback({
                        "type": "motion_progress",
                        "mode": "fly",
                        "iteration": iteration,
                        "progress": round(progress * 100.0, 2),
                        "time_elapsed": round(time_elapsed, 4),
                        "delta_time": round(dt, 6),
                        "q_measured": np.round(q_measured, 6).tolist(),
                        "dq_measured": np.round(
                            np.asarray(robot_state.dq, dtype=float), 6
                        ).tolist(),
                        "q_command": np.round(q_command, 6).tolist(),
                        "tcp_position": {
                            "x": float(tcp_position[0]),
                            "y": float(tcp_position[1]),
                            "z": float(tcp_position[2]),
                        },
                        "contact": cartesian_contact.tolist(),
                        "collision": cartesian_collision.tolist(),
                    })

                if finished:
                    control_finished = True
                    break

            q_final = np.asarray(self.robot.get_state().q, dtype=float)
            final_error = float(np.max(np.abs(target - q_final)))
            print(f"[FLY] Errore massimo finale: {final_error:.6f} rad")
            if final_error > tolerance:
                raise RuntimeError(
                    "Tolleranza non rispettata: "
                    f"errore={final_error:.6f} rad, tolleranza={tolerance:.6f} rad."
                )

            self.robot.mode = self.robot.mode.__class__.READY
            print("✓ Fly motion completata")
            return True

        except pylibfranka.ControlException as exc:
            error = self.robot._enter_error_locked(
                operation="motion/execute_trajectory_fly (ControlException)",
                exception=exc,
                recoverable=False,
            )
            print(f"✗ ControlException durante la fly motion: {error.message}")
            traceback.print_exc()
            return False

        except RuntimeError as exc:
            error = self.robot._enter_error_locked(
                operation="motion/execute_trajectory_fly (RuntimeError)",
                exception=exc,
                recoverable=False,
            )
            print(f"✗ Fly motion interrotta: {error.message}")
            traceback.print_exc()
            return False

        except Exception as exc:
            error = self.robot._enter_error_locked(
                operation="motion/execute_trajectory_fly (Exception)",
                exception=exc,
                recoverable=True,
            )
            print(f"✗ Errore durante la fly motion: {error.message}")
            traceback.print_exc()
            return False

        finally:
            if active_control is not None and not control_finished:
                try:
                    self.robot.robot.stop()
                except Exception as cleanup_exc:
                    print(f"[FLY] Arresto fallito: {cleanup_exc}")

    # ------------------------------------------------------------
    # Cartesian placeholder
    # ------------------------------------------------------------

    def move_to_cartesian_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        speed_factor: float = 0.2,
        tolerance: float = 0.01,
        orientation_tolerance: float = 0.05,
        settling_timeout: float = 5.0,
        progress_callback: Optional[
            Callable[[dict], None]
        ] = None,
        ) -> bool:
        """
        Muove l'end-effector verso una posa cartesiana assoluta.

        Unità:
            x, y, z                 metri
            roll, pitch, yaw        radianti
            tolerance               metri
            orientation_tolerance   radianti
            settling_timeout        secondi

        Convenzione RPY:
            R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

        Comportamento:
            1. Genera una traiettoria minimum-jerk.
            2. Al termine della durata nominale continua a comandare
            la posa target.
            3. Termina quando posizione e orientamento entrano nelle
            rispettive tolleranze.
            4. Se scade settling_timeout, termina con warning e
            restituisce False senza impostare ERROR_LOCKED.
            5. Collisioni e ControlException impostano ERROR_LOCKED.
        """

        self.robot._ensure_can_command()

        active_control = None
        control_finished = False
        last_command_pose = None

        collision_detected = False
        contact_detected = False

        try:
            # ============================================================
            # Validazione
            # ============================================================

            self._validate_speed_factor(speed_factor)

            if not np.isfinite(tolerance) or tolerance <= 0.0:
                raise ValueError(
                    "tolerance deve essere positiva e finita."
                )

            if (
                not np.isfinite(orientation_tolerance)
                or orientation_tolerance <= 0.0
            ):
                raise ValueError(
                    "orientation_tolerance deve essere "
                    "positiva e finita."
                )

            if (
                not np.isfinite(settling_timeout)
                or settling_timeout < 0.0
            ):
                raise ValueError(
                    "settling_timeout deve essere finito e "
                    "maggiore o uguale a zero."
                )

            target_values = np.asarray(
                [x, y, z, roll, pitch, yaw],
                dtype=float,
            )

            if not np.all(np.isfinite(target_values)):
                raise ValueError(
                    "La posa target contiene valori non finiti."
                )

            # ============================================================
            # Costruzione target
            # ============================================================

            target_pose = _build_cartesian_pose(
                x=x,
                y=y,
                z=z,
                roll=roll,
                pitch=pitch,
                yaw=yaw,
            )

            target_position = target_pose[:3, 3].copy()
            target_rotation = target_pose[:3, :3].copy()

            target_quat = _rotation_matrix_to_quaternion(
                target_rotation
            )

            print(
                "[MOTION] Posizione target:",
                np.round(target_position, 6).tolist(),
            )

            print(
                "[MOTION] Orientamento target RPY:",
                np.round([roll, pitch, yaw], 6).tolist(),
            )

            print(
                f"[MOTION] Speed factor: {speed_factor:.3f}"
            )

            print(
                f"[MOTION] Tolleranza posizione: "
                f"{tolerance:.6f} m"
            )

            print(
                f"[MOTION] Tolleranza orientamento: "
                f"{orientation_tolerance:.6f} rad "
                f"({np.rad2deg(orientation_tolerance):.3f}°)"
            )

            print(
                f"[MOTION] Timeout assestamento: "
                f"{settling_timeout:.3f} s"
            )

            # ============================================================
            # Avvio controllo cartesiano
            # ============================================================

            self.robot.mode = self.robot.mode.__class__.RUNNING

            active_control = (
                self.robot.robot.start_cartesian_pose_control(
                    pylibfranka.ControllerMode.JointImpedance
                )
            )

            print("[MOTION] Controllo cartesiano avviato.")

            # ============================================================
            # Lettura posa iniziale
            # ============================================================

            initial_state, _ = active_control.readOnce()

            if hasattr(initial_state, "O_T_EE_d"):
                initial_pose_data = initial_state.O_T_EE_d
            else:
                initial_pose_data = initial_state.O_T_EE

            initial_pose = np.asarray(
                initial_pose_data,
                dtype=float,
            ).reshape((4, 4), order="F")

            initial_position = initial_pose[:3, 3].copy()
            initial_rotation = initial_pose[:3, :3].copy()

            initial_quat = _rotation_matrix_to_quaternion(
                initial_rotation
            )

            last_command_pose = initial_pose.copy()

            print(
                "[MOTION] Posizione iniziale:",
                np.round(initial_position, 6).tolist(),
            )

            initial_position_distance = float(
                np.linalg.norm(
                    target_position - initial_position
                )
            )

            initial_quaternion_dot = abs(
                float(np.dot(initial_quat, target_quat))
            )

            initial_orientation_distance = float(
                2.0
                * np.arccos(
                    np.clip(
                        initial_quaternion_dot,
                        0.0,
                        1.0,
                    )
                )
            )

            print(
                f"[MOTION] Spostamento richiesto: "
                f"{initial_position_distance:.6f} m"
            )

            print(
                f"[MOTION] Rotazione richiesta: "
                f"{initial_orientation_distance:.6f} rad "
                f"({np.rad2deg(initial_orientation_distance):.3f}°)"
            )

            # Mantiene la stessa legge temporale del metodo originale.
            trajectory_duration = 5.0 / speed_factor

            maximum_duration = (
                trajectory_duration + settling_timeout
            )

            print(
                f"[MOTION] Durata nominale: "
                f"{trajectory_duration:.3f} s"
            )

            # ============================================================
            # Variabili del controllo
            # ============================================================

            time_elapsed = 0.0
            iteration_count = 0

            settling_warning_printed = False
            settling_timeout_reached = False
            target_reached = False

            last_robot_pose = initial_pose.copy()

            last_position_error = initial_position_distance
            last_orientation_error = (
                initial_orientation_distance
            )

            # ============================================================
            # Ciclo real-time
            # ============================================================

            while True:
                robot_state, delta_time = (
                    active_control.readOnce()
                )

                dt = float(delta_time.to_sec())

                if not np.isfinite(dt) or dt < 0.0:
                    raise RuntimeError(
                        f"Delta time non valido: {dt}."
                    )

                time_elapsed += dt
                iteration_count += 1

                # --------------------------------------------------------
                # Posa misurata: O_T_EE
                # --------------------------------------------------------

                robot_pose = np.asarray(
                    robot_state.O_T_EE,
                    dtype=float,
                ).reshape((4, 4), order="F")

                robot_position = robot_pose[:3, 3].copy()
                robot_rotation = robot_pose[:3, :3].copy()

                robot_quat = _rotation_matrix_to_quaternion(
                    robot_rotation
                )

                last_robot_pose = robot_pose.copy()

                # --------------------------------------------------------
                # Ultima posa comandata: O_T_EE_c
                # --------------------------------------------------------

                commanded_robot_pose = np.asarray(
                    robot_state.O_T_EE_c,
                    dtype=float,
                ).reshape((4, 4), order="F")

                commanded_robot_position = (
                    commanded_robot_pose[:3, 3].copy()
                )

                # --------------------------------------------------------
                # Posa desiderata interna: O_T_EE_d
                # --------------------------------------------------------

                desired_robot_pose = np.asarray(
                    robot_state.O_T_EE_d,
                    dtype=float,
                ).reshape((4, 4), order="F")

                desired_robot_position = (
                    desired_robot_pose[:3, 3].copy()
                )

                control_command_success_rate = float(
                    robot_state.control_command_success_rate
                )

                # --------------------------------------------------------
                # Contatto e collisione
                # --------------------------------------------------------

                cartesian_contact = np.asarray(
                    robot_state.cartesian_contact,
                    dtype=bool,
                )

                cartesian_collision = np.asarray(
                    robot_state.cartesian_collision,
                    dtype=bool,
                )

                if self.robot.stop_requested:
                    stop_command = _pose_to_cartesian_command(
                        last_command_pose
                    )

                    stop_command.motion_finished = True

                    active_control.writeOnce(stop_command)
                    control_finished = True

                    raise EmergencyStop(EMERGENCY_STOP_MESSAGE)

                if np.any(cartesian_collision):
                    collision_detected = True

                    stop_command = _pose_to_cartesian_command(
                        last_command_pose
                    )

                    stop_command.motion_finished = True

                    active_control.writeOnce(stop_command)
                    control_finished = True

                    raise RuntimeError(
                        "Collisione cartesiana rilevata: "
                        "movimento interrotto."
                    )

                if np.any(cartesian_contact):
                    contact_detected = True

                    stop_command = _pose_to_cartesian_command(
                        last_command_pose
                    )

                    stop_command.motion_finished = True

                    active_control.writeOnce(stop_command)
                    control_finished = True

                    raise RuntimeError(
                        "Contatto cartesiano rilevato: "
                        "movimento interrotto."
                    )

                # --------------------------------------------------------
                # Fase temporale
                # --------------------------------------------------------

                trajectory_complete = (
                    time_elapsed >= trajectory_duration
                )

                progress = min(
                    time_elapsed / trajectory_duration,
                    1.0,
                )

                s = compute_minimum_jerk_trajectory(
                    progress
                )

                # --------------------------------------------------------
                # Comando cartesiano
                # --------------------------------------------------------

                if not trajectory_complete:
                    position_command = (
                        initial_position
                        + s
                        * (
                            target_position
                            - initial_position
                        )
                    )

                    quaternion_command = _slerp_quaternion(
                        initial_quat,
                        target_quat,
                        s,
                    )

                    rotation_command = (
                        _quaternion_to_rotation_matrix(
                            quaternion_command
                        )
                    )

                    phase = "TRAJECTORY"

                else:
                    # Mantiene esattamente il riferimento target durante
                    # la fase di assestamento.
                    position_command = target_position.copy()
                    rotation_command = target_rotation.copy()

                    phase = "SETTLING"

                    if not settling_warning_printed:
                        print(
                            "⚠ [MOTION] Durata nominale "
                            "superata. Il robot sta completando "
                            "l'assestamento verso il target."
                        )

                        settling_warning_printed = True

                pose_command = np.eye(4, dtype=float)
                pose_command[:3, :3] = rotation_command
                pose_command[:3, 3] = position_command

                last_command_pose = pose_command.copy()

                # --------------------------------------------------------
                # Errori finali misurati
                # --------------------------------------------------------

                position_error = float(
                    np.linalg.norm(
                        target_position - robot_position
                    )
                )

                quaternion_dot = abs(
                    float(np.dot(robot_quat, target_quat))
                )

                quaternion_dot = float(
                    np.clip(
                        quaternion_dot,
                        0.0,
                        1.0,
                    )
                )

                orientation_error = float(
                    2.0 * np.arccos(quaternion_dot)
                )

                last_position_error = position_error
                last_orientation_error = orientation_error

                # Si termina per successo solo dopo che la traiettoria
                # nominale è stata completata.
                target_reached = (
                    trajectory_complete
                    and position_error <= tolerance
                    and orientation_error
                    <= orientation_tolerance
                )

                settling_timeout_reached = (
                    trajectory_complete
                    and time_elapsed >= maximum_duration
                    and not target_reached
                )

                finish_control = (
                    target_reached
                    or settling_timeout_reached
                )

                # --------------------------------------------------------
                # Errori diagnostici
                # --------------------------------------------------------

                # O_T_EE_c e O_T_EE_d descrivono il comando elaborato
                # nel ciclo precedente rispetto a quello che verrà
                # inviato tra poco.
                software_to_commanded_error = float(
                    np.linalg.norm(
                        position_command
                        - commanded_robot_position
                    )
                )

                commanded_to_desired_error = float(
                    np.linalg.norm(
                        commanded_robot_position
                        - desired_robot_position
                    )
                )

                desired_to_measured_error = float(
                    np.linalg.norm(
                        desired_robot_position
                        - robot_position
                    )
                )

                target_to_commanded_error = float(
                    np.linalg.norm(
                        target_position
                        - commanded_robot_position
                    )
                )

                target_to_desired_error = float(
                    np.linalg.norm(
                        target_position
                        - desired_robot_position
                    )
                )

                # --------------------------------------------------------
                # Creazione comando pylibfranka
                # --------------------------------------------------------

                cartesian_command = (
                    _pose_to_cartesian_command(
                        pose_command
                    )
                )

                cartesian_command.motion_finished = (
                    finish_control
                )

                # --------------------------------------------------------
                # Logging sintetico
                # --------------------------------------------------------

                if (
                    iteration_count % 100 == 0
                    or finish_control
                ):
                    print(
                        f"[MOTION][{phase}] "
                        f"{progress * 100:5.1f}% | "
                        f"errore posizione: "
                        f"{position_error:.6f} m | "
                        f"errore orientamento: "
                        f"{orientation_error:.6f} rad "
                        f"({np.rad2deg(orientation_error):.3f}°)"
                    )

                # --------------------------------------------------------
                # Diagnostica dettagliata
                # --------------------------------------------------------

                if (
                    iteration_count % 1000 == 0
                    or finish_control
                ):
                    print("[DEBUG CARTESIAN]")

                    print(
                        "  Target finale:       ",
                        np.round(
                            target_position,
                            6,
                        ).tolist(),
                    )

                    print(
                        "  Comando software:    ",
                        np.round(
                            position_command,
                            6,
                        ).tolist(),
                    )

                    print(
                        "  O_T_EE_c comandata:  ",
                        np.round(
                            commanded_robot_position,
                            6,
                        ).tolist(),
                    )

                    print(
                        "  O_T_EE_d desiderata: ",
                        np.round(
                            desired_robot_position,
                            6,
                        ).tolist(),
                    )

                    print(
                        "  O_T_EE misurata:     ",
                        np.round(
                            robot_position,
                            6,
                        ).tolist(),
                    )

                    print(
                        "  Software → command:  "
                        f"{software_to_commanded_error:.6f} m"
                    )

                    print(
                        "  Command → desired:   "
                        f"{commanded_to_desired_error:.6f} m"
                    )

                    print(
                        "  Desired → measured:  "
                        f"{desired_to_measured_error:.6f} m"
                    )

                    print(
                        "  Target → command:    "
                        f"{target_to_commanded_error:.6f} m"
                    )

                    print(
                        "  Target → desired:    "
                        f"{target_to_desired_error:.6f} m"
                    )

                    print(
                        "  Command success rate:"
                        f" {control_command_success_rate:.3f}"
                    )

                # --------------------------------------------------------
                # Callback
                # --------------------------------------------------------

                if (
                    progress_callback is not None
                    and (
                        iteration_count % 50 == 0
                        or finish_control
                    )
                ):
                    progress_callback({
                        "type": "cartesian_progress",
                        "iteration": iteration_count,
                        "phase": phase.lower(),
                        "progress": round(
                            progress * 100.0,
                            2,
                        ),
                        "time_elapsed": round(
                            time_elapsed,
                            4,
                        ),
                        "settling_time": round(
                            max(
                                0.0,
                                time_elapsed
                                - trajectory_duration,
                            ),
                            4,
                        ),
                        "delta_time": round(dt, 6),
                        "pose_command": (
                            pose_command.tolist()
                        ),
                        "pose_commanded_robot": (
                            commanded_robot_pose.tolist()
                        ),
                        "pose_desired_robot": (
                            desired_robot_pose.tolist()
                        ),
                        "pose_measured": (
                            robot_pose.tolist()
                        ),
                        "tcp_position": {
                            "x": float(robot_position[0]),
                            "y": float(robot_position[1]),
                            "z": float(robot_position[2]),
                        },
                        "position_error": round(
                            position_error,
                            6,
                        ),
                        "orientation_error": round(
                            orientation_error,
                            6,
                        ),
                        "orientation_error_deg": round(
                            float(
                                np.rad2deg(
                                    orientation_error
                                )
                            ),
                            4,
                        ),
                        "software_to_commanded_error": round(
                            software_to_commanded_error,
                            6,
                        ),
                        "commanded_to_desired_error": round(
                            commanded_to_desired_error,
                            6,
                        ),
                        "desired_to_measured_error": round(
                            desired_to_measured_error,
                            6,
                        ),
                        "target_to_commanded_error": round(
                            target_to_commanded_error,
                            6,
                        ),
                        "target_to_desired_error": round(
                            target_to_desired_error,
                            6,
                        ),
                        "control_command_success_rate": round(
                            control_command_success_rate,
                            4,
                        ),
                        "target_reached": target_reached,
                        "contact": (
                            cartesian_contact.tolist()
                        ),
                        "collision": (
                            cartesian_collision.tolist()
                        ),
                    })

                # --------------------------------------------------------
                # Invio comando
                # --------------------------------------------------------

                active_control.writeOnce(cartesian_command)

                if finish_control:
                    control_finished = True

                    if settling_timeout_reached:
                        print(
                            "⚠ [MOTION] Timeout di "
                            "assestamento raggiunto: "
                            f"{settling_timeout:.3f} s oltre "
                            "la durata nominale."
                        )

                    break

            # ============================================================
            # Verifica finale
            # ============================================================

            final_pose = np.asarray(
                self.robot.get_current_cartesian_pose(),
                dtype=float,
            ).reshape((4, 4))

            final_position = final_pose[:3, 3].copy()
            final_rotation = final_pose[:3, :3].copy()

            final_quat = _rotation_matrix_to_quaternion(
                final_rotation
            )

            final_position_error = float(
                np.linalg.norm(
                    target_position - final_position
                )
            )

            final_quaternion_dot = abs(
                float(np.dot(final_quat, target_quat))
            )

            final_quaternion_dot = float(
                np.clip(
                    final_quaternion_dot,
                    0.0,
                    1.0,
                )
            )

            final_orientation_error = float(
                2.0
                * np.arccos(
                    final_quaternion_dot
                )
            )

            print(
                "[MOTION] Posizione finale:",
                np.round(final_position, 6).tolist(),
            )

            print(
                f"[MOTION] Errore finale posizione: "
                f"{final_position_error:.6f} m"
            )

            print(
                f"[MOTION] Errore finale orientamento: "
                f"{final_orientation_error:.6f} rad "
                f"({np.rad2deg(final_orientation_error):.3f}°)"
            )

            position_ok = (
                final_position_error <= tolerance
            )

            orientation_ok = (
                final_orientation_error
                <= orientation_tolerance
            )

            success = position_ok and orientation_ok

            # Il controllo è terminato regolarmente, quindi il wrapper
            # torna nello stato READY anche se il target non è stato
            # raggiunto entro il timeout.
            self.robot.mode = self.robot.mode.__class__.READY

            if not success:
                print(
                    "⚠ [MOTION] Movimento terminato senza "
                    "raggiungere completamente le tolleranze. "
                    "Il robot NON viene posto in ERROR_LOCKED."
                )

                if not position_ok:
                    print(
                        "⚠ [MOTION] Errore posizione: "
                        f"{final_position_error:.6f} m, "
                        f"tolleranza: {tolerance:.6f} m."
                    )

                if not orientation_ok:
                    print(
                        "⚠ [MOTION] Errore orientamento: "
                        f"{final_orientation_error:.6f} rad, "
                        f"tolleranza: "
                        f"{orientation_tolerance:.6f} rad."
                    )

                return False

            settling_time = max(
                0.0,
                time_elapsed - trajectory_duration,
            )

            if settling_time > 0.0:
                print(
                    "✓ Movimento cartesiano completato dopo "
                    f"{settling_time:.3f} s di assestamento."
                )
            else:
                print(
                    "✓ Movimento cartesiano completato "
                    "con successo."
                )

            return True

        # ================================================================
        # Input non validi: nessun ERROR_LOCKED
        # ================================================================

        except ValueError as exc:
            self.robot.mode = self.robot.mode.__class__.READY

            print(
                "⚠ [MOTION] Parametri cartesiani non validi: "
                f"{exc}"
            )

            return False

        # ================================================================
        # Errori libfranka: ERROR_LOCKED
        # ================================================================

        except pylibfranka.ControlException as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/move_to_cartesian_pose "
                    "(ControlException)"
                ),
                exception=exc,
                recoverable=False,
            )

            print(
                "✗ ControlException durante il movimento "
                f"cartesiano: {error.message}"
            )

            traceback.print_exc()
            return False

        # ================================================================
        # Arresto software richiesto dall'utente: ERROR_LOCKED fino al recovery
        # (senza questo handler una RuntimeError qui riporterebbe a READY)
        # ================================================================

        except EmergencyStop as exc:
            error = self.robot._enter_error_locked(
                operation="motion/move_to_cartesian_pose (emergency stop)",
                exception=exc,
                recoverable=False,
            )
            print(f"✗ Movimento cartesiano arrestato: {error.message}")
            return False

        # ================================================================
        # RuntimeError
        # ================================================================

        except RuntimeError as exc:
            if collision_detected:
                error = self.robot._enter_error_locked(
                    operation=(
                        "motion/move_to_cartesian_pose "
                        "(collision)"
                    ),
                    exception=exc,
                    recoverable=False,
                )

                print(
                    "✗ Movimento interrotto per collisione: "
                    f"{error.message}"
                )

                traceback.print_exc()
                return False

            # Il contatto ferma il movimento, ma non imposta
            # automaticamente ERROR_LOCKED.
            if contact_detected:
                self.robot.mode = (
                    self.robot.mode.__class__.READY
                )

                print(
                    "⚠ [MOTION] Movimento interrotto per "
                    f"contatto: {exc}"
                )

                return False

            # Una RuntimeError software non blocca automaticamente
            # il robot.
            self.robot.mode = self.robot.mode.__class__.READY

            print(
                "⚠ [MOTION] Movimento terminato con warning: "
                f"{exc}"
            )

            traceback.print_exc()
            return False

        # ================================================================
        # Errore imprevisto
        # ================================================================

        except Exception as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/move_to_cartesian_pose "
                    "(unexpected exception)"
                ),
                exception=exc,
                recoverable=True,
            )

            print(
                "✗ Errore imprevisto durante il movimento "
                f"cartesiano: {error.message}"
            )

            traceback.print_exc()
            return False

        # ================================================================
        # Cleanup
        # ================================================================

        finally:
            if (
                active_control is not None
                and not control_finished
            ):
                try:
                    self.robot.robot.stop()

                except Exception as cleanup_exc:
                    print(
                        "[MOTION] Arresto del controllo "
                        f"fallito: {cleanup_exc}"
                    )
    def move_cartesian_relative(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        droll: float = 0.0,
        dpitch: float = 0.0,
        dyaw: float = 0.0,
        speed_factor: float = 0.2,
        tolerance: float = 0.01,
        progress_callback: Optional[Callable[[dict], None]] = None,
    ) -> bool:
        self.robot._ensure_can_command()

        current_pose = self.robot.get_current_cartesian_pose()
        relative_transform = np.eye(4, dtype=float)
        relative_transform[:3, :3] = _rpy_to_rotation_matrix(droll, dpitch, dyaw)
        relative_transform[:3, 3] = [dx, dy, dz]

        target_pose = current_pose @ relative_transform
        target_roll, target_pitch, target_yaw = _rotation_matrix_to_rpy(target_pose[:3, :3])
        target_position = target_pose[:3, 3]

        return self.move_to_cartesian_pose(
            float(target_position[0]),
            float(target_position[1]),
            float(target_position[2]),
            target_roll,
            target_pitch,
            target_yaw,
            speed_factor=speed_factor,
            tolerance=tolerance,
            progress_callback=progress_callback,
        )

    # ------------------------------------------------------------
    # Impedance control
    # ------------------------------------------------------------
    def impedance_control(
        self,
        target_positions: List[float],
        stiffness: Optional[List[float]] = None,
        damping: Optional[List[float]] = None,
        duration: float = 5.0,
        tolerance: float = 0.04,
        progress_callback: Optional[Callable[[dict], None]] = None,
    ) -> bool:
        """
        Esegue un vero joint impedance control mediante torque control.

        La configurazione di equilibrio viene interpolata dalla posizione
        iniziale alla configurazione target con una traiettoria minimum-jerk.

        Legge di controllo:

            tau_d = K * (q_desired - q)
                    - D * dq
                    + coriolis

        Parametri:
            target_positions:
                Configurazione articolare target [rad], 7 elementi.

            stiffness:
                Rigidezze articolari K [Nm/rad], 7 elementi.

            damping:
                Smorzamenti articolari D [Nm*s/rad], 7 elementi.
                Se None, viene usato D = 2*sqrt(K).

            duration:
                Durata dell'interpolazione minimum-jerk [s].

            tolerance:
                Errore articolare massimo accettabile alla fine [rad].

            progress_callback:
                Callback opzionale per la telemetria.
        """

        self.robot._ensure_can_command()

        active_control = None
        control_finished = False

        try:
            # --------------------------------------------------------
            # Validazione degli input
            # --------------------------------------------------------

            self._validate_joint_vector(
                target_positions,
                "target_positions",
            )
            self._validate_duration(duration)

            if tolerance <= 0.0:
                raise ValueError(
                    "tolerance deve essere positiva."
                )

            if stiffness is None:
                stiffness = [
                    600.0,
                    600.0,
                    600.0,
                    600.0,
                    250.0,
                    150.0,
                    50.0,
                ]

            self._validate_joint_vector(
                stiffness,
                "stiffness",
            )

            target = np.asarray(
                target_positions,
                dtype=float,
            )

            K = np.asarray(
                stiffness,
                dtype=float,
            )

            if np.any(K <= 0.0):
                raise ValueError(
                    "Tutti i valori di stiffness devono "
                    "essere positivi."
                )

            if damping is None:
                D = 2.0 * np.sqrt(K)
            else:
                self._validate_joint_vector(
                    damping,
                    "damping",
                )

                D = np.asarray(
                    damping,
                    dtype=float,
                )

                if np.any(D < 0.0):
                    raise ValueError(
                        "I valori di damping non possono "
                        "essere negativi."
                    )

            # --------------------------------------------------------
            # Stato iniziale e modello dinamico
            # --------------------------------------------------------

            initial_state = self.robot.get_state()

            q_start = np.asarray(
                initial_state.q,
                dtype=float,
            )

            print(
                "[IMPEDANCE] Posizione iniziale:",
                np.round(q_start, 3).tolist(),
            )
            print(
                "[IMPEDANCE] Target:",
                np.round(target, 3).tolist(),
            )
            print(
                "[IMPEDANCE] Stiffness:",
                np.round(K, 3).tolist(),
            )
            print(
                "[IMPEDANCE] Damping:",
                np.round(D, 3).tolist(),
            )
            print(
                f"[IMPEDANCE] Durata: {duration:.3f} s"
            )

            # Il modello serve per calcolare i termini di Coriolis.
            model = self.robot.robot.load_model()

            self.robot.mode = (
                self.robot.mode.__class__.RUNNING
            )

            # --------------------------------------------------------
            # Avvio del torque control
            # --------------------------------------------------------

            active_control = (
                self.robot.robot.start_torque_control()
            )

            time_elapsed = 0.0
            iteration = 0

            # --------------------------------------------------------
            # Ciclo di controllo
            # --------------------------------------------------------

            while True:
                robot_state, period = (
                    active_control.readOnce()
                )

                dt = period.to_sec()
                time_elapsed += dt
                iteration += 1

                q_measured = np.asarray(
                    robot_state.q,
                    dtype=float,
                )

                dq_measured = np.asarray(
                    robot_state.dq,
                    dtype=float,
                )

                cartesian_contact = np.asarray(
                    robot_state.cartesian_contact,
                    dtype=bool,
                )

                cartesian_collision = np.asarray(
                    robot_state.cartesian_collision,
                    dtype=bool,
                )

                # Nel controllo di impedenza un contatto non comporta
                # necessariamente l'arresto. Una collisione, invece,
                # interrompe il controllo.
                if self.robot.stop_requested:
                    raise EmergencyStop(EMERGENCY_STOP_MESSAGE)

                if np.any(cartesian_collision):
                    raise RuntimeError(
                        "Collisione cartesiana rilevata: "
                        "torque control interrotto."
                    )

                # Avanzamento normalizzato della traiettoria.
                progress = min(
                    time_elapsed / duration,
                    1.0,
                )

                # Minimum-jerk già presente nella classe.
                s = compute_minimum_jerk_trajectory(
                    progress
                )

                # Configurazione di equilibrio variabile nel tempo.
                q_desired = (
                    q_start
                    + s * (target - q_start)
                )

                position_error = (
                    q_desired - q_measured
                )

                # Compensazione di Coriolis e centrifuga.
                coriolis = np.asarray(
                    model.coriolis(robot_state),
                    dtype=float,
                )

                # Legge di joint impedance.
                tau_command = (
                    K * position_error
                    - D * dq_measured
                    + coriolis
                )

                torque_command = pylibfranka.Torques(
                    tau_command.tolist()
                )

                torque_command.motion_finished = (
                    progress >= 1.0
                )

                active_control.writeOnce(
                    torque_command
                )

                # ----------------------------------------------------
                # Telemetria non eseguita a ogni ciclo
                # ----------------------------------------------------

                if iteration % 100 == 0:
                    target_error = float(
                        np.max(
                            np.abs(
                                target - q_measured
                            )
                        )
                    )

                    print(
                        f"[IMPEDANCE] "
                        f"{progress * 100:5.1f}% | "
                        f"errore target: "
                        f"{target_error:.5f} rad | "
                        f"tau max: "
                        f"{np.max(np.abs(tau_command)):.3f} Nm"
                    )

                if (
                    progress_callback is not None
                    and (
                        iteration % 50 == 0
                        or progress >= 1.0
                    )
                ):
                    progress_callback({
                        "type": "impedance_progress",
                        "iteration": iteration,
                        "progress": round(
                            progress * 100.0,
                            2,
                        ),
                        "time_elapsed": round(
                            time_elapsed,
                            4,
                        ),
                        "delta_time": round(dt, 6),
                        "q_measured": (
                            np.round(q_measured, 6)
                            .tolist()
                        ),
                        "dq_measured": (
                            np.round(dq_measured, 6)
                            .tolist()
                        ),
                        "q_desired": (
                            np.round(q_desired, 6)
                            .tolist()
                        ),
                        "position_error": (
                            np.round(position_error, 6)
                            .tolist()
                        ),
                        "tau_command": (
                            np.round(tau_command, 6)
                            .tolist()
                        ),
                        "coriolis": (
                            np.round(coriolis, 6)
                            .tolist()
                        ),
                        "contact": (
                            cartesian_contact.tolist()
                        ),
                        "collision": (
                            cartesian_collision.tolist()
                        ),
                    })

                if progress >= 1.0:
                    control_finished = True
                    break

            # --------------------------------------------------------
            # Verifica finale
            # --------------------------------------------------------

            final_state = self.robot.get_state()

            q_final = np.asarray(
                final_state.q,
                dtype=float,
            )

            final_error = float(
                np.max(
                    np.abs(target - q_final)
                )
            )

            print(
                "[IMPEDANCE] Posizione finale:",
                np.round(q_final, 4).tolist(),
            )

            print(
                "[IMPEDANCE] Errore massimo finale: "
                f"{final_error:.6f} rad"
            )

            if final_error > tolerance:
                raise RuntimeError(
                    "Tolleranza non rispettata: "
                    f"errore={final_error:.6f} rad, "
                    f"tolleranza={tolerance:.6f} rad."
                )

            self.robot.mode = (
                self.robot.mode.__class__.READY
            )

            print(
                "✓ Joint impedance control completato"
            )
            return True

        except pylibfranka.ControlException as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/impedance_control "
                    "(ControlException)"
                ),
                exception=exc,
                recoverable=False,
            )

            print(
                "✗ ControlException durante "
                f"impedance_control: {error.message}"
            )
            traceback.print_exc()
            return False

        except RuntimeError as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/impedance_control "
                    "(RuntimeError)"
                ),
                exception=exc,
                recoverable=False,
            )

            print(
                "✗ Impedance control interrotto: "
                f"{error.message}"
            )
            traceback.print_exc()
            return False

        except Exception as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/impedance_control "
                    "(Exception)"
                ),
                exception=exc,
                recoverable=True,
            )

            print(
                "✗ Errore durante impedance_control: "
                f"{error.message}"
            )
            traceback.print_exc()
            return False

        finally:
            if (
                active_control is not None
                and not control_finished
            ):
                try:
                    self.robot.robot.stop()
                except Exception as cleanup_exc:
                    print(
                        "[IMPEDANCE] Arresto fallito: "
                        f"{cleanup_exc}"
                    )

    # ------------------------------------------------------------
    # Force control: spinta con forza controllata, inserimento, avvitamento
    # ------------------------------------------------------------

    def apply_force(
        self,
        force: float,
        duration: float = 5.0,
        axis: Sequence[float] = (0.0, 0.0, 1.0),
        progress_callback: Optional[Callable[[dict], None]] = None,
        **tuning: Any,
    ) -> bool:
        """Spinge lungo `axis` (frame del tool, default z) con `force` N per `duration` s."""
        params = make_params("hold", force, hold_time=duration, axis_tool=tuple(axis), **tuning)
        return self.run_force_task(params, progress_callback)

    def insert_with_force(
        self,
        force: float,
        target_depth: float,
        axis: Sequence[float] = (0.0, 0.0, 1.0),
        progress_callback: Optional[Callable[[dict], None]] = None,
        **tuning: Any,
    ) -> bool:
        """Inserisce spingendo con `force` N lungo `axis` fino a `target_depth` m dal contatto."""
        params = make_params(
            "insert", force, target_depth=target_depth, axis_tool=tuple(axis), **tuning
        )
        return self.run_force_task(params, progress_callback)

    def screw(
        self,
        angle: float,
        force: float = 10.0,
        axis: Sequence[float] = (0.0, 0.0, 1.0),
        progress_callback: Optional[Callable[[dict], None]] = None,
        **tuning: Any,
    ) -> bool:
        """
        Avvita: spinge con `force` N lungo `axis` e ruota di `angle` rad attorno ad esso.

        La rotazione è quella del giunto 7 (escursione ~±3 rad): per più giri
        serve riprendere la vite (regrasp). La coppia è limitata a `max_torque`.
        """
        params = make_params("screw", force, angle=angle, axis_tool=tuple(axis), **tuning)
        return self.run_force_task(params, progress_callback)

    def run_force_task(
        self,
        params: ForceTaskParams,
        progress_callback: Optional[Callable[[dict], None]] = None,
    ) -> bool:
        """
        Esegue un task di forza (hold / insert / screw) in torque control.

        Ritorna True se l'obiettivo è raggiunto. Se non lo è (nessun contatto,
        blocco, timeout, coppia massima…) il controllo termina in ordine, con
        la forza riportata a zero senza bloccare il robot, e viene sollevata
        ForceTaskFailed con il motivo. Una forza oltre il limite di sicurezza o
        una collisione fermano subito il controllo (ERROR_LOCKED).

        Durante il task le soglie di collisione di libfranka vengono alzate
        (quelle di default, 10 N, scatterebbero appena si spinge) e al termine
        ripristinate ai valori di default di FrankaRobot.set_collision_behavior().
        """
        self.robot._ensure_can_command()

        initial_state = self.robot.get_state()
        q0 = np.asarray(initial_state.q, dtype=float)
        T0 = np.asarray(initial_state.O_T_EE, dtype=float).reshape((4, 4), order="F")
        precheck_wrist_range(params, q0)

        controller = ForceTaskController(params, q0, T0[:3, 3], T0[:3, :3])
        model = self.robot.robot.load_model()

        print(
            f"[FORCE] {params.mode}: {params.force:.1f} N lungo asse tool "
            f"{np.round(params.axis_unit, 3).tolist()}"
            + (f", profondità {params.target_depth * 1000:.1f} mm" if params.target_depth else "")
            + (f", rotazione {params.angle:.2f} rad" if params.mode == "screw" else "")
            + f" | limite di sicurezza {params.abort_force:.0f} N"
        )

        active_control = None
        control_finished = False
        thresholds_set = False

        try:
            self.robot.set_collision_behavior(**params.collision_thresholds())
            thresholds_set = True

            self.robot.mode = self.robot.mode.__class__.RUNNING
            active_control = self.robot.robot.start_torque_control()

            time_elapsed = 0.0
            iteration = 0

            while True:
                robot_state, period = active_control.readOnce()
                dt = period.to_sec()
                time_elapsed += dt
                iteration += 1

                if self.robot.stop_requested:
                    raise EmergencyStop(EMERGENCY_STOP_MESSAGE)
                if np.any(np.asarray(robot_state.cartesian_collision, dtype=bool)):
                    raise RuntimeError("Collisione cartesiana rilevata: torque control interrotto.")

                q = np.asarray(robot_state.q, dtype=float)
                dq = np.asarray(robot_state.dq, dtype=float)
                pose = np.asarray(robot_state.O_T_EE, dtype=float).reshape((4, 4), order="F")
                out = controller.step(
                    t=time_elapsed,
                    dt=dt,
                    q=q,
                    dq=dq,
                    x=pose[:3, 3],
                    R=pose[:3, :3],
                    J=as_jacobian(model.zero_jacobian(robot_state)),
                    coriolis=np.asarray(model.coriolis(robot_state), dtype=float),
                    ext_wrench=np.asarray(robot_state.O_F_ext_hat_K, dtype=float),
                    tau_prev=np.asarray(robot_state.tau_J_d, dtype=float),
                )

                command = pylibfranka.Torques(out.tau.tolist())
                command.motion_finished = out.finished
                active_control.writeOnce(command)

                if iteration % 100 == 0:
                    print(
                        f"[FORCE] {out.info['phase']:8s} F={out.info['force_measured']:6.2f} N "
                        f"(des {out.info['force_desired']:5.2f}) | profondità "
                        f"{out.info['depth_m'] * 1000:6.2f} mm | angolo {out.info['angle_rad']:6.3f} rad"
                    )

                if progress_callback is not None and (iteration % 50 == 0 or out.finished):
                    progress_callback({
                        "type": "force_progress",
                        "mode": params.mode,
                        "iteration": iteration,
                        "time_elapsed": round(time_elapsed, 4),
                        "delta_time": round(dt, 6),
                        "q_measured": np.round(q, 6).tolist(),
                        "tcp_position": {
                            "x": float(pose[0, 3]),
                            "y": float(pose[1, 3]),
                            "z": float(pose[2, 3]),
                        },
                        **out.info,
                    })

                if out.finished:
                    control_finished = True
                    break

            result = controller.result()
            self.last_force_result = result
            self.robot.mode = self.robot.mode.__class__.READY
            if not result["success"]:
                raise ForceTaskFailed(result["reason"], result)

            print(f"✓ Force task completato: {result['reason']} | {result}")
            return True

        except ForceTaskFailed as exc:
            self.robot.mode = self.robot.mode.__class__.READY
            print(f"⚠ [FORCE] Obiettivo non raggiunto: {exc} | {exc.result}")
            raise

        except pylibfranka.ControlException as exc:
            error = self.robot._enter_error_locked(
                operation="motion/run_force_task (ControlException)",
                exception=exc,
                recoverable=False,
            )
            print(f"✗ ControlException durante il force task: {error.message}")
            traceback.print_exc()
            return False

        except RuntimeError as exc:
            error = self.robot._enter_error_locked(
                operation="motion/run_force_task (RuntimeError)",
                exception=exc,
                recoverable=False,
            )
            print(f"✗ Force task interrotto: {error.message}")
            traceback.print_exc()
            return False

        except Exception as exc:
            error = self.robot._enter_error_locked(
                operation="motion/run_force_task (Exception)",
                exception=exc,
                recoverable=True,
            )
            print(f"✗ Errore durante il force task: {error.message}")
            traceback.print_exc()
            return False

        finally:
            if active_control is not None and not control_finished:
                try:
                    self.robot.robot.stop()
                except Exception as cleanup_exc:
                    print(f"[FORCE] Arresto fallito: {cleanup_exc}")
            if thresholds_set:
                try:
                    self.robot.set_collision_behavior(allow_locked=True)
                except Exception as cleanup_exc:
                    print(f"[FORCE] Ripristino soglie di collisione fallito: {cleanup_exc}")

    def run_screw_passes(
        self,
        params: ForceTaskParams,
        regrasp: RegraspConfig,
        iterations: int = 1,
        progress_callback: Optional[Callable[[dict], None]] = None,
    ) -> bool:
        """
        Avvitamento a più passate come UNICA operazione.

        Ripete `iterations` volte la passata di avvitamento descritta da
        `params` (angolo per passata, forza, coppia…). Tra una passata e la
        successiva esegue la ripresa configurata in `regrasp`: il giunto 7
        viene riportato a vuoto all'estremo opposto dell'escursione, così ogni
        passata ha a disposizione tutta la rotazione. Si ferma prima se una
        passata raggiunge la coppia di serraggio (stop_at_torque): la vite è
        a battuta e le passate restanti non servono.

        Il pre-check (escursione del giunto 7, parametri di ripresa) avviene
        prima di muovere il robot. Un fallimento in una passata o nella
        ripresa solleva ForceTaskFailed indicando il numero di passata.
        """
        self.robot._ensure_can_command()
        precheck_screw_passes(
            params, regrasp, iterations, np.asarray(self.robot.get_state().q, dtype=float)
        )

        total_angle = 0.0
        for index in range(iterations):
            label = f"passata {index + 1}/{iterations}"
            print(f"[SCREW] {label}: {params.angle:.2f} rad con {params.force:.1f} N")

            def pass_callback(data: dict, _index: int = index) -> None:
                if progress_callback is not None:
                    progress_callback({**data, "pass": _index + 1, "passes": iterations})

            try:
                self.run_force_task(params, pass_callback)
            except ForceTaskFailed as exc:
                raise ForceTaskFailed(f"{label}: {exc}", exc.result) from exc
            if self.robot.mode.value == "error_locked":
                return False
            result = self.last_force_result or {}
            total_angle += float(result.get("angle_rad", 0.0))

            if result.get("torque_reached"):
                print(f"[SCREW] {label}: coppia di serraggio raggiunta, passate restanti saltate.")
                break
            if index < iterations - 1:
                self._regrasp(params, regrasp, label, progress_callback)

        print(f"✓ Avvitamento completato: rotazione totale ~{total_angle:.2f} rad")
        return True

    def _regrasp(
        self,
        params: ForceTaskParams,
        regrasp: RegraspConfig,
        label: str,
        progress_callback: Optional[Callable[[dict], None]],
    ) -> None:
        def fail(step: str) -> None:
            raise ForceTaskFailed(f"{label}: ripresa fallita ({step})")

        self.robot.check_stop()
        print(f"[SCREW] {label}: ripresa ({regrasp.mode})")
        gripper = self.robot.gripper if regrasp.mode == "gripper" else None

        if gripper is not None and not gripper.open(
            width=regrasp.open_width_mm / 1000.0, speed=regrasp.gripper_speed
        ):
            fail("apertura del gripper")

        if regrasp.mode == "retract":
            axis = params.axis_unit * -regrasp.retract_distance
            if not self.move_cartesian_relative(
                float(axis[0]), float(axis[1]), float(axis[2]), speed_factor=0.1,
                progress_callback=progress_callback,
            ):
                fail("arretramento lungo l'asse")

        q = np.asarray(self.robot.get_state().q, dtype=float)
        target = q.copy()
        target[6] = wrist_rewind_target(params)
        distance = abs(target[6] - q[6])
        if distance > 1e-3:
            duration = max(1.0, 1.875 * distance / regrasp.rewind_speed)
            if not self.move_to_joint_positions(
                target.tolist(), speed_factor=0.2, tolerance=0.04,
                progress_callback=progress_callback, duration_override=duration,
            ):
                fail("riavvolgimento del giunto 7")

        self.robot.check_stop()
        if gripper is not None and not gripper.grasp(
            width=regrasp.grasp_width_mm / 1000.0,
            force=regrasp.grasp_force,
            speed=regrasp.gripper_speed,
        ):
            fail("presa della vite")

    #------------------------------------------------------------
    # Move joint positions with stiffness wrapper
    # ------------------------------------------------------------**
    def move_to_joint_positions_with_impedance(
        self,
        target_positions: List[float],
        stiffness: Optional[List[float]] = None,
        duration: float = 5.0,
        progress_callback: Optional[Callable[[dict], None]] = None,
    ) -> bool:
        self.robot._ensure_can_command()

        try:
            self._validate_joint_vector(target_positions, "target_positions")
            self._validate_duration(duration)

            if stiffness is None:
                stiffness = [600, 600, 600, 600, 250, 150, 50]

            self._validate_joint_vector(stiffness, "stiffness")
            self.robot.set_joint_impedance(stiffness)

            print(f"Controllo impedenza verso: {np.round(target_positions, 3).tolist()}")

            speed_factor = min(0.5, 5.0 / duration)
            return self.move_to_joint_positions(
                target_positions, speed_factor=speed_factor, progress_callback=progress_callback
            )

        except Exception as e:
            error = self.robot._enter_error_locked(
                operation="motion/impedance_control",
                exception=e,
                recoverable=True,
            )
            print(f"✗ Errore durante impedance_control: {error.message}")
            return False

    # ------------------------------------------------------------
    # Velocity control
    # ------------------------------------------------------------

    def velocity_control(
        self,
        velocity_callback: Callable[[pylibfranka.RobotState, float], List[float]],
        duration: float = 5.0,
        progress_callback: Optional[Callable[[dict], None]] = None,
    ) -> bool:
        self.robot._ensure_can_command()

        active_control = None
        q_last: Optional[np.ndarray] = None

        try:
            self._validate_duration(duration)
            print(f"Avvio controllo velocità per {duration}s")
            self.robot.mode = self.robot.mode.__class__.RUNNING

            active_control = self.robot.robot.start_joint_velocity_control(
                pylibfranka.ControllerMode.CartesianImpedance
                #pylibfranka.ControllerMode.JointImpedance
            )

            time_elapsed = 0.0
            iteration_count = 0
            motion_finished = False

            while not motion_finished:
                iteration_count += 1
                robot_state, delta_time = active_control.readOnce()
                q_last = np.array(robot_state.q)
                if self.robot.stop_requested:
                    raise EmergencyStop(EMERGENCY_STOP_MESSAGE)
                dt = delta_time.to_sec()
                time_elapsed += dt

                dq = velocity_callback(robot_state, dt)
                self._validate_joint_vector(dq, "dq")

                velocity_cmd = pylibfranka.JointVelocities(dq)

                if time_elapsed >= duration:
                    velocity_cmd.motion_finished = True
                    motion_finished = True
                else:
                    velocity_cmd.motion_finished = False

                if progress_callback is not None:
                    progress_callback({
                        "type": "velocity_progress",
                        "iteration": iteration_count,
                        "time_elapsed": round(time_elapsed, 4),
                        "delta_time": round(dt, 6),
                        "q_measured": list(robot_state.q),
                        "dq_commanded": dq,
                        "progress": round(min(time_elapsed / duration, 1.0) * 100, 2),
                    })

                active_control.writeOnce(velocity_cmd)

            self.robot.mode = self.robot.mode.__class__.READY
            print("✓ Controllo velocità completato")
            return True

        except Exception as e:
            error = self.robot._enter_error_locked(
                operation="motion/velocity_control",
                exception=e,
                recoverable=True,
            )
            print(f"✗ Errore durante controllo velocità: {error.message}")
            traceback.print_exc()
            return False

        finally:
            if active_control is not None:
                try:
                    if q_last is None:
                        q_last = np.array(self.robot.get_state().q)
                    stop_cmd = pylibfranka.JointVelocities([0.0] * 7)
                    stop_cmd.motion_finished = True
                    active_control.writeOnce(stop_cmd)
                except Exception as cleanup_exc:
                    print(f"[MOTION] Cleanup velocity control failed: {cleanup_exc}")
                try:
                    self.robot.robot.stop()
                except Exception as cleanup_exc:
                    print(f"[MOTION] Robot.stop() failed during velocity cleanup: {cleanup_exc}")

    # ------------------------------------------------------------
    # Home
    # ------------------------------------------------------------

    def go_to_home(
        self,
        speed_factor: float = 0.2,
        progress_callback: Optional[Callable[[dict], None]] = None,
    ) -> bool:
        self.robot._ensure_can_command()

        home_position = [0.059821926057338715,-0.7743019461631775, -0.023823734372854233,-2.373579978942871,
                         -0.037449367344379425, 1.5682077407836914, 0.8321403861045837]
        print("Ritorno alla posizione home...")

        return self.move_to_joint_positions(
            home_position, speed_factor=speed_factor, progress_callback=progress_callback
        )
