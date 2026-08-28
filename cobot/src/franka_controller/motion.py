"""
Controller per il movimento del robot Franka usando pylibfranka.
"""

import traceback
import numpy as np
from typing import List, Optional, Callable, Tuple

import pylibfranka
from .utils import compute_minimum_jerk_trajectory




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

            duration = 5.0 / speed_factor
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
    ) -> bool:
        """
        Esegue in sequenza una lista di waypoint articolari (7 valori ciascuno),
        tipicamente i punti di una traiettoria pianificata da moveit_api.

        Ogni segmento riusa move_to_joint_positions, quindi eredita le stesse
        verifiche di sicurezza (collision/contact) e la stessa interpolazione
        minimum-jerk. Si interrompe al primo segmento fallito senza tentare i
        successivi. Si assume che l'ordine dei 7 valori corrisponda all'ordine
        articolare usato altrove in questo modulo (fr3_joint1..7).
        """
        for waypoint in waypoints:
            self._validate_joint_vector(waypoint, "waypoint")

        print(f"[TRAJECTORY] Esecuzione di {len(waypoints)} waypoint")

        for index, waypoint in enumerate(waypoints):
            print(
                f"[TRAJECTORY] Waypoint {index + 1}/{len(waypoints)}: "
                f"{np.round(waypoint, 3).tolist()}"
            )
            success = self.move_to_joint_positions(
                waypoint,
                speed_factor=speed_factor,
                tolerance=tolerance,
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
        progress_callback: Optional[
            Callable[[dict], None]
        ] = None,
    ) -> bool:
        """
        Muove l'end-effector verso una posa cartesiana target.

        Unità:
            x, y, z               metri
            roll, pitch, yaw      radianti
            tolerance             metri
            orientation_tolerance radianti

        Convenzione RPY:
            R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        """

        self.robot._ensure_can_command()

        active_control = None
        control_finished = False
        last_command_pose = None

        try:
            # --------------------------------------------------------
            # Validazione degli input
            # --------------------------------------------------------

            self._validate_speed_factor(speed_factor)

            if tolerance <= 0.0:
                raise ValueError(
                    "tolerance deve essere positiva."
                )

            if orientation_tolerance <= 0.0:
                raise ValueError(
                    "orientation_tolerance deve essere "
                    "positiva."
                )

            target_values = np.asarray(
                [x, y, z, roll, pitch, yaw],
                dtype=float,
            )

            if not np.all(np.isfinite(target_values)):
                raise ValueError(
                    "La posa target contiene valori "
                    "non finiti."
                )

            # --------------------------------------------------------
            # Costruzione della posa target
            # --------------------------------------------------------

            target_pose = _build_cartesian_pose(
                x,
                y,
                z,
                roll,
                pitch,
                yaw,
            )

            target_position = (
                target_pose[:3, 3].copy()
            )

            target_rotation = (
                target_pose[:3, :3].copy()
            )

            target_quat = (
                _rotation_matrix_to_quaternion(
                    target_rotation
                )
            )

            print(
                "[MOTION] Posizione target:",
                np.round(
                    target_position,
                    4,
                ).tolist(),
            )

            print(
                "[MOTION] Orientamento target RPY:",
                np.round(
                    [roll, pitch, yaw],
                    4,
                ).tolist(),
            )

            print(
                f"[MOTION] Speed factor: "
                f"{speed_factor}"
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

            # --------------------------------------------------------
            # Avvio del controllo cartesiano
            # --------------------------------------------------------

            self.robot.mode = (
                self.robot.mode.__class__.RUNNING
            )

            active_control = (
                self.robot.robot
                .start_cartesian_pose_control(
                    pylibfranka.ControllerMode
                    .CartesianImpedance
                )
            )

            print(
                "[MOTION] Controllo cartesiano avviato."
            )

            # --------------------------------------------------------
            # Lettura della posa iniziale
            # --------------------------------------------------------

            initial_state, _ = (
                active_control.readOnce()
            )

            # Si usa la posa desiderata precedente per
            # garantire continuità del riferimento.
            #
            # Se O_T_EE_d non fosse disponibile, viene
            # utilizzata la posa misurata O_T_EE.
            if hasattr(initial_state, "O_T_EE_d"):
                initial_pose_data = (
                    initial_state.O_T_EE_d
                )
            else:
                initial_pose_data = (
                    initial_state.O_T_EE
                )

            current_pose = np.asarray(
                initial_pose_data,
                dtype=float,
            ).reshape(
                (4, 4),
                order="F",
            )

            current_position = (
                current_pose[:3, 3].copy()
            )

            current_rotation = (
                current_pose[:3, :3].copy()
            )

            current_quat = (
                _rotation_matrix_to_quaternion(
                    current_rotation
                )
            )

            last_command_pose = (
                current_pose.copy()
            )

            print(
                "[MOTION] Posizione iniziale:",
                np.round(
                    current_position,
                    4,
                ).tolist(),
            )

            # --------------------------------------------------------
            # Parametri temporali
            # --------------------------------------------------------

            duration = 5.0 / speed_factor
            time_elapsed = 0.0
            iteration_count = 0

            print(
                f"[MOTION] Durata prevista: "
                f"{duration:.3f} s"
            )

            # --------------------------------------------------------
            # Ciclo di controllo
            # --------------------------------------------------------

            while True:
                robot_state, delta_time = (
                    active_control.readOnce()
                )

                dt = delta_time.to_sec()
                time_elapsed += dt
                iteration_count += 1

                # Posa misurata del robot.
                robot_pose = np.asarray(
                    robot_state.O_T_EE,
                    dtype=float,
                ).reshape(
                    (4, 4),
                    order="F",
                )

                robot_position = (
                    robot_pose[:3, 3].copy()
                )

                robot_rotation = (
                    robot_pose[:3, :3].copy()
                )

                robot_quat = (
                    _rotation_matrix_to_quaternion(
                        robot_rotation
                    )
                )

                cartesian_contact = np.asarray(
                    robot_state.cartesian_contact,
                    dtype=bool,
                )

                cartesian_collision = np.asarray(
                    robot_state.cartesian_collision,
                    dtype=bool,
                )

                # ----------------------------------------------------
                # Gestione di contatto e collisione
                # ----------------------------------------------------

                if (
                    np.any(cartesian_contact)
                    or np.any(cartesian_collision)
                ):
                    # Termina usando l'ultima posa comandata.
                    # Usare direttamente la posa misurata potrebbe
                    # introdurre una discontinuità nel riferimento.
                    stop_command = (
                        _pose_to_cartesian_command(
                            last_command_pose
                        )
                    )

                    stop_command.motion_finished = True

                    active_control.writeOnce(
                        stop_command
                    )

                    control_finished = True

                    if np.any(cartesian_collision):
                        raise RuntimeError(
                            "Collisione cartesiana rilevata: "
                            "movimento interrotto."
                        )

                    raise RuntimeError(
                        "Contatto cartesiano rilevato: "
                        "movimento interrotto."
                    )

                # ----------------------------------------------------
                # Avanzamento temporale
                # ----------------------------------------------------

                progress = min(
                    time_elapsed / duration,
                    1.0,
                )

                s = compute_minimum_jerk_trajectory(
                    progress
                )

                # ----------------------------------------------------
                # Interpolazione della posizione
                # ----------------------------------------------------

                position_command = (
                    current_position
                    + s
                    * (
                        target_position
                        - current_position
                    )
                )

                # ----------------------------------------------------
                # Interpolazione dell'orientamento
                # ----------------------------------------------------

                quaternion_command = (
                    _slerp_quaternion(
                        current_quat,
                        target_quat,
                        s,
                    )
                )

                rotation_command = (
                    _quaternion_to_rotation_matrix(
                        quaternion_command
                    )
                )

                # ----------------------------------------------------
                # Ricostruzione della posa comandata
                # ----------------------------------------------------

                pose_command = np.eye(
                    4,
                    dtype=float,
                )

                pose_command[:3, :3] = (
                    rotation_command
                )

                pose_command[:3, 3] = (
                    position_command
                )

                last_command_pose = (
                    pose_command.copy()
                )

                # ----------------------------------------------------
                # Calcolo degli errori
                # ----------------------------------------------------

                position_error = float(
                    np.linalg.norm(
                        target_position
                        - robot_position
                    )
                )

                quaternion_dot = abs(
                    float(
                        np.dot(
                            robot_quat,
                            target_quat,
                        )
                    )
                )

                orientation_error = float(
                    2.0
                    * np.arccos(
                        np.clip(
                            quaternion_dot,
                            -1.0,
                            1.0,
                        )
                    )
                )

                # ----------------------------------------------------
                # Invio del comando
                # ----------------------------------------------------

                cartesian_command = (
                    _pose_to_cartesian_command(
                        pose_command
                    )
                )

                cartesian_command.motion_finished = (
                    progress >= 1.0
                )

                active_control.writeOnce(
                    cartesian_command
                )

                # ----------------------------------------------------
                # Stampa limitata per il ciclo real-time
                # ----------------------------------------------------

                if (
                    iteration_count % 100 == 0
                    or progress >= 1.0
                ):
                    print(
                        f"[MOTION] "
                        f"{progress * 100:5.1f}% | "
                        f"errore posizione: "
                        f"{position_error:.6f} m | "
                        f"errore orientamento: "
                        f"{orientation_error:.6f} rad "
                        f"({np.rad2deg(orientation_error):.3f}°)"
                    )

                # ----------------------------------------------------
                # Callback limitata
                # ----------------------------------------------------

                if (
                    progress_callback is not None
                    and (
                        iteration_count % 50 == 0
                        or progress >= 1.0
                    )
                ):
                    progress_callback({
                        "type": "cartesian_progress",
                        "iteration": iteration_count,
                        "progress": round(
                            progress * 100.0,
                            2,
                        ),
                        "time_elapsed": round(
                            time_elapsed,
                            4,
                        ),
                        "delta_time": round(
                            dt,
                            6,
                        ),
                        "pose_command": (
                            pose_command.tolist()
                        ),
                        "pose_measured": (
                            robot_pose.tolist()
                        ),
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
            # Verifica della posa finale
            # --------------------------------------------------------

            final_pose = (
                self.robot.get_current_cartesian_pose()
            )

            final_pose = np.asarray(
                final_pose,
                dtype=float,
            ).reshape(
                (4, 4),
            )

            final_position = (
                final_pose[:3, 3]
            )

            final_rotation = (
                final_pose[:3, :3]
            )

            final_quat = (
                _rotation_matrix_to_quaternion(
                    final_rotation
                )
            )

            final_position_error = float(
                np.linalg.norm(
                    target_position
                    - final_position
                )
            )

            final_quaternion_dot = abs(
                float(
                    np.dot(
                        final_quat,
                        target_quat,
                    )
                )
            )

            final_orientation_error = float(
                2.0
                * np.arccos(
                    np.clip(
                        final_quaternion_dot,
                        -1.0,
                        1.0,
                    )
                )
            )

            print(
                "[MOTION] Posizione finale:",
                np.round(
                    final_position,
                    4,
                ).tolist(),
            )

            print(
                "[MOTION] Errore finale posizione: "
                f"{final_position_error:.6f} m"
            )

            print(
                "[MOTION] Errore finale orientamento: "
                f"{final_orientation_error:.6f} rad "
                f"({np.rad2deg(final_orientation_error):.3f}°)"
            )

            if final_position_error > tolerance:
                raise RuntimeError(
                    "Tolleranza di posizione non rispettata: "
                    f"errore={final_position_error:.6f} m, "
                    f"tolleranza={tolerance:.6f} m."
                )

            if (
                final_orientation_error
                > orientation_tolerance
            ):
                raise RuntimeError(
                    "Tolleranza di orientamento "
                    "non rispettata: "
                    f"errore="
                    f"{final_orientation_error:.6f} rad, "
                    f"tolleranza="
                    f"{orientation_tolerance:.6f} rad."
                )

            self.robot.mode = (
                self.robot.mode.__class__.READY
            )

            print(
                "✓ Movimento cartesiano completato "
                "con successo."
            )

            return True

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
                "✗ ControlException durante il "
                f"movimento cartesiano: {error.message}"
            )

            traceback.print_exc()
            return False

        except RuntimeError as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/move_to_cartesian_pose "
                    "(RuntimeError)"
                ),
                exception=exc,
                recoverable=False,
            )

            print(
                "✗ Movimento cartesiano interrotto: "
                f"{error.message}"
            )

            traceback.print_exc()
            return False

        except Exception as exc:
            error = self.robot._enter_error_locked(
                operation=(
                    "motion/move_to_cartesian_pose "
                    "(Exception)"
                ),
                exception=exc,
                recoverable=True,
            )

            print(
                "✗ Errore durante il movimento "
                f"cartesiano: {error.message}"
            )

            traceback.print_exc()
            return False

        finally:
            # Se motion_finished=True è già stato inviato,
            # non viene scritto un secondo comando.
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
            )

            time_elapsed = 0.0
            iteration_count = 0
            motion_finished = False

            while not motion_finished:
                iteration_count += 1
                robot_state, delta_time = active_control.readOnce()
                q_last = np.array(robot_state.q)
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

        home_position = [0, 0, 0, -3.03005749, 1.55690476, 1.56836934, -0.29296873]
        print("Ritorno alla posizione home...")

        return self.move_to_joint_positions(
            home_position, speed_factor=speed_factor, progress_callback=progress_callback
        )
