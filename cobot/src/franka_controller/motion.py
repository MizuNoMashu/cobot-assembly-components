"""
Controller per il movimento del robot Franka usando pylibfranka.
"""

import traceback
import numpy as np
from typing import List, Optional, Callable, Tuple

import pylibfranka


def _rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, np.cos(roll), -np.sin(roll)],
        [0.0, np.sin(roll), np.cos(roll)],
    ])
    Ry = np.array([
        [np.cos(pitch), 0.0, np.sin(pitch)],
        [0.0, 1.0, 0.0],
        [-np.sin(pitch), 0.0, np.cos(pitch)],
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0.0],
        [np.sin(yaw), np.cos(yaw), 0.0],
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
    q = q / np.linalg.norm(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)],
    ], dtype=float)


def _slerp_quaternion(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    dot = np.dot(q0, q1)
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    return (s0 * q0) + (s1 * q1)


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
    pose = np.eye(4, dtype=float)
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
        """
        Muove il robot verso una configurazione target dei giunti.

        progress_callback viene chiamata ad ogni iterazione del loop real-time
        con un dict di telemetria live. Se None, i dati vengono solo stampati.
        """
        from .utils import compute_minimum_jerk_trajectory

        self.robot._ensure_can_command()

        try:
            self._validate_joint_vector(target_positions, "target_positions")
            self._validate_speed_factor(speed_factor)

            target = np.array(target_positions)

            print(f"[MOTION] Movimento verso posizioni target: {np.round(target, 3).tolist()}")
            print(f"[MOTION] Speed factor: {speed_factor}, Tolerance: {tolerance}")
            print("[MOTION] Leggendo posizione iniziale...")

            initial_state = self.robot.get_state()
            q_start = np.array(initial_state.q)

            print(f"[MOTION] Posizione iniziale: {np.round(q_start, 3).tolist()}")

            self.robot.mode = self.robot.mode.__class__.RUNNING
            active_control = None
            q_last: Optional[np.ndarray] = None

            print("[MOTION] Avviando controllo posizione giunti...")
            active_control = self.robot.robot.start_joint_position_control(
                pylibfranka.ControllerMode.CartesianImpedance
            )
            print("[MOTION] ✓ Controllo posizione avviato")

            duration = 5.0 / speed_factor
            time_elapsed = 0.0
            motion_finished = False
            iteration_count = 0
            monitoring_log = []
            previous_measured_q = q_start.copy()

            print(f"[MOTION] Durata prevista: {duration:.2f}s")
            print("[MOTION] --- Inizio loop di controllo ---")

            while not motion_finished:
                iteration_count += 1

                robot_state, delta_time = active_control.readOnce()
                cartesian_contact = np.array(robot_state.cartesian_contact, dtype=bool)
                cartesian_collision = np.array(robot_state.cartesian_collision, dtype=bool)

                if np.any(cartesian_collision):
                    hold_cmd = pylibfranka.JointPositions(list(robot_state.q))
                    hold_cmd.motion_finished = True
                    active_control.writeOnce(hold_cmd)
                    raise RuntimeError("Collisione cartesiana rilevata: movimento interrotto.")

                if np.any(cartesian_contact):
                    hold_cmd = pylibfranka.JointPositions(list(robot_state.q))
                    hold_cmd.motion_finished = True
                    active_control.writeOnce(hold_cmd)
                    raise RuntimeError("Contatto cartesiano rilevato: movimento interrotto.")

                time_elapsed += delta_time.to_sec()
                progress = min(time_elapsed / duration, 1.0)

                s = compute_minimum_jerk_trajectory(progress)
                q_current = q_start + (target - q_start) * s
                q_measured = np.array(robot_state.q)
                measured_step = q_measured - previous_measured_q
                previous_measured_q = q_measured.copy()

                joint_cmd = pylibfranka.JointPositions(q_current.tolist())
                progress_pct = progress * 100.0
                max_delta_cmd = float(np.max(np.abs(target - q_current)))

 
                print(f"[MOTION][LIVE] q_measured: {np.round(q_measured, 5).tolist()}")
                print(f"[MOTION][LIVE] dq_measured: {np.round(measured_step, 5).tolist()}")
                print(f"[MOTION][LIVE] q_command : {np.round(q_current, 5).tolist()}")
                print(f"[MOTION][LIVE] Contact: {cartesian_contact.tolist()} | Collision: {cartesian_collision.tolist()}")

                if progress_callback is not None:
                    progress_callback({
                        "type": "motion_progress",
                        "iteration": iteration_count,
                        "progress": round(progress_pct, 2),
                        "time_elapsed": round(time_elapsed, 4),
                        "delta_time": round(delta_time.to_sec(), 6),
                        "q_measured": np.round(q_measured, 5).tolist(),
                        "dq_measured": np.round(measured_step, 5).tolist(),
                        "q_command": np.round(q_current, 5).tolist(),
                        "contact": cartesian_contact.tolist(),
                        "collision": cartesian_collision.tolist(),
                        "max_delta_cmd": round(max_delta_cmd, 6),
                    })

                if progress >= 1.0:
                    joint_cmd.motion_finished = True
                    motion_finished = True
                    monitoring_log.append(
                        f"[MOTION] Iter {iteration_count} | Progress: 100.0% | "
                        f"Tempo: {time_elapsed:.3f}s | ✓ MOVIMENTO FINITO"
                    )
                else:
                    joint_cmd.motion_finished = False
                    if iteration_count % 20 == 0 or iteration_count == 1:
                        monitoring_log.append(
                            f"[MOTION] Iter {iteration_count:4d} | "
                            f"Progress: {progress_pct:5.1f}% | "
                            f"Tempo: {time_elapsed:6.3f}s | "
                            f"Max delta: {np.max(np.abs(target - q_current)):.4f} rad | "
                            f"dT: {delta_time.to_sec():.4f}s"
                        )

                active_control.writeOnce(joint_cmd)


            final_state = self.robot.get_state()
            q_final_real = np.array(final_state.q)
            final_error_vector = target - q_final_real
            final_error = np.max(np.abs(final_error_vector))

            print(f"[MOTION] --- Fine loop ({iteration_count} iterazioni) ---")
            print(f"[MOTION] Errore finale massimo: {final_error:.6f} rad")

            if final_error > tolerance:
                raise RuntimeError(
                    f"Tolleranza non rispettata: errore={final_error:.6f} rad, "
                    f"tolleranza={tolerance:.6f} rad."
                )

            self.robot.mode = self.robot.mode.__class__.READY
            print("✓ Movimento completato con successo")
            return True

        except pylibfranka.ControlException as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_to_joint_positions (ControlException)",
                exception=e,
                recoverable=False,
            )
            print(f"✗ ControlException durante il movimento: {error.message}")
            if "cartesian_reflex" in str(e):
                print("✗ [SAFETY] Cartesian reflex: contatto rilevato.")
            traceback.print_exc()
            return False

        except RuntimeError as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_to_joint_positions (RuntimeError)",
                exception=e,
                recoverable=False,
            )
            print(f"✗ Errore critico durante il movimento: {error.message}")
            traceback.print_exc()
            return False

        except Exception as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_to_joint_positions (Exception)",
                exception=e,
                recoverable=True,
            )
            print(f"✗ Errore durante il movimento: {error.message}")
            traceback.print_exc()
            return False

        finally:
            if active_control is not None:
                try:
                    if q_last is None:
                        q_last = np.array(self.robot.get_state().q)
                    hold_cmd = pylibfranka.JointPositions(q_last.tolist())
                    hold_cmd.motion_finished = True
                    active_control.writeOnce(hold_cmd)
                except Exception as cleanup_exc:
                    print(f"[MOTION] Cleanup active control failed: {cleanup_exc}")
                try:
                    self.robot.robot.stop()
                except Exception as cleanup_exc:
                    print(f"[MOTION] Robot.stop() failed during cleanup: {cleanup_exc}")

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
        progress_callback: Optional[Callable[[dict], None]] = None,
    ) -> bool:
        """
        Muove l'end-effector verso una posa cartesiana target.

        Il target è espresso come posizione + orientamento RPY.
        """
        self.robot._ensure_can_command()

        try:
            self._validate_speed_factor(speed_factor)

            target_pose = _build_cartesian_pose(x, y, z, roll, pitch, yaw)
            current_pose = self.robot.get_current_cartesian_pose()
            current_rot = current_pose[:3, :3]
            current_quat = _rotation_matrix_to_quaternion(current_rot)
            target_quat = _rotation_matrix_to_quaternion(target_pose[:3, :3])

            print(f"[MOTION] Movimento cartesiano verso posa target: {[x, y, z, roll, pitch, yaw]}")
            print(f"[MOTION] Speed factor: {speed_factor}, Tolerance: {tolerance}")
            print("[MOTION] Posa iniziale:")
            print(current_pose)

            self.robot.mode = self.robot.mode.__class__.RUNNING
            active_control = self.robot.robot.start_cartesian_pose_control(
                pylibfranka.ControllerMode.CartesianImpedance
            )
            print("[MOTION] ✓ Controllo cartesiano avviato")

            duration = 5.0 / speed_factor
            time_elapsed = 0.0
            motion_finished = False
            iteration_count = 0
            pose_last: Optional[np.ndarray] = None

            while not motion_finished:
                iteration_count += 1
                robot_state, delta_time = active_control.readOnce()
                time_elapsed += delta_time.to_sec()
                progress = min(time_elapsed / duration, 1.0)
                s = compute_minimum_jerk_trajectory(progress)

                position_command = current_pose[:3, 3] + (target_pose[:3, 3] - current_pose[:3, 3]) * s
                interp_quat = _slerp_quaternion(current_quat, target_quat, s)
                rotation_command = _quaternion_to_rotation_matrix(interp_quat)

                pose_command = np.eye(4, dtype=float)
                pose_command[:3, :3] = rotation_command
                pose_command[:3, 3] = position_command

                cartesian_cmd = _pose_to_cartesian_command(pose_command)
                cartesian_cmd.motion_finished = progress >= 1.0

                robot_pose = np.array(robot_state.O_T_EE).reshape(4, 4, order="F")
                q_measured = np.array(robot_state.q)
                position_error = np.linalg.norm(target_pose[:3, 3] - robot_pose[:3, 3])
                orientation_error = np.arccos(
                    np.clip(
                        np.dot(
                            _rotation_matrix_to_quaternion(robot_pose[:3, :3]),
                            target_quat,
                        ),
                        -1.0,
                        1.0,
                    )
                )

                if progress_callback is not None:
                    progress_callback({
                        "type": "cartesian_progress",
                        "iteration": iteration_count,
                        "progress": round(progress * 100.0, 2),
                        "time_elapsed": round(time_elapsed, 4),
                        "pose_command": pose_command.tolist(),
                        "pose_measured": robot_pose.tolist(),
                        "position_error": round(float(position_error), 6),
                        "orientation_error": round(float(orientation_error), 6),
                    })

                if np.any(np.array(robot_state.cartesian_collision, dtype=bool)):
                    hold_cmd = _pose_to_cartesian_command(robot_pose)
                    hold_cmd.motion_finished = True
                    active_control.writeOnce(hold_cmd)
                    raise RuntimeError("Collisione cartesiana rilevata: movimento interrotto.")

                if np.any(np.array(robot_state.cartesian_contact, dtype=bool)):
                    hold_cmd = _pose_to_cartesian_command(robot_pose)
                    hold_cmd.motion_finished = True
                    active_control.writeOnce(hold_cmd)
                    raise RuntimeError("Contatto cartesiano rilevato: movimento interrotto.")

                if progress >= 1.0:
                    motion_finished = True
                    pose_last = robot_pose
                else:
                    pose_last = robot_pose

                active_control.writeOnce(cartesian_cmd)

            final_pose = self.robot.get_current_cartesian_pose()
            position_error = np.linalg.norm(target_pose[:3, 3] - final_pose[:3, 3])
            if position_error > tolerance:
                raise RuntimeError(
                    f"Tolleranza non rispettata: errore posizione={position_error:.6f} m, "
                    f"tolleranza={tolerance:.6f} m."
                )

            self.robot.mode = self.robot.mode.__class__.READY
            print("✓ Movimento cartesiano completato con successo")
            return True

        except pylibfranka.ControlException as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_to_cartesian_pose (ControlException)",
                exception=e,
                recoverable=False,
            )
            print(f"✗ ControlException durante il movimento cartesiano: {error.message}")
            traceback.print_exc()
            return False

        except RuntimeError as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_to_cartesian_pose (RuntimeError)",
                exception=e,
                recoverable=False,
            )
            print(f"✗ Errore critico durante il movimento cartesiano: {error.message}")
            traceback.print_exc()
            return False

        except Exception as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_to_cartesian_pose (Exception)",
                exception=e,
                recoverable=True,
            )
            print(f"✗ Errore durante il movimento cartesiano: {error.message}")
            traceback.print_exc()
            return False

        finally:
            if 'active_control' in locals() and active_control is not None:
                try:
                    final_pose = pose_last if pose_last is not None else self.robot.get_current_cartesian_pose()
                    hold_cmd = _pose_to_cartesian_command(final_pose)
                    hold_cmd.motion_finished = True
                    active_control.writeOnce(hold_cmd)
                except Exception as cleanup_exc:
                    print(f"[MOTION] Cleanup active control failed: {cleanup_exc}")
                try:
                    self.robot.robot.stop()
                except Exception as cleanup_exc:
                    print(f"[MOTION] Robot.stop() failed during cleanup: {cleanup_exc}")

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
