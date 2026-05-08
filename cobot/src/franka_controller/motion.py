"""
Controller per il movimento del robot Franka usando pylibfranka.
"""

from copy import error
import traceback
import numpy as np
from typing import List, Optional, Callable

import pylibfranka


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
    # Cartesian placeholder
    # ------------------------------------------------------------

    def move_to_cartesian_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, speed_factor=0.2):
        self.robot._ensure_can_command()
        raise NotImplementedError("Movimento cartesiano non implementato: serve inverse kinematics.")

    def move_cartesian_relative(self, dx=0.0, dy=0.0, dz=0.0, droll=0.0, dpitch=0.0, dyaw=0.0, speed_factor=0.2):
        self.robot._ensure_can_command()
        raise NotImplementedError("Movimento cartesiano non implementato: serve inverse kinematics.")

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
