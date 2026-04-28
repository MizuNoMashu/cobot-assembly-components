"""
Controller per il movimento del robot Franka usando pylibfranka.
Fornisce interfacce ad alto livello per movimenti in spazio dei giunti e cartesiano.
"""

import traceback
import numpy as np
from typing import List, Optional, Callable

import pylibfranka


class MotionController:
    """
    Controller per gestire il movimento del robot Franka con pylibfranka.
    """

    def __init__(self, franka_robot):
        """
        Args:
            franka_robot: Istanza di FrankaRobot
        """
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
    ) -> bool:
        """
        Muove il robot verso una configurazione target dei giunti.
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

                # Legge stato robot e timing del ciclo corrente.
                robot_state, delta_time = active_control.readOnce()
                cartesian_contact = np.array(robot_state.cartesian_contact, dtype=bool)
                cartesian_collision = np.array(robot_state.cartesian_collision, dtype=bool)

                if np.any(cartesian_collision):
                    hold_cmd = pylibfranka.JointPositions(list(robot_state.q))
                    hold_cmd.motion_finished = True
                    active_control.writeOnce(hold_cmd)
                    raise RuntimeError(...)

                if np.any(cartesian_contact):
                    hold_cmd = pylibfranka.JointPositions(list(robot_state.q))
                    hold_cmd.motion_finished = True
                    active_control.writeOnce(hold_cmd)
                    raise RuntimeError(...)

                time_elapsed += delta_time.to_sec()
                progress = min(time_elapsed / duration, 1.0)

                # Interpolazione minimum-jerk tra posa iniziale e target.
                s = compute_minimum_jerk_trajectory(progress)
                q_current = q_start + (target - q_start) * s
                q_measured = np.array(robot_state.q)
                measured_step = q_measured - previous_measured_q
                previous_measured_q = q_measured.copy()

                joint_cmd = pylibfranka.JointPositions(q_current.tolist())
                progress_pct = progress * 100.0
                max_delta_cmd = float(np.max(np.abs(target - q_current)))

                # Stampa live ad ogni iterazione: andamento e variazione giunti.
                print(
                    f"[MOTION][LIVE] Iter {iteration_count:4d} | "
                    f"Progress: {progress_pct:6.2f}% | "
                    f"Tempo: {time_elapsed:7.4f}s | "
                    f"dT: {delta_time.to_sec():.4f}s | "
                    f"Max delta cmd: {max_delta_cmd:.6f} rad"
                )
                print(f"[MOTION][LIVE] q_measured: {np.round(q_measured, 5).tolist()}")
                print(f"[MOTION][LIVE] dq_measured: {np.round(measured_step, 5).tolist()}")
                print(f"[MOTION][LIVE] q_command : {np.round(q_current, 5).tolist()}")

                if progress >= 1.0:
                    joint_cmd.motion_finished = True
                    motion_finished = True

                    monitoring_log.append(
                        f"[MOTION] Iter {iteration_count} | "
                        f"Progress: 100.0% | Tempo: {time_elapsed:.3f}s | "
                        f"✓ MOVIMENTO FINITO"
                    )
                    monitoring_log.append(
                        f"[MOTION] Comando finale: {np.round(q_current, 3).tolist()}"
                    )

                else:
                    joint_cmd.motion_finished = False

                    if iteration_count % 20 == 0 or iteration_count == 1:
                        progress_pct = progress * 100
                        delta_to_target = target - q_current
                        max_delta = np.max(np.abs(delta_to_target))

                        monitoring_log.append(
                            f"[MOTION] Iter {iteration_count:4d} | "
                            f"Progress: {progress_pct:5.1f}% | "
                            f"Tempo: {time_elapsed:6.3f}s | "
                            f"Max delta: {max_delta:.4f} rad | "
                            f"dT: {delta_time.to_sec():.4f}s"
                        )

                active_control.writeOnce(joint_cmd)

            for log_line in monitoring_log:
                print(log_line)

            # Lettura dello stato reale finale del robot.
            # Nota: questa lettura viene fatta DOPO la fine del controllo attivo.
            final_state = self.robot.get_state()
            q_final_real = np.array(final_state.q)

            final_error_vector = target - q_final_real
            final_error = np.max(np.abs(final_error_vector))

            print(f"[MOTION] --- Fine loop di controllo (totale {iteration_count} iterazioni) ---")
            print(f"[MOTION] Posizione finale comandata: {np.round(q_current, 3).tolist()}")
            print(f"[MOTION] Posizione finale reale: {np.round(q_final_real, 3).tolist()}")
            print(
                f"[MOTION] Errore finale per giunto: {np.round(final_error_vector, 6).tolist()} rad"
            )
            print(f"[MOTION] Errore finale massimo reale: {final_error:.6f} rad")

            if final_error > tolerance:
                raise RuntimeError(
                    f"Movimento terminato ma tolleranza non rispettata. "
                    f"Errore massimo reale: {final_error:.6f} rad, "
                    f"tolleranza: {tolerance:.6f} rad."
                )

            self.robot.mode = self.robot.mode.__class__.READY

            print("✓ Movimento completato con successo")
            return True

        except Exception as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_to_joint_positions",
                exception=e,
                recoverable=True,
            )

            print(f"✗ Errore durante il movimento: {error.message}")
            traceback.print_exc()
            return False

    # ------------------------------------------------------------
    # Relative joint motion
    # ------------------------------------------------------------

    def move_relative(
        self,
        delta_positions: List[float],
        speed_factor: float = 0.2,
    ) -> bool:
        """Esegue un movimento relativo in spazio giunti partendo dalla posa attuale."""
        self.robot._ensure_can_command()

        try:
            self._validate_joint_vector(delta_positions, "delta_positions")
            self._validate_speed_factor(speed_factor)

            current = self.robot.get_current_joint_positions()
            target = current + np.array(delta_positions)

            print(f"Movimento relativo: Δ{np.round(delta_positions, 3).tolist()}")
            print(f"Posizione target: {np.round(target, 3).tolist()}")

            return self.move_to_joint_positions(target.tolist(), speed_factor)

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

    def move_to_cartesian_pose(
        self,
        x: float,
        y: float,
        z: float,
        roll: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        speed_factor: float = 0.2,
    ) -> bool:
        """Placeholder per movimento cartesiano assoluto (IK non ancora implementata)."""
        self.robot._ensure_can_command()

        try:
            self._validate_speed_factor(speed_factor)

            print(
                f"Movimento verso posa cartesiana: "
                f"x={x:.3f}, y={y:.3f}, z={z:.3f}, "
                f"roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}"
            )

            raise NotImplementedError(
                "Movimento cartesiano non implementato: serve inverse kinematics."
            )

        except Exception as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_to_cartesian_pose",
                exception=e,
                recoverable=False,
            )

            print(f"✗ Errore movimento cartesiano: {error.message}")
            return False

    def move_cartesian_relative(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        droll: float = 0.0,
        dpitch: float = 0.0,
        dyaw: float = 0.0,
        speed_factor: float = 0.2,
    ) -> bool:
        """Applica un delta cartesiano rispetto alla posa corrente dell'end-effector."""
        self.robot._ensure_can_command()

        try:
            current_pose = self.robot.get_current_cartesian_pose()
            current_pos = current_pose[:3, 3]

            target_x = current_pos[0] + dx
            target_y = current_pos[1] + dy
            target_z = current_pos[2] + dz

            print(f"Movimento cartesiano relativo: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")

            return self.move_to_cartesian_pose(
                target_x,
                target_y,
                target_z,
                droll,
                dpitch,
                dyaw,
                speed_factor,
            )

        except Exception as e:
            error = self.robot._enter_error_locked(
                operation="motion/move_cartesian_relative",
                exception=e,
                recoverable=False,
            )

            print(f"✗ Errore movimento cartesiano relativo: {error.message}")
            return False

    # ------------------------------------------------------------
    # Impedance control
    # ------------------------------------------------------------

    def impedance_control(
        self,
        target_positions: List[float],
        stiffness: Optional[List[float]] = None,
        duration: float = 5.0,
    ) -> bool:
        """Configura impedenza giunti e raggiunge il target con traiettoria a giunti."""
        self.robot._ensure_can_command()

        try:
            self._validate_joint_vector(target_positions, "target_positions")
            self._validate_duration(duration)

            if stiffness is None:
                stiffness = [600, 600, 600, 600, 250, 150, 50]

            self._validate_joint_vector(stiffness, "stiffness")

            self.robot.set_joint_impedance(stiffness)

            print(f"Controllo impedenza verso: {np.round(target_positions, 3).tolist()}")
            print(f"Stiffness: {stiffness}")

            speed_factor = min(0.5, 5.0 / duration)

            return self.move_to_joint_positions(
                target_positions,
                speed_factor=speed_factor,
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
    ) -> bool:
        """Controllo in velocita': ad ogni ciclo il callback fornisce le dq da comandare."""
        self.robot._ensure_can_command()

        try:
            self._validate_duration(duration)

            print(f"Avvio controllo velocità per {duration}s")

            self.robot.mode = self.robot.mode.__class__.RUNNING

            active_control = self.robot.robot.start_joint_velocity_control(
                pylibfranka.ControllerMode.CartesianImpedance
            )

            time_elapsed = 0.0
            motion_finished = False

            while not motion_finished:
                robot_state, delta_time = active_control.readOnce()

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

    # ------------------------------------------------------------
    # Home
    # ------------------------------------------------------------

    def go_to_home(self, speed_factor: float = 0.2) -> bool:
        """Riporta il robot alla configurazione home predefinita."""
        self.robot._ensure_can_command()

        home_position = [
            0,
            0,
            0,
            -3.03005749,
            1.55690476,
            1.56836934,
            -0.29296873,
        ]

        print("Ritorno alla posizione home...")

        return self.move_to_joint_positions(
            home_position,
            speed_factor=speed_factor,
        )
