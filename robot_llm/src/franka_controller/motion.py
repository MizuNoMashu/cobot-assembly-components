"""
Controller per il movimento del robot Franka usando pylibfranka.
Fornisce interfacce ad alto livello per movimenti in spazio dei giunti e cartesiano.
"""

import time
import numpy as np
from typing import List, Optional, Callable, Dict
import pylibfranka


class MotionController:
    """
    Controller per gestire il movimento del robot Franka con pylibfranka.

    Fornisce funzioni per:
    - Controllo in posizione dei giunti tramite motion generators
    - Controllo di velocità dei giunti
    - Controllo di impedenza cartesiana
    - Movimento basato su coordinate cartesiane (con inverse kinematics approssimata)
    """

    def __init__(self, franka_robot):
        """
        Inizializza il controller di movimento.

        Args:
            franka_robot: Istanza di FrankaRobot
        """
        self.robot = franka_robot

    def move_to_joint_positions(
        self,
        target_positions: List[float],
        speed_factor: float = 0.2,
        tolerance: float = 0.01,
    ) -> bool:
        """
        Muove il robot verso una configurazione target dei giunti usando un motion generator.

        Args:
            target_positions: Lista di 7 posizioni target in radianti
            speed_factor: Fattore di velocità (0.0 - 1.0)
            tolerance: Tolleranza per considerare la posizione raggiunta [rad]

        Returns:
            True se il movimento è completato con successo
        """
        from .utils import validate_joint_positions, compute_minimum_jerk_trajectory

        validate_joint_positions(target_positions)
        target = np.array(target_positions)

        print(f"Movimento verso posizioni target: {np.round(target, 3).tolist()}")

        # Ottieni posizione iniziale
        initial_state = self.robot.get_state()
        q_start = np.array(initial_state.q)

        # Parametri del motion generator
        duration = 5.0 / speed_factor  # Durata del movimento in secondi
        time_start = time.time()

        def motion_generator_callback(robot_state: pylibfranka.RobotState, period: float) -> Dict:
            """
            Callback per generare posizioni target in real-time.
            Ritorna un dizionario con le posizioni dei giunti e un flag finished.
            """
            elapsed = time.time() - time_start
            progress = min(elapsed / duration, 1.0)

            # Traiettoria minimum jerk dal punto iniziale al target
            q_current = q_start + (target - q_start) * compute_minimum_jerk_trajectory(progress)

            finished = progress >= 1.0

            return {"q_goal": q_current.tolist(), "finished": finished}

        try:
            # Avvia il controllo in posizione dei giunti
            self.robot.robot.start_joint_position_control(motion_generator_callback)
            print("✓ Movimento completato")
            return True

        except Exception as e:
            print(f"✗ Errore durante il movimento: {e}")
            return False

    def move_relative(self, delta_positions: List[float], speed_factor: float = 0.2) -> bool:
        """
        Muove il robot in modo relativo rispetto alla posizione corrente.

        Args:
            delta_positions: Incrementi per ogni giunto in radianti
            speed_factor: Fattore di velocità

        Returns:
            True se il movimento è completato con successo
        """
        current = self.robot.get_current_joint_positions()
        target = current + np.array(delta_positions)

        return self.move_to_joint_positions(target.tolist(), speed_factor)

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
        """
        Muove l'end-effector verso una posa cartesiana target.

        NOTA: Questa è un'implementazione semplificata che usa inverse kinematics numerica
        o differenziale. Per IK più robusta, considera di usare librerie esterne come PyKDL.

        Args:
            x, y, z: Coordinate cartesiane in metri
            roll, pitch, yaw: Orientamento in radianti (Euler angles ZYX)
            speed_factor: Fattore di velocità (0.0 - 1.0)

        Returns:
            True se il movimento è completato con successo
        """
        print(
            f"Movimento verso posa cartesiana: x={x:.3f}, y={y:.3f}, z={z:.3f}, "
            + f"roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}"
        )

        print("⚠ Movimento cartesiano richiede inverse kinematics (non implementata)")
        print("  Usa move_to_joint_positions con configurazione pre-calcolata")
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
        """
        Muove l'end-effector relativamente alla posizione corrente.

        Args:
            dx, dy, dz: Delta di posizione in metri
            droll, dpitch, dyaw: Delta di orientamento in radianti
            speed_factor: Fattore di velocità

        Returns:
            True se il movimento è completato con successo
        """
        # Ottieni posa corrente
        current_pose = self.robot.get_current_cartesian_pose()
        current_pos = current_pose[:3, 3]

        # Calcola posa target
        target_x = current_pos[0] + dx
        target_y = current_pos[1] + dy
        target_z = current_pos[2] + dz

        # Per ora ignoriamo i delta di orientamento
        print(f"Movimento cartesiano relativo: dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f}")

        return self.move_to_cartesian_pose(
            target_x, target_y, target_z, droll, dpitch, dyaw, speed_factor
        )

    def impedance_control(
        self,
        target_positions: List[float],
        stiffness: Optional[List[float]] = None,
        duration: float = 5.0,
    ) -> bool:
        """
        Esegue un movimento con controllo impedenza dei giunti.

        Imposta l'impedenza e poi muove verso la configurazione target usando
        un motion generator.

        Args:
            target_positions: Posizioni target dei giunti [rad]
            stiffness: Rigidità dei giunti [Nm/rad]. Se None, usa valori di default
            duration: Durata del movimento in secondi

        Returns:
            True se il movimento è completato con successo
        """
        from .utils import validate_joint_positions

        validate_joint_positions(target_positions)

        # Imposta impedenza
        if stiffness is None:
            stiffness = [600, 600, 600, 600, 250, 150, 50]  # Valori di default

        self.robot.set_joint_impedance(stiffness)

        print(f"Controllo impedenza verso: {np.round(target_positions, 3).tolist()}")
        print(f"Stiffness: {stiffness}")

        # Muovi con velocità adattata alla durata
        speed_factor = min(0.5, 5.0 / duration) if duration > 0 else 0.2

        return self.move_to_joint_positions(target_positions, speed_factor)

    def velocity_control(
        self,
        velocity_callback: Callable[[pylibfranka.RobotState, float], List[float]],
        duration: float = 5.0,
    ) -> bool:
        """
        Esegue controllo di velocità dei giunti usando un callback.

        Args:
            velocity_callback: Funzione che riceve (RobotState, period: float) e
                             ritorna velocità desiderate [rad/s] per i 7 giunti
            duration: Durata del controllo in secondi

        Returns:
            True se il controllo è completato con successo
        """
        print(f"Avvio controllo velocità per {duration}s")

        time_start = time.time()

        def velocity_generator(robot_state: pylibfranka.RobotState, period: float) -> Dict:
            """Wrapper del callback dell'utente."""
            elapsed = time.time() - time_start

            if elapsed >= duration:
                return {"dq_goal": [0.0] * 7, "finished": True}

            dq = velocity_callback(robot_state, period)
            return {"dq_goal": dq, "finished": False}

        try:
            self.robot.robot.start_joint_velocity_control(velocity_generator)
            print("✓ Controllo velocità completato")
            return True
        except Exception as e:
            print(f"✗ Errore durante controllo velocità: {e}")
            return False

    def go_to_home(self, speed_factor: float = 0.2) -> bool:
        """
        Porta il robot alla configurazione home standard.

        Args:
            speed_factor: Fattore di velocità

        Returns:
            True se il movimento è completato con successo
        """
        # Configurazione home tipica per Franka Panda
        home_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

        print("Ritorno alla posizione home...")
        return self.move_to_joint_positions(home_position, speed_factor)
