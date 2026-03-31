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
        Muove il robot verso una configurazione target dei giunti usando un controllo esterno.
        Muove il robot verso una configurazione target dei giunti usando un controllo esterno.

        Args:
            target_positions: Lista di 7 posizioni target in radianti
            speed_factor: Fattore di velocità (0.0 - 1.0)
            tolerance: Tolleranza per considerare la posizione raggiunta [rad]

        Returns:
            True se il movimento è completato con successo
        """
        from .utils import compute_minimum_jerk_trajectory

        target = np.array(target_positions)

        print(f"[MOTION] Movimento verso posizioni target: {np.round(target, 3).tolist()}")
        print(f"[MOTION] Speed factor: {speed_factor}, Tolerance: {tolerance}")

        try:
            # Ottieni posizione iniziale PRIMA di avviare il controllo
            # (read_once non è compatibile con un'operazione di controllo attiva)
            print("[MOTION] Leggendo posizione iniziale...")
            initial_state = self.robot.get_state()
            q_start = np.array(initial_state.q)
            print(f"[MOTION] Posizione iniziale: {np.round(q_start, 3).tolist()}")

            # Avvia il controllo in posizione dei giunti
            print("[MOTION] Avviando controllo posizione giunti con CartesianImpedance...")
            active_control = self.robot.robot.start_joint_position_control(
                pylibfranka.ControllerMode.CartesianImpedance
            )
            print("[MOTION] ✓ Controllo posizione avviato")

            # Parametri del movimento
            duration = 5.0 / speed_factor
            time_elapsed = 0.0
            motion_finished = False
            iteration_count = 0
            monitoring_log = []  # Buffer per log senza I/O bloccante nel loop

            print(f"[MOTION] Durata prevista: {duration:.2f}s")
            print("[MOTION] --- Inizio loop di controllo ---")

            # Loop di controllo esterno
            while not motion_finished:
                iteration_count += 1
                
                # Leggi stato del robot
                robot_state, delta_time = active_control.readOnce()
                
                # Aggiorna tempo
                time_elapsed += delta_time.to_sec()
                progress = min(time_elapsed / duration, 1.0)

                # Calcola posizione intermedia con traiettoria minimum jerk
                q_current = q_start + (target - q_start) * compute_minimum_jerk_trajectory(progress)

                # Crea comando di posizione
                joint_cmd = pylibfranka.JointPositions(q_current.tolist())
                
                # Imposta flag di movimento finito e raccogli dati di monitoraggio (NO print nel loop!)
                if progress >= 1.0:
                    joint_cmd.motion_finished = True
                    motion_finished = True
                    monitoring_log.append(
                        f"[MOTION] Iter {iteration_count} | Progress: 100.0% | Tempo: {time_elapsed:.3f}s | ✓ MOVIMENTO FINITO"
                    )
                    monitoring_log.append(f"[MOTION] Comando finale: {np.round(q_current, 3).tolist()}")
                else:
                    joint_cmd.motion_finished = False
                    # Raccogli dati ogni 20 iterazioni (ma NO print - potrebbe causare communication_constraints_violation)
                    if iteration_count % 20 == 0 or iteration_count == 1:
                        progress_pct = progress * 100
                        delta_to_target = target - q_current
                        max_delta = np.max(np.abs(delta_to_target))
                        monitoring_log.append(
                            f"[MOTION] Iter {iteration_count:4d} | Progress: {progress_pct:5.1f}% | Tempo: {time_elapsed:6.3f}s | Max delta: {max_delta:.4f}rad | dT: {delta_time.to_sec():.4f}s"
                        )

                # Invia comando al robot (CRITICO: nessun I/O bloccante qui)
                active_control.writeOnce(joint_cmd)

            # Stampa tutti i log DOPO il loop di controllo (I/O non critico)
            for log_line in monitoring_log:
                print(log_line)
            
            print(f"[MOTION] --- Fine loop di controllo (totale {iteration_count} iterazioni) ---")
            print(f"[MOTION] Posizione finale raggiunta: {np.round(q_current, 3).tolist()}")
            print("✓ Movimento completato con successo")
            return True

        except Exception as e:
            print(f"✗ Errore durante il movimento: {e}")
            import traceback
            traceback.print_exc()
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
        print(f"Movimento relativo: Δ{np.round(delta_positions, 3).tolist()}")
        print(f"Posizione target: {np.round(target, 3).tolist()}")
        
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
            velocity_callback: Funzione che riceve (RobotState, delta_time: float) e
            velocity_callback: Funzione che riceve (RobotState, delta_time: float) e
                             ritorna velocità desiderate [rad/s] per i 7 giunti
            duration: Durata del controllo in secondi

        Returns:
            True se il controllo è completato con successo
        """
        print(f"Avvio controllo velocità per {duration}s")

        try:
            # Avvia il controllo di velocità dei giunti
            active_control = self.robot.robot.start_joint_velocity_control(
                pylibfranka.ControllerMode.CartesianImpedance
            )

            time_elapsed = 0.0
            motion_finished = False

            while not motion_finished:
                # Leggi stato del robot
                robot_state, delta_time = active_control.readOnce()
                
                # Aggiorna tempo
                time_elapsed += delta_time.to_sec()

                # Chiama il callback dell'utente
                dq = velocity_callback(robot_state, delta_time.to_sec())

                # Crea comando di velocità
                velocity_cmd = pylibfranka.JointVelocities(dq)
                
                # Verifica se abbiamo raggiunto il tempo massimo
                if time_elapsed >= duration:
                    velocity_cmd.motion_finished = True
                    motion_finished = True
                else:
                    velocity_cmd.motion_finished = False

                # Invia comando al robot
                active_control.writeOnce(velocity_cmd)

        try:
            # Avvia il controllo di velocità dei giunti
            active_control = self.robot.robot.start_joint_velocity_control(
                pylibfranka.ControllerMode.CartesianImpedance
            )

            time_elapsed = 0.0
            motion_finished = False

            while not motion_finished:
                # Leggi stato del robot
                robot_state, delta_time = active_control.readOnce()
                
                # Aggiorna tempo
                time_elapsed += delta_time.to_sec()

                # Chiama il callback dell'utente
                dq = velocity_callback(robot_state, delta_time.to_sec())

                # Crea comando di velocità
                velocity_cmd = pylibfranka.JointVelocities(dq)
                
                # Verifica se abbiamo raggiunto il tempo massimo
                if time_elapsed >= duration:
                    velocity_cmd.motion_finished = True
                    motion_finished = True
                else:
                    velocity_cmd.motion_finished = False

                # Invia comando al robot
                active_control.writeOnce(velocity_cmd)

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
        home_position = [0, 0, 0, -3.03005749, 1.55690476, 1.56836934, -0.29296873]

        print("Ritorno alla posizione home...")
        return self.move_to_joint_positions(home_position, speed_factor)
