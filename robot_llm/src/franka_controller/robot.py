"""
Wrapper ad alto livello per pylibfranka.Robot con gestione errori e funzionalità avanzate.
"""

import os
import numpy as np
from typing import Optional, List, Callable, Dict, Any, Tuple
import pylibfranka


class FrankaRobot:
    """
    Classe wrapper per pylibfranka.Robot che fornisce un'interfaccia ad alto livello
    per controllare il robot Franka Panda.

    Gestisce la connessione, lo stato del robot, impedenza, collision behavior,
    e fornisce accesso ai controller di movimento e gripper.
    """

    def __init__(self, robot_ip: Optional[str] = None):
        """
        Inizializza la connessione con il robot Franka.

        Args:
            robot_ip: Indirizzo IP del robot.
                     Se None, usa FRANKA_ROBOT_IP; fallback: "172.16.0.2"

        Raises:
            RuntimeError: Se la connessione al robot fallisce
        """
        self.robot_ip = robot_ip or os.getenv("FRANKA_ROBOT_IP", "172.16.0.2")
        self._robot: Optional[pylibfranka.Robot] = None
        self._connect()

        # Inizializza i controller
        from .motion import MotionController
        from .gripper import GripperController

        self.motion = MotionController(self)
        self.gripper = GripperController(self.robot_ip)

    def _connect(self) -> None:
        """
        Stabilisce la connessione con il robot.

        Raises:
            RuntimeError: Se la connessione fallisce
        """
        try:
            self._robot = pylibfranka.Robot(self.robot_ip)
            print(f"✓ Connesso al robot Franka all'indirizzo {self.robot_ip}")
        except Exception as e:
            raise ConnectionError(f"Impossibile connettersi al robot: {e}")

    @property
    def robot(self) -> pylibfranka.Robot:
        """Restituisce l'oggetto Robot di pylibfranka"""
        if self._robot is None:
            raise RuntimeError("Robot non connesso")
        return self._robot

    def get_state(self) -> pylibfranka.RobotState:
        """
        Ottiene lo stato corrente del robot usando read_once().

        Returns:
            RobotState contenente tutte le informazioni sul robot
        """
        return self.robot.read_once()

    def get_current_joint_positions(self) -> np.ndarray:
        """Ottiene le posizioni correnti dei giunti.

        Returns:
            Array numpy con le posizioni dei 7 giunti in radianti
        """
        state = self.get_state()
        return np.array(state.q)

    def get_current_cartesian_pose(self) -> np.ndarray:
        """Ottiene la posa cartesiana corrente dell'end-effector (O_T_EE).

        Returns:
            Array 4x4 rappresentante la matrice di trasformazione omogenea
            dalla base del robot (O) all'end-effector (EE)
        """
        state = self.get_state()
        # O_T_EE è un array 16 elementi (column-major 4x4 matrix)
        return np.array(state.O_T_EE).reshape(4, 4, order="F")  # Fortran order

    def get_current_cartesian_velocity(self) -> np.ndarray:
        """
        Ottiene la velocità cartesiana corrente dell'end-effector.

        Returns:
            Array numpy di 6 elementi [vx, vy, vz, wx, wy, wz]
        """
        state = self.get_state()
        return np.array(state.O_dP_EE_d)  # Velocity in base frame (O) expressed in EE frame

    def get_external_forces(self) -> np.ndarray:
        """
        Ottiene le forze esterne stimate nel frame K (stiffness frame).

        Returns:
            Array numpy di 6 elementi [fx, fy, fz, tx, ty, tz]
        """
        state = self.get_state()
        return np.array(state.K_F_ext_hat_K)  # External forces in stiffness frame (K)

    def set_collision_behavior(
        self,
        lower_torque_thresholds: Optional[List[float]] = None,
        upper_torque_thresholds: Optional[List[float]] = None,
        lower_force_thresholds: Optional[List[float]] = None,
        upper_force_thresholds: Optional[List[float]] = None,
    ) -> None:
        """
        Configura il comportamento in caso di collisione.

        Args:
            lower_torque_thresholds: Soglie inferiori di torque per i 7 giunti [Nm]
            upper_torque_thresholds: Soglie superiori di torque per i 7 giunti [Nm]
            lower_force_thresholds: Soglie inferiori di forza cartesiana [fx,fy,fz,tx,ty,tz]
            upper_force_thresholds: Soglie superiori di forza cartesiana [fx,fy,fz,tx,ty,tz]
        """
        # Valori di default sicuri
        if lower_torque_thresholds is None:
            lower_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        if upper_torque_thresholds is None:
            upper_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        if lower_force_thresholds is None:
            lower_force_thresholds = [20.0, 20.0, 18.0, 25.0, 25.0, 25.0]
        if upper_force_thresholds is None:
            upper_force_thresholds = [20.0, 20.0, 18.0, 25.0, 25.0, 25.0]

        try:
            self.robot.set_collision_behavior(
                lower_torque_thresholds,
                upper_torque_thresholds,
                lower_force_thresholds,
                upper_force_thresholds,
            )
            print("✓ Comportamento di collisione configurato")
        except Exception as e:
            print(f"⚠ Errore nella configurazione collision behavior: {e}")

    def set_cartesian_impedance(self, stiffness: List[float]) -> None:
        """
        Imposta l'impedenza cartesiana del robot.

        Args:
            stiffness: Lista di 6 valori [x,y,z,roll,pitch,yaw] che definiscono
                      la rigidità traslazionale e rotazionale [N/m, Nm/rad]
        """
        try:
            self.robot.set_cartesian_impedance(stiffness)
            print(f"✓ Impedenza cartesiana impostata: {stiffness}")
        except Exception as e:
            print(f"⚠ Errore nell'impostazione impedenza cartesiana: {e}")

    def set_joint_impedance(self, stiffness: List[float]) -> None:
        """
        Imposta l'impedenza dei giunti del robot.

        Args:
            stiffness: Lista di 7 valori che definiscono la rigidità di ogni giunto [Nm/rad]
        """
        try:
            self.robot.set_joint_impedance(stiffness)
            print(f"✓ Impedenza giunti impostata: {stiffness}")
        except Exception as e:
            print(f"⚠ Errore nell'impostazione impedenza giunti: {e}")

    def set_load(self, mass: float, center_of_mass: List[float], inertia: List[float]) -> None:
        """
        Imposta i parametri del carico montato sull'end-effector.

        Args:
            mass: Massa del carico in kg
            center_of_mass: Centro di massa [x, y, z] rispetto al flange [m]
            inertia: Matrice di inerzia [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] [kg*m²]
        """
        try:
            self.robot.set_load(mass, center_of_mass, inertia)
            print(f"✓ Carico impostato: massa={mass}kg, CoM={center_of_mass}")
        except Exception as e:
            print(f"⚠ Errore nell'impostazione del carico: {e}")

    def set_end_effector_frame(self, transform: List[float]) -> None:
        """
        Imposta la trasformazione dal flange all'end-effector (NE_T_EE).

        Args:
            transform: Array di 16 elementi rappresentante la matrice 4x4 di trasformazione
                      in column-major order
        """
        try:
            self.robot.set_EE(transform)
            print("✓ Frame end-effector impostato")
        except Exception as e:
            print(f"⚠ Errore nell'impostazione del frame EE: {e}")

    def automatic_error_recovery(self) -> bool:
        """
        Esegue il recovery automatico dagli errori del robot.

        Returns:
            True se il recovery ha avuto successo, False altrimenti
        """
        try:
            self.robot.automatic_error_recovery()
            print("✓ Recovery automatico completato")
            return True
        except Exception as e:
            print(f"✗ Recovery automatico fallito: {e}")
            return False

    def load_model(self) -> Any:
        """
        Carica il modello dinamico del robot.

        Returns:
            Oggetto Model contenente i parametri cinematici e dinamici
        """
        return self.robot.load_model()

    def print_state(self) -> None:
        """Stampa lo stato corrente del robot in modo formattato."""
        state = self.get_state()
        print("\n=== Stato Robot Franka ===")
        print(f"Posizioni giunti [rad]: {np.array(state.q)}")
        print(f"Velocità giunti [rad/s]: {np.array(state.dq)}")

        pose = self.get_current_cartesian_pose()
        pos = pose[:3, 3]
        print(f"Posizione EE [m]: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")

        ext_force = self.get_external_forces()
        print(f"Forze esterne [N, Nm]: {ext_force}")
        print("=" * 30)

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - cleanup se necessario"""
        if exc_type is not None:
            print(f"[Errore durante l'esecuzione: {exc_val}]")
        # pylibfranka gestisce automaticamente la disconnessione
        pass
