import os
from enum import Enum
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, List

import numpy as np
import pylibfranka


from dataclasses import dataclass
import datetime
from enum import Enum
from typing import Optional, List

class RobotMode(Enum):
    DISCONNECTED = "disconnected"
    READY = "ready"
    RUNNING = "running"
    ERROR_LOCKED = "error_locked"


class RobotErrorType(Enum):
    REALTIME = "realtime_error"
    CONNECTION = "connection_error"
    COMMUNICATION = "communication_error"
    VALIDATION = "validation_error"
    COMMAND = "command_error"
    RECOVERY = "recovery_error"
    UNKNOWN = "unknown_error"


@dataclass
class RobotError:
    operation: str
    error_type: RobotErrorType
    message: str
    timestamp: datetime
    recoverable: bool
    original_exception: Optional[Exception] = None



class FrankaRobot:
    """
    Classe wrapper per pylibfranka.Robot.

    Gestisce:
    - connessione al robot;
    - stato operativo tramite pylibfranka.RobotMode;
    - classificazione e collezione degli errori;
    - blocco dei comandi in caso di errore critico.
    """

    def __init__(
        self,
        robot_ip: Optional[str] = None,
        enforce_realtime: bool = True,
    ):
        self.robot_ip = robot_ip or os.getenv("FRANKA_ROBOT_IP", "172.16.0.2")
        self.enforce_realtime = enforce_realtime

        self._robot: Optional[pylibfranka.Robot] = None

        self.mode: RobotMode = RobotMode.DISCONNECTED
        self.errors: List[RobotError] = []
        self.last_error: Optional[RobotError] = None

        self._connect()

        from .motion import MotionController
        from .gripper import GripperController

        self.motion = MotionController(self)
        self.gripper = GripperController(self.robot_ip)

    # ------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------

    def _connect(self) -> None:
        try:
            realtime_config = (
                pylibfranka.RealtimeConfig.kEnforce
                if self.enforce_realtime
                else pylibfranka.RealtimeConfig.kIgnore
            )

            self._robot = pylibfranka.Robot(
                self.robot_ip,
                realtime_config,
            )

            self.mode = RobotMode.READY

            print(
                f"✓ Connesso al robot Franka all'indirizzo {self.robot_ip} "
                f"con realtime_config={realtime_config}"
            )

        except Exception as e:
            error = self._collect_error(
                operation="_connect",
                exception=e,
                recoverable=False,
            )

            self.mode = RobotMode.ERROR_LOCKED

            raise ConnectionError(
                f"[{error.error_type.value}] Connessione fallita: {error.message}"
            ) from e

    @property
    def robot(self) -> pylibfranka.Robot:
        if self._robot is None:
            raise RuntimeError("Robot non connesso.")
        return self._robot

    # ------------------------------------------------------------
    # Error handling
    # ------------------------------------------------------------

    def _classify_error(self, exception: Exception) ->RobotErrorType:
        msg = str(exception).lower()
        cls_name = exception.__class__.__name__.lower()

        if "realtime" in msg or "priority" in msg or "scheduler" in msg:
            return RobotErrorType.REALTIME

        if (
            "connect" in msg
            or "connection" in msg
            or "network" in msg
            or "net exception" in msg
        ):
            return RobotErrorType.CONNECTION

        if "communication" in msg or "timeout" in msg or "packet" in msg:
            return RobotErrorType.COMMUNICATION

        if isinstance(exception, ValueError):
            return RobotErrorType.VALIDATION

        if "invalid" in msg or "dimension" in msg or "size" in msg:
            return RobotErrorType.VALIDATION

        if "command" in msg or "control" in msg or "motion" in msg:
            return RobotErrorType.COMMAND

        if "recovery" in msg:
            return RobotErrorType.RECOVERY

        return RobotErrorType.UNKNOWN

    def _collect_error(
        self,
        operation: str,
        exception: Exception,
        recoverable: bool,
    ) -> RobotError:
        error = RobotError(
            operation=operation,
            error_type=self._classify_error(exception),
            message=str(exception),
            timestamp=datetime.datetime.now(),
            recoverable=recoverable,
            original_exception=exception,
        )

        self.errors.append(error)
        self.last_error = error

        return error

    def _enter_error_locked(
        self,
        operation: str,
        exception: Exception,
        recoverable: bool = True,
    ) -> RobotError:
        error = self._collect_error(
            operation=operation,
            exception=exception,
            recoverable=recoverable,
        )

        self.mode = RobotMode.ERROR_LOCKED

        print("✗ Robot entrato in ERROR_LOCKED.")
        print(f"Operazione: {operation}")
        print(f"Tipo errore: {error.error_type.value}")
        print(f"Messaggio: {error.message}")

        return error

    # ------------------------------------------------------------
    # Safety gates
    # ------------------------------------------------------------

    def _ensure_connected(self) -> None:
        if self._robot is None or self.mode == RobotMode.DISCONNECTED:
            raise RuntimeError("Robot non connesso.")

    def _ensure_can_command(self) -> None:
        self._ensure_connected()

        if self.mode == RobotMode.ERROR_LOCKED:
            raise RuntimeError(
                "Comando bloccato: il robot è in ERROR_LOCKED. "
                "Eseguire automatic_error_recovery() prima di inviare nuovi comandi."
            )

    def _check_length(self, values: List[float], expected: int, name: str) -> None:
        if len(values) != expected:
            raise ValueError(
                f"{name} deve contenere {expected} valori, ricevuti {len(values)}."
            )

    # ------------------------------------------------------------
    # Read methods
    # ------------------------------------------------------------

    def get_state(self) -> pylibfranka.RobotState:
        """
        Lettura dello stato corrente.

        La lettura è consentita anche in ERROR_LOCKED,
        purché la connessione sia ancora attiva.
        """
        self._ensure_connected()

        try:
            return self.robot.read_once()

        except Exception as e:
            error = self._enter_error_locked(
                operation="get_state/read_once",
                exception=e,
                recoverable=True,
            )

            raise RuntimeError(
                f"[{error.error_type.value}] Errore durante la lettura dello stato: "
                f"{error.message}"
            ) from e

    def get_current_joint_positions(self) -> np.ndarray:
        state = self.get_state()
        return np.array(state.q)

    def get_current_cartesian_pose(self) -> np.ndarray:
        state = self.get_state()
        return np.array(state.O_T_EE).reshape(4, 4, order="F")

    def get_current_cartesian_velocity(self) -> np.ndarray:
        state = self.get_state()
        return np.array(state.O_dP_EE_d)

    def get_external_forces(self) -> np.ndarray:
        state = self.get_state()
        return np.array(state.K_F_ext_hat_K)
    def get_cartesian_collision(self):
        state = self.get_state()
        return np.array(state.cartesian_collision)
    def get_cartesian_contact(self):
        state = self.get_state()
        return np.array(state.cartesian_contact)
    # ------------------------------------------------------------
    # Command/configuration methods
    # ------------------------------------------------------------

    def set_collision_behavior(
        self,
        lower_torque_thresholds: Optional[List[float]] = None,
        upper_torque_thresholds: Optional[List[float]] = None,
        lower_force_thresholds: Optional[List[float]] = None,
        upper_force_thresholds: Optional[List[float]] = None,
    ) -> None:
        self._ensure_can_command()

        if lower_torque_thresholds is None:
            lower_torque_thresholds = [10,10,10,10,10,10,10]#[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]

        if upper_torque_thresholds is None:
            upper_torque_thresholds =  [10,10,10,10,10,10,10]#[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]

        if lower_force_thresholds is None:
            lower_force_thresholds =  [10,10,10,10,10,10]#[20.0, 20.0, 18.0, 25.0, 25.0, 25.0]

        if upper_force_thresholds is None:
            upper_force_thresholds = [10,10,10,10,10,10]#[20.0, 20.0, 18.0, 25.0, 25.0, 25.0]

        try:
            self._check_length(lower_torque_thresholds, 7, "lower_torque_thresholds")
            self._check_length(upper_torque_thresholds, 7, "upper_torque_thresholds")
            self._check_length(lower_force_thresholds, 6, "lower_force_thresholds")
            self._check_length(upper_force_thresholds, 6, "upper_force_thresholds")

            self.robot.set_collision_behavior(
                lower_torque_thresholds,
                upper_torque_thresholds,
                lower_force_thresholds,
                upper_force_thresholds,
            )

            print("✓ Comportamento di collisione configurato")

        except Exception as e:
            error = self._enter_error_locked(
                operation="set_collision_behavior",
                exception=e,
                recoverable=True,
            )

            raise RuntimeError(
                f"[{error.error_type.value}] Errore in set_collision_behavior: "
                f"{error.message}"
            ) from e

    def set_cartesian_impedance(self, stiffness: List[float]) -> None:
        self._ensure_can_command()

        try:
            self._check_length(stiffness, 6, "stiffness cartesiana")

            self.robot.set_cartesian_impedance(stiffness)

            print(f"✓ Impedenza cartesiana impostata: {stiffness}")

        except Exception as e:
            error = self._enter_error_locked(
                operation="set_cartesian_impedance",
                exception=e,
                recoverable=True,
            )

            raise RuntimeError(
                f"[{error.error_type.value}] Errore in set_cartesian_impedance: "
                f"{error.message}"
            ) from e

    def set_joint_impedance(self, stiffness: List[float]) -> None:
        self._ensure_can_command()

        try:
            self._check_length(stiffness, 7, "stiffness giunti")

            self.robot.set_joint_impedance(stiffness)

            print(f"✓ Impedenza giunti impostata: {stiffness}")

        except Exception as e:
            error = self._enter_error_locked(
                operation="set_joint_impedance",
                exception=e,
                recoverable=True,
            )

            raise RuntimeError(
                f"[{error.error_type.value}] Errore in set_joint_impedance: "
                f"{error.message}"
            ) from e

    def set_end_effector_frame(self, transform: List[float]) -> None:
        self._ensure_can_command()

        try:
            self._check_length(transform, 16, "transform EE")

            self.robot.set_EE(transform)

            print("✓ Frame end-effector impostato")

        except Exception as e:
            error = self._enter_error_locked(
                operation="set_end_effector_frame",
                exception=e,
                recoverable=True,
            )

            raise RuntimeError(
                f"[{error.error_type.value}] Errore in set_end_effector_frame: "
                f"{error.message}"
            ) from e

    # ------------------------------------------------------------
    # Recovery
    # ------------------------------------------------------------

    def automatic_error_recovery(self) -> bool:
        """
        Recovery consentito anche in ERROR_LOCKED.
        """
        self._ensure_connected()

        try:
            self.robot.automatic_error_recovery()

            self.mode =RobotMode.READY
            self.last_error = None

            print("✓ Recovery automatico completato. Robot nuovamente READY.")
            return True

        except Exception as e:
            error = self._collect_error(
                operation="automatic_error_recovery",
                exception=e,
                recoverable=False,
            )

            self.mode =RobotMode.ERROR_LOCKED

            print(f"✗ Recovery automatico fallito: {error.message}")
            return False

    # ------------------------------------------------------------
    # Debug utilities
    # ------------------------------------------------------------

    def print_state(self) -> None:
        state = self.get_state()

        print("\n=== Stato Robot Franka ===")
        print(f"Mode: {self.mode.value}")
        print(f"Posizioni giunti [rad]: {np.array(state.q)}")
        print(f"Velocità giunti [rad/s]: {np.array(state.dq)}")

        pose = self.get_current_cartesian_pose()
        pos = pose[:3, 3]

        print(
            f"Posizione EE [m]: "
            f"x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}"
        )

        ext_force = self.get_external_forces()
        print(f"Forze esterne [N, Nm]: {ext_force}")
        print("=" * 30)
        self.cartesian_collision = self.get_cartesian_collision()
        self.cartesian_contact = self.get_cartesian_contact()
        print(f"Collisione cartesiana: {self.cartesian_collision}")
        print(f"Contatto cartesiano: {self.cartesian_contact}")

    def print_errors(self) -> None:
        if not self.errors:
            print("Nessun errore registrato.")
            return

        print("\n=== Errori Robot Registrati ===")

        for i, err in enumerate(self.errors, start=1):
            print(f"\nErrore #{i}")
            print(f"Timestamp: {err.timestamp}")
            print(f"Operation: {err.operation}")
            print(f"Type: {err.error_type.value}")
            print(f"Recoverable: {err.recoverable}")
            print(f"Message: {err.message}")

    def clear_errors(self) -> None:
        self.errors.clear()
        self.last_error = None
        print("✓ Lista errori svuotata")

    # ------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None:
            print(f"[Errore durante l'esecuzione: {exc_val}]")

        pass