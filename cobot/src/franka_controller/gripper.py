"""
Controller per il gripper del robot Franka usando pylibfranka.Gripper.
"""

from typing import Optional, Dict, Any, List
from enum import Enum
from dataclasses import dataclass
from datetime import datetime

import pylibfranka

from dataclasses import dataclass
import datetime
from enum import Enum

from typing import Optional
class GripperMode(Enum):
    DISCONNECTED = "disconnected"
    READY = "ready"
    RUNNING = "running"
    ERROR_LOCKED = "error_locked"


class GripperErrorType(Enum):
    CONNECTION = "connection_error"
    COMMUNICATION = "communication_error"
    VALIDATION = "validation_error"
    COMMAND = "command_error"
    HOMING = "homing_error"
    UNKNOWN = "unknown_error"


@dataclass
class GripperError:
    operation: str
    error_type: GripperErrorType
    message: str
    timestamp: datetime
    recoverable: bool
    original_exception: Optional[Exception] = None
class GripperController:
    """
    Controller per gestire il gripper del robot Franka con pylibfranka.

    Include:
    - stato operativo del gripper;
    - collezione degli errori;
    - blocco dei comandi in caso di errore critico.
    """

    def __init__(self, robot_ip: str):
        self.robot_ip = robot_ip
        self._gripper: Optional[pylibfranka.Gripper] = None
        self._is_homed = False

        self.mode: GripperMode = GripperMode.DISCONNECTED
        self.errors: List[GripperError] = []
        self.last_error: Optional[GripperError] = None

        self._connect()

    # ------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------

    def _connect(self) -> None:
        try:
            self._gripper = pylibfranka.Gripper(self.robot_ip)
            self.mode = GripperMode.READY
            print("✓ Connesso al gripper Franka")

        except Exception as e:
            error = self._enter_error_locked(
                operation="_connect",
                exception=e,
                recoverable=False,
            )

            raise ConnectionError(
                f"[{error.error_type.value}] Impossibile connettersi al gripper: "
                f"{error.message}"
            ) from e

    @property
    def gripper(self) -> pylibfranka.Gripper:
        if self._gripper is None:
            raise RuntimeError("Gripper non connesso")
        return self._gripper

    # ------------------------------------------------------------
    # Error handling
    # ------------------------------------------------------------

    def _classify_error(self, exception: Exception) -> GripperErrorType:
        msg = str(exception).lower()

        if "connect" in msg or "connection" in msg or "network" in msg:
            return GripperErrorType.CONNECTION

        if "communication" in msg or "timeout" in msg or "packet" in msg:
            return GripperErrorType.COMMUNICATION

        if isinstance(exception, ValueError):
            return GripperErrorType.VALIDATION

        if "invalid" in msg or "width" in msg or "speed" in msg or "force" in msg:
            return GripperErrorType.VALIDATION

        if "homing" in msg or "home" in msg:
            return GripperErrorType.HOMING

        if "command" in msg or "move" in msg or "grasp" in msg or "stop" in msg:
            return GripperErrorType.COMMAND

        return GripperErrorType.UNKNOWN

    def _collect_error(
        self,
        operation: str,
        exception: Exception,
        recoverable: bool,
    ) -> GripperError:
        error = GripperError(
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
    ) -> GripperError:
        error = self._collect_error(
            operation=operation,
            exception=exception,
            recoverable=recoverable,
        )

        self.mode =     GripperMode.ERROR_LOCKED

        print("✗ Gripper entrato in ERROR_LOCKED.")
        print(f"Operazione: {operation}")
        print(f"Tipo errore: {error.error_type.value}")
        print(f"Messaggio: {error.message}")

        return error

    # ------------------------------------------------------------
    # Safety gates
    # ------------------------------------------------------------

    def _ensure_connected(self) -> None:
        if self._gripper is None or self.mode == GripperMode.DISCONNECTED:
            raise RuntimeError("Gripper non connesso.")

    def _ensure_can_command(self) -> None:
        self._ensure_connected()

        if self.mode == GripperMode.ERROR_LOCKED:
            raise RuntimeError(
                "Comando gripper bloccato: il gripper è in ERROR_LOCKED. "
                "Eseguire reset_error_lock() oppure riconnettere il gripper."
            )

    def _validate_width(self, width: float) -> None:
        if width < 0.0 or width > 0.08:
            raise ValueError(
                f"width fuori range: {width}. Deve essere tra 0.0 m e 0.08 m."
            )

    def _validate_speed(self, speed: float) -> None:
        if speed <= 0.0:
            raise ValueError(
                f"speed non valida: {speed}. Deve essere positiva."
            )

    def _validate_force(self, force: float) -> None:
        if force <= 0.0:
            raise ValueError(
                f"force non valida: {force}. Deve essere positiva."
            )

    # ------------------------------------------------------------
    # Recovery / reset
    # ------------------------------------------------------------

    def reset_error_lock(self) -> None:
        """
        Sblocca il gripper lato software.

        Attenzione: questo non esegue un recovery hardware.
        Serve solo a permettere nuovi comandi dopo aver valutato l'errore.
        """
        self._ensure_connected()

        self.mode =  GripperMode.READY
        self.last_error = None

        print("✓ Gripper riportato in READY lato software")

    def reconnect(self) -> None:
        """
        Riconnette il gripper dopo un errore grave.
        """
        self._gripper = None
        self._is_homed = False
        self.mode = GripperMode.DISCONNECTED
        self._connect()

    # ------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------

    def homing(self) -> bool:
        self._ensure_can_command()

        try:
            self.mode = GripperMode.RUNNING

            print("Esecuzione homing del gripper...")
            success = self.gripper.homing()

            self._is_homed = bool(success)
            self.mode = GripperMode.READY

            if success:
                print("✓ Homing completato")
            else:
                print("✗ Homing fallito")

            return bool(success)

        except Exception as e:
            self._is_homed = False

            error = self._enter_error_locked(
                operation="homing",
                exception=e,
                recoverable=True,
            )

            print(f"✗ Errore durante homing: {error.message}")
            return False

    def open(self, width: float = 0.08, speed: float = 0.1) -> bool:
        self._ensure_can_command()

        try:
            self._validate_width(width)
            self._validate_speed(speed)

            if not self._is_homed:
                print("⚠ Gripper non inizializzato. Esecuzione homing...")
                if not self.homing():
                    return False

            self.mode = GripperMode.RUNNING

            print(
                f"Apertura gripper a {width * 1000:.1f} mm "
                f"con velocità {speed} m/s"
            )

            result = self.gripper.move(width, speed)

            self.mode = GripperMode.READY

            if result:
                print("✓ Movimento gripper completato")
            else:
                print("✗ Movimento gripper non completato")

            return bool(result)

        except Exception as e:
            error = self._enter_error_locked(
                operation="open/move",
                exception=e,
                recoverable=True,
            )

            print(f"✗ Errore durante apertura: {error.message}")
            return False

    def close(self, speed: float = 0.1) -> bool:
        return self.open(width=0.0, speed=speed)

    def grasp(
        self,
        width: float,
        force: float = 60.0,
        speed: float = 0.1,
        epsilon_inner: float = 0.005,
        epsilon_outer: float = 0.005,
    ) -> bool:
        self._ensure_can_command()

        try:
            self._validate_width(width)
            self._validate_speed(speed)
            self._validate_force(force)

            if epsilon_inner < 0.0 or epsilon_outer < 0.0:
                raise ValueError("epsilon_inner ed epsilon_outer devono essere >= 0.")

            if not self._is_homed:
                print("⚠ Gripper non inizializzato. Esecuzione homing...")
                if not self.homing():
                    return False

            self.mode = GripperMode.RUNNING

            print(
                f"Tentativo di presa: width={width * 1000:.1f} mm, "
                f"force={force} N, speed={speed} m/s"
            )

            success = self.gripper.grasp(
                width,
                speed,
                force,
                epsilon_inner,
                epsilon_outer,
            )

            self.mode = GripperMode.READY

            if success:
                print("✓ Oggetto afferrato con successo")
            else:
                print("✗ Presa fallita - oggetto non rilevato")

            return bool(success)

        except Exception as e:
            error = self._enter_error_locked(
                operation="grasp",
                exception=e,
                recoverable=True,
            )

            print(f"✗ Errore durante la presa: {error.message}")
            return False

    def release(self, speed: float = 0.1) -> bool:
        print("Rilascio oggetto...")
        return self.open(width=0.08, speed=speed)

    def stop(self) -> bool:
        """
        Lo stop deve essere consentito anche in ERROR_LOCKED.
        """
        self._ensure_connected()

        try:
            self.gripper.stop()
            self.mode = GripperMode.READY
            print("✓ Gripper fermato")
            return True

        except Exception as e:
            error = self._enter_error_locked(
                operation="stop",
                exception=e,
                recoverable=True,
            )

            print(f"✗ Errore durante l'arresto: {error.message}")
            return False

    # ------------------------------------------------------------
    # Read methods
    # ------------------------------------------------------------

    def get_state(self) -> Optional[Dict[str, Any]]:
        """
        La lettura è consentita anche in ERROR_LOCKED,
        purché il gripper sia ancora connesso.
        """
        self._ensure_connected()

        try:
            state = self.gripper.read_once()

            return {
                "width": state.width if hasattr(state, "width") else 0.0,
                "max_width": state.max_width if hasattr(state, "max_width") else 0.08,
                "is_grasped": state.is_grasped if hasattr(state, "is_grasped") else False,
                "temperature": state.temperature if hasattr(state, "temperature") else 0.0,
            }

        except Exception as e:
            error = self._enter_error_locked(
                operation="get_state/read_once",
                exception=e,
                recoverable=True,
            )

            print(f"✗ Errore nella lettura dello stato: {error.message}")
            return None

    def print_state(self) -> None:
        state = self.get_state()

        if state is None:
            print("Impossibile leggere lo stato del gripper")
            return

        print("\n=== Stato Gripper Franka ===")
        print(f"Mode: {self.mode.value}")
        print(f"Larghezza corrente: {state['width'] * 1000:.1f} mm")
        print(f"Larghezza massima: {state['max_width'] * 1000:.1f} mm")
        print(f"Temperatura: {state['temperature']:.1f} °C")
        print(f"Oggetto afferrato: {'Sì' if state['is_grasped'] else 'No'}")
        print("=" * 30)

    def is_grasping(self) -> bool:
        state = self.get_state()
        return state["is_grasped"] if state else False

    def get_server_version(self) -> str:
        self._ensure_connected()

        try:
            return str(self.gripper.server_version())

        except Exception as e:
            error = self._enter_error_locked(
                operation="get_server_version",
                exception=e,
                recoverable=True,
            )

            print(f"✗ Errore nell'ottenere la versione: {error.message}")
            return "Unknown"

    # ------------------------------------------------------------
    # Debug utilities
    # ------------------------------------------------------------

    def print_errors(self) -> None:
        if not self.errors:
            print("Nessun errore gripper registrato.")
            return

        print("\n=== Errori Gripper Registrati ===")

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
        print("✓ Lista errori gripper svuotata")

    # ------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None:
            print(f"[Errore durante l'esecuzione del gripper: {exc_val}]")
        pass