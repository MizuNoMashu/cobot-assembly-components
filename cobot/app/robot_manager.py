"""
Singleton che gestisce il ciclo di vita di FrankaRobot e serializza le operazioni.

Una sola operazione hardware può girare alla volta (lock). Le operazioni di
movimento vengono eseguite in un thread di background; il WebSocket riceve
la telemetria live via progress_callback.
"""

import inspect
import threading
import traceback
from typing import Optional, Callable, Any

import numpy as np

from franka_controller import FrankaRobot


class RobotManager:
    _instance: Optional["RobotManager"] = None
    _init_lock = threading.Lock()

    def __new__(cls):
        with cls._init_lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
                cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True

        self.robot: Optional[FrankaRobot] = None
        self._operation_lock = threading.Lock()
        self._cached_state: dict = {}
        self._socketio = None

    # ------------------------------------------------------------------
    # Wiring
    # ------------------------------------------------------------------

    def set_socketio(self, socketio) -> None:
        self._socketio = socketio

    # ------------------------------------------------------------------
    # Status helpers
    # ------------------------------------------------------------------

    @property
    def is_connected(self) -> bool:
        return self.robot is not None

    @property
    def is_busy(self) -> bool:
        return self._operation_lock.locked()

    def status_dict(self) -> dict:
        return {
            "connected": self.is_connected,
            "busy": self.is_busy,
            "mode": self.robot.mode.value if self.is_connected else "disconnected",
        }

    # ------------------------------------------------------------------
    # SocketIO emit (thread-safe)
    # ------------------------------------------------------------------

    def _emit(self, event: str, data: Any) -> None:
        if self._socketio is not None:
            self._socketio.emit(event, data)

    # ------------------------------------------------------------------
    # progress_callback – chiamata dal loop real-time di motion.py
    # ------------------------------------------------------------------

    def _progress_callback(self, data: dict) -> None:
        self._cached_state = data
        self._emit("motion_progress", data)

    # ------------------------------------------------------------------
    # Async motion runner
    # ------------------------------------------------------------------

    def run_async(
        self,
        func: Callable,
        *args,
        on_complete_event: str = "motion_complete",
        **kwargs,
    ) -> bool:
        """
        Prova ad acquisire il lock e lancia func in un thread di background.

        Ritorna True se il thread è stato avviato, False se il robot è occupato.
        Inietta progress_callback=self._progress_callback solo se il target lo supporta.
        """
        if not self._operation_lock.acquire(blocking=False):
            return False

        signature = inspect.signature(func)
        injected_kwargs = dict(kwargs)
        if "progress_callback" in signature.parameters or any(
            p.kind == inspect.Parameter.VAR_KEYWORD for p in signature.parameters.values()
        ):
            injected_kwargs["progress_callback"] = self._progress_callback

        def _run():
            try:
                print(
                    f"[RobotManager] Async task started: {func.__name__} args={args} "
                    f"kwargs={list(injected_kwargs.keys())}"
                )
                result = func(*args, **injected_kwargs)
                self._emit(on_complete_event, {"success": bool(result)})
            except Exception as exc:
                error_info = traceback.format_exc()
                print(f"[RobotManager] Async task failed: {exc!r}\n{error_info}")
                self._emit("motion_error", {"error": str(exc), "traceback": error_info})
            finally:
                self._cached_state = {}
                self._operation_lock.release()

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()
        return True

    # ------------------------------------------------------------------
    # State reading (safe anche durante il movimento)
    # ------------------------------------------------------------------

    def get_robot_state(self) -> dict:
        """
        Ritorna lo stato del robot.

        - Se il robot è in movimento restituisce la cache aggiornata dal loop.
        - Se è idle acquisisce il lock ed esegue una read diretta.
        """
        if not self.is_connected:
            raise RuntimeError("Robot non connesso.")

        if self.robot.mode.value == "error_locked":
            error = self.robot.last_error
            message = error.message if error is not None else "Robot in ERROR_LOCKED."
            raise RuntimeError(f"Robot in ERROR_LOCKED: {message}")

        if self.is_busy:
            if self._cached_state:
                return dict(self._cached_state)
            raise RuntimeError("Robot occupato, nessuno stato in cache ancora.")

        with self._operation_lock:
            state = self.robot.get_state()
            pose = np.array(state.O_T_EE).reshape(4, 4, order="F")
            pos = pose[:3, 3]
            orientation = pose[:3, :3]
            return {
                "mode": self.robot.mode.value,
                "joint_positions": list(state.q),
                "joint_velocities": list(state.dq),
                "cartesian_pose": {
                    "position": {
                        "x": float(pos[0]),
                        "y": float(pos[1]),
                        "z": float(pos[2]),
                    },
                    "orientation": {
                        "matrix": orientation.tolist(),
                    },
                },
                "external_forces": list(self.robot.get_external_forces()),
                "cartesian_contact": list(self.robot.get_cartesian_contact()),
                "cartesian_collision": list(self.robot.get_cartesian_collision()),
            }
