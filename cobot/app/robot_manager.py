"""
Singleton che gestisce il ciclo di vita di FrankaRobot e serializza le operazioni.

Una sola operazione hardware può girare alla volta (lock). Le operazioni di
movimento vengono eseguite in un thread di background; il WebSocket riceve
la telemetria live via progress_callback.
"""

import inspect
import traceback
from datetime import datetime, timezone
from typing import Optional, Callable, Any

import numpy as np

try:
    from eventlet.patcher import original as original_module

    threading = original_module("threading")
except ImportError:
    import threading

from franka_controller import FrankaRobot
from franka_controller.robot import EMERGENCY_STOP_MESSAGE, EmergencyStop


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
        self._current_operation: Optional[str] = None
        self._last_operation: dict = {}
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
        busy = self.is_busy
        progress = self._cached_state.get("progress") if busy else None
        return {
            "connected": self.is_connected,
            "busy": busy,
            "mode": self.robot.mode.value if self.is_connected else "disconnected",
            "stop_requested": self.robot.stop_requested if self.is_connected else False,
            "current_operation": self._current_operation if busy else None,
            "progress": progress,
            "last_operation": dict(self._last_operation) if self._last_operation else None,
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

        # uno stop richiesto per l'operazione precedente non deve fermare questa
        if self.robot is not None:
            self.robot.clear_stop()

        operation_name = func.__name__
        if operation_name == "_workflow":
            operation_name = "pick_and_place"
        elif operation_name in ("execute_trajectory", "execute_trajectory_fly") and args:
            operation_name = f"{operation_name} ({len(args[0])} waypoint)"
        elif operation_name == "run_force_task" and args:
            operation_name = f"force_{args[0].mode}"
        elif operation_name == "run_screw_passes" and len(args) >= 3:
            operation_name = f"force_screw ({args[2]} passate)"
        self._current_operation = operation_name

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
                self._last_operation = {
                    "operation": operation_name,
                    "success": bool(result),
                    "completed_at": datetime.now(timezone.utc).isoformat(),
                    "error": None if result else "Operazione terminata con esito negativo.",
                }
                self._emit(on_complete_event, {"success": bool(result)})
            except Exception as exc:
                error_info = traceback.format_exc()
                print(f"[RobotManager] Async task failed: {exc!r}\n{error_info}")
                self._last_operation = {
                    "operation": operation_name,
                    "success": False,
                    "completed_at": datetime.now(timezone.utc).isoformat(),
                    "error": str(exc),
                }
                self._emit("motion_error", {"error": str(exc), "traceback": error_info})
            finally:
                self._apply_emergency_stop()
                self._cached_state = {}
                self._current_operation = None
                self._operation_lock.release()

        thread = threading.Thread(target=_run, daemon=True)
        thread.start()
        return True

    # ------------------------------------------------------------------
    # Arresto software
    # ------------------------------------------------------------------

    def request_stop(self) -> dict:
        """
        Chiede di fermare subito l'operazione in corso, qualunque sia il tipo.

        Non prende il lock (lo tiene l'operazione da fermare): imposta solo un
        flag che i loop di controllo leggono a ogni ciclo (1 kHz). Il robot va
        in ERROR_LOCKED e si sblocca solo con POST /api/robot/recovery.
        Se non c'è nessuna operazione in corso non fa nulla.
        """
        if not self.is_connected:
            raise RuntimeError("Robot non connesso.")

        operation = self._current_operation
        if not self.is_busy:
            return {"stop_requested": False, "operation": None,
                    "message": "Nessuna operazione in corso."}

        self.robot.request_stop()
        if not self.is_busy:  # l'operazione è finita proprio ora
            self.robot.clear_stop()
            return {"stop_requested": False, "operation": None,
                    "message": "Nessuna operazione in corso."}

        if operation == "homing":
            # l'homing è una chiamata bloccante che non legge il flag
            try:
                self.robot.gripper.stop()
            except Exception as exc:
                print(f"[RobotManager] Stop gripper fallito: {exc}")

        print(f"[RobotManager] ARRESTO richiesto durante: {operation}")
        return {"stop_requested": True, "operation": operation,
                "message": "Arresto richiesto: il robot resta in ERROR_LOCKED "
                           "fino a POST /api/robot/recovery."}

    def _apply_emergency_stop(self) -> None:
        """
        A fine operazione: se è stato richiesto lo stop il robot resta bloccato.

        Vale per qualunque tipo di operazione (anche quelle che non leggono il
        flag, come l'homing, o che sono finite nel frattempo).
        """
        robot = self.robot
        if robot is None or not robot.stop_requested:
            return
        if robot.mode.value != "error_locked":
            robot._enter_error_locked(
                operation="emergency_stop",
                exception=EmergencyStop(EMERGENCY_STOP_MESSAGE),
                recoverable=False,
            )
        last = dict(self._last_operation) if self._last_operation else {}
        last["emergency_stop"] = True
        if not last.get("success"):
            last["error"] = EMERGENCY_STOP_MESSAGE
        self._last_operation = last
        self._emit("emergency_stop", {
            "operation": last.get("operation"),
            "message": EMERGENCY_STOP_MESSAGE,
        })

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
