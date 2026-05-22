import os
import time
from enum import Enum
from dataclasses import dataclass
from datetime import datetime
from typing import Optional, Dict, Any, List

import requests
from requests.auth import HTTPBasicAuth


# ------------------------------------------------------------------
# Enums
# ------------------------------------------------------------------

class DeskSystemStatus(Enum):
    STARTING = "Starting"
    FIRST_START = "FirstStart"
    STARTED = "Started"
    RESCUE_SYSTEM = "RescueSystem"
    REBOOT_REQUIRED = "RebootRequired"
    UNKNOWN = "Unknown"


class DeskOperatingMode(Enum):
    PROGRAMMING = "Programming"
    EXECUTION = "Execution"
    SAFETY_RECOVERY = "SafetyRecovery"
    SELF_TEST = "SelfTest"
    UNDEFINED = "Undefined"
    UNKNOWN = "Unknown"


class DeskFCIStatus(Enum):
    ACTIVE = "Active"
    INACTIVE = "Inactive"
    UNKNOWN = "Unknown"


class DeskErrorType(Enum):
    AUTHENTICATION = "authentication_error"
    PERMISSION = "permission_error"
    CONTROL_TOKEN = "control_token_error"
    FAILED_DEPENDENCY = "failed_dependency_error"
    SERVICE_UNAVAILABLE = "service_unavailable"
    CONNECTION = "connection_error"
    HTTP = "http_error"
    VALIDATION = "validation_error"
    UNKNOWN = "unknown_error"


# ------------------------------------------------------------------
# Dataclasses
# ------------------------------------------------------------------

@dataclass
class DeskError:
    operation: str
    error_type: DeskErrorType
    message: str
    timestamp: datetime
    status_code: Optional[int] = None
    details: Optional[Any] = None
    recoverable: bool = True
    original_exception: Optional[Exception] = None


@dataclass
class ControlToken:
    token: str
    token_id: Optional[int] = None
    owner: Optional[str] = None
    timestamp: Optional[datetime] = None


# ------------------------------------------------------------------
# Main class
# ------------------------------------------------------------------

class FrankaDeskAPI:
    """
    Wrapper per la Franka Desk API HTTP.

    Gestisce:
    - connessione HTTP al Control Franka;
    - Basic Auth;
    - SPoC control token;
    - stato sistema;
    - operating mode;
    - lock/unlock joints;
    - attivazione/disattivazione FCI;
    - safety recovery;
    - reboot/shutdown.

    Questa classe NON sostituisce pylibfranka.
    Serve per preparare e gestire il sistema prima/durante l'uso real-time.
    """

    def __init__(
        self,
        robot_ip: Optional[str] = None,
        username: Optional[str] = None,
        password: Optional[str] = None,
        owner: str = "franka-backend",
        timeout: float = 5.0,
        verify_ssl: bool = False,
        scheme: str = "https",
    ):
        self.robot_ip = robot_ip or os.getenv("FRANKA_ROBOT_IP", "172.16.0.3")
        self.username = username or os.getenv("FRANKA_USERNAME", "fixed_arm")
        self.password = password or os.getenv("FRANKA_PASSWORD", "")
        self.owner = owner
        self.timeout = timeout
        self.verify_ssl = verify_ssl
        self.scheme = scheme

        self.base_url = f"{self.scheme}://{self.robot_ip}"

        self.session = requests.Session()
        #self.session.auth = HTTPBasicAuth(self.username, self.password)

        self.control_token: Optional[ControlToken] = None

        self.errors: List[DeskError] = []
        self.last_error: Optional[DeskError] = None

    # ------------------------------------------------------------
    # Internal utilities
    # ------------------------------------------------------------

    def _url(self, path: str) -> str:
        if not path.startswith("/"):
            path = "/" + path
        return self.base_url + path

    def _headers(
        self,
        require_token: bool = False,
        extra_headers: Optional[Dict[str, str]] = None,
    ) -> Dict[str, str]:

        headers = {
            "Accept": "application/json",
        }

        if require_token:
            if self.control_token is None:
                raise RuntimeError(
                    "Control token assente. Eseguire take_control_token() prima."
                )

            headers["X-Control-Token"] = self.control_token.token

        if extra_headers:
            headers.update(extra_headers)

        return headers

    def _classify_status_code(self, status_code: int) -> DeskErrorType:
        if status_code == 401:
            return DeskErrorType.AUTHENTICATION

        if status_code == 403:
            return DeskErrorType.PERMISSION

        if status_code == 423:
            return DeskErrorType.CONTROL_TOKEN

        if status_code == 424:
            return DeskErrorType.FAILED_DEPENDENCY

        if status_code == 503:
            return DeskErrorType.SERVICE_UNAVAILABLE

        if 400 <= status_code < 600:
            return DeskErrorType.HTTP

        return DeskErrorType.UNKNOWN

    def _parse_error_response(self, response: requests.Response) -> Dict[str, Any]:
        try:
            return response.json()
        except Exception:
            return {
                "code": "HTTPError",
                "message": response.text,
            }

    def _collect_error(
        self,
        operation: str,
        message: str,
        error_type: DeskErrorType = DeskErrorType.UNKNOWN,
        status_code: Optional[int] = None,
        details: Optional[Any] = None,
        recoverable: bool = True,
        original_exception: Optional[Exception] = None,
    ) -> DeskError:

        error = DeskError(
            operation=operation,
            error_type=error_type,
            message=message,
            timestamp=datetime.now(),
            status_code=status_code,
            details=details,
            recoverable=recoverable,
            original_exception=original_exception,
        )

        self.errors.append(error)
        self.last_error = error

        return error

    def _request(
        self,
        method: str,
        path: str,
        operation: str,
        require_token: bool = False,
        json: Optional[Dict[str, Any]] = None,
        data: Optional[Any] = None,
        extra_headers: Optional[Dict[str, str]] = None,
        expected_status: Optional[List[int]] = None,
    ) -> Optional[Any]:

        if expected_status is None:
            expected_status = [200, 204]

        try:
            response = self.session.request(
                method=method.upper(),
                url=self._url(path),
                headers=self._headers(
                    require_token=require_token,
                    extra_headers=extra_headers,
                ),
                json=json,
                data=data,
                timeout=self.timeout,
                verify=self.verify_ssl,
            )

            if response.status_code not in expected_status:
                payload = self._parse_error_response(response)

                error_type = self._classify_status_code(response.status_code)

                error = self._collect_error(
                    operation=operation,
                    error_type=error_type,
                    message=payload.get("message", payload.get("code", response.text)),
                    status_code=response.status_code,
                    details=payload.get("details", payload),
                    recoverable=response.status_code in [423, 424, 503],
                )

                raise RuntimeError(
                    f"[{error.error_type.value}] {operation} failed "
                    f"with status {response.status_code}: {error.message}"
                )

            if response.status_code == 204:
                return None

            if not response.text:
                return None

            content_type = response.headers.get("Content-Type", "")

            if "application/json" in content_type:
                return response.json()

            return response.text

        except requests.exceptions.RequestException as e:
            error = self._collect_error(
                operation=operation,
                error_type=DeskErrorType.CONNECTION,
                message=str(e),
                recoverable=True,
                original_exception=e,
            )

            raise RuntimeError(
                f"[{error.error_type.value}] Errore di connessione Desk API: "
                f"{error.message}"
            ) from e

    # ------------------------------------------------------------
    # System
    # ------------------------------------------------------------

    def get_system_state(self) -> Dict[str, Any]:
        return self._request(
            method="GET",
            path="/api/system",
            operation="get_system_state",
        )

    def get_system_status(self) -> DeskSystemStatus:
        state = self.get_system_state()
        value = state.get("status", "Unknown")

        try:
            return DeskSystemStatus(value)
        except ValueError:
            return DeskSystemStatus.UNKNOWN

    def is_started(self) -> bool:
        return self.get_system_status() == DeskSystemStatus.STARTED

    def is_rescue_system(self) -> bool:
        return self.get_system_status() == DeskSystemStatus.RESCUE_SYSTEM

    def get_operating_mode(self) -> DeskOperatingMode:
        state = self._request(
            method="GET",
            path="/api/system/operating-mode",
            operation="get_operating_mode",
        )

        value = state.get("status", "Unknown")

        try:
            return DeskOperatingMode(value)
        except ValueError:
            return DeskOperatingMode.UNKNOWN

    def change_to_execution_mode(self) -> None:
        self._request(
            method="POST",
            path="/api/system/operating-mode:change",
            operation="change_to_execution_mode",
            require_token=True,
            json={
                "desiredOperatingMode": "Execution",
            },
            expected_status=[204],
        )

    def reboot(self, to_rescue: bool = False) -> None:
        path = "/api/system:reboot"

        if to_rescue:
            path += "?to_rescue=true"

        self._request(
            method="POST",
            path=path,
            operation="reboot",
            expected_status=[204],
        )

    def shutdown(self) -> None:
        self._request(
            method="POST",
            path="/api/system:shutdown",
            operation="shutdown",
            expected_status=[204],
        )

    # ------------------------------------------------------------
    # SPoC control token
    # ------------------------------------------------------------

    def get_control_token_state(self) -> Dict[str, Any]:
        return self._request(
            method="GET",
            path="/api/system/control-token",
            operation="get_control_token_state",
        )

    def take_control_token(
        self,
        timeout: Optional[float] = 1.0,
    ) -> ControlToken:

        body = {
            "owner": self.owner,
        }

        if timeout is not None:
            body["timeout"] = timeout

        response = self._request(
            method="POST",
            path="/api/system/control-token:take",
            operation="take_control_token",
            json=body,
        )

        self.control_token = ControlToken(
            token=response["token"],
            token_id=response.get("tokenId"),
            owner=self.owner,
            timestamp=datetime.now(),
        )

        return self.control_token

    def release_control_token(self) -> None:
        self._request(
            method="POST",
            path="/api/system/control-token:release",
            operation="release_control_token",
            require_token=True,
            expected_status=[204],
        )

        self.control_token = None

    def ensure_control_token(self) -> ControlToken:
        if self.control_token is None:
            return self.take_control_token()

        return self.control_token

    # ------------------------------------------------------------
    # Arm / joints
    # ------------------------------------------------------------

    def get_arm_info(self) -> Dict[str, Any]:
        return self._request(
            method="GET",
            path="/api/arm",
            operation="get_arm_info",
        )

    def get_joints(self) -> List[Dict[str, Any]]:
        return self._request(
            method="GET",
            path="/api/arm/joints",
            operation="get_joints",
        )

    def get_arm_warnings(self) -> Dict[str, Any]:
        return self._request(
            method="GET",
            path="/api/arm/warnings",
            operation="get_arm_warnings",
        )

    def unlock_joints(self) -> None:
        self.ensure_control_token()

        self._request(
            method="POST",
            path="/api/arm/joints:unlock",
            operation="unlock_joints",
            require_token=True,
            expected_status=[204],
        )

    def lock_joints(self) -> None:
        self._request(
            method="POST",
            path="/api/arm/joints:lock",
            operation="lock_joints",
            expected_status=[204],
        )

    def are_joints_unlocked(self) -> bool:
        joints = self.get_joints()

        return all(
            joint.get("brakeStatus") == "Unlocked"
            for joint in joints
        )

    # ------------------------------------------------------------
    # FCI
    # ------------------------------------------------------------

    def get_fci_state(self) -> DeskFCIStatus:
        state = self._request(
            method="GET",
            path="/api/fci",
            operation="get_fci_state",
        )

        value = state.get("status", "Unknown")

        try:
            return DeskFCIStatus(value)
        except ValueError:
            return DeskFCIStatus.UNKNOWN

    def activate_fci(self) -> None:
        self.ensure_control_token()

        self._request(
            method="POST",
            path="/api/fci:activate",
            operation="activate_fci",
            require_token=True,
            expected_status=[204],
        )

    def deactivate_fci(self) -> None:
        self.ensure_control_token()

        self._request(
            method="POST",
            path="/api/fci:deactivate",
            operation="deactivate_fci",
            require_token=True,
            expected_status=[204],
        )

    def is_fci_active(self) -> bool:
        return self.get_fci_state() == DeskFCIStatus.ACTIVE

    # ------------------------------------------------------------
    # Safety
    # ------------------------------------------------------------

    def get_self_tests_state(self) -> Dict[str, Any]:
        return self._request(
            method="GET",
            path="/api/safety/self-tests",
            operation="get_self_tests_state",
        )

    def execute_self_tests(self) -> None:
        self.ensure_control_token()

        self._request(
            method="POST",
            path="/api/safety/self-tests:execute",
            operation="execute_self_tests",
            require_token=True,
            expected_status=[204],
        )

    def get_safety_recovery(self) -> Dict[str, Any]:
        return self._request(
            method="GET",
            path="/api/safety/recovery",
            operation="get_safety_recovery",
        )

    def has_active_recovery(self) -> bool:
        recovery = self.get_safety_recovery()
        return recovery.get("recovery") is not None

    def confirm_recovery(self, recovery_type: str) -> None:
        self.ensure_control_token()

        self._request(
            method="POST",
            path="/api/safety/recovery:confirm",
            operation="confirm_recovery",
            require_token=True,
            json={
                "type": recovery_type,
            },
            expected_status=[204],
        )

    # ------------------------------------------------------------
    # High-level preparation
    # ------------------------------------------------------------

    def prepare_for_fci(self) -> Dict[str, Any]:
        """
        Sequenza tipica prima di usare pylibfranka/libfranka:

        1. controlla stato sistema;
        2. prende control token;
        3. controlla recovery safety;
        4. porta il robot in Execution se necessario;
        5. sblocca i giunti;
        6. attiva FCI.
        """

        report = {
            "system_status": None,
            "operating_mode": None,
            "recovery": None,
            "joints_unlocked": None,
            "fci_status": None,
            "steps": [],
        }

        system_status = self.get_system_status()
        report["system_status"] = system_status.value

        if system_status == DeskSystemStatus.RESCUE_SYSTEM:
            raise RuntimeError(
                "Robot in Rescue System. Sono disponibili solo endpoint limitati."
            )

        if system_status != DeskSystemStatus.STARTED:
            raise RuntimeError(
                f"Robot non pronto. Stato corrente: {system_status.value}"
            )

        self.ensure_control_token()
        report["steps"].append("control_token_acquired")

        recovery = self.get_safety_recovery()
        report["recovery"] = recovery

        if recovery.get("recovery") is not None:
            raise RuntimeError(
                f"Recovery safety attiva: {recovery['recovery']}"
            )

        operating_mode = self.get_operating_mode()
        report["operating_mode"] = operating_mode.value

        if operating_mode != DeskOperatingMode.EXECUTION:
            self.change_to_execution_mode()
            report["steps"].append("changed_to_execution")

        if not self.are_joints_unlocked():
            self.unlock_joints()
            report["steps"].append("joints_unlocked")

        report["joints_unlocked"] = self.are_joints_unlocked()

        if not self.is_fci_active():
            self.activate_fci()
            report["steps"].append("fci_activated")

        report["fci_status"] = self.get_fci_state().value

        return report

    # ------------------------------------------------------------
    # Debug
    # ------------------------------------------------------------

    def print_status(self) -> None:
        print("\n=== Franka Desk API Status ===")

        try:
            system = self.get_system_state()
            print(f"System status: {system.get('status')}")
            print(f"Operating mode: {system.get('operatingMode', {}).get('status')}")
            print(f"Control serial: {system.get('controlSerialNumber')}")
        except Exception as e:
            print(f"Errore lettura sistema: {e}")

        try:
            joints = self.get_joints()
            print("Joints:")
            for joint in joints:
                print(
                    f"  Joint {joint.get('joint')}: "
                    f"{joint.get('brakeStatus')}"
                )
        except Exception as e:
            print(f"Errore lettura joints: {e}")

        try:
            print(f"FCI: {self.get_fci_state().value}")
        except Exception as e:
            print(f"Errore lettura FCI: {e}")

        print("=" * 35)

    def print_errors(self) -> None:
        if not self.errors:
            print("Nessun errore Desk API registrato.")
            return

        print("\n=== Errori Desk API Registrati ===")

        for i, err in enumerate(self.errors, start=1):
            print(f"\nErrore #{i}")
            print(f"Timestamp: {err.timestamp}")
            print(f"Operation: {err.operation}")
            print(f"Type: {err.error_type.value}")
            print(f"Status code: {err.status_code}")
            print(f"Recoverable: {err.recoverable}")
            print(f"Message: {err.message}")
            print(f"Details: {err.details}")

    def clear_errors(self) -> None:
        self.errors.clear()
        self.last_error = None
        print("✓ Lista errori Desk API svuotata")

    # ------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        try:
            if self.control_token is not None:
                self.release_control_token()
        except Exception:
            pass

        if exc_type is not None:
            print(f"[Errore durante l'esecuzione Desk API: {exc_val}]")

        return False