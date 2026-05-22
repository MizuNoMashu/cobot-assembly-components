"""
Endpoint REST per Franka Desk API.

Questi endpoint NON usano pylibfranka direttamente.
Servono per gestire:
- stato sistema Franka;
- SPoC control token;
- operating mode;
- joints lock/unlock;
- FCI activation/deactivation;
- safety recovery;
- self-tests;
- reboot/shutdown.

La Desk API è l'API HTTP del robot Franka.
"""

from flask import Blueprint, request, jsonify

from app.robot_manager import RobotManager
from franka_controller.desk_api import FrankaDeskAPI

try:
    from franka_controller.robot import FrankaRobot
except ImportError:
    FrankaRobot = None


desk_bp = Blueprint("desk", __name__, url_prefix="/api/desk")
manager = RobotManager()


def _desk_not_connected():
    return jsonify({"error": "Desk API non inizializzata"}), 503


def _busy():
    return jsonify({
        "error": "robot occupato, attendere il completamento dell'operazione corrente"
    }), 409


def _serialize_desk_error(error):
    return {
        "operation": error.operation,
        "type": error.error_type.value,
        "message": error.message,
        "timestamp": error.timestamp.isoformat(),
        "status_code": error.status_code,
        "details": error.details,
        "recoverable": error.recoverable,
    }


def _get_desk():
    if getattr(manager, "desk", None) is None:
        return None
    return manager.desk


# ------------------------------------------------------------------
# Desk API connection/configuration
# ------------------------------------------------------------------

@desk_bp.route("/connect", methods=["POST"])
def desk_connect():
    """Initialize Franka Desk API client.
    ---
    tags:
      - Desk API
    summary: Connect to Franka Desk API
    description: >
      Initializes the HTTP client for the Franka Desk API.
      This does not open a real-time FCI/pylibfranka connection.
    parameters:
      - in: body
        name: body
        required: false
        schema:
          type: object
          properties:
            robot_ip:
              type: string
              example: "172.16.0.3"
            username:
              type: string
              example: "fixed_arm"
            password:
              type: string
              example: "password"
            owner:
              type: string
              example: "franka-backend"
            timeout:
              type: number
              example: 5.0
            scheme:
              type: string
              example: "http"
    responses:
      200:
        description: Desk API connected successfully.
      500:
        description: Desk API connection failed.
    """
    body = request.get_json(silent=True) or {}

    robot_ip = body.get("robot_ip", "172.16.0.3")
    username = body.get("username")
    password = body.get("password")
    owner = body.get("owner", "franka-backend")
    timeout = float(body.get("timeout", 5.0))
    scheme = body.get("scheme", "http")

    try:
        manager.desk = FrankaDeskAPI(
            robot_ip=robot_ip,
            username=username,
            password=password,
            owner=owner,
            timeout=timeout,
            scheme=scheme,
        )

        system_state = manager.desk.get_system_state()

        return jsonify({
            "status": "connected",
            "robot_ip": robot_ip,
            "system": system_state,
        }), 200

    except Exception as exc:
        manager.desk = None
        return jsonify({
            "error": "connessione Desk API fallita",
            "message": str(exc),
        }), 500


@desk_bp.route("/disconnect", methods=["POST"])
def desk_disconnect():
    """Disconnect Franka Desk API client.
    ---
    tags:
      - Desk API
    summary: Disconnect from Franka Desk API
    description: >
      Releases the current Desk API control token if present and removes
      the Desk API client from the RobotManager.
    responses:
      200:
        description: Desk API disconnected successfully.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        if desk.control_token is not None:
            desk.release_control_token()
    except Exception:
        pass

    manager.desk = None

    return jsonify({"status": "disconnected"}), 200


@desk_bp.route("/config", methods=["GET"])
def desk_config():
    """Get Desk API client configuration.
    ---
    tags:
      - Desk API
    summary: Get Desk API configuration
    description: Returns the current Desk API client configuration and connection status.
    responses:
      200:
        description: Current Desk API configuration.
    """
    desk = _get_desk()

    if desk is None:
        return jsonify({
            "connected": False,
            "robot_ip": "172.16.0.3",
            "scheme": "http",
            "note": "Desk API non inizializzata",
        }), 200

    return jsonify({
        "connected": True,
        "robot_ip": desk.robot_ip,
        "base_url": desk.base_url,
        "owner": desk.owner,
        "timeout": desk.timeout,
        "has_control_token": desk.control_token is not None,
    }), 200


# ------------------------------------------------------------------
# System
# ------------------------------------------------------------------

@desk_bp.route("/system", methods=["GET"])
def desk_system_state():
    """Get Franka system state.
    ---
    tags:
      - Desk API - System
    summary: Get Franka system state
    description: Equivalent to Franka Desk API GET /api/system.
    responses:
      200:
        description: Franka system state.
      500:
        description: Error while reading system state.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        return jsonify(desk.get_system_state()), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/system/status", methods=["GET"])
def desk_system_status():
    """Get simplified Franka system status.
    ---
    tags:
      - Desk API - System
    summary: Get simplified Franka system status
    description: Returns the simplified system status and whether the robot is started or in Rescue System.
    responses:
      200:
        description: Simplified Franka system status.
      500:
        description: Error while reading system status.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        status = desk.get_system_status()

        return jsonify({
            "status": status.value,
            "is_started": desk.is_started(),
            "is_rescue_system": desk.is_rescue_system(),
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/system/operating-mode", methods=["GET"])
def desk_operating_mode():
    """Get current Franka operating mode.
    ---
    tags:
      - Desk API - System
    summary: Get operating mode
    description: Equivalent to Franka Desk API GET /api/system/operating-mode.
    responses:
      200:
        description: Current Franka operating mode.
      500:
        description: Error while reading operating mode.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        mode = desk.get_operating_mode()
        return jsonify({"operating_mode": mode.value}), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/system/operating-mode/execution", methods=["POST"])
def desk_change_to_execution():
    """Change Franka operating mode to Execution.
    ---
    tags:
      - Desk API - System
    summary: Change operating mode to Execution
    description: >
      Changes the Franka operating mode to Execution.
      Requires Basic Auth and a valid SPoC control token on the Franka Desk API side.
    responses:
      200:
        description: Operating mode changed to Execution.
      409:
        description: Robot is busy.
      500:
        description: Error while changing operating mode.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    try:
        desk.ensure_control_token()
        desk.change_to_execution_mode()

        return jsonify({
            "status": "ok",
            "operating_mode": desk.get_operating_mode().value,
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/system/reboot", methods=["POST"])
def desk_reboot():
    """Reboot Franka system.
    ---
    tags:
      - Desk API - System
    summary: Reboot Franka system
    description: >
      Reboots the Franka system. If to_rescue is true, the robot is rebooted
      into the Rescue System.
    parameters:
      - in: body
        name: body
        required: false
        schema:
          type: object
          properties:
            to_rescue:
              type: boolean
              example: false
    responses:
      202:
        description: Reboot requested.
      409:
        description: Robot is busy.
      500:
        description: Error while requesting reboot.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    to_rescue = bool(body.get("to_rescue", False))

    try:
        desk.reboot(to_rescue=to_rescue)

        return jsonify({
            "status": "reboot_requested",
            "to_rescue": to_rescue,
        }), 202

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/system/shutdown", methods=["POST"])
def desk_shutdown():
    """Shutdown Franka system.
    ---
    tags:
      - Desk API - System
    summary: Shutdown Franka system
    description: Requests a shutdown of the Franka system through the Desk API.
    responses:
      202:
        description: Shutdown requested.
      409:
        description: Robot is busy.
      500:
        description: Error while requesting shutdown.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    try:
        desk.shutdown()
        return jsonify({"status": "shutdown_requested"}), 202

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


# ------------------------------------------------------------------
# SPoC control token
# ------------------------------------------------------------------

@desk_bp.route("/control-token", methods=["GET"])
def desk_get_control_token_state():
    """Get current SPoC control token state.
    ---
    tags:
      - Desk API - SPoC
    summary: Get SPoC control token state
    description: Equivalent to Franka Desk API GET /api/system/control-token.
    responses:
      200:
        description: Current SPoC control token state.
      500:
        description: Error while reading control token state.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        return jsonify(desk.get_control_token_state()), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/control-token/take", methods=["POST"])
def desk_take_control_token():
    """Take SPoC control token.
    ---
    tags:
      - Desk API - SPoC
    summary: Take SPoC control token
    description: Equivalent to Franka Desk API POST /api/system/control-token:take.
    parameters:
      - in: body
        name: body
        required: false
        schema:
          type: object
          properties:
            timeout:
              type: number
              example: 1.0
    responses:
      200:
        description: Control token acquired.
      500:
        description: Error while taking control token.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    body = request.get_json(silent=True) or {}
    timeout = body.get("timeout", 1.0)

    try:
        token = desk.take_control_token(timeout=timeout)

        return jsonify({
            "status": "ok",
            "token_id": token.token_id,
            "owner": token.owner,
            "timestamp": token.timestamp.isoformat() if token.timestamp else None,
            "has_token": True,
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/control-token/release", methods=["POST"])
def desk_release_control_token():
    """Release current SPoC control token.
    ---
    tags:
      - Desk API - SPoC
    summary: Release SPoC control token
    description: Equivalent to Franka Desk API POST /api/system/control-token:release.
    responses:
      200:
        description: Control token released.
      500:
        description: Error while releasing control token.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        desk.release_control_token()

        return jsonify({
            "status": "released",
            "has_token": False,
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


# ------------------------------------------------------------------
# Arm / joints
# ------------------------------------------------------------------

@desk_bp.route("/arm", methods=["GET"])
def desk_arm_info():
    """Get arm information.
    ---
    tags:
      - Desk API - Arm
    summary: Get arm information
    description: Equivalent to Franka Desk API GET /api/arm.
    responses:
      200:
        description: Arm information.
      500:
        description: Error while reading arm information.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        return jsonify(desk.get_arm_info()), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/arm/joints", methods=["GET"])
def desk_joints():
    """Get joint brake status.
    ---
    tags:
      - Desk API - Arm
    summary: Get joint brake status
    description: Equivalent to Franka Desk API GET /api/arm/joints.
    responses:
      200:
        description: Joint brake status.
      500:
        description: Error while reading joint state.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        joints = desk.get_joints()

        return jsonify({
            "joints": joints,
            "all_unlocked": desk.are_joints_unlocked(),
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/arm/joints/unlock", methods=["POST"])
def desk_unlock_joints():
    """Unlock all robot joints.
    ---
    tags:
      - Desk API - Arm
    summary: Unlock all robot joints
    description: >
      Unlocks all Franka joints through the Desk API.
      Requires Basic Auth and a SPoC control token on the Franka Desk API side.
    responses:
      200:
        description: Joints unlocked.
      409:
        description: Robot is busy.
      500:
        description: Error while unlocking joints.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    try:
        desk.ensure_control_token()
        desk.unlock_joints()

        return jsonify({
            "status": "unlocked",
            "joints": desk.get_joints(),
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/arm/joints/lock", methods=["POST"])
def desk_lock_joints():
    """Lock all robot joints.
    ---
    tags:
      - Desk API - Arm
    summary: Lock all robot joints
    description: Equivalent to Franka Desk API POST /api/arm/joints:lock.
    responses:
      200:
        description: Joints locked.
      409:
        description: Robot is busy.
      500:
        description: Error while locking joints.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    try:
        desk.lock_joints()

        return jsonify({
            "status": "locked",
            "joints": desk.get_joints(),
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/arm/warnings", methods=["GET"])
def desk_arm_warnings():
    """Get arm warnings.
    ---
    tags:
      - Desk API - Arm
    summary: Get arm warnings
    description: Equivalent to Franka Desk API GET /api/arm/warnings.
    responses:
      200:
        description: Current arm warnings.
      500:
        description: Error while reading arm warnings.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        return jsonify(desk.get_arm_warnings()), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


# ------------------------------------------------------------------
# FCI
# ------------------------------------------------------------------

@desk_bp.route("/fci", methods=["GET"])
def desk_fci_state():
    """Get FCI state.
    ---
    tags:
      - Desk API - FCI
    summary: Get FCI state
    description: Equivalent to Franka Desk API GET /api/fci.
    responses:
      200:
        description: Current Franka Control Interface state.
      500:
        description: Error while reading FCI state.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        fci_status = desk.get_fci_state()

        return jsonify({
            "status": fci_status.value,
            "active": desk.is_fci_active(),
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/fci/activate", methods=["POST"])
def desk_activate_fci():
    """Activate Franka Control Interface.
    ---
    tags:
      - Desk API - FCI
    summary: Activate FCI
    description: >
      Activates the Franka Control Interface.
      Requires Basic Auth, SPoC control token, Execution mode,
      valid brakes/joints state, and installed FCI feature.
    responses:
      200:
        description: FCI activated.
      409:
        description: Robot is busy.
      500:
        description: Error while activating FCI.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    try:
        desk.ensure_control_token()
        desk.activate_fci()

        return jsonify({
            "status": "activated",
            "fci": desk.get_fci_state().value,
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/fci/deactivate", methods=["POST"])
def desk_deactivate_fci():
    """Deactivate Franka Control Interface.
    ---
    tags:
      - Desk API - FCI
    summary: Deactivate FCI
    description: Equivalent to Franka Desk API POST /api/fci:deactivate.
    responses:
      200:
        description: FCI deactivated.
      409:
        description: Robot is busy.
      500:
        description: Error while deactivating FCI.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    try:
        desk.ensure_control_token()
        desk.deactivate_fci()

        return jsonify({
            "status": "deactivated",
            "fci": desk.get_fci_state().value,
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


# ------------------------------------------------------------------
# Safety / self-tests / recovery
# ------------------------------------------------------------------

@desk_bp.route("/safety/self-tests", methods=["GET"])
def desk_self_tests_state():
    """Get self-tests state.
    ---
    tags:
      - Desk API - Safety
    summary: Get self-tests state
    description: Equivalent to Franka Desk API GET /api/safety/self-tests.
    responses:
      200:
        description: Current self-tests state.
      500:
        description: Error while reading self-tests state.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        return jsonify(desk.get_self_tests_state()), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/safety/self-tests/execute", methods=["POST"])
def desk_execute_self_tests():
    """Execute Franka self-tests.
    ---
    tags:
      - Desk API - Safety
    summary: Execute self-tests
    description: >
      Executes Franka self-tests. The arm may power-cycle and joints may be
      locked/unlocked. Requires Basic Auth and a SPoC control token on the
      Franka Desk API side.
    responses:
      200:
        description: Self-tests executed.
      409:
        description: Robot is busy.
      500:
        description: Error while executing self-tests.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    try:
        desk.ensure_control_token()
        desk.execute_self_tests()

        return jsonify({"status": "self_tests_executed"}), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/safety/recovery", methods=["GET"])
def desk_safety_recovery():
    """Get current safety recovery status.
    ---
    tags:
      - Desk API - Safety
    summary: Get safety recovery status
    description: Equivalent to Franka Desk API GET /api/safety/recovery.
    responses:
      200:
        description: Current safety recovery status.
      500:
        description: Error while reading safety recovery status.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    try:
        recovery = desk.get_safety_recovery()

        return jsonify({
            "active": recovery.get("recovery") is not None,
            "recovery": recovery.get("recovery"),
            "raw": recovery,
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/safety/recovery/confirm", methods=["POST"])
def desk_confirm_recovery():
    """Confirm recovery of a safety error or violation.
    ---
    tags:
      - Desk API - Safety
    summary: Confirm safety recovery
    description: >
      Confirms recovery of a safety error or violation.
      Some recovery types may require safety-operator role on the Franka side.
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          required:
            - type
          properties:
            type:
              type: string
              example: "SafetyError"
              description: >
                Recovery type, e.g. SafetyError, SafeInputUnacknowledged,
                JointLimitViolation, JointPositionError.
    responses:
      200:
        description: Recovery confirmed.
      400:
        description: Missing required field type.
      409:
        description: Robot is busy.
      500:
        description: Error while confirming recovery.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    recovery_type = body.get("type")

    if not recovery_type:
        return jsonify({"error": "campo obbligatorio mancante: type"}), 400

    try:
        desk.ensure_control_token()
        desk.confirm_recovery(recovery_type)

        return jsonify({
            "status": "confirmed",
            "type": recovery_type,
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


# ------------------------------------------------------------------
# High-level prepare FCI
# ------------------------------------------------------------------

@desk_bp.route("/prepare-fci", methods=["POST"])
def desk_prepare_fci():
    """Prepare robot for pylibfranka/libfranka usage.
    ---
    tags:
      - Desk API - Prepare
    summary: Prepare robot for FCI
    description: >
      Runs the Desk API preparation sequence before pylibfranka/libfranka usage:
      check system state, take SPoC control token, check safety recovery,
      switch to Execution if needed, unlock joints if needed, and activate FCI if needed.
      This endpoint does not create a pylibfranka robot object.
    responses:
      200:
        description: Robot prepared for FCI.
      409:
        description: Robot is busy.
      500:
        description: Error while preparing FCI.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    try:
        report = desk.prepare_for_fci()

        return jsonify({
            "status": "prepared",
            "report": report,
        }), 200

    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@desk_bp.route("/prepare-and-connect", methods=["POST"])
def desk_prepare_and_connect_robot():
    """Prepare Desk API and connect pylibfranka robot.
    ---
    tags:
      - Desk API - Prepare
    summary: Prepare FCI and connect robot
    description: >
      Runs prepare_for_fci through the Desk API and then creates a FrankaRobot
      pylibfranka/libfranka connection. Useful for a single GUI button such as
      Prepare robot.
    parameters:
      - in: body
        name: body
        required: false
        schema:
          type: object
          properties:
            enforce_realtime:
              type: boolean
              example: true
    responses:
      200:
        description: Robot prepared and connected.
      409:
        description: Robot is busy or pylibfranka is already connected.
      500:
        description: Error while preparing or connecting.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    if manager.is_busy:
        return _busy()

    if manager.is_connected:
        return jsonify({
            "error": "pylibfranka già connesso",
            "mode": manager.robot.mode.value,
        }), 409

    if FrankaRobot is None:
        return jsonify({
            "error": "FrankaRobot non importato",
            "message": "Correggi l'import di FrankaRobot in app/routes/desk_api.py",
        }), 500

    body = request.get_json(silent=True) or {}
    enforce_realtime = bool(body.get("enforce_realtime", True))

    try:
        report = desk.prepare_for_fci()

        manager.robot = FrankaRobot(
            robot_ip=desk.robot_ip,
            enforce_realtime=enforce_realtime,
        )

        return jsonify({
            "status": "prepared_and_connected",
            "desk_report": report,
            "robot": {
                "connected": True,
                "mode": manager.robot.mode.value,
                "robot_ip": manager.robot.robot_ip,
            },
        }), 200

    except Exception as exc:
        manager.robot = None

        return jsonify({
            "error": "prepare/connect fallito",
            "message": str(exc),
        }), 500


# ------------------------------------------------------------------
# Desk API errors
# ------------------------------------------------------------------

@desk_bp.route("/errors", methods=["GET"])
def desk_errors():
    """Get collected Desk API errors.
    ---
    tags:
      - Desk API - Errors
    summary: Get Desk API errors
    description: Returns the list of collected Desk API errors.
    responses:
      200:
        description: List of collected Desk API errors.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    return jsonify({
        "errors": [_serialize_desk_error(e) for e in desk.errors],
    }), 200


@desk_bp.route("/errors", methods=["DELETE"])
def desk_clear_errors():
    """Clear collected Desk API errors.
    ---
    tags:
      - Desk API - Errors
    summary: Clear Desk API errors
    description: Clears the list of collected Desk API errors.
    responses:
      200:
        description: Desk API errors cleared.
      503:
        description: Desk API not initialized.
    """
    desk = _get_desk()

    if desk is None:
        return _desk_not_connected()

    desk.clear_errors()

    return jsonify({"status": "cleared"}), 200