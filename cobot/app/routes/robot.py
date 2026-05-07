"""
Endpoint REST per robot e motion.

Tutti gli endpoint di movimento sono asincroni: ritornano 202 Accepted
immediatamente e streamano la telemetria via WebSocket (event: motion_progress).
"""

from flask import Blueprint, request, jsonify
from app.robot_manager import RobotManager
from franka_controller import FrankaRobot

robot_bp = Blueprint("robot", __name__, url_prefix="/api")
manager = RobotManager()


def _busy():
    return jsonify({"error": "robot occupato, attendere il completamento dell'operazione corrente"}), 409


def _not_connected():
    return jsonify({"error": "robot non connesso"}), 503


# ------------------------------------------------------------------
# Connection
# ------------------------------------------------------------------

@robot_bp.route("/robot/disconnect", methods=["POST"])
def disconnect():
    if not manager.is_connected:
        return jsonify({"error": "non connesso"}), 409
    if manager.is_busy:
        return _busy()

    # Ferma il gripper prima di mollare il riferimento (come faceva Ctrl+C nel main originale)
    try:
        manager.robot.gripper.stop()
    except Exception:
        pass

    # Impostare a None → GC Python → destructor C++ pylibfranka chiude la connessione FCI
    manager.robot = None
    return jsonify({"status": "disconnected"}), 200


@robot_bp.route("/robot/connect", methods=["POST"])
def connect():
    if manager.is_connected:
        return jsonify({"error": "già connesso", "mode": manager.robot.mode.value}), 409

    body = request.get_json(silent=True) or {}
    robot_ip = body.get("robot_ip", "172.16.0.3")
    enforce_realtime = bool(body.get("enforce_realtime", True))

    try:
        manager.robot = FrankaRobot(robot_ip=robot_ip, enforce_realtime=enforce_realtime)
        return jsonify({"status": "connected", "mode": manager.robot.mode.value}), 200
    except Exception as exc:
        manager.robot = None
        return jsonify({"error": "connessione fallita", "message": str(exc)}), 500


# ------------------------------------------------------------------
# Robot config
# ------------------------------------------------------------------

@robot_bp.route("/robot/config", methods=["GET"])
def robot_config():
    if manager.is_connected:
        ip = manager.robot.robot_ip
        enforce = manager.robot.enforce_realtime
    else:
        ip = "172.16.0.3"
        enforce = True

    return jsonify({
        "robot_ip": ip,
        "enforce_realtime": enforce,
        "fci_note": "FCI ports are fixed by libfranka (1337/1338), not configurable via pylibfranka",
    }), 200


# ------------------------------------------------------------------
# Robot state & errors
# ------------------------------------------------------------------

@robot_bp.route("/robot/state", methods=["GET"])
def robot_state():
    if not manager.is_connected:
        return _not_connected()
    try:
        return jsonify(manager.get_robot_state()), 200
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@robot_bp.route("/robot/status", methods=["GET"])
def robot_status():
    return jsonify(manager.status_dict()), 200


@robot_bp.route("/robot/errors", methods=["GET"])
def robot_errors():
    if not manager.is_connected:
        return _not_connected()
    errors = [
        {
            "operation": e.operation,
            "type": e.error_type.value,
            "message": e.message,
            "timestamp": e.timestamp.isoformat(),
            "recoverable": e.recoverable,
        }
        for e in manager.robot.errors
    ]
    return jsonify({"errors": errors}), 200


@robot_bp.route("/robot/errors", methods=["DELETE"])
def clear_robot_errors():
    if not manager.is_connected:
        return _not_connected()
    manager.robot.clear_errors()
    return jsonify({"status": "cleared"}), 200


@robot_bp.route("/robot/recovery", methods=["POST"])
def recovery():
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()
    success = manager.robot.automatic_error_recovery()
    return jsonify({"success": success, "mode": manager.robot.mode.value}), 200


@robot_bp.route("/robot/collision-behavior", methods=["POST"])
def collision_behavior():
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    try:
        manager.robot.set_collision_behavior(
            lower_torque_thresholds=body.get("lower_torque"),
            upper_torque_thresholds=body.get("upper_torque"),
            lower_force_thresholds=body.get("lower_force"),
            upper_force_thresholds=body.get("upper_force"),
        )
        return jsonify({"status": "ok"}), 200
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


# ------------------------------------------------------------------
# Motion endpoints (asincroni – 202 Accepted)
# ------------------------------------------------------------------

@robot_bp.route("/motion/move-home", methods=["POST"])
def move_home():
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    speed_factor = float(body.get("speed_factor", 0.2))

    started = manager.run_async(manager.robot.motion.go_to_home, speed_factor=speed_factor)
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "move_home"}), 202


@robot_bp.route("/motion/move-joints", methods=["POST"])
def move_joints():
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    positions = body.get("positions")
    if not positions or len(positions) != 7:
        return jsonify({"error": "positions deve essere una lista di 7 float"}), 400

    speed_factor = float(body.get("speed_factor", 0.2))
    tolerance = float(body.get("tolerance", 0.04))

    started = manager.run_async(
        manager.robot.motion.move_to_joint_positions,
        [float(p) for p in positions],
        speed_factor=speed_factor,
        tolerance=tolerance,
    )
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "move_joints"}), 202


@robot_bp.route("/motion/move-relative", methods=["POST"])
def move_relative():
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    deltas = body.get("deltas")
    if not deltas or len(deltas) != 7:
        return jsonify({"error": "deltas deve essere una lista di 7 float"}), 400

    speed_factor = float(body.get("speed_factor", 0.2))

    started = manager.run_async(
        manager.robot.motion.move_relative,
        [float(d) for d in deltas],
        speed_factor=speed_factor,
    )
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "move_relative"}), 202


@robot_bp.route("/motion/impedance", methods=["POST"])
def impedance():
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    positions = body.get("positions")
    if not positions or len(positions) != 7:
        return jsonify({"error": "positions deve essere una lista di 7 float"}), 400

    stiffness = body.get("stiffness")
    duration = float(body.get("duration", 5.0))

    started = manager.run_async(
        manager.robot.motion.impedance_control,
        [float(p) for p in positions],
        stiffness=[float(s) for s in stiffness] if stiffness else None,
        duration=duration,
    )
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "impedance"}), 202


@robot_bp.route("/motion/pick-and-place", methods=["POST"])
def pick_and_place():
    """
    Workflow pick-and-place completo (asincrono).

    Body opzionale:
      pick_position  : [7 float]  – default pick simulato
      place_position : [7 float]  – default place simulato
      grasp_width_mm : float      – larghezza oggetto in mm (default 50)
      grasp_force    : float      – forza presa in N (default 60)
      speed_factor   : float      – velocità movimenti (default 0.15)

    Sequenza:
      1. home  2. apri gripper  3. vai a pick  4. grasp
      5. vai a place  6. release  7. home
    """
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    pick_position = body.get("pick_position", [0.3, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785])
    place_position = body.get("place_position", [-0.3, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785])
    grasp_width_m = float(body.get("grasp_width_mm", 50.0)) / 1000.0
    grasp_force = float(body.get("grasp_force", 60.0))
    speed_factor = float(body.get("speed_factor", 0.15))

    def _workflow(progress_callback=None):
        robot = manager.robot
        sio = manager._socketio

        def _step(msg):
            print(f"[PICK&PLACE] {msg}")
            if sio:
                sio.emit("workflow_step", {"message": msg})

        try:
            _step("Step 1/7: home position")
            robot.motion.go_to_home(speed_factor=speed_factor, progress_callback=progress_callback)

            _step("Step 2/7: apertura gripper")
            robot.gripper.open(width=0.08, speed=0.1)

            _step("Step 3/7: movimento verso pick position")
            robot.motion.move_to_joint_positions(
                [float(p) for p in pick_position],
                speed_factor=speed_factor,
                progress_callback=progress_callback,
            )

            _step("Step 4/7: presa oggetto")
            robot.gripper.grasp(width=grasp_width_m, force=grasp_force, speed=0.1)

            _step("Step 5/7: movimento verso place position")
            robot.motion.move_to_joint_positions(
                [float(p) for p in place_position],
                speed_factor=speed_factor,
                progress_callback=progress_callback,
            )

            _step("Step 6/7: rilascio oggetto")
            robot.gripper.release()

            _step("Step 7/7: ritorno a home")
            robot.motion.go_to_home(speed_factor=speed_factor, progress_callback=progress_callback)

            _step("✓ Workflow completato")
            return True

        except Exception as exc:
            _step(f"✗ Errore durante il workflow: {exc}")
            robot.automatic_error_recovery()
            return False

    started = manager.run_async(_workflow, on_complete_event="workflow_complete")
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "pick_and_place"}), 202
