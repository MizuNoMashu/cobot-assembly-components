"""
Endpoint REST per il gripper.

Le operazioni gripper sono bloccanti lato pylibfranka ma hanno durata breve
(< 2s tipicamente), quindi vengono eseguite in modo sincrono con timeout implicito.
Le operazioni lunghe (homing) usano lo stesso pattern async del motion.
"""

from flask import Blueprint, request, jsonify
from app.robot_manager import RobotManager

gripper_bp = Blueprint("gripper", __name__, url_prefix="/api/gripper")
manager = RobotManager()


def _not_connected():
    return jsonify({"error": "robot/gripper non connesso"}), 503


def _busy():
    return jsonify({"error": "robot occupato"}), 409


def _gripper():
    if not manager.is_connected:
        return None
    return manager.robot.gripper


# ------------------------------------------------------------------
# State & errors
# ------------------------------------------------------------------

@gripper_bp.route("/state", methods=["GET"])
def gripper_state():
    g = _gripper()
    if g is None:
        return _not_connected()
    state = g.get_state()
    if state is None:
        return jsonify({"error": "impossibile leggere lo stato del gripper"}), 500
    return jsonify({
        "mode": g.mode.value,
        "width_mm": round(state["width"] * 1000, 2),
        "max_width_mm": round(state["max_width"] * 1000, 2),
        "is_grasped": state["is_grasped"],
        "temperature": state["temperature"],
    }), 200


@gripper_bp.route("/errors", methods=["GET"])
def gripper_errors():
    g = _gripper()
    if g is None:
        return _not_connected()
    errors = [
        {
            "operation": e.operation,
            "type": e.error_type.value,
            "message": e.message,
            "timestamp": e.timestamp.isoformat(),
            "recoverable": e.recoverable,
        }
        for e in g.errors
    ]
    return jsonify({"errors": errors}), 200


@gripper_bp.route("/errors", methods=["DELETE"])
def clear_gripper_errors():
    g = _gripper()
    if g is None:
        return _not_connected()
    g.clear_errors()
    return jsonify({"status": "cleared"}), 200


# ------------------------------------------------------------------
# Commands
# ------------------------------------------------------------------

@gripper_bp.route("/home", methods=["POST"])
def gripper_home():
    g = _gripper()
    if g is None:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    # homing è bloccante (~2-5s): lo eseguiamo async per non bloccare Flask
    started = manager.run_async(g.homing, on_complete_event="gripper_complete")
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "homing"}), 202


@gripper_bp.route("/open", methods=["POST"])
def gripper_open():
    g = _gripper()
    if g is None:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    width_mm = float(body.get("width_mm", 80.0))
    speed = float(body.get("speed", 0.1))

    try:
        result = g.open(width=width_mm / 1000.0, speed=speed)
        return jsonify({"success": result}), 200
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@gripper_bp.route("/close", methods=["POST"])
def gripper_close():
    g = _gripper()
    if g is None:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    speed = float(body.get("speed", 0.1))

    try:
        result = g.close(speed=speed)
        return jsonify({"success": result}), 200
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@gripper_bp.route("/grasp", methods=["POST"])
def gripper_grasp():
    g = _gripper()
    if g is None:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    width_mm = body.get("width_mm")
    if width_mm is None:
        return jsonify({"error": "width_mm richiesto"}), 400

    force = float(body.get("force", 60.0))
    speed = float(body.get("speed", 0.1))
    epsilon_inner = float(body.get("epsilon_inner", 0.005))
    epsilon_outer = float(body.get("epsilon_outer", 0.005))

    try:
        result = g.grasp(
            width=float(width_mm) / 1000.0,
            force=force,
            speed=speed,
            epsilon_inner=epsilon_inner,
            epsilon_outer=epsilon_outer,
        )
        return jsonify({"success": result}), 200
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@gripper_bp.route("/release", methods=["POST"])
def gripper_release():
    g = _gripper()
    if g is None:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    speed = float(body.get("speed", 0.1))

    try:
        result = g.release(speed=speed)
        return jsonify({"success": result}), 200
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@gripper_bp.route("/stop", methods=["POST"])
def gripper_stop():
    g = _gripper()
    if g is None:
        return _not_connected()
    try:
        result = g.stop()
        return jsonify({"success": result}), 200
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@gripper_bp.route("/reset-lock", methods=["POST"])
def gripper_reset_lock():
    g = _gripper()
    if g is None:
        return _not_connected()
    try:
        g.reset_error_lock()
        return jsonify({"status": "ok", "mode": g.mode.value}), 200
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@gripper_bp.route("/reconnect", methods=["POST"])
def gripper_reconnect():
    g = _gripper()
    if g is None:
        return _not_connected()
    if manager.is_busy:
        return _busy()
    try:
        g.reconnect()
        return jsonify({"status": "reconnected", "mode": g.mode.value}), 200
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500
