"""
Endpoint REST per robot e motion.

Tutti gli endpoint di movimento sono asincroni: ritornano 202 Accepted
immediatamente e streamano la telemetria via WebSocket (event: motion_progress).
"""

import math

from flask import Blueprint, request, jsonify
from app.robot_manager import RobotManager
from franka_controller import FrankaRobot
from franka_controller.robot import EmergencyStop
from franka_controller.force_control import (
    make_params,
    make_regrasp,
    precheck_screw_passes,
)
from franka_controller.trajectory_validation import ValidationLimits

robot_bp = Blueprint("robot", __name__, url_prefix="/api")
manager = RobotManager()


def _busy():
    return jsonify(
        {"error": "robot occupato, attendere il completamento dell'operazione corrente"}
    ), 409


def _not_connected():
    return jsonify({"error": "robot non connesso"}), 503


def _parse_trajectory_body(body):
    """Valida il body comune alle route di esecuzione traiettoria.

    Ritorna ((waypoints, speed_factor, tolerance, segment_duration), None)
    oppure (None, risposta_di_errore_400).
    """
    waypoints = body.get("waypoints")
    if not waypoints or not isinstance(waypoints, list):
        return None, (
            jsonify({"error": "waypoints deve essere una lista non vuota di liste di 7 float"}),
            400,
        )
    for wp in waypoints:
        if not isinstance(wp, list) or len(wp) != 7:
            return None, (
                jsonify({"error": "ogni waypoint deve avere esattamente 7 valori"}),
                400,
            )

    try:
        speed_factor = float(body.get("speed_factor", 0.2))
        tolerance = float(body.get("tolerance", 0.04))
        segment_duration = body.get("segment_duration")
        if segment_duration is not None:
            segment_duration = float(segment_duration)
            if not math.isfinite(segment_duration) or segment_duration <= 0.0:
                raise ValueError("segment_duration deve essere un numero finito positivo")
        waypoints = [[float(v) for v in wp] for wp in waypoints]
    except (TypeError, ValueError) as exc:
        return None, (jsonify({"error": f"Parametri traiettoria non validi: {exc}"}), 400)

    return (waypoints, speed_factor, tolerance, segment_duration), None


# ------------------------------------------------------------------
# Connection
# ------------------------------------------------------------------


@robot_bp.route("/robot/disconnect", methods=["POST"])
def disconnect():
    """Disconnect the robot.

    Disconnects the current Franka robot session and releases the connection.
    ---
    tags:
      - Robot
    responses:
      200:
        description: Robot disconnected successfully.
      409:
        description: Robot is not connected or is busy.
    """
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
    """Connect to a Franka robot.

    The request body may contain `robot_ip` and `enforce_realtime`.
    ---
    tags:
      - Robot
    parameters:
      - in: body
        name: body
        schema:
          type: object
          properties:
            robot_ip:
              type: string
              example: 172.16.0.3
            enforce_realtime:
              type: boolean
              example: true
    responses:
      200:
        description: Robot connected.
      409:
        description: Already connected.
      500:
        description: Connection failed.
    """
    if manager.is_connected:
        return jsonify(
            {
                "status": "already_connected",
                "mode": manager.robot.mode.value,
                "robot_ip": manager.robot.robot_ip,
            }
        ), 200

    body = request.get_json(silent=True) or request.values.to_dict(flat=True) or {}
    robot_ip = body.get("robot_ip", "172.16.0.3")
    enforce_realtime = bool(body.get("enforce_realtime", True))

    try:
        manager.robot = FrankaRobot(
            robot_ip=robot_ip, enforce_realtime=enforce_realtime
        )
        return jsonify({"status": "connected", "mode": manager.robot.mode.value}), 200
    except Exception as exc:
        manager.robot = None
        return jsonify({"error": "connessione fallita", "message": str(exc)}), 500


# ------------------------------------------------------------------
# Robot config
# ------------------------------------------------------------------


@robot_bp.route("/robot/config", methods=["GET"])
def robot_config():
    """Get robot configuration.

    Returns the current robot IP and realtime enforcement settings.
    ---
    tags:
      - Robot
    responses:
      200:
        description: Robot configuration.
        schema:
          type: object
          properties:
            robot_ip:
              type: string
              example: 172.16.0.3
            enforce_realtime:
              type: boolean
              example: true
            fci_note:
              type: string
    """
    if manager.is_connected:
        ip = manager.robot.robot_ip
        enforce = manager.robot.enforce_realtime
    else:
        ip = "172.16.0.3"
        enforce = True

    return jsonify(
        {
            "robot_ip": ip,
            "enforce_realtime": enforce,
            "fci_note": "FCI ports are fixed by libfranka (1337/1338), not configurable via pylibfranka",
        }
    ), 200


# ------------------------------------------------------------------
# Robot state & errors
# ------------------------------------------------------------------


@robot_bp.route("/robot/state", methods=["GET"])
def robot_state():
    """Get robot state.

    Returns the current robot joint positions, velocities, and other state information.
    ---
    tags:
      - Robot
    responses:
      200:
        description: Robot state data.
      503:
        description: Robot not connected.
      500:
        description: Error reading robot state.
    """
    if not manager.is_connected:
        return _not_connected()
    try:
        return jsonify(manager.get_robot_state()), 200
    except Exception as exc:
      status_code = 503 if manager.robot.mode.value == "error_locked" else 500
      return jsonify({"error": str(exc)}), status_code


@robot_bp.route("/robot/status", methods=["GET"])
def robot_status():
    """Get robot status.

    Returns connection status and current operation state.
    ---
    tags:
      - Robot
    responses:
      200:
        description: Robot status.
        schema:
          type: object
          properties:
            connected:
              type: boolean
            busy:
              type: boolean
            mode:
              type: string
    """
    return jsonify(manager.status_dict()), 200


@robot_bp.route("/robot/errors", methods=["GET"])
def robot_errors():
    """Get robot errors.

    Returns a list of current robot errors.
    ---
    tags:
      - Robot
    responses:
      200:
        description: List of robot errors.
        schema:
          type: object
          properties:
            errors:
              type: array
              items:
                type: object
                properties:
                  operation:
                    type: string
                  type:
                    type: string
                  message:
                    type: string
                  timestamp:
                    type: string
                  recoverable:
                    type: boolean
      503:
        description: Robot not connected.
    """
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
    """Clear robot errors.

    Clears all current robot errors.
    ---
    tags:
      - Robot
    responses:
      200:
        description: Errors cleared.
      503:
        description: Robot not connected.
    """
    if not manager.is_connected:
        return _not_connected()
    manager.robot.clear_errors()
    return jsonify({"status": "cleared"}), 200


@robot_bp.route("/robot/recovery", methods=["POST"])
def recovery():
    """Perform automatic error recovery.

    Attempts to recover from robot errors automatically.
    ---
    tags:
      - Robot
    responses:
      200:
        description: Recovery attempt result.
        schema:
          type: object
          properties:
            success:
              type: boolean
            mode:
              type: string
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()
    success = manager.robot.automatic_error_recovery()
    return jsonify({"success": success, "mode": manager.robot.mode.value}), 200


@robot_bp.route("/robot/stop", methods=["POST"])
def emergency_stop():
    """Software emergency stop.

    Ferma subito l'operazione in corso, di qualunque tipo (movimenti, forza,
    impedenza, traiettorie, pick-and-place, homing). Nei loop di controllo il
    taglio avviene al ciclo successivo (1 kHz). Il robot resta in ERROR_LOCKED
    fino a POST /api/robot/recovery. Se non c'e' nessuna operazione in corso
    non fa nulla (200, stop_requested=false).

    NON e' una funzione di sicurezza certificata: dipende dal processo Python
    e dalla rete. Non sostituisce il pulsante di emergenza fisico.
    ---
    tags:
      - Robot
    responses:
      200:
        description: Stop richiesto (stop_requested=true) oppure nessuna operazione in corso.
        schema:
          type: object
          properties:
            stop_requested:
              type: boolean
            operation:
              type: string
            message:
              type: string
      503:
        description: Robot not connected.
    """
    if not manager.is_connected:
        return _not_connected()
    return jsonify(manager.request_stop()), 200


@robot_bp.route("/robot/collision-behavior", methods=["POST"])
def collision_behavior():
    """Set collision behavior.

    Configures collision detection thresholds for torque and force.
    ---
    tags:
      - Robot
    parameters:
      - in: body
        name: body
        schema:
          type: object
          properties:
            lower_torque:
              type: array
              items:
                type: number
            upper_torque:
              type: array
              items:
                type: number
            lower_force:
              type: array
              items:
                type: number
            upper_force:
              type: array
              items:
                type: number
    responses:
      200:
        description: Collision behavior set.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
      500:
        description: Error setting collision behavior.
    """
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
    """Move robot to home position.

    Moves the robot to its predefined home joint positions.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        schema:
          type: object
          properties:
            speed_factor:
              type: number
              example: 0.2
    responses:
      202:
        description: Motion accepted.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    speed_factor = float(body.get("speed_factor", 0.2))

    started = manager.run_async(
        manager.robot.motion.go_to_home, speed_factor=speed_factor
    )
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "move_home"}), 202


@robot_bp.route("/motion-position-control/move", methods=["POST"])
def move_joints():
    """Move to joint positions.

    Moves the robot to specified joint positions.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          properties:
            positions:
              type: array
              items:
                type: number
              minItems: 7
              maxItems: 7
            speed_factor:
              type: number
              example: 0.2
            tolerance:
              type: number
              example: 0.04
    responses:
      202:
        description: Motion accepted.
      400:
        description: Invalid positions.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
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


@robot_bp.route("/motion-cartesian/move-relative", methods=["POST"])
def move_relative_cartesian():
    """Move relative to current position.

    Moves the robot by relative Cartesian deltas from current position.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          properties:
            dx:
              type: number
            dy:
              type: number
            dz:
              type: number
            droll:
              type: number
            dpitch:
              type: number
            dyaw:
              type: number
            speed_factor:
              type: number
              example: 0.2
    responses:
      202:
        description: Motion accepted.
      400:
        description: Invalid Cartesian deltas.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    try:
        deltas = [
            float(body.get(value, 0.0))
            for value in ("dx", "dy", "dz", "droll", "dpitch", "dyaw")
        ]
        speed_factor = float(body.get("speed_factor", 0.2))
        tolerance = float(body.get("tolerance", 0.01))
    except (TypeError, ValueError):
        return jsonify(
            {"error": "i parametri dello spostamento devono essere float"}
        ), 400

    started = manager.run_async(
        manager.robot.motion.move_relative,
        *deltas,
        speed_factor=speed_factor,
        tolerance=tolerance,
    )
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "move_relative_cartesian"}), 202


@robot_bp.route("/motion-cartesian/move", methods=["POST"])
def move_cartesian():
    """Move to a cartesian pose.

    Moves the robot to the specified cartesian position and orientation.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          properties:
            x:
              type: number
            y:
              type: number
            z:
              type: number
            roll:
              type: number
              default: 0.0
            pitch:
              type: number
              default: 0.0
            yaw:
              type: number
              default: 0.0
            speed_factor:
              type: number
              example: 0.2
            tolerance:
              type: number
              example: 0.01
            orientation_tolerance:
              type: number
              example: 0.05
    responses:
      202:
        description: Motion accepted.
      400:
        description: Invalid cartesian pose.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    required = ("x", "y", "z")
    if any(body.get(value) is None for value in required):
        return jsonify(
            {"error": "x, y e z sono obbligatori e devono essere float"}
        ), 400

    try:
        pose = [
            float(body.get(value, 0.0))
            for value in ("x", "y", "z", "roll", "pitch", "yaw")
        ]
        speed_factor = float(body.get("speed_factor", 0.2))
        tolerance = float(body.get("tolerance", 0.01))
        orientation_tolerance = float(body.get("orientation_tolerance", 0.05))
    except (TypeError, ValueError):
        return jsonify({"error": "i parametri della posa devono essere float"}), 400

    started = manager.run_async(
        manager.robot.motion.move_to_cartesian_pose,
        *pose,
        speed_factor=speed_factor,
        tolerance=tolerance,
        orientation_tolerance=orientation_tolerance,
    )
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "move_cartesian_pose"}), 202


@robot_bp.route("/motion-position-control/move-relative", methods=["POST"])
def move_relative_joints():
    """Move relative to current position.

    Moves the robot by relative joint deltas from current position.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          properties:
            deltas:
              type: array
              items:
                type: number
              minItems: 7
              maxItems: 7
            speed_factor:
              type: number
              example: 0.2
    responses:
      202:
        description: Motion accepted.
      400:
        description: Invalid deltas.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
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


@robot_bp.route("/motion/execute-trajectory", methods=["POST"])
def execute_trajectory():
    """Execute a multi-waypoint joint trajectory.

    Esegue in sequenza una lista di waypoint articolari (es. una traiettoria
    pianificata esternamente da moveit_api, che pianifica ma non esegue mai:
    l'attuazione resta interamente qui in cobot). Ogni waypoint viene
    eseguito come un movimento verso un target (stessa interpolazione
    minimum-jerk di /motion/move-joints), nell'ordine ricevuto. Si assume che
    i 7 valori di ogni waypoint siano nello stesso ordine articolare usato da
    /motion/move-joints e /motion/move-relative (fr3_joint1..7).

    Prima di muovere il robot l'intera traiettoria passa un controllo
    pre-esecuzione (limiti di giunto, velocità/accelerazioni, singolarità);
    se fallisce il robot non si muove e l'errore è in last_operation.error
    (evento WebSocket motion_error). Il controllo gira nel thread di movimento,
    quindi la risposta 202 resta immediata. Per la versione senza soste nei
    punti intermedi vedi /motion/execute-trajectory-fly.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          properties:
            waypoints:
              type: array
              items:
                type: array
                items:
                  type: number
                minItems: 7
                maxItems: 7
            speed_factor:
              type: number
              example: 0.2
            tolerance:
              type: number
              example: 0.04
            segment_duration:
              type: number
              description: Optional minimum-jerk duration in seconds for each waypoint (overrides the legacy 5 / speed_factor duration).
              example: 2.0
    responses:
      202:
        description: Motion accepted.
      400:
        description: Invalid waypoints.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    parsed, error = _parse_trajectory_body(request.get_json(silent=True) or {})
    if error is not None:
        return error
    waypoints, speed_factor, tolerance, segment_duration = parsed

    started = manager.run_async(
        manager.robot.motion.execute_trajectory,
        waypoints,
        speed_factor=speed_factor,
        tolerance=tolerance,
        segment_duration=segment_duration,
    )
    if not started:
        return _busy()
    return jsonify(
        {
            "status": "accepted",
            "operation": "execute_trajectory",
            "num_waypoints": len(waypoints),
            "segment_duration": segment_duration,
            "precheck_timeout_sec": ValidationLimits.from_env().timeout_sec,
        }
    ), 202


@robot_bp.route("/motion/execute-trajectory-fly", methods=["POST"])
def execute_trajectory_fly():
    """Execute a joint trajectory as a fly motion (no stop at intermediate points).

    Stessi waypoint di /motion/execute-trajectory, ma eseguiti come un unico
    percorso continuo (fly-by / continuous path): i punti intermedi non sono
    destinazioni ma via-point, e attorno a ciascuno il percorso curva entro una
    zona di raccordo di raggio `blend_radius` (rad, spazio dei giunti). Il robot
    si ferma solo al punto finale. Prima di muovere il robot il percorso
    raccordato viene validato (limiti di giunto, velocità, singolarità).
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          properties:
            waypoints:
              type: array
              items:
                type: array
                items:
                  type: number
                minItems: 7
                maxItems: 7
            speed_factor:
              type: number
              description: Frazione del limite di velocità dei giunti (usata se segment_duration è assente).
              example: 0.2
            tolerance:
              type: number
              example: 0.04
            segment_duration:
              type: number
              description: Durata totale richiesta = len(waypoints) * segment_duration. Può essere allungata se i limiti di velocità/accelerazione lo richiedono.
              example: 0.7
            blend_radius:
              type: number
              description: Raggio delle zone di raccordo in rad (0 = nessun raccordo).
              example: 0.15
    responses:
      202:
        description: Motion accepted (il pre-check gira nel thread di movimento; un eventuale rifiuto appare in last_operation.error / evento motion_error).
      400:
        description: Invalid waypoints or parameters.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    parsed, error = _parse_trajectory_body(body)
    if error is not None:
        return error
    waypoints, speed_factor, tolerance, segment_duration = parsed

    try:
        blend_radius = float(body.get("blend_radius", 0.15))
        if not math.isfinite(blend_radius) or blend_radius < 0.0:
            raise ValueError("blend_radius deve essere un numero finito >= 0")
    except (TypeError, ValueError) as exc:
        return jsonify({"error": f"Parametri traiettoria non validi: {exc}"}), 400

    started = manager.run_async(
        manager.robot.motion.execute_trajectory_fly,
        waypoints,
        speed_factor=speed_factor,
        tolerance=tolerance,
        segment_duration=segment_duration,
        blend_radius=blend_radius,
    )
    if not started:
        return _busy()
    return jsonify(
        {
            "status": "accepted",
            "operation": "execute_trajectory_fly",
            "num_waypoints": len(waypoints),
            "segment_duration": segment_duration,
            "blend_radius": blend_radius,
            "precheck_timeout_sec": ValidationLimits.from_env().timeout_sec,
        }
    ), 202


_REGRASP_KEYS = {
    "regrasp_mode", "rewind_speed", "retract_distance",
    "open_width_mm", "grasp_width_mm", "grasp_force", "gripper_speed",
}


def _force_task_response(mode, body, required, aliases=None, passes=False):
    """Valida il body di una route di force control e avvia il task.

    I parametri sono validati in modo sincrono (400 se non validi, anche il
    limite del giunto 7 per l'avvitamento); il task gira poi in background
    come le altre route di movimento. Con passes=True accetta `iterations` e
    i parametri di ripresa e avvia un avvitamento a più passate.
    """
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    fields = dict(body)
    for name, target in (aliases or {}).items():
        if name in fields:
            fields[target] = fields.pop(name)
    try:
        for name in required:
            if fields.get(name) is None:
                raise ValueError(f"{name} è obbligatorio")
        force = fields.pop("force")
        if "axis_tool" in fields:
            fields["axis_tool"] = tuple(float(v) for v in fields["axis_tool"])
        regrasp_fields = {}
        iterations = 1
        if passes:
            regrasp_fields = {k: fields.pop(k) for k in list(fields) if k in _REGRASP_KEYS}
            raw_iterations = fields.pop("iterations", 1)
            if isinstance(raw_iterations, bool) or int(raw_iterations) != raw_iterations:
                raise ValueError("iterations deve essere un intero")
            iterations = int(raw_iterations)
        params = make_params(mode, force, **fields)
        # con una sola passata non serve nessuna ripresa: i relativi parametri sono ignorati
        regrasp = (
            make_regrasp(**regrasp_fields) if passes and iterations > 1
            else make_regrasp(regrasp_mode="none") if passes else None
        )
        if passes:
            # il limite del giunto 7 dipende dalla posizione corrente: errore subito
            precheck_screw_passes(params, regrasp, iterations, manager.robot.get_state().q)
    except (TypeError, ValueError) as exc:
        return jsonify({"error": f"Parametri di forza non validi: {exc}"}), 400

    if passes:
        started = manager.run_async(
            manager.robot.motion.run_screw_passes, params, regrasp, iterations
        )
    else:
        started = manager.run_async(manager.robot.motion.run_force_task, params)
    if not started:
        return _busy()
    response = {
        "status": "accepted",
        "operation": f"force_{mode}",
        "force": params.force,
        "abort_force": params.abort_force,
        "axis_tool": list(params.axis_tool),
    }
    if passes:
        response["iterations"] = iterations
        response["regrasp_mode"] = regrasp.mode
    return jsonify(response), 202


@robot_bp.route("/motion-force/apply", methods=["POST"])
def force_apply():
    """Press with a controlled force.

    Spinge lungo un asse del tool (default z) con una forza costante, dopo un
    avvicinamento a velocità limitata fino al contatto. Torque control con
    forza lungo l'asse e impedenza nelle altre direzioni.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          required: [force]
          properties:
            force:
              type: number
              description: Forza di spinta in N (max 40).
              example: 10
            duration:
              type: number
              description: Tempo di mantenimento della forza in s.
              example: 5
            axis_tool:
              type: array
              items:
                type: number
              description: Asse di spinta nel frame del tool (default [0,0,1]).
            approach_speed:
              type: number
              description: Velocità di avvicinamento in m/s (0 = già a contatto, default 0.01).
            max_approach_distance:
              type: number
              description: Corsa massima senza contatto in m (default 0.05).
    responses:
      202:
        description: Task accepted (l'esito finale è in last_operation).
      400:
        description: Invalid parameters.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    return _force_task_response(
        "hold", request.get_json(silent=True) or {}, ["force"], {"duration": "hold_time"}
    )


@robot_bp.route("/motion-force/insert", methods=["POST"])
def force_insert():
    """Insert by pushing with a controlled force.

    Dopo il contatto spinge con `force` N finché l'oggetto è avanzato di
    `target_depth` m. Termina con errore (senza bloccare il robot) se
    l'inserimento si blocca, va in timeout o supera max_depth.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          required: [force, target_depth]
          properties:
            force:
              type: number
              example: 15
            target_depth:
              type: number
              description: Profondità di inserimento in m dal punto di contatto.
              example: 0.012
            max_time:
              type: number
              example: 30
            stall_time:
              type: number
              description: Secondi senza avanzamento prima di dichiarare il blocco.
              example: 1.5
            lateral_stiffness:
              type: number
              description: Cedevolezza laterale in N/m (più bassa = più autoallineante).
              example: 300
    responses:
      202:
        description: Task accepted.
      400:
        description: Invalid parameters.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    return _force_task_response(
        "insert", request.get_json(silent=True) or {}, ["force", "target_depth"]
    )


@robot_bp.route("/motion-force/screw", methods=["POST"])
def force_screw():
    """Screw: push with a controlled force while rotating about the tool axis.

    Spinge con `force` N e, a forza stabilizzata, ruota di `angle` rad attorno
    all'asse con coppia limitata a `max_torque` Nm. La rotazione è quella del
    giunto 7 (escursione ~±3 rad): una passata oltre il limite viene rifiutata
    subito con 400. Con `iterations` > 1 l'operazione resta unica ma ripete la
    passata più volte; tra una passata e l'altra il giunto 7 viene riportato a
    vuoto all'estremo opposto con la ripresa scelta in `regrasp_mode`:
    "gripper" (apre, riavvolge, richiude sulla vite), "retract" (arretra
    lungo l'asse e riavvolge: cacciavite/bit) o "none" (cricchetto).
    Con `stop_at_torque` la coppia massima è l'obiettivo (serraggio) e
    ferma le passate restanti; altrimenti raggiungerla è un errore.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          required: [force, angle]
          properties:
            force:
              type: number
              example: 10
            angle:
              type: number
              description: Rotazione PER PASSATA in rad con segno, regola della mano destra attorno all'asse z del tool (se z punta verso la vite, positivo = avvitare una vite destrorsa).
              example: 5.0
            iterations:
              type: integer
              description: Numero di passate (1-20). Rotazione totale = iterations x angle.
              example: 2
            regrasp_mode:
              type: string
              enum: [gripper, retract, none]
              example: gripper
            grasp_width_mm:
              type: number
              description: Larghezza della presa sulla vite in mm (obbligatorio con regrasp_mode gripper).
              example: 8
            open_width_mm:
              type: number
              example: 20
            grasp_force:
              type: number
              example: 40
            retract_distance:
              type: number
              description: Arretramento in m (regrasp_mode retract).
              example: 0.005
            rewind_speed:
              type: number
              description: Velocità di riavvolgimento a vuoto del giunto 7 in rad/s.
              example: 1.0
            angular_speed:
              type: number
              example: 0.3
            max_torque:
              type: number
              description: Coppia massima attorno all'asse in Nm (max 6).
              example: 2.0
            stop_at_torque:
              type: boolean
              example: false
    responses:
      202:
        description: Task accepted.
      400:
        description: Invalid parameters (anche rotazione oltre il limite del giunto 7).
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    return _force_task_response(
        "screw", request.get_json(silent=True) or {}, ["force", "angle"], passes=True
    )


@robot_bp.route("/motion-position-control/move-impedance", methods=["POST"])
def move_to_joint_positions_with_impedance():
    """Move to joint positions with impedance control.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          properties:
            positions:
              type: array
              items:
                type: number
              minItems: 7
              maxItems: 7
            stiffness:
              type: array
              items:
                type: number
            damping:
              type: array
              items:
                type: number
            duration:
              type: number
              default: 5.0
            tolerance:
              type: number
              default: 0.04
    responses:
      202:
        description: Motion accepted.
      400:
        description: Invalid joint positions.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
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
        manager.robot.motion.move_to_joint_positions_with_impedance,
        [float(p) for p in positions],
        stiffness=[float(s) for s in stiffness] if stiffness else None,
        duration=duration,
    )
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "impedance"}), 202


@robot_bp.route("/motion-joint-control/move", methods=["POST"])
def impedance_control():
    """Move joints with impedance control.
    ---
    tags:
      - Motion
    parameters:
      - in: body
        name: body
        required: true
        schema:
          type: object
          properties:
            positions:
              type: array
              items:
                type: number
              minItems: 7
              maxItems: 7
            stiffness:
              type: array
              items:
                type: number
            damping:
              type: array
              items:
                type: number
            duration:
              type: number
              default: 5.0
            tolerance:
              type: number
              default: 0.04
    responses:
      202:
        description: Motion accepted.
      400:
        description: Invalid joint positions.
      503:
        description: Robot not connected.
      409:
        description: Robot is busy.
    """
    if not manager.is_connected:
        return _not_connected()
    if manager.is_busy:
        return _busy()

    body = request.get_json(silent=True) or {}
    positions = body.get("positions")
    if not positions or len(positions) != 7:
        return jsonify({"error": "positions deve essere una lista di 7 float"}), 400

    stiffness = body.get("stiffness")
    damping = body.get("damping")
    duration = float(body.get("duration", 5.0))
    tolerance = float(body.get("tolerance", 0.04))

    started = manager.run_async(
        manager.robot.motion.impedance_control,
        [float(p) for p in positions],
        stiffness=[float(s) for s in stiffness] if stiffness else None,
        damping=[float(d) for d in damping] if damping else None,
        duration=duration,
        tolerance=tolerance,
    )
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "impedance_control"}), 202


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
    place_position = body.get(
        "place_position", [-0.3, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785]
    )
    grasp_width_m = float(body.get("grasp_width_mm", 50.0)) / 1000.0
    grasp_force = float(body.get("grasp_force", 60.0))
    speed_factor = float(body.get("speed_factor", 0.15))

    def _workflow(progress_callback=None):
        robot = manager.robot
        sio = manager._socketio

        def _notify(msg):
            print(f"[PICK&PLACE] {msg}")
            if sio:
                sio.emit("workflow_step", {"message": msg})

        def _step(msg):
            # dopo uno stop nessun passo successivo (es. apertura gripper) deve partire
            robot.check_stop()
            _notify(msg)

        try:
            _step("Step 1/7: home position")
            robot.motion.go_to_home(
                speed_factor=speed_factor, progress_callback=progress_callback
            )

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
            robot.motion.go_to_home(
                speed_factor=speed_factor, progress_callback=progress_callback
            )

            _notify("✓ Workflow completato")
            return True

        except EmergencyStop:
            _notify("✗ Workflow interrotto: arresto di emergenza software")
            return False

        except Exception as exc:
            _notify(f"✗ Errore durante il workflow: {exc}")
            if not robot.stop_requested:
                robot.automatic_error_recovery()
            return False

    started = manager.run_async(_workflow, on_complete_event="workflow_complete")
    if not started:
        return _busy()
    return jsonify({"status": "accepted", "operation": "pick_and_place"}), 202
