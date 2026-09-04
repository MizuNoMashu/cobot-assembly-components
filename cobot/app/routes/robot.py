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
    return jsonify(
        {"error": "robot occupato, attendere il completamento dell'operazione corrente"}
    ), 409


def _not_connected():
    return jsonify({"error": "robot non connesso"}), 503


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


@robot_bp.route("/motion-position-control/execute-trajectory", methods=["POST"])
def execute_trajectory():
    """Execute a multi-waypoint joint trajectory.

    Esegue in sequenza una lista di waypoint articolari (es. una traiettoria
    pianificata esternamente da moveit_api, che pianifica ma non esegue mai:
    l'attuazione resta interamente qui in cobot). Ogni waypoint viene
    eseguito come un movimento verso un target (stessa interpolazione
    minimum-jerk di /motion/move-joints), nell'ordine ricevuto. Si assume che
    i 7 valori di ogni waypoint siano nello stesso ordine articolare usato da
    /motion/move-joints e /motion/move-relative (fr3_joint1..7).
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

    body = request.get_json(silent=True) or {}
    waypoints = body.get("waypoints")
    if not waypoints or not isinstance(waypoints, list):
        return jsonify(
            {"error": "waypoints deve essere una lista non vuota di liste di 7 float"}
        ), 400
    for wp in waypoints:
        if not isinstance(wp, list) or len(wp) != 7:
            return jsonify(
                {"error": "ogni waypoint deve avere esattamente 7 valori"}
            ), 400

    speed_factor = float(body.get("speed_factor", 0.2))
    tolerance = float(body.get("tolerance", 0.04))
    waypoints = [[float(v) for v in wp] for wp in waypoints]

    started = manager.run_async(
        manager.robot.motion.execute_trajectory,
        waypoints,
        speed_factor=speed_factor,
        tolerance=tolerance,
    )
    if not started:
        return _busy()
    return jsonify(
        {
            "status": "accepted",
            "operation": "execute_trajectory",
            "num_waypoints": len(waypoints),
        }
    ), 202


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

        def _step(msg):
            print(f"[PICK&PLACE] {msg}")
            if sio:
                sio.emit("workflow_step", {"message": msg})

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
