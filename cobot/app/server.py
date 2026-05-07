"""
Entry point Flask + Flask-SocketIO per il cobot API server.

eventlet deve essere importato e monkey-patched PRIMA di qualsiasi altro import
per garantire il funzionamento corretto di Flask-SocketIO in modalità asincrona.
"""

import eventlet
eventlet.monkey_patch()

import os
from flask import Flask, jsonify
from flask_socketio import SocketIO

from app.robot_manager import RobotManager
from app.routes.robot import robot_bp
from app.routes.gripper import gripper_bp

# ------------------------------------------------------------------
# App setup
# ------------------------------------------------------------------

app = Flask(__name__)
app.config["SECRET_KEY"] = os.getenv("SECRET_KEY", "franka-cobot-secret")

socketio = SocketIO(
    app,
    async_mode="eventlet",
    cors_allowed_origins="*",
    logger=False,
    engineio_logger=False,
)

# Wire SocketIO into the singleton manager so background threads possono emettere eventi
manager = RobotManager()
manager.set_socketio(socketio)

# ------------------------------------------------------------------
# Blueprints
# ------------------------------------------------------------------

app.register_blueprint(robot_bp)
app.register_blueprint(gripper_bp)

# ------------------------------------------------------------------
# Health check
# ------------------------------------------------------------------

@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok", **manager.status_dict()}), 200

# ------------------------------------------------------------------
# WebSocket handlers
# ------------------------------------------------------------------

@socketio.on("connect")
def on_connect():
    print(f"[WS] Client connesso: {socketio}")
    socketio.emit("status", manager.status_dict())


@socketio.on("disconnect")
def on_disconnect():
    print("[WS] Client disconnesso")


@socketio.on("get_state")
def on_get_state():
    """Il client può richiedere lo stato corrente via WebSocket."""
    try:
        state = manager.get_robot_state()
        socketio.emit("robot_state", state)
    except Exception as exc:
        socketio.emit("error", {"message": str(exc)})

# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------

if __name__ == "__main__":
    port = int(os.getenv("PORT", 5000))
    host = os.getenv("HOST", "0.0.0.0")
    print(f"[SERVER] Avvio su {host}:{port}")
    print("[SERVER] Robot non connesso — usa POST /api/robot/connect per connetterti")
    socketio.run(app, host=host, port=port)
