"""
Entry point Flask + Flask-SocketIO per il cobot API server.

eventlet deve essere importato e monkey-patched PRIMA di qualsiasi altro import
per garantire il funzionamento corretto di Flask-SocketIO in modalità asincrona.
"""

import os

# Monkey patch eventlet solo se non siamo in modalità development (debug)
if os.getenv("FLASK_ENV") != "development":
    import eventlet
    eventlet.monkey_patch()

from flask import Flask, jsonify, redirect
from flask_socketio import SocketIO
from flasgger import Swagger

from app.robot_manager import RobotManager
from app.routes.robot import robot_bp
from app.routes.gripper import gripper_bp

# ------------------------------------------------------------------
# App setup
# ------------------------------------------------------------------

app = Flask(__name__)
app.config["SECRET_KEY"] = os.getenv("SECRET_KEY", "franka-cobot-secret")

swagger_template = {
    "swagger": "2.0",
    "info": {
        "title": "Cobot API",
        "description": "REST API for Franka cobot control and telemetry.",
        "version": "1.0.0",
    },
    "basePath": "/",
    "schemes": ["http"],
}

swagger_config = {
    "headers": [],
    "specs": [
        {
            "endpoint": "apispec_1",
            "route": "/api/docs/apispec.json",
            "rule_filter": lambda rule: True,
            "model_filter": lambda tag: True,
        }
    ],
    "static_url_path": "/api/docs/flasgger_static",
    "swagger_ui": True,
    "specs_route": "/api/docs/",
}

swagger = Swagger(app, template=swagger_template, config=swagger_config)

socketio = SocketIO(
    app,
    async_mode="eventlet" if os.getenv("FLASK_ENV") != "development" else "threading",
    cors_allowed_origins="*",
    logger=False,
    engineio_logger=False,
)


@app.route("/openapi.json", methods=["GET"])
def openapi_spec():
    """OpenAPI spec endpoint.

    Ritorna il documento OpenAPI/Swagger in formato JSON.
    """
    specs = swagger.get_apispecs()
    if specs:
        return jsonify(specs[0])
    return jsonify({}), 404




@app.route("/", methods=["GET"])
def root_redirect():
    """Redirect root alla UI di Swagger."""
    return redirect("/api/docs/")


@app.route("/swagger", methods=["GET"])
def swagger_redirect():
    """Redirect alias per la UI di Swagger."""
    return redirect("/api/docs/")


@app.route("/docs", methods=["GET"])
def docs_redirect():
    """Redirect alias per la documentazione Swagger."""
    return redirect("/api/docs/")


@app.route("/openapi", methods=["GET"])
def openapi_redirect():
    """Redirect alias al documento OpenAPI JSON."""
    return redirect("/openapi.json")


@app.route("/api/openapi", methods=["GET"])
def api_openapi_redirect():
    return redirect("/openapi.json")


@app.route("/api/swagger", methods=["GET"])
def api_swagger_redirect():
    return redirect("/api/docs/")


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
    """Health check endpoint.

    Returns the current status of the cobot API server and robot connection.
    ---
    tags:
      - Health
    responses:
      200:
        description: Server health status.
        schema:
          type: object
          properties:
            status:
              type: string
              example: ok
            connected:
              type: boolean
            busy:
              type: boolean
            mode:
              type: string
    """
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
