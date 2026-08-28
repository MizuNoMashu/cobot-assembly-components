#!/usr/bin/env python3
"""
Script di debug per testare le operazioni di movimento del `MotionController`.

Stile e flow simili a `debug_desk_api.py`:
- intestazione informativa
- passi numerati
- try/except che stampa tipo ed errore

Questo script legge le variabili d'ambiente da `credential.env` (se presente)
e poi permette di eseguire i test di movimento uno per uno con prompt interattivo
in modo da poter mettere breakpoint prima di ogni invio di comando.
"""

import sys
import os
import time
from pathlib import Path

# Aggiunge automaticamente la cartella src se lo script viene eseguito dalla root
script_dir = Path(__file__).resolve().parent
src_candidates = [script_dir / "src", script_dir.parent / "src", Path.cwd() / "cobot" / "src"]
for p in src_candidates:
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))
        break


def load_env_file(path: Path) -> None:
    if not path.exists():
        return
    with open(path, "r", encoding="utf-8") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if "#" in line:
                line = line.split("#", 1)[0].strip()
            if line.endswith(","):
                line = line[:-1].strip()
            if "=" not in line:
                continue
            key, val = line.split("=", 1)
            key = key.strip()
            val = val.strip().strip('"').strip("'")
            os.environ.setdefault(key, val)


repo_root = script_dir.parent
load_env_file(repo_root / "credential.env")


print("=" * 70)
print("TEST: MotionController via pylibfranka")
print("=" * 70)

try:
    print("\n[1/4] Creazione client FrankaRobot...")
    from franka_controller import FrankaRobot

    robot_ip = os.environ.get("robot_ip") or os.environ.get("FRANKA_ROBOT_IP")
    if not robot_ip:
        raise RuntimeError("FRANKA_ROBOT_IP non impostato in credential.env")

    # Per il debug in locale usiamo enforce_realtime=False di default
    enforce_realtime = os.environ.get("ENFORCE_REALTIME", "false").lower() in ("1", "true", "yes")

    robot = FrankaRobot(robot_ip=robot_ip, enforce_realtime=enforce_realtime)
    print("  ✓ Client FrankaRobot creato")

    print("\n[2/4] Stato iniziale del robot (get_state/print_state)...")
    try:
        robot.print_state()
        print("  ✓ Stato letto")
    except Exception as e:
        print(f"  ! Warning: errore lettura stato: {e}")

    # Lista semplificata di test: manteniamo i singoli passi con prompt
    tests = [
        ("go_to_home", lambda r: r.motion.go_to_home(speed_factor=0.5)),
        ("move_joints", lambda r: r.motion.move_to_joint_positions((r.get_current_joint_positions() + [0.05,0,0,0,0,0,0]).tolist(), speed_factor=0.5)),
        ("move_relative", lambda r: r.motion.move_relative([0.02,0,0,0,0,0,0], speed_factor=0.5)),
        ("execute_trajectory", lambda r: r.motion.execute_trajectory([r.get_current_joint_positions().tolist(), (r.get_current_joint_positions()+[0.03,0,-0.03,0,0,0,0]).tolist(), r.get_current_joint_positions().tolist()], speed_factor=0.4)),
        ("move_cartesian_relative", lambda r: r.motion.move_cartesian_relative(dz=0.03, speed_factor=0.4)),
        ("impedance_control", lambda r: r.motion.impedance_control((r.get_current_joint_positions()+[0.02,0,0,0,0,0,0]).tolist(), duration=3.0)),
    ]

    print("\n[3/4] Esecuzione test di movimento (interattivo):")
    for name, fn in tests:
        print(f"\n--- Test: {name} ---")
        input(f"Premi Enter per eseguire '{name}' (metti breakpoint ora, Ctrl-C per abortire)...")
        try:
            fn(robot)
            print(f"  ✓ Test '{name}' eseguito")
        except Exception as e:
            print(f"  ✗ Errore durante '{name}': {type(e).__name__}: {e}")
            raise

    print("\n[4/4] Pulizia e recovery finale")
    try:
        robot.automatic_error_recovery()
    except Exception:
        pass

    print("\n" + "=" * 70)
    print("✓✓✓ TUTTI I TEST ESEGUITI (verifica output/telemetria)")
    print("=" * 70)

except Exception as e:
    print(f"\n✗✗✗ ERRORE:\n  Tipo: {type(e).__name__}\n  Messaggio: {e}\n")
    print("\n" + "=" * 70)
