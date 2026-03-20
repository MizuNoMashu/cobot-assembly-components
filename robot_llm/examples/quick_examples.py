#!/usr/bin/env python3
"""
Quick examples - Esempi rapidi di utilizzo del controller Franka Robot.

Questo script contiene esempi semplici e diretti per:
- Connessione al robot
- Movimento dei giunti
- Controllo del gripper
- Lettura dello stato
- Semplici workflow

Ogni esempio è una funzione standalone che può essere eseguita indipendentemente.
"""

import numpy as np
from franka_controller import FrankaRobot


# ========== CONFIGURAZIONE ==========
ROBOT_IP = "172.16.0.2"  # Modifica con l'IP del tuo robot


def example_01_connect():
    """Esempio 1: Connessione al robot."""
    print("\n" + "=" * 60)
    print("ESEMPIO 1: Connessione al robot")
    print("=" * 60)

    try:
        robot = FrankaRobot(ROBOT_IP)
        print("✓ Connessione riuscita!")
        return robot
    except Exception as e:
        print(f"✗ Errore: {e}")
        return None


def example_02_read_state(robot: FrankaRobot):
    """Esempio 2: Lettura dello stato del robot."""
    print("\n" + "=" * 60)
    print("ESEMPIO 2: Lettura dello stato del robot")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # Metodo 1: Print formattato
        robot.print_state()

        # Metodo 2: Accesso raw ai dati
        state = robot.get_state()
        print("\n[Dati raw RobotState]")
        print(f"Posizioni giunti: {np.array(state.q)}")
        print(f"Velocità giunti: {np.array(state.dq)}")

        # Metodo 3: Helper functions specifiche
        joint_pos = robot.get_current_joint_positions()
        cartesian_pose = robot.get_current_cartesian_pose()

        print("\n[Helper functions]")
        print("Posizioni giunti (numpy):", joint_pos)
        print("Posizione end-effector (x,y,z):", cartesian_pose[:3, 3])

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_03_move_home(robot: FrankaRobot):
    """Esempio 3: Movimento alla posizione home."""
    print("\n" + "=" * 60)
    print("ESEMPIO 3: Movimento alla posizione home")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # Posizione home standard per Franka Panda
        home_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

        print(f"Target: {np.round(home_position, 3).tolist()}")
        robot.motion.go_to_home(speed_factor=0.2)
        print("✓ Home raggiunta")

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_04_move_joints(robot: FrankaRobot):
    """Esempio 4: Movimento dei giunti verso una configurazione custom."""
    print("\n" + "=" * 60)
    print("ESEMPIO 4: Movimento giunti custom")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # Configurazione target (radianti)
        target_position = [0.5, -0.5, 0.0, -2.0, 0.0, 1.5, 1.0]

        print(f"Target: {np.round(target_position, 3).tolist()}")
        robot.motion.move_to_joint_positions(target_position, speed_factor=0.2)
        print("✓ Posizione raggiunta")

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_05_move_relative(robot: FrankaRobot):
    """Esempio 5: Movimento relativo dei giunti."""
    print("\n" + "=" * 60)
    print("ESEMPIO 5: Movimento relativo")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # Stampa posizione corrente
        current_pos = robot.get_current_joint_positions()
        print(f"Posizione corrente: {np.round(current_pos, 3).tolist()}")

        # Definisci movimento relativo (solo joint 1 si muove di 0.2 rad)
        delta = [0.2, 0, 0, 0, 0, 0, 0]

        print(f"Delta: {delta}")
        robot.motion.move_relative(delta, speed_factor=0.2)

        # Stampa nuova posizione
        new_pos = robot.get_current_joint_positions()
        print(f"Nuova posizione: {np.round(new_pos, 3).tolist()}")
        print("✓ Movimento relativo completato")

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_06_gripper_homing(robot: FrankaRobot):
    """Esempio 6: Homing del gripper."""
    print("\n" + "=" * 60)
    print("ESEMPIO 6: Homing del gripper")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        robot.gripper.homing()
        print("✓ Homing completato")
    except Exception as e:
        print(f"✗ Errore: {e}")


def example_07_gripper_open_close(robot: FrankaRobot):
    """Esempio 7: Apertura e chiusura del gripper."""
    print("\n" + "=" * 60)
    print("ESEMPIO 7: Apertura e chiusura gripper")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # Apri completamente (80mm)
        print("Apertura gripper a 80mm...")
        robot.gripper.open(width=0.08, speed=0.1)

        import time

        time.sleep(1)

        # Chiudi completamente
        print("Chiusura gripper...")
        robot.gripper.close(speed=0.1)

        print("✓ Sequenza completata")

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_08_gripper_grasp(robot: FrankaRobot):
    """Esempio 8: Afferrare un oggetto con il gripper."""
    print("\n" + "=" * 60)
    print("ESEMPIO 8: Presa di un oggetto")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # Parametri dell'oggetto
        object_width = 0.05  # 50mm
        grasp_force = 60.0  # 60N

        print(f"Tentativo di presa: larghezza={object_width * 1000}mm, forza={grasp_force}N")

        success = robot.gripper.grasp(width=object_width, force=grasp_force, speed=0.1)

        if success:
            print("✓ Oggetto afferrato con successo")

            # Verifica stato
            state = robot.gripper.get_state()
            if state:
                print(f"Larghezza gripper: {state['width'] * 1000:.1f}mm")
                print(f"Oggetto afferrato: {state['is_grasped']}")
        else:
            print("✗ Presa fallita")

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_09_gripper_state(robot: FrankaRobot):
    """Esempio 9: Lettura dello stato del gripper."""
    print("\n" + "=" * 60)
    print("ESEMPIO 9: Stato del gripper")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # Metodo 1: Print formattato
        robot.gripper.print_state()

        # Metodo 2: Accesso raw ai dati
        state = robot.gripper.get_state()
        if state:
            print("\n[Dati raw]")
            print(f"Larghezza: {state['width']}m")
            print(f"Max width: {state['max_width']}m")
            print(f"Temperatura: {state['temperature']}°C")
            print(f"Is grasping: {state['is_grasped']}")

        # Metodo 3: Check rapido
        is_grasping = robot.gripper.is_grasping()
        print(f"\n[Check rapido] È afferrato un oggetto? {is_grasping}")

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_10_impedance_control(robot: FrankaRobot):
    """Esempio 10: Controllo impedenza."""
    print("\n" + "=" * 60)
    print("ESEMPIO 10: Controllo impedenza")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # Configurazione target
        target_position = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]

        # Stiffness personalizzata (più bassa = più compliant)
        stiffness = [400, 400, 400, 400, 200, 100, 50]

        print(f"Target: {np.round(target_position, 3).tolist()}")
        print(f"Stiffness: {stiffness}")

        robot.motion.impedance_control(target_position, stiffness=stiffness, duration=5.0)

        print("✓ Controllo impedenza completato")

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_11_collision_behavior(robot: FrankaRobot):
    """Esempio 11: Configurazione del collision behavior."""
    print("\n" + "=" * 60)
    print("ESEMPIO 11: Configurazione collision behavior")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # Soglie custom (più basse = più sensibile)
        lower_torque = [15.0, 15.0, 15.0, 15.0, 12.0, 10.0, 8.0]
        upper_torque = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        lower_force = [15.0, 15.0, 15.0, 20.0, 20.0, 20.0]
        upper_force = [20.0, 20.0, 18.0, 25.0, 25.0, 25.0]

        print("Configurazione soglie di collisione...")
        robot.set_collision_behavior(lower_torque, upper_torque, lower_force, upper_force)

        print("✓ Collision behavior configurato")

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_12_error_recovery(robot: FrankaRobot):
    """Esempio 12: Recovery automatico da errori."""
    print("\n" + "=" * 60)
    print("ESEMPIO 12: Recovery automatico da errori")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        print("Tentativo di recovery automatico...")
        success = robot.automatic_error_recovery()

        if success:
            print("✓ Recovery completato con successo")
        else:
            print("✗ Recovery fallito")

    except Exception as e:
        print(f"✗ Errore: {e}")


def example_13_simple_pick_place(robot: FrankaRobot):
    """Esempio 13: Semplice workflow di pick-and-place."""
    print("\n" + "=" * 60)
    print("ESEMPIO 13: Simple Pick-and-Place Workflow")
    print("=" * 60)

    if robot is None:
        print("✗ Robot non connesso")
        return

    try:
        # 1. Home position
        print("\n[1/6] Movimento a home...")
        robot.motion.go_to_home(speed_factor=0.2)

        # 2. Open gripper
        print("\n[2/6] Apertura gripper...")
        robot.gripper.open(width=0.08, speed=0.1)

        # 3. Move to pick position (simulata)
        print("\n[3/6] Movimento verso pick position...")
        pick_pos = [0.3, -0.5, 0, -2.0, 0, 1.5, 0.785]
        robot.motion.move_to_joint_positions(pick_pos, speed_factor=0.15)

        # 4. Grasp object
        print("\n[4/6] Presa oggetto...")
        robot.gripper.grasp(width=0.05, force=60.0, speed=0.1)

        # 5. Move to place position (simulata)
        print("\n[5/6] Movimento verso place position...")
        place_pos = [-0.3, -0.5, 0, -2.0, 0, 1.5, 0.785]
        robot.motion.move_to_joint_positions(place_pos, speed_factor=0.15)

        # 6. Release object
        print("\n[6/6] Rilascio oggetto...")
        robot.gripper.release()

        print("\n✓ Pick-and-place completato con successo!")

    except Exception as e:
        print(f"✗ Errore: {e}")
        print("Tentativo di recovery...")
        robot.automatic_error_recovery()


def example_14_context_manager(robot_ip: str = ROBOT_IP):
    """Esempio 14: Uso del context manager (with statement)."""
    print("\n" + "=" * 60)
    print("ESEMPIO 14: Context Manager")
    print("=" * 60)

    try:
        # Il context manager gestisce automaticamente connessione/disconnessione
        with FrankaRobot(robot_ip) as robot:
            print("✓ Robot connesso")

            # Esegui operazioni
            robot.print_state()

            print("✓ Operazioni completate")

        print("✓ Context manager chiuso correttamente")

    except Exception as e:
        print(f"✗ Errore: {e}")


# ========== MAIN - Esegui tutti gli esempi ==========


def main():
    """Esegue tutti gli esempi in sequenza."""
    print("=" * 60)
    print("QUICK EXAMPLES - FRANKA ROBOT CONTROLLER")
    print("=" * 60)
    print(f"IP Robot: {ROBOT_IP}")
    print("")

    # Connessione
    robot = example_01_connect()

    if robot is None:
        print("\n✗ Impossibile connettersi al robot. Verifica l'IP e la connessione.")
        return

    # Menu di selezione
    print("\n" + "=" * 60)
    print("SELEZIONA ESEMPI DA ESEGUIRE")
    print("=" * 60)
    print("1. Stato del robot")
    print("2. Movimento a home")
    print("3. Movimento giunti custom")
    print("4. Movimento relativo")
    print("5. Gripper homing")
    print("6. Gripper apri/chiudi")
    print("7. Gripper grasp")
    print("8. Stato gripper")
    print("9. Controllo impedenza")
    print("10. Collision behavior")
    print("11. Error recovery")
    print("12. Simple pick-and-place")
    print("13. Context manager demo")
    print("0. Esegui tutti")
    print("=" * 60)

    choice = input("\nScegli un esempio (0-13): ")

    examples = {
        "1": lambda: example_02_read_state(robot),
        "2": lambda: example_03_move_home(robot),
        "3": lambda: example_04_move_joints(robot),
        "4": lambda: example_05_move_relative(robot),
        "5": lambda: example_06_gripper_homing(robot),
        "6": lambda: example_07_gripper_open_close(robot),
        "7": lambda: example_08_gripper_grasp(robot),
        "8": lambda: example_09_gripper_state(robot),
        "9": lambda: example_10_impedance_control(robot),
        "10": lambda: example_11_collision_behavior(robot),
        "11": lambda: example_12_error_recovery(robot),
        "12": lambda: example_13_simple_pick_place(robot),
        "13": lambda: example_14_context_manager(),
    }

    if choice == "0":
        # Esegui tutti gli esempi
        print("\n[Esecuzione di tutti gli esempi...]")
        for key in sorted(examples.keys()):
            try:
                examples[key]()
                input("\nPremi ENTER per continuare...")
            except KeyboardInterrupt:
                print("\n\n[Interruzione utente]")
                break
            except Exception as e:
                print(f"\n✗ Errore imprevisto: {e}")
                input("\nPremi ENTER per continuare...")
    elif choice in examples:
        # Esegui esempio selezionato
        try:
            examples[choice]()
        except Exception as e:
            print(f"\n✗ Errore: {e}")
    else:
        print("✗ Scelta non valida")

    print("\n" + "=" * 60)
    print("Quick Examples completati!")
    print("=" * 60)


if __name__ == "__main__":
    main()
